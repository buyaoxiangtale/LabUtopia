#!/usr/bin/env python3
"""
路径规划失败分析工具

功能：
1. 分析失败的路径规划图像
2. 识别与障碍物发生碰撞的具体物体
3. 支持修改场景 JSON 文件以移除/调整障碍物
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import cv2
from dataclasses import dataclass
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


@dataclass
class BoundingBox:
    """2D 边界框"""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float) -> bool:
        """检查点是否在边界框内"""
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def expand(self, radius: float) -> 'BoundingBox':
        """膨胀边界框"""
        return BoundingBox(
            x_min=self.x_min - radius,
            x_max=self.x_max + radius,
            y_min=self.y_min - radius,
            y_max=self.y_max + radius
        )

    def distance_to_edge(self, x: float, y: float) -> float:
        """
        计算点到边界框边缘的最短距离

        Args:
            x, y: 点的坐标

        Returns:
            到边界框边缘的最短距离（如果点在边界框内，返回负值表示穿透深度）
        """
        # 如果点在边界框内，返回负值（表示穿透深度）
        if self.contains(x, y):
            # 计算到各边的距离，取最小值（负数）
            dx_min = x - self.x_min
            dx_max = self.x_max - x
            dy_min = y - self.y_min
            dy_max = self.y_max - y
            return -min(dx_min, dx_max, dy_min, dy_max)

        # 点在边界框外，计算到边界的最短距离
        dx = max(self.x_min - x, 0, x - self.x_max)
        dy = max(self.y_min - y, 0, y - self.y_max)

        return (dx**2 + dy**2)**0.5


@dataclass
class SceneObject:
    """场景物体"""
    id: str
    name: str
    position: Tuple[float, float, float]  # (x, y, z)
    rotation: Tuple[float, float, float]  # (x, y, z) degrees
    bbox_2d: BoundingBox
    original_bbox: BoundingBox  # 原始边界框（未膨胀）
    asset_info: Optional[Dict] = None


class CollisionAnalyzer:
    """碰撞分析器"""

    def __init__(
        self,
        scene_json_path: str,
        assets_json_path: str,
        offset_radius: float = 0.3,
        exclude_infrastructure: bool = True
    ):
        """
        初始化分析器

        Args:
            scene_json_path: 场景布局 JSON 文件
            assets_json_path: 资产库 JSON 文件
            offset_radius: 障碍物膨胀半径（米）
            exclude_infrastructure: 是否排除基础设施类物体（如 LaboratoryRoom）
        """
        self.scene_json_path = Path(scene_json_path)
        self.assets_json_path = Path(assets_json_path)
        self.offset_radius = offset_radius
        self.exclude_infrastructure = exclude_infrastructure

        # 加载数据
        with open(scene_json_path, 'r') as f:
            self.scene_data = json.load(f)

        with open(assets_json_path, 'r') as f:
            self.assets_data = json.load(f)

        # 构建资产查找表
        self.asset_lookup = {}
        for asset in self.assets_data['assets']:
            self.asset_lookup[asset['id']] = asset

        # 解析场景物体
        self.objects = self._parse_scene_objects()

        print(f"✓ 加载场景: {self.scene_json_path.name}")
        print(f"  物体数量: {len(self.objects)}")
        print(f"  膨胀半径: {offset_radius} 米")
        if exclude_infrastructure:
            print(f"  排除基础设施: 是")

    def _parse_scene_objects(self) -> List[SceneObject]:
        """解析场景中的物体"""
        objects = []

        for obj_data in self.scene_data.get('objects', []):
            obj_id = obj_data['id']
            position = obj_data['position']
            rotation = obj_data.get('rotation', {'x': 0, 'y': 0, 'z': 0})

            # 获取资产信息
            asset = self.asset_lookup.get(obj_id)
            
            # 如果启用了排除基础设施，检查物体类别
            if self.exclude_infrastructure and asset:
                if 'semantic' in asset:
                    category = asset['semantic'].get('category', '')
                    subtype = asset['semantic'].get('subtype', '')
                    # 排除房间基础设施
                    if category == 'infrastructure' or subtype in ['spatial_entity', 'room']:
                        continue

            if asset and 'geometry' in asset and 'bbox' in asset['geometry']:
                bbox_data = asset['geometry']['bbox']

                # 2D 边界框（考虑旋转）
                bbox_2d = self._calculate_2d_bbox(
                    position['x'], position['y'],
                    bbox_data['long'], bbox_data['short'],
                    rotation.get('z', 0)
                )

                # 原始边界框
                original_bbox = bbox_2d

                # 膨胀后的边界框
                expanded_bbox = bbox_2d.expand(self.offset_radius)
            else:
                # 没有几何信息，使用默认大小
                default_size = 0.5
                bbox_2d = BoundingBox(
                    x_min=position['x'] - default_size/2,
                    x_max=position['x'] + default_size/2,
                    y_min=position['y'] - default_size/2,
                    y_max=position['y'] + default_size/2
                )
                original_bbox = bbox_2d
                expanded_bbox = bbox_2d.expand(self.offset_radius)

            obj = SceneObject(
                id=obj_id,
                name=obj_id,
                position=(position['x'], position['y'], position.get('z', 0)),
                rotation=(rotation.get('x', 0), rotation.get('y', 0), rotation.get('z', 0)),
                bbox_2d=expanded_bbox,
                original_bbox=original_bbox,
                asset_info=asset
            )
            objects.append(obj)

        return objects

    def _calculate_2d_bbox(
        self,
        cx: float, cy: float,  # 中心点
        length: float, width: float,  # 长和宽
        rotation_z: float  # Z轴旋转（度）
    ) -> BoundingBox:
        """
        计算2D边界框（考虑旋转）

        Args:
            cx, cy: 中心坐标
            length: 长度（沿 Y 轴，如果未旋转）
            width: 宽度（沿 X 轴，如果未旋转）
            rotation_z: Z轴旋转角度（度）

        Returns:
            BoundingBox: 2D边界框
        """
        # 如果旋转是 90 度的倍数，简化计算
        rot = rotation_z % 180
        if rot == 0 or rot == 180:
            # 未旋转或翻转
            half_l = length / 2
            half_w = width / 2
            return BoundingBox(
                x_min=cx - half_w,
                x_max=cx + half_w,
                y_min=cy - half_l,
                y_max=cy + half_l
            )
        elif rot == 90:
            # 旋转 90 度
            half_l = length / 2
            half_w = width / 2
            return BoundingBox(
                x_min=cx - half_l,
                x_max=cx + half_l,
                y_min=cy - half_w,
                y_max=cy + half_w
            )

        # 对于任意旋转角度，使用外接矩形
        # 这是一种简化处理，更精确的方法是使用旋转矩形的四个角点
        half_l = length / 2
        half_w = width / 2

        # 旋转前的四个角点
        corners = [
            (-half_w, -half_l),
            (half_w, -half_l),
            (half_w, half_l),
            (-half_w, half_l)
        ]

        # 旋转并平移
        rad = np.radians(rotation_z)
        cos_r = np.cos(rad)
        sin_r = np.sin(rad)

        rotated_corners = []
        for x, y in corners:
            rx = x * cos_r - y * sin_r + cx
            ry = x * sin_r + y * cos_r + cy
            rotated_corners.append((rx, ry))

        # 计算外接矩形
        xs = [p[0] for p in rotated_corners]
        ys = [p[1] for p in rotated_corners]

        return BoundingBox(
            x_min=min(xs),
            x_max=max(xs),
            y_min=min(ys),
            y_max=max(ys)
        )

    def find_colliding_objects(
        self,
        x: float,
        y: float,
        check_expanded: bool = True
    ) -> List[SceneObject]:
        """
        查找与指定点碰撞的物体

        Args:
            x, y: 世界坐标
            check_expanded: 是否检查膨胀后的边界框

        Returns:
            碰撞的物体列表
        """
        colliding = []

        for obj in self.objects:
            bbox = obj.bbox_2d if check_expanded else obj.original_bbox

            if bbox.contains(x, y):
                colliding.append(obj)

        return colliding

    def find_colliding_objects_along_path(
        self,
        path: List[Tuple[float, float]],
        check_expanded: bool = True,
        find_first_only: bool = False
    ) -> List[SceneObject]:
        """
        查找与路径碰撞的物体

        Args:
            path: 路径点列表 [(x1, y1), (x2, y2), ...]
            check_expanded: 是否检查膨胀后的边界框
            find_first_only: 是否只返回第一个碰撞点（最近的失败点）

        Returns:
            碰撞的物体列表
        """
        if find_first_only:
            # 只返回第一个碰撞点（最近的失败点）
            for i, (x, y) in enumerate(path):
                objs = self.find_colliding_objects(x, y, check_expanded)
                if objs:
                    # 找到第一个碰撞点，返回该点碰撞的物体
                    return objs
            # 没有碰撞
            return []
        else:
            # 返回所有碰撞的物体（原有逻辑）
            colliding_objects = {}  # 使用字典来去重（key为物体ID）

            for x, y in path:
                objs = self.find_colliding_objects(x, y, check_expanded)
                for obj in objs:
                    colliding_objects[obj.id] = obj

            return list(colliding_objects.values())

    def find_first_collision_point(
        self,
        path: List[Tuple[float, float]],
        check_expanded: bool = True
    ) -> Optional[Dict]:
        """
        查找路径上第一个碰撞点（最近的失败点）

        Args:
            path: 路径点列表 [(x1, y1), (x2, y2), ...]
            check_expanded: 是否检查膨胀后的边界框

        Returns:
            如果找到碰撞，返回包含碰撞信息的字典：
            {
                'index': 碰撞点在路径中的索引,
                'position': (x, y) 碰撞点坐标,
                'objects': [SceneObject] 碰撞的物体列表
            }
            如果没有碰撞，返回 None
        """
        for i, (x, y) in enumerate(path):
            objs = self.find_colliding_objects(x, y, check_expanded)
            if objs:
                return {
                    'index': i,
                    'position': (x, y),
                    'objects': objs
                }
        return None

    def analyze_failed_task(
        self,
        failed_task_path: str,
        barrier_image_path: Optional[str] = None,
        find_first_collision_only: bool = False
    ) -> Dict:
        """
        分析失败的路径规划任务

        Args:
            failed_task_path: 失败任务 JSON 文件路径
            barrier_image_path: 障碍物图像路径（可选，用于可视化）
            find_first_collision_only: 是否只分析第一个碰撞点

        Returns:
            分析结果字典
        """
        # 加载失败任务
        with open(failed_task_path, 'r') as f:
            failed_task = json.load(f)

        results = []

        for task in failed_task:
            task_id = task['task_id']

            start_pos = (task['start'][0], task['start'][1])
            end_pos = (task['end'][0], task['end'][1])

            # 检查起点和终点
            start_collisions = self.find_colliding_objects(*start_pos)
            end_collisions = self.find_colliding_objects(*end_pos)

            # 生成详细的碰撞报告
            collision_report = {
                'start_blocked': len(start_collisions) > 0,
                'end_blocked': len(end_collisions) > 0,
                'blocking_objects': {
                    'start': [],
                    'end': []
                }
            }

            # 详细报告起点阻隔物体
            if start_collisions:
                for obj in start_collisions:
                    # 计算到膨胀边界框边缘的距离
                    distance_to_bbox_edge = obj.bbox_2d.distance_to_edge(*start_pos)

                    obj_info = {
                        'id': obj.id,
                        'position': (obj.position[0], obj.position[1]),
                        'distance': distance_to_bbox_edge,  # 到边界框边缘的距离
                        'bbox_edge': f"[{obj.bbox_2d.x_min:.2f}, {obj.bbox_2d.x_max:.2f}] x [{obj.bbox_2d.y_min:.2f}, {obj.bbox_2d.y_max:.2f}]"
                    }
                    if obj.asset_info:
                        if 'semantic' in obj.asset_info:
                            obj_info['category'] = obj.asset_info['semantic'].get('category', 'Unknown')
                        if 'geometry' in obj.asset_info and 'bbox' in obj.asset_info['geometry']:
                            bbox = obj.asset_info['geometry']['bbox']
                            obj_info['size'] = {
                                'long': bbox.get('long', 0),
                                'short': bbox.get('short', 0)
                            }
                    collision_report['blocking_objects']['start'].append(obj_info)

            # 详细报告终点阻隔物体
            if end_collisions:
                for obj in end_collisions:
                    # 计算到膨胀边界框边缘的距离
                    distance_to_bbox_edge = obj.bbox_2d.distance_to_edge(*end_pos)

                    obj_info = {
                        'id': obj.id,
                        'position': (obj.position[0], obj.position[1]),
                        'distance': distance_to_bbox_edge,  # 到边界框边缘的距离
                        'bbox_edge': f"[{obj.bbox_2d.x_min:.2f}, {obj.bbox_2d.x_max:.2f}] x [{obj.bbox_2d.y_min:.2f}, {obj.bbox_2d.y_max:.2f}]"
                    }
                    if obj.asset_info:
                        if 'semantic' in obj.asset_info:
                            obj_info['category'] = obj.asset_info['semantic'].get('category', 'Unknown')
                        if 'geometry' in obj.asset_info and 'bbox' in obj.asset_info['geometry']:
                            bbox = obj.asset_info['geometry']['bbox']
                            obj_info['size'] = {
                                'long': bbox.get('long', 0),
                                'short': bbox.get('short', 0)
                            }
                    collision_report['blocking_objects']['end'].append(obj_info)

            # 如果有路径信息，检查路径
            path_collisions = []
            first_collision_point = None

            if 'path' in task:
                path = [(p[0], p[1]) for p in task['path']]

                if find_first_collision_only:
                    # 只获取第一个碰撞点
                    first_collision_point = self.find_first_collision_point(path)
                    if first_collision_point:
                        path_collisions = first_collision_point['objects']

                        # 添加第一个碰撞点的详细信息
                        collision_report['path_blocked'] = True
                        collision_report['first_collision_point'] = {
                            'index': first_collision_point['index'],
                            'position': first_collision_point['position'],
                            'blocking_objects': []
                        }
                        for obj in first_collision_point['objects']:
                            obj_info = {
                                'id': obj.id,
                                'position': (obj.position[0], obj.position[1])
                            }
                            if obj.asset_info:
                                if 'semantic' in obj.asset_info:
                                    obj_info['category'] = obj.asset_info['semantic'].get('category', 'Unknown')
                            collision_report['first_collision_point']['blocking_objects'].append(obj_info)
                else:
                    # 获取所有碰撞物体
                    path_collisions = self.find_colliding_objects_along_path(path)
                    collision_report['path_blocked'] = len(path_collisions) > 0

            result = {
                'task_id': task_id,
                'start_position': start_pos,
                'end_position': end_pos,
                'collision_report': collision_report,
                'start_collisions': [obj.id for obj in start_collisions],
                'end_collisions': [obj.id for obj in end_collisions],
                'path_collisions': [obj.id for obj in path_collisions],
                'all_colliding_objects': list(set(
                    [obj.id for obj in start_collisions] +
                    [obj.id for obj in end_collisions] +
                    [obj.id for obj in path_collisions]
                )),
                'failure_reason': task.get('failure_reason', ''),
                'failure_category': task.get('failure_category', '')
            }

            results.append(result)

        return {
            'failed_tasks': results,
            'offset_radius': self.offset_radius
        }

    def visualize_collisions(
        self,
        failed_task_path: str,
        barrier_image_path: Optional[str] = None,
        output_path: Optional[str] = None
    ):
        """
        可视化碰撞情况

        Args:
            failed_task_path: 失败任务文件
            barrier_image_path: 障碍物图像（可选）
            output_path: 输出图像路径
        """
        analysis = self.analyze_failed_task(failed_task_path, barrier_image_path)

        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        # 左图：场景布局
        ax1 = axes[0]

        # 绘制所有物体
        for obj in self.objects:
            # 绘制原始边界框（虚线）
            bbox = obj.original_bbox
            width = bbox.x_max - bbox.x_min
            height = bbox.y_max - bbox.y_min
            rect = Rectangle(
                (bbox.x_min, bbox.y_min),
                width, height,
                fill=False, edgecolor='blue', linestyle='--', linewidth=1,
                label='Original' if obj == self.objects[0] else ''
            )
            ax1.add_patch(rect)

            # 绘制膨胀边界框（实线）
            bbox_exp = obj.bbox_2d
            width_exp = bbox_exp.x_max - bbox_exp.x_min
            height_exp = bbox_exp.y_max - bbox_exp.y_min
            rect_exp = Rectangle(
                (bbox_exp.x_min, bbox_exp.y_min),
                width_exp, height_exp,
                fill=True, facecolor='red', alpha=0.1,
                edgecolor='red', linestyle='-', linewidth=1,
                label='Expanded' if obj == self.objects[0] else ''
            )
            ax1.add_patch(rect_exp)

            # 标注物体名称
            ax1.text(
                obj.position[0], obj.position[1],
                obj.id[:10] + '...',
                fontsize=6, ha='center', va='center'
            )

        # 绘制失败的路径
        for task in analysis['failed_tasks']:
            start = task['start_position']
            end = task['end_position']

            # 绘制起点和终点
            ax1.plot(start[0], start[1], 'go', markersize=8, label='Start' if task == analysis['failed_tasks'][0] else '')
            ax1.plot(end[0], end[1], 'ro', markersize=8, label='End' if task == analysis['failed_tasks'][0] else '')

            # 绘制连线
            ax1.plot([start[0], end[0]], [start[1], end[1]], 'g--', linewidth=1, alpha=0.5)

            # 标注碰撞物体
            if task['all_colliding_objects']:
                for obj_id in task['all_colliding_objects']:
                    obj = next((o for o in self.objects if o.id == obj_id), None)
                    if obj:
                        ax1.text(
                            obj.position[0], obj.position[1] + 0.2,
                            f"✗ {obj_id}",
                            fontsize=7, color='red', ha='center', weight='bold'
                        )

        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Scene Layout with Collisions')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')

        # 右图：分析结果文本
        ax2 = axes[1]
        ax2.axis('off')

        text = "COLLISION ANALYSIS RESULTS\n\n"
        text += f"Offset Radius: {analysis['offset_radius']} m\n\n"

        for i, task in enumerate(analysis['failed_tasks'], 1):
            text += f"Task {i}: {task['task_id']}\n"
            text += f"  Start: {task['start_position']}\n"
            text += f"  End: {task['end_position']}\n"
            text += f"  Failure: {task['failure_reason']}\n"

            if task['start_collisions']:
                text += f"  ✗ Collisions at start: {', '.join(task['start_collisions'])}\n"

            if task['end_collisions']:
                text += f"  ✗ Collisions at end: {', '.join(task['end_collisions'])}\n"

            if task['path_collisions']:
                text += f"  ✗ Collisions along path: {', '.join(task['path_collisions'])}\n"

            text += "\n"

        ax2.text(0.05, 0.95, text, transform=ax2.transAxes,
                fontsize=9, verticalalignment='top', family='monospace')

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"✓ 可视化保存到: {output_path}")
        else:
            plt.show()

        plt.close()

    def remove_object_from_scene(
        self,
        object_id: str,
        output_json_path: str
    ):
        """
        从场景中移除指定物体

        Args:
            object_id: 物体 ID
            output_json_path: 输出 JSON 文件路径
        """
        # 过滤掉指定物体
        new_objects = [
            obj for obj in self.scene_data['objects']
            if obj['id'] != object_id
        ]

        if len(new_objects) == len(self.scene_data['objects']):
            print(f"⚠️  物体 '{object_id}' 未找到")
            return

        # 更新场景数据
        self.scene_data['objects'] = new_objects

        # 保存
        output_path = Path(output_json_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w') as f:
            json.dump(self.scene_data, f, indent=2, ensure_ascii=False)

        print(f"✓ 已移除物体 '{object_id}'")
        print(f"  保存到: {output_json_path}")

        # 重新解析
        self.objects = self._parse_scene_objects()

    def adjust_object_position(
        self,
        object_id: str,
        new_x: float,
        new_y: float,
        output_json_path: str,
        new_z: Optional[float] = None
    ):
        """
        调整物体位置

        Args:
            object_id: 物体 ID
            new_x, new_y, new_z: 新位置
            output_json_path: 输出 JSON 文件路径
        """
        found = False

        for obj in self.scene_data['objects']:
            if obj['id'] == object_id:
                obj['position']['x'] = new_x
                obj['position']['y'] = new_y
                if new_z is not None:
                    obj['position']['z'] = new_z
                found = True
                break

        if not found:
            print(f"⚠️  物体 '{object_id}' 未找到")
            return

        # 保存
        output_path = Path(output_json_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w') as f:
            json.dump(self.scene_data, f, indent=2, ensure_ascii=False)

        print(f"✓ 已调整物体 '{object_id}' 位置")
        print(f"  新位置: ({new_x}, {new_y}, {new_z})")
        print(f"  保存到: {output_json_path}")

        # 重新解析
        self.objects = self._parse_scene_objects()


def main():
    import sys

    if len(sys.argv) < 4:
        print("用法:")
        print("  python collision_analyzer.py <scene_json> <assets_json> <failed_tasks_json> [options]")
        print()
        print("选项:")
        print("  --visualize <output_path>     生成可视化图像")
        print("  --remove <object_id>          移除物体")
        print("  --move <object_id> <x> <y>    移动物体到新位置")
        print("  --output <json_path>          输出修改后的 JSON")
        print("  --offset-radius <radius>      设置膨胀半径")
        print()
        print("示例:")
        print("  # 分析失败任务")
        print("  python collision_analyzer.py \\")
        print("    roomlayout/layout_new/.../room.json \\")
        print("    roomlayout/layout_new/assets_annotated.json \\")
        print("    outputs/.../failed_tasks.json")
        print()
        print("  # 分析并可视化")
        print("  python collision_analyzer.py ... failed_tasks.json --visualize collision.png")
        print()
        print("  # 移除碰撞物体")
        print("  python collision_analyzer.py ... failed_tasks.json --remove FumeHood --output room_modified.json")
        sys.exit(1)

    scene_json = sys.argv[1]
    assets_json = sys.argv[2]
    failed_tasks_json = sys.argv[3]

    # 解析选项
    offset_radius = 0.3
    visualize_path = None
    remove_object = None
    move_object = None
    move_position = None
    output_json = None

    i = 4
    while i < len(sys.argv):
        if sys.argv[i] == '--visualize' and i + 1 < len(sys.argv):
            visualize_path = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '--remove' and i + 1 < len(sys.argv):
            remove_object = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '--move' and i + 3 < len(sys.argv):
            move_object = sys.argv[i + 1]
            move_position = (float(sys.argv[i + 2]), float(sys.argv[i + 3]))
            i += 4
        elif sys.argv[i] == '--output' and i + 1 < len(sys.argv):
            output_json = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '--offset-radius' and i + 1 < len(sys.argv):
            offset_radius = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1

    # 创建分析器
    analyzer = CollisionAnalyzer(scene_json, assets_json, offset_radius)

    # 分析失败任务
    print(f"\n{'='*70}")
    print("分析失败任务")
    print(f"{'='*70}\n")

    analysis = analyzer.analyze_failed_task(failed_tasks_json)

    for task in analysis['failed_tasks']:
        print(f"\n任务: {task['task_id']}")
        print(f"  起点: ({task['start_position'][0]:.2f}, {task['start_position'][1]:.2f})")
        print(f"  终点: ({task['end_position'][0]:.2f}, {task['end_position'][1]:.2f})")
        print(f"  失败原因: {task['failure_reason']}")
        print()

        # 显示详细的碰撞报告
        report = task['collision_report']

        # 起点阻隔分析
        if report['start_blocked']:
            print(f"  ✗ 起点被阻隔！")
            for obj_info in report['blocking_objects']['start']:
                distance = obj_info['distance']
                distance_text = f"{abs(distance):.2f}m (穿透)" if distance < 0 else f"{distance:.2f}m"

                print(f"    • 阻隔物体: {obj_info['id']}")
                print(f"      - 物体中心: ({obj_info['position'][0]:.2f}, {obj_info['position'][1]:.2f})")
                print(f"      - 膨胀边界框: {obj_info['bbox_edge']}")
                print(f"      - 距离边界框边缘: {distance_text}")
                if 'category' in obj_info:
                    print(f"      - 类型: {obj_info['category']}")
                if 'size' in obj_info:
                    print(f"      - 原始尺寸: 长={obj_info['size']['long']:.2f}m, 宽={obj_info['size']['short']:.2f}m")
            print()
        else:
            print(f"  ✓ 起点畅通")

        # 终点阻隔分析
        if report['end_blocked']:
            print(f"  ✗ 终点被阻隔！")
            for obj_info in report['blocking_objects']['end']:
                distance = obj_info['distance']
                distance_text = f"{abs(distance):.2f}m (穿透)" if distance < 0 else f"{distance:.2f}m"

                print(f"    • 阻隔物体: {obj_info['id']}")
                print(f"      - 物体中心: ({obj_info['position'][0]:.2f}, {obj_info['position'][1]:.2f})")
                print(f"      - 膨胀边界框: {obj_info['bbox_edge']}")
                print(f"      - 距离边界框边缘: {distance_text}")
                if 'category' in obj_info:
                    print(f"      - 类型: {obj_info['category']}")
                if 'size' in obj_info:
                    print(f"      - 原始尺寸: 长={obj_info['size']['long']:.2f}m, 宽={obj_info['size']['short']:.2f}m")
            print()
        else:
            print(f"  ✓ 终点畅通")

        # 路径阻隔分析
        if report.get('path_blocked', False):
            print(f"  ✗ 路径被阻隔！")
            if 'first_collision_point' in report:
                fc = report['first_collision_point']
                print(f"    • 第一个碰撞点: 索引={fc['index']}, 位置=({fc['position'][0]:.2f}, {fc['position'][1]:.2f})")
                print(f"    • 阻隔物体:")
                for obj_info in fc['blocking_objects']:
                    print(f"      - {obj_info['id']}")
                    if 'category' in obj_info:
                        print(f"        类型: {obj_info['category']}")
            elif task['path_collisions']:
                print(f"    • 路径上的阻隔物体:")
                for obj_id in task['path_collisions']:
                    obj = next((o for o in analyzer.objects if o.id == obj_id), None)
                    if obj:
                        print(f"      - {obj_id}")
                        print(f"        位置: ({obj.position[0]:.2f}, {obj.position[1]:.2f})")
                        if obj.asset_info and 'semantic' in obj.asset_info:
                            print(f"        类型: {obj.asset_info['semantic'].get('category', 'N/A')}")

        # 如果没有任何碰撞
        if not report['start_blocked'] and not report['end_blocked'] and not report.get('path_blocked', False):
            print(f"  ✓ 未检测到碰撞（可能是其他原因）")

        print()

    # 可视化
    if visualize_path:
        print(f"\n生成可视化...")
        analyzer.visualize_collisions(failed_tasks_json, None, visualize_path)

    # 修改场景
    if remove_object:
        output = output_json or scene_json.replace('.json', '_modified.json')
        analyzer.remove_object_from_scene(remove_object, output)

    if move_object and move_position:
        output = output_json or scene_json.replace('.json', '_modified.json')
        analyzer.adjust_object_position(
            move_object,
            move_position[0],
            move_position[1],
            output_json=output
        )

    print(f"\n{'='*70}")
    print("分析完成")
    print(f"{'='*70}\n")


if __name__ == "__main__":
    main()



# cd /home/pjlab/fbh/LabUtopia

# conda activate labutopia

# PYTHONPATH=. python3 utils/collision_analyzer.py \
#   "roomlayout/layout_new/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/Boc_Deprotection_of_Piperazine_Derivative__Model___room_isaacsim.json" \
#   "roomlayout/layout_new/assets_annotated.json" \
#   "outputs/path_planning_batch_results/run_2026-01-06_18-22-53/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725_failed_tasks.json" \
#   --offset-radius 0.4 \
#   --visualize "outputs/path_planning_batch_results/run_2026-01-06_18-22-53/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/collision_analysis_detailed.png"