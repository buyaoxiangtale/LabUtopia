#!/usr/bin/env python3
"""
路径规划障碍物分析工具 v4.0 (OBB矩形碰撞检测 + 旋转支持)

改进内容：
1. ✅ 从圆形包围盒改为矩形包围盒（OBB - Oriented Bounding Box）
2. ✅ 支持物体旋转角度（rotation.z）
3. ✅ 更精确的碰撞检测，减少假阳性

主要解决的问题：
- 问题A：长条形物体的"虚空碰撞" - 使用精确的矩形而不是圆形
- 问题B：忽略物体旋转 - 完整支持rotation.z角度
"""

import os
import json
import sys
import math
import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from collections import defaultdict


# ==============================================================================
# 1. OBB碰撞分析器（有向包围盒）
# ==============================================================================
class OBBCollisionAnalyzer:
    """
    使用有向包围盒（OBB）进行精确碰撞检测

    存储格式：
        objects = [{
            'id': str,
            'center': np.array([x, y]),      # 中心点
            'half_extents': np.array([dx, dy]), # 半长和半宽
            'angle': float,                    # 旋转角度（弧度）
            'vertices': np.array([[x1,y1], [x2,y2], [x3,y3], [x4,y4]]) # 四个顶点
        }]
    """

    def __init__(self, scene_json_path: str, assets_json_path: str, default_inflation: float = 0.3):
        self.objects = []
        self.default_inflation = default_inflation
        self.is_loaded = self.load_scene(scene_json_path, assets_json_path)

    def load_scene(self, scene_path: str, assets_path: str) -> bool:
        """加载场景和资产尺寸"""
        try:
            if not os.path.exists(scene_path):
                print(f"  ❌ 文件不存在: {scene_path}")
                return False

            with open(scene_path, 'r', encoding='utf-8') as f:
                scene_data = json.load(f)

            if not isinstance(scene_data, dict):
                return False

            obj_list = scene_data.get('objects', [])

            # ✅ 统计：记录房间信息
            total_objects = len(obj_list)
            room_objects = [obj for obj in obj_list if obj.get('id', '') in ['LaboratoryRoom', 'laboratoryroom']]

            # 加载资产库
            assets_db = {}
            if os.path.exists(assets_path):
                try:
                    with open(assets_path, 'r', encoding='utf-8') as f:
                        assets_data = json.load(f)
                        if isinstance(assets_data, dict) and 'assets' in assets_data:
                            if isinstance(assets_data['assets'], list):
                                for asset in assets_data['assets']:
                                    if 'id' in asset:
                                        assets_db[asset['id']] = asset
                            elif isinstance(assets_data['assets'], dict):
                                assets_db = assets_data['assets']
                        else:
                            assets_db = assets_data
                except Exception as e:
                    print(f"  ⚠️  Assets 文件加载失败: {e}")

            count = 0
            for obj in obj_list:
                obj_id = obj.get('id', 'unknown')

                # ✅ 跳过房间（LaboratoryRoom）- 不参与碰撞检测
                if obj_id == 'LaboratoryRoom' or obj_id.lower() == 'laboratoryroom':
                    continue

                # 读取位置
                pos_raw = obj.get('position', [0, 0, 0])
                if isinstance(pos_raw, dict):
                    pos = np.array([pos_raw.get('x', 0), pos_raw.get('y', 0)])
                elif isinstance(pos_raw, list):
                    pos = np.array([pos_raw[0], pos_raw[1]])
                else:
                    pos = np.array([0.0, 0.0])

                # ✅ 改进点1：读取旋转角度
                rotation_raw = obj.get('rotation', {'x': 0, 'y': 0, 'z': 0})
                if isinstance(rotation_raw, dict):
                    angle_deg = rotation_raw.get('z', 0)
                elif isinstance(rotation_raw, list) and len(rotation_raw) > 2:
                    angle_deg = rotation_raw[2]
                else:
                    angle_deg = 0.0

                # 转换为弧度
                angle_rad = math.radians(angle_deg)

                scale = obj.get('scale', [1, 1, 1])
                scale_x = scale[0] if isinstance(scale, list) else 1.0
                scale_y = scale[1] if isinstance(scale, list) else 1.0

                # ✅ 改进：通过物体名称查找资产信息
                # 场景文件中没有assetId字段，需要通过物体名称（id）匹配
                asset_info = None
                obj_name = obj.get('id', 'unknown')

                # 方法1：直接匹配
                if obj_name in assets_db:
                    asset_info = assets_db[obj_name]

                # 方法2：忽略大小写匹配
                else:
                    for db_id, db_asset in assets_db.items():
                        if obj_name.lower() == db_id.lower():
                            asset_info = db_asset
                            break

                short_edge, long_edge = 0.5, 0.5  # 默认尺寸

                if asset_info:
                    bbox = None
                    if 'geometry' in asset_info and 'bbox' in asset_info['geometry']:
                        bbox = asset_info['geometry']['bbox']
                    elif 'boundingBox' in asset_info:
                        bbox = asset_info['boundingBox']

                    if bbox:
                        if 'short' in bbox and 'long' in bbox:
                            short_edge = bbox['short']
                            long_edge = bbox['long']
                        elif 'dimensions' in bbox:
                            dims = bbox['dimensions']
                            short_edge, long_edge = min(dims[0], dims[1]), max(dims[0], dims[1])
                        else:
                            short_edge, long_edge = 0.5, 0.5
                    else:
                        short_edge, long_edge = 0.5, 0.5
                else:
                    # 找不到资产信息，跳过该物体
                    continue

                # ✅ 改进点2：应用缩放系数到长宽
                short_edge *= scale_x
                long_edge *= scale_y

                # ✅ 改进点3：使用矩形而不是圆形
                # half_extents 是从中心到边缘的距离（半长和半宽）
                half_extents = np.array([
                    long_edge / 2.0,  # 沿着物体朝向（长边）
                    short_edge / 2.0  # 垂直于物体朝向（短边）
                ])

                # 计算OBB的四个顶点
                vertices = self._compute_obb_vertices(pos, half_extents, angle_rad)

                self.objects.append({
                    'id': obj_id,
                    'center': pos,
                    'half_extents': half_extents,
                    'angle': angle_rad,
                    'vertices': vertices,
                    'angle_deg': angle_deg,  # 保留度数用于调试
                    'short_edge': short_edge,
                    'long_edge': long_edge
                })
                count += 1

            # ✅ 显示加载信息（包含跳过房间的统计）
            if len(room_objects) > 0:
                print(f"  ✓ 成功加载场景 (OBB模式): {Path(scene_path).name}")
                print(f"    总物体数: {total_objects}, 跳过房间: {len(room_objects)}, 实际加载: {count}")
            else:
                print(f"  ✓ 成功加载场景 (OBB模式): {Path(scene_path).name} (包含 {count} 个物体)")
            return True

        except Exception as e:
            print(f"  ❌ 场景加载异常: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _compute_obb_vertices(self, center: np.ndarray, half_extents: np.ndarray, angle: float) -> np.ndarray:
        """
        计算OBB的四个顶点

        顶点顺序：[前右, 前左, 后左, 后右] (逆时针)
        """
        # 旋转矩阵
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        rotation_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a,  cos_a]
        ])

        # 局部坐标系中的四个角点（未旋转）
        local_corners = np.array([
            [ half_extents[0],  half_extents[1]],  # 前右
            [-half_extents[0],  half_extents[1]],  # 前左
            [-half_extents[0], -half_extents[1]],  # 后左
            [ half_extents[0], -half_extents[1]]   # 后右
        ])

        # 旋转并平移到世界坐标
        world_corners = local_corners @ rotation_matrix.T + center

        return world_corners

    def get_collisions_at_point(
        self,
        x: float,
        y: float,
        robot_radius: float = 0.0,
        only_largest: bool = False
    ) -> List[str]:
        """
        检测某一点是否与物体的OBB碰撞（考虑机器人半径）

        使用分离轴定理（SAT）进行精确的矩形-圆形碰撞检测
        """
        if not self.objects:
            return []

        point = np.array([x, y])
        hit_objects = []

        for obj in self.objects:
            if self._check_point_obb_collision(point, robot_radius, obj):
                hit_objects.append(obj)

        if not hit_objects:
            return []

        if only_largest:
            # 按面积排序（长 × 宽）
            hit_objects.sort(key=lambda o: o['long_edge'] * o['short_edge'], reverse=True)
            return [hit_objects[0]['id']]
        else:
            return [o['id'] for o in hit_objects]

    def _check_point_obb_collision(
        self,
        point: np.ndarray,
        robot_radius: float,
        obj: Dict
    ) -> bool:
        """
        检测点（或圆）是否与OBB碰撞

        使用分离轴定理（SAT）的简化版本：
        1. 将点转换到OBB的局部坐标系
        2. 检查点是否在扩展的OBB范围内（考虑机器人半径）
        """
        # 提取OBB参数
        center = obj['center']
        half_extents = obj['half_extents']
        angle = obj['angle']

        # 1. 将点转换到OBB局部坐标系
        # 平移
        translated_point = point - center

        # 旋转（逆向旋转）
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        local_x = translated_point[0] * cos_a - translated_point[1] * sin_a
        local_y = translated_point[0] * sin_a + translated_point[1] * cos_a

        # 2. 在局部坐标系中，OBB变成轴对齐的矩形
        # 扩展的边界（考虑机器人半径）
        expanded_half_extents = half_extents + robot_radius

        # 3. 检查点是否在扩展的矩形内
        if (abs(local_x) <= expanded_half_extents[0] and
            abs(local_y) <= expanded_half_extents[1]):
            return True

        return False

    def visualize_collision(self, point: np.ndarray, robot_radius: float, output_path: str):
        """可视化碰撞检测结果（用于调试）"""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Circle

        fig, ax = plt.subplots(figsize=(10, 10))

        # 绘制所有物体的OBB
        for obj in self.objects:
            vertices = obj['vertices']
            is_collision = self._check_point_obb_collision(point, robot_radius, obj)

            # 绘制OBB
            color = 'red' if is_collision else 'lightblue'
            alpha = 0.5 if is_collision else 0.3
            poly = Polygon(vertices, closed=True, facecolor=color, edgecolor='blue', alpha=alpha)
            ax.add_patch(poly)

            # 标注物体ID
            center = obj['center']
            ax.text(center[0], center[1], obj['id'], fontsize=8, ha='center')

        # 绘制检测点
        point_circle = Circle(point, robot_radius, facecolor='green', edgecolor='darkgreen', alpha=0.7)
        ax.add_patch(point_circle)
        ax.plot(point[0], point[1], 'g+', markersize=15, markeredgewidth=2)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'OBB Collision Detection (Point: [{point[0]:.2f}, {point[1]:.2f}])')

        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  ✓ 可视化保存到: {output_path}")


# ==============================================================================
# 2. 核心分析逻辑（保持不变）
# ==============================================================================
def analyze_choke_points(analyzer, start, end, robot_radius=0.3):
    if not analyzer.is_loaded or not analyzer.objects:
        return []

    dist = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    if dist < 0.1:
        return []

    step_size = 0.1
    steps = int(dist / step_size)
    choke_events = []
    last_collision_set = set()

    start_step = int(0.5 / step_size)
    end_step = steps - int(0.5 / step_size)

    if start_step >= end_step:
        return []

    for i in range(start_step, end_step):
        t = i / steps
        rx = start[0] + (end[0] - start[0]) * t
        ry = start[1] + (end[1] - start[1]) * t

        hit_ids = analyzer.get_collisions_at_point(rx, ry, robot_radius, only_largest=False)
        current_set = set(hit_ids)

        if hit_ids and current_set != last_collision_set:
            event_type = "narrow_gap" if len(hit_ids) > 1 else "single_block"
            choke_events.append({
                'dist': f"{dist * t:.1f}m",
                'type': event_type,
                'objects': hit_ids
            })
        last_collision_set = current_set

    return choke_events


# ==============================================================================
# 3. 主流程
# ==============================================================================
def find_scene_file(scene_name: str, scene_base_dir: str):
    import re
    core_name = re.sub(r'_pair$', '', scene_name)
    core_name = re.sub(r'_20\d{6}_\d{6}$', '', core_name)
    base = Path(scene_base_dir)
    candidate_dirs = [d for d in base.iterdir() if d.is_dir() and core_name in d.name]
    for d in candidate_dirs:
        json_files = list(d.glob("*_room_isaacsim.json"))
        if json_files:
            return str(json_files[0])
    return None


def main():
    base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
    scene_base_dir = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
    assets_json = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"
    robot_radius = 0.3

    if len(sys.argv) > 1:
        base_dir = sys.argv[1]

    print("="*70)
    print("OBB碰撞检测分析工具 v4.0")
    print("="*70)
    print(f"分析目录: {base_dir}")
    print(f"机器人半径: {robot_radius}m")
    print("-"*70)

    # 加载结果
    results = []
    for f in Path(base_dir).rglob("*_all_results.json"):
        try:
            with open(f) as fp:
                results.extend(json.load(fp))
        except:
            pass

    failed_tasks = [t for t in results if not t.get('is_success', False)]
    print(f"失败任务数: {len(failed_tasks)}")

    # 分组
    tasks_by_scene = defaultdict(list)
    for t in failed_tasks:
        tid = t.get('task_id', 'unknown')
        s_name = '_'.join(tid.split('_')[:-1]) if '_' in tid else tid
        tasks_by_scene[s_name].append(t)

    analyzers = {}

    for scene_name, tasks in tasks_by_scene.items():
        print(f"\n场景: {scene_name}")

        # 查找场景文件
        scene_path = find_scene_file(scene_name, scene_base_dir)
        if not scene_path:
            print(f"  ❌ 找不到场景文件夹/文件 (搜索名称: {scene_name})")
            continue

        # 使用新的OBB分析器
        if scene_path not in analyzers:
            analyzers[scene_path] = OBBCollisionAnalyzer(scene_path, assets_json)
        analyzer = analyzers[scene_path]

        if not analyzer.is_loaded:
            print(f"  ❌ 跳过分析 (地图加载失败)")
            continue

        # 分析任务
        for task in tasks:
            start = task.get('start')
            end = task.get('end')
            tid = task.get('task_id')[-3:]

            # 使用only_largest=True只报告最大的障碍物
            s_hits = analyzer.get_collisions_at_point(start[0], start[1], robot_radius, only_largest=True)
            e_hits = analyzer.get_collisions_at_point(end[0], end[1], robot_radius, only_largest=True)

            chokes = analyze_choke_points(analyzer, start, end, robot_radius)

            print(f"  任务 {tid}:")
            if s_hits or e_hits:
                if s_hits:
                    print(f"    ❌ 起点碰撞: {s_hits}")
                if e_hits:
                    print(f"    ❌ 终点碰撞: {e_hits}")
            elif chokes:
                print(f"    ⚠️  路径阻隔 ({len(chokes)}处):")
                for c in chokes:
                    print(f"      - {c['dist']} 处被 {c['objects']} ({'夹击' if c['type']=='narrow_gap' else '挡住'})")
            else:
                print(f"    ❓ 路径通畅（可能原因：算法参数或局部极小值）")


if __name__ == "__main__":
    main()
