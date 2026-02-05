#!/usr/bin/env python3
"""
路径规划障碍物分析工具 v6.0 (基于goal_pairs输入)

使用 goal_pairs YAML 文件作为输入，进行精确的碰撞分析。
数据来源: /home/pjlab/fbh/LabUtopia/outputs/gemini_flash_nav_targets_1_13_18_47/goal_pairs
"""

import os
import json
import sys
import math
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from collections import defaultdict


# ==============================================================================
# 1. OBB碰撞分析器
# ==============================================================================
class OBBCollisionAnalyzer:
    """
    使用有向包围盒（OBB）进行精确碰撞检测
    """

    def __init__(self, scene_json_path: str, assets_json_path: str, default_inflation: float = 0.3):
        self.objects = []
        self.default_inflation = default_inflation
        self.scene_name = Path(scene_json_path).stem.replace('_room_isaacsim', '')
        self.is_loaded = self.load_scene(scene_json_path, assets_json_path)

    def load_scene(self, scene_path: str, assets_path: str) -> bool:
        """加载场景和资产尺寸"""
        try:
            if not os.path.exists(scene_path):
                print(f"    ❌ 场景文件不存在: {scene_path}")
                return False

            with open(scene_path, 'r', encoding='utf-8') as f:
                scene_data = json.load(f)

            if not isinstance(scene_data, dict):
                return False

            obj_list = scene_data.get('objects', [])

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
                    print(f"    ⚠️  Assets 文件加载失败: {e}")

            count = 0
            for obj in obj_list:
                obj_id = obj.get('id', 'unknown')

                # 跳过房间
                if obj_id.lower() == 'laboratoryroom':
                    continue

                # 读取位置
                pos_raw = obj.get('position', [0, 0, 0])
                if isinstance(pos_raw, dict):
                    pos = np.array([pos_raw.get('x', 0), pos_raw.get('y', 0)])
                elif isinstance(pos_raw, list):
                    pos = np.array([pos_raw[0], pos_raw[1]])
                else:
                    pos = np.array([0.0, 0.0])

                # 读取旋转角度
                rotation_raw = obj.get('rotation', {'x': 0, 'y': 0, 'z': 0})
                if isinstance(rotation_raw, dict):
                    angle_deg = rotation_raw.get('z', 0)
                elif isinstance(rotation_raw, list) and len(rotation_raw) > 2:
                    angle_deg = rotation_raw[2]
                else:
                    angle_deg = 0.0

                angle_rad = math.radians(angle_deg)

                # 读取缩放
                scale = obj.get('scale', [1, 1, 1])
                scale_x = scale[0] if isinstance(scale, list) else 1.0
                scale_y = scale[1] if isinstance(scale, list) else 1.0

                # 查找资产信息
                asset_info = None
                obj_name = obj.get('id', 'unknown')

                if obj_name in assets_db:
                    asset_info = assets_db[obj_name]
                else:
                    for db_id, db_asset in assets_db.items():
                        if obj_name.lower() == db_id.lower():
                            asset_info = db_asset
                            break

                short_edge, long_edge = 0.5, 0.5

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

                # 应用缩放
                short_edge *= scale_x
                long_edge *= scale_y

                # 计算半长和半宽
                half_extents = np.array([
                    long_edge / 2.0,
                    short_edge / 2.0
                ])

                # 计算OBB顶点
                vertices = self._compute_obb_vertices(pos, half_extents, angle_rad)

                self.objects.append({
                    'id': obj_id,
                    'center': pos,
                    'half_extents': half_extents,
                    'angle': angle_rad,
                    'vertices': vertices,
                    'short_edge': short_edge,
                    'long_edge': long_edge
                })
                count += 1

            print(f"    ✓ 成功加载场景: {Path(scene_path).parent.name} (包含 {count} 个物体)")
            return True

        except Exception as e:
            print(f"    ❌ 场景加载异常: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _compute_obb_vertices(self, center: np.ndarray, half_extents: np.ndarray, angle: float) -> np.ndarray:
        """计算OBB的四个顶点"""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        rotation_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a,  cos_a]
        ])

        local_corners = np.array([
            [ half_extents[0],  half_extents[1]],
            [-half_extents[0],  half_extents[1]],
            [-half_extents[0], -half_extents[1]],
            [ half_extents[0], -half_extents[1]]
        ])

        world_corners = local_corners @ rotation_matrix.T + center
        return world_corners

    def _check_point_obb_collision(self, point: np.ndarray, robot_radius: float, obj: Dict) -> bool:
        """检测点（或圆）是否与OBB碰撞"""
        center = obj['center']
        half_extents = obj['half_extents']
        angle = obj['angle']

        # 将点转换到OBB局部坐标系
        translated_point = point - center

        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        local_x = translated_point[0] * cos_a - translated_point[1] * sin_a
        local_y = translated_point[0] * sin_a + translated_point[1] * cos_a

        # 扩展的边界
        expanded_half_extents = half_extents + robot_radius

        # 检查点是否在扩展的矩形内
        if (abs(local_x) <= expanded_half_extents[0] and
            abs(local_y) <= expanded_half_extents[1]):
            return True

        return False

    def get_collisions_at_point(
        self,
        x: float,
        y: float,
        robot_radius: float = 0.0,
        only_largest: bool = False
    ) -> List[str]:
        """检测某一点是否与物体的OBB碰撞"""
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
            hit_objects.sort(key=lambda o: o['long_edge'] * o['short_edge'], reverse=True)
            return [hit_objects[0]['id']]
        else:
            return [o['id'] for o in hit_objects]

    def visualize_task(
        self,
        start: np.ndarray,
        end: np.ndarray,
        task_id: str,
        output_path: str,
        robot_radius: float,
        collisions: Dict = None
    ):
        """可视化任务场景"""
        fig, ax = plt.subplots(figsize=(12, 12))

        # 绘制所有障碍物
        for obj in self.objects:
            vertices = obj['vertices']

            # 判断是否碰撞
            is_collision = False
            if collisions:
                is_collision = obj['id'] in collisions.get('start', []) or \
                             obj['id'] in collisions.get('end', [])

            color = 'red' if is_collision else 'lightblue'
            alpha = 0.6 if is_collision else 0.3

            poly = Polygon(vertices, closed=True, facecolor=color, edgecolor='blue', alpha=alpha)
            ax.add_patch(poly)

            # 标注物体ID
            center = obj['center']
            ax.text(center[0], center[1], obj['id'][:8], fontsize=7, ha='center', alpha=0.7)

        # 绘制起点和终点
        start_circle = Circle(start, robot_radius, facecolor='green', edgecolor='darkgreen', alpha=0.5)
        end_circle = Circle(end, robot_radius, facecolor='red', edgecolor='darkred', alpha=0.5)
        ax.add_patch(start_circle)
        ax.add_patch(end_circle)
        ax.plot(start[0], start[1], 'g+', markersize=15, markeredgewidth=2)
        ax.plot(end[0], end[1], 'r+', markersize=15, markeredgewidth=2)

        # 绘制直线路径
        ax.plot([start[0], end[0]], [start[1], end[1]], 'k--', linewidth=1.5, alpha=0.6, label='Straight Path')

        # 设置范围
        all_x = [v[0] for o in self.objects for v in o['vertices']] + [start[0], end[0]]
        all_y = [v[1] for o in self.objects for v in o['vertices']] + [start[1], end[1]]

        if all_x:
            margin = 1.0
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f"Scene: {self.scene_name}\nTask: {task_id}\nRobot Radius: {robot_radius}m")

        # 添加图例
        if collisions:
            legend_text = []
            if collisions.get('start'):
                legend_text.append(f"Start Collision: {', '.join(collisions['start'])}")
            if collisions.get('end'):
                legend_text.append(f"End Collision: {', '.join(collisions['end'])}")
            if legend_text:
                ax.text(0.02, 0.02, '\n'.join(legend_text),
                       transform=ax.transAxes, fontsize=9,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()


# ==============================================================================
# 2. 辅助函数
# ==============================================================================
def find_scene_file(scene_name: str, scene_base_dir: str):
    """查找场景文件"""
    import re

    # 去掉日期后缀
    core_name = re.sub(r'_20\d{6}_\d{6}$', '', scene_name)
    base = Path(scene_base_dir)

    # 查找匹配的目录
    candidate_dirs = [d for d in base.iterdir() if d.is_dir() and core_name in d.name]

    for d in candidate_dirs:
        json_files = list(d.glob("*_room_isaacsim.json"))
        if json_files:
            return str(json_files[0])

    return None


def analyze_choke_points(analyzer, start, end, robot_radius=0.3):
    """分析路径上的阻隔点"""
    if not analyzer.is_loaded or not analyzer.objects:
        return []

    dist = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    if dist < 0.1:
        return []

    step_size = 0.1
    steps = int(dist / step_size)
    choke_events = []
    last_collision_set = set()

    # 跳过起点和终点附近 0.5m
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
def main():
    # 配置参数
    GOAL_PAIRS_DIR = "/home/pjlab/fbh/LabUtopia/outputs/gemini_flash_nav_targets_1_13_18_47/goal_pairs"
    SCENE_BASE_DIR = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
    ASSETS_JSON = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"
    OUTPUT_DIR = "/home/pjlab/fbh/LabUtopia/outputs/collision_analysis_goal_pairs"
    ROBOT_RADIUS = 0.3

    print("=" * 80)
    print("OBB碰撞检测分析工具 v6.0 (基于goal_pairs输入)")
    print("=" * 80)
    print(f"Goal Pairs目录: {GOAL_PAIRS_DIR}")
    print(f"场景基础目录: {SCENE_BASE_DIR}")
    print(f"Assets文件: {ASSETS_JSON}")
    print(f"输出目录: {OUTPUT_DIR}")
    print(f"机器人半径: {ROBOT_RADIUS}m")
    print("-" * 80)

    # 创建输出目录
    output_path = Path(OUTPUT_DIR)
    output_path.mkdir(parents=True, exist_ok=True)

    # 查找所有YAML文件
    goal_pairs_dir = Path(GOAL_PAIRS_DIR)
    yaml_files = list(goal_pairs_dir.glob("*_goal_pairs.yaml"))

    if not yaml_files:
        print(f"未找到任何goal_pairs文件")
        return

    print(f"找到 {len(yaml_files)} 个goal_pairs文件\n")

    # 统计信息
    total_tasks = 0
    total_start_collisions = 0
    total_end_collisions = 0
    total_path_blocked = 0

    # 分析器缓存
    analyzers = {}

    # 处理每个场景
    for yaml_file in sorted(yaml_files):
        print(f"\n{'='*80}")
        print(f"场景: {yaml_file.stem.replace('_goal_pairs', '')}")
        print(f"{'='*80}")

        # 加载YAML文件
        try:
            with open(yaml_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            print(f"  ❌ YAML加载失败: {e}")
            continue

        goal_pairs = data.get('goal_pairs', [])
        metadata = data.get('metadata', {})
        scene_name = metadata.get('scene_name', yaml_file.stem.replace('_goal_pairs', ''))

        if not goal_pairs:
            print(f"  ℹ 没有goal_pairs数据")
            continue

        print(f"  目标对数量: {len(goal_pairs)}")

        # 查找场景文件
        scene_path = find_scene_file(scene_name, SCENE_BASE_DIR)
        if not scene_path:
            print(f"  ❌ 找不到场景文件")
            continue

        # 创建或获取分析器
        if scene_path not in analyzers:
            analyzers[scene_path] = OBBCollisionAnalyzer(scene_path, ASSETS_JSON)

        analyzer = analyzers[scene_path]
        if not analyzer.is_loaded:
            print(f"  ❌ 场景加载失败")
            continue

        # 创建场景输出目录
        scene_output_dir = output_path / scene_name
        scene_output_dir.mkdir(parents=True, exist_ok=True)

        # 分析每个目标对
        scene_stats = {
            'total': 0,
            'start_collision': 0,
            'end_collision': 0,
            'path_blocked': 0,
            'clear': 0
        }

        for idx, goal_pair in enumerate(goal_pairs):
            start = goal_pair.get('start', [0, 0, 0])
            end = goal_pair.get('end', [0, 0, 0])

            # 只取 x, y 坐标
            start_pos = np.array([start[0], start[1]])
            end_pos = np.array([end[0], end[1]])

            task_id = f"pair_{idx:02d}"
            scene_stats['total'] += 1
            total_tasks += 1

            # 检测起点和终点碰撞
            start_hits = analyzer.get_collisions_at_point(
                start_pos[0], start_pos[1], ROBOT_RADIUS, only_largest=True
            )
            end_hits = analyzer.get_collisions_at_point(
                end_pos[0], end_pos[1], ROBOT_RADIUS, only_largest=True
            )

            # 分析路径阻隔
            chokes = analyze_choke_points(analyzer, start_pos, end_pos, ROBOT_RADIUS)

            # 打印结果
            status_symbol = "✓"
            status_text = "通畅"

            if start_hits:
                status_symbol = "❌"
                status_text = f"起点碰撞: {start_hits[0]}"
                scene_stats['start_collision'] += 1
                total_start_collisions += 1
            elif end_hits:
                status_symbol = "❌"
                status_text = f"终点碰撞: {end_hits[0]}"
                scene_stats['end_collision'] += 1
                total_end_collisions += 1
            elif chokes:
                status_symbol = "⚠️"
                status_text = f"路径阻隔 ({len(chokes)}处)"
                scene_stats['path_blocked'] += 1
                total_path_blocked += 1
            else:
                scene_stats['clear'] += 1

            print(f"  [{status_symbol}] {task_id}: {status_text}")

            if chokes:
                for c in chokes:
                    print(f"      - {c['dist']} 处被 {c['objects']} ({'夹击' if c['type']=='narrow_gap' else '挡住'})")

            # 生成可视化
            collisions = {
                'start': start_hits,
                'end': end_hits
            }
            vis_path = scene_output_dir / f"{task_id}.png"
            analyzer.visualize_task(
                start_pos, end_pos, task_id,
                str(vis_path), ROBOT_RADIUS, collisions
            )

        # 打印场景统计
        print(f"\n  场景统计:")
        print(f"    总任务数: {scene_stats['total']}")
        print(f"    起点碰撞: {scene_stats['start_collision']}")
        print(f"    终点碰撞: {scene_stats['end_collision']}")
        print(f"    路径阻隔: {scene_stats['path_blocked']}")
        print(f"    路径通畅: {scene_stats['clear']}")
        print(f"    可视化保存在: {scene_output_dir}")

    # 打印总体统计
    print("\n" + "=" * 80)
    print("总体统计")
    print("=" * 80)
    print(f"总任务数: {total_tasks}")
    print(f"起点碰撞: {total_start_collisions} ({total_start_collisions/total_tasks*100:.1f}%)")
    print(f"终点碰撞: {total_end_collisions} ({total_end_collisions/total_tasks*100:.1f}%)")
    print(f"路径阻隔: {total_path_blocked} ({total_path_blocked/total_tasks*100:.1f}%)")
    print(f"路径通畅: {total_tasks - total_start_collisions - total_end_collisions - total_path_blocked} ({(total_tasks - total_start_collisions - total_end_collisions - total_path_blocked)/total_tasks*100:.1f}%)")
    print(f"\n输出目录: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
