#!/usr/bin/env python3
"""
路径规划障碍物分析工具 v5.0 (可视化调试版)
新增功能：
1. ✅ 生成现场还原图：画出所有障碍物OBB、起点、终点和连线
2. ✅ 自动保存到 debug_images 文件夹
"""

import os
import json
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from collections import defaultdict

# ==============================================================================
# 1. OBB碰撞分析器 (带可视化功能)
# ==============================================================================
class OBBCollisionAnalyzer:
    def __init__(self, scene_json_path: str, assets_json_path: str, default_inflation: float = 0.3):
        self.objects = []
        self.default_inflation = default_inflation
        self.scene_name = Path(scene_json_path).stem
        self.is_loaded = self.load_scene(scene_json_path, assets_json_path)

    def load_scene(self, scene_path: str, assets_path: str) -> bool:
        # ... (加载逻辑与之前相同，省略重复代码以节省空间) ...
        # 请保留你 v4.0 版本中的 load_scene 和 _compute_obb_vertices 方法
        try:
            if not os.path.exists(scene_path): return False
            with open(scene_path, 'r', encoding='utf-8') as f: scene_data = json.load(f)
            
            # 加载 Assets
            assets_db = {}
            if os.path.exists(assets_path):
                with open(assets_path, 'r') as f: 
                    d = json.load(f)
                    assets_db = {a['id']: a for a in d.get('assets', [])} if isinstance(d.get('assets'), list) else d.get('assets', {})

            obj_list = scene_data.get('objects', [])
            for obj in obj_list:
                obj_id = obj.get('id', 'unknown')
                # ⚠️ 注意：这里跳过了房间，如果墙壁属于LaboratoryRoom，碰撞检测就会失效！
                if obj_id.lower() == 'laboratoryroom': continue 

                # 读取基本信息
                pos = np.array([obj.get('position', {}).get('x', 0), obj.get('position', {}).get('y', 0)]) if isinstance(obj.get('position'), dict) else np.array(obj.get('position', [0,0])[:2])
                rot_z = obj.get('rotation', {}).get('z', 0) if isinstance(obj.get('rotation'), dict) else (obj.get('rotation', [0,0,0])[2] if len(obj.get('rotation', []))>2 else 0)
                angle_rad = math.radians(rot_z)
                scale = obj.get('scale', [1,1,1])
                
                # 查找尺寸
                asset = assets_db.get(obj_id) or next((v for k,v in assets_db.items() if k.lower() == obj_id.lower()), None)
                short, long = 0.5, 0.5
                if asset and 'geometry' in asset and 'bbox' in asset['geometry']:
                    b = asset['geometry']['bbox']
                    short, long = b.get('short', 0.5), b.get('long', 0.5)
                
                # 应用缩放
                short *= scale[0]
                long *= scale[1]
                
                half_extents = np.array([long/2.0, short/2.0])
                vertices = self._compute_obb_vertices(pos, half_extents, angle_rad)
                
                self.objects.append({
                    'id': obj_id,
                    'center': pos,
                    'half_extents': half_extents,
                    'angle': angle_rad,
                    'vertices': vertices
                })
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def _compute_obb_vertices(self, center, half_extents, angle):
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        corners = np.array([[half_extents[0], half_extents[1]], [-half_extents[0], half_extents[1]], 
                           [-half_extents[0], -half_extents[1]], [half_extents[0], -half_extents[1]]])
        return corners @ R.T + center

    def _check_point_obb_collision(self, point, robot_radius, obj):
        # 你的 SAT 检测逻辑 (保留原样)
        center, half, angle = obj['center'], obj['half_extents'], obj['angle']
        tp = point - center
        cos_a, sin_a = math.cos(-angle), math.sin(-angle)
        lx = tp[0]*cos_a - tp[1]*sin_a
        ly = tp[0]*sin_a + tp[1]*cos_a
        expanded = half + robot_radius
        return abs(lx) <= expanded[0] and abs(ly) <= expanded[1]

    def get_collisions_at_point(self, x, y, robot_radius=0.0, only_largest=False):
        # 你的检测逻辑 (保留原样)
        hits = [o for o in self.objects if self._check_point_obb_collision(np.array([x,y]), robot_radius, o)]
        if not hits: return []
        if only_largest: hits.sort(key=lambda o: o['half_extents'][0]*o['half_extents'][1], reverse=True); return [hits[0]['id']]
        return [o['id'] for o in hits]

    # ✅ 新增：可视化绘图功能
    def visualize_task(self, start, end, task_id, output_dir, robot_radius):
        plt.figure(figsize=(10, 10))
        ax = plt.gca()

        # 1. 绘制所有障碍物
        # 先确定哪些物体会显示标签（避免重叠）
        labeled_positions = []  # 已标注的位置
        min_label_distance = 1.5  # 最小标签间距（米）

        for obj in self.objects:
            # 绘制实体轮廓
            poly = Polygon(obj['vertices'], closed=True, facecolor='lightblue', edgecolor='blue', alpha=0.5, label='Obstacle')
            ax.add_patch(poly)

            # 检查是否应该显示标签（避免重叠）
            center = obj['center']
            should_label = True

            for labeled_pos in labeled_positions:
                distance = np.linalg.norm(center - labeled_pos)
                if distance < min_label_distance:
                    should_label = False
                    break

            if should_label:
                # 显示完整的物体名称，字体更大
                ax.text(center[0], center[1], obj['id'],
                       fontsize=12, ha='center', va='center',
                       fontweight='bold', alpha=0.9,
                       bbox=dict(boxstyle='round,pad=0.3',
                                facecolor='white',
                                edgecolor='blue',
                                alpha=0.7))
                labeled_positions.append(center)

        # 2. 绘制起点和终点
        start_circle = Circle(start, radius=robot_radius, color='green', alpha=0.5, label='Start')
        end_circle = Circle(end, radius=robot_radius, color='red', alpha=0.5, label='End')
        ax.add_patch(start_circle)
        ax.add_patch(end_circle)
        ax.plot(start[0], start[1], 'gx', markersize=10)
        ax.plot(end[0], end[1], 'rx', markersize=10)

        # 3. 绘制直线路径
        ax.plot([start[0], end[0]], [start[1], end[1]], 'k--', linewidth=1, label='Straight Line')

        # 4. 设置范围
        all_x = [v[0] for o in self.objects for v in o['vertices']] + [start[0], end[0]]
        all_y = [v[1] for o in self.objects for v in o['vertices']] + [start[1], end[1]]
        if all_x:
            ax.set_xlim(min(all_x)-1, max(all_x)+1)
            ax.set_ylim(min(all_y)-1, max(all_y)+1)

        ax.set_aspect('equal')
        plt.title(f"Scene: {self.scene_name}\nTask: {task_id}")

        # 保存
        os.makedirs(output_dir, exist_ok=True)
        save_path = os.path.join(output_dir, f"{self.scene_name}_{task_id}.png")
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"    🖼️  已生成现场图: {save_path}")

    # ✅ 新增：生成所有物体的碰撞信息图
    def visualize_all_objects(self, tasks=None, output_path="all_objects_collision_map.png"):
        """
        生成场景中所有物体的碰撞信息图

        Args:
            tasks: 任务列表，包含起点和终点信息 (可选)
            output_path: 输出图片路径

        包括：
        - 所有物体的OBB包围盒
        - 物体名称和尺寸信息
        - 不同的颜色表示不同大小的物体
        - 所有任务的起点和终点（如果提供tasks）
        """
        plt.figure(figsize=(16, 16))
        ax = plt.gca()

        # 按物体大小分类着色
        colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A', '#98D8C8',
                 '#F7DC6F', '#BB8FCE', '#85C1E2', '#F8B500', '#52B788']

        labeled_positions = []
        min_label_distance = 1.5

        for i, obj in enumerate(self.objects):
            center = obj['center']
            half_extents = obj['half_extents']
            long_edge = half_extents[0] * 2
            short_edge = half_extents[1] * 2

            # 根据物体大小选择颜色
            area = long_edge * short_edge
            color_idx = min(int(area / 0.5), len(colors) - 1)
            color = colors[color_idx]

            # 绘制OBB包围盒
            vertices = obj['vertices']
            poly = Polygon(vertices, closed=True,
                          facecolor=color, edgecolor='darkblue',
                          alpha=0.6, linewidth=2)
            ax.add_patch(poly)

            # 绘制中心点
            ax.plot(center[0], center[1], 'ko', markersize=4)

            # 绘制朝向箭头（显示物体的旋转方向）
            angle = obj['angle']
            arrow_len = min(long_edge, short_edge) * 0.6
            dx = arrow_len * math.cos(angle)
            dy = arrow_len * math.sin(angle)
            ax.arrow(center[0], center[1], dx, dy,
                    head_width=0.15, head_length=0.1,
                    fc='red', ec='red', alpha=0.7, linewidth=2)

            # 显示标签（避免重叠）
            should_label = True
            for labeled_pos in labeled_positions:
                distance = np.linalg.norm(center - labeled_pos)
                if distance < min_label_distance:
                    should_label = False
                    break

            if should_label:
                # 物体信息标签
                info_text = f"{obj['id']}\n{long_edge:.2f}m × {short_edge:.2f}m"
                ax.text(center[0], center[1], info_text,
                       fontsize=14, ha='center', va='center',
                       fontweight='bold', alpha=1.0,
                       bbox=dict(boxstyle='round,pad=0.5',
                                facecolor='white',
                                edgecolor='darkblue',
                                linewidth=2,
                                alpha=0.9))
                labeled_positions.append(center)

        # 绘制所有任务的起点和终点
        if tasks:
            start_points = []
            end_points = []

            for task in tasks:
                start = task.get('start')
                end = task.get('end')
                if start and end:
                    start_points.append(start)
                    end_points.append(end)

                    # 绘制连接线
                    ax.plot([start[0], end[0]], [start[1], end[1]],
                           'g--', alpha=0.3, linewidth=1)

            # 绘制起点（绿色圆圈）
            if start_points:
                start_x = [p[0] for p in start_points]
                start_y = [p[1] for p in start_points]
                ax.scatter(start_x, start_y, s=200, c='lime', edgecolors='green',
                          linewidths=2, alpha=0.8, label='Start Points', zorder=10)

            # 绘制终点（红色圆圈）
            if end_points:
                end_x = [p[0] for p in end_points]
                end_y = [p[1] for p in end_points]
                ax.scatter(end_x, end_y, s=200, c='red', edgecolors='darkred',
                          linewidths=2, alpha=0.8, label='End Points', zorder=10)

        # 设置范围和比例
        all_x = [v[0] for obj in self.objects for v in obj['vertices']]
        all_y = [v[1] for obj in self.objects for v in obj['vertices']]

        # 包含起点和终点的范围
        if tasks:
            for task in tasks:
                if task.get('start'):
                    all_x.append(task['start'][0])
                    all_y.append(task['start'][1])
                if task.get('end'):
                    all_x.append(task['end'][0])
                    all_y.append(task['end'][1])

        if all_x:
            margin = 1.0
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=1)
        ax.set_xlabel('X (meters)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Y (meters)', fontsize=14, fontweight='bold')
        ax.set_title(f'All Objects Collision Map - {self.scene_name}\n'
                    f'Total Objects: {len(self.objects)}',
                    fontsize=16, fontweight='bold', pad=20)

        # 添加图例说明（英文）
        num_tasks = len(tasks) if tasks else 0
        legend_text = (
            f"📊 Statistics:\n"
            f"• Total Objects: {len(self.objects)}\n"
            f"• Total Tasks: {num_tasks}\n"
            f"• Red Arrow: Object Orientation\n"
            f"• Label: Object Name | Size (L×W)\n"
            f"• Green Circle: Start Points\n"
            f"• Red Circle: End Points\n"
            f"• Dashed Line: Task Path"
        )
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax.text(0.02, 0.98, legend_text, transform=ax.transAxes,
               fontsize=12, verticalalignment='top', bbox=props,
               fontfamily='monospace')

        # 添加图例
        if tasks:
            ax.legend(loc='upper right', fontsize=12, framealpha=0.9)

        # 保存图片
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"✅ Generated collision map: {output_path}")
        print(f"   Objects: {len(self.objects)}, Tasks: {num_tasks}")
        return output_path

# ... (main 函数保持之前的逻辑，但在循环末尾增加 visualize_task 调用) ...

def main():
    # ... (前面的路径设置代码不变) ...
    base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
    scene_base_dir = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
    assets_json = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"
    robot_radius = 0.3
    output_dir = "./collision_maps"  # 输出目录

    # ... (加载 results 代码不变) ...
    results = []
    for f in Path(base_dir).rglob("*_all_results.json"):
        try:
            with open(f) as fp: results.extend(json.load(fp))
        except: pass

    failed_tasks = [t for t in results if not t.get('is_success', False)]
    tasks_by_scene = defaultdict(list)
    for t in failed_tasks:
        tid = t.get('task_id', 'unknown')
        s_name = '_'.join(tid.split('_')[:-1]) if '_' in tid else tid
        tasks_by_scene[s_name].append(t)

    analyzers = {}

    print(f"Analyzing {len(failed_tasks)} failed tasks...")
    print("="*70)

    for scene_name, tasks in tasks_by_scene.items():
        # ... (find_scene_file 代码不变) ...
        import re
        core_name = re.sub(r'_pair$', '', scene_name); core_name = re.sub(r'_20\d{6}_\d{6}$', '', core_name)
        scene_path = None
        for d in Path(scene_base_dir).iterdir():
            if d.is_dir() and core_name in d.name:
                jsons = list(d.glob("*_room_isaacsim.json"))
                if jsons: scene_path = str(jsons[0]); break

        if not scene_path: continue

        if scene_path not in analyzers: analyzers[scene_path] = OBBCollisionAnalyzer(scene_path, assets_json)
        analyzer = analyzers[scene_path]
        if not analyzer.is_loaded: continue

        # ✅ 生成该场景的所有物体碰撞信息图（包含所有任务的起点和终点）
        os.makedirs(output_dir, exist_ok=True)
        scene_map_path = os.path.join(output_dir, f"{analyzer.scene_name}_all_objects.png")
        analyzer.visualize_all_objects(tasks=tasks, output_path=scene_map_path)

        for task in tasks:
            start = task.get('start'); end = task.get('end'); tid = task.get('task_id')[-3:]

            # 检测碰撞
            s_hits = analyzer.get_collisions_at_point(start[0], start[1], robot_radius, only_largest=True)
            e_hits = analyzer.get_collisions_at_point(end[0], end[1], robot_radius, only_largest=True)

            # 打印结果
            status = "❓ Unknown (collision-free)"
            if s_hits: status = f"❌ Start collision: {s_hits[0]}"
            elif e_hits: status = f"❌ End collision: {e_hits[0]}"

            print(f"  Task {tid}: {status}")

            # ✅ 只要是未知的/通畅的，就画图看看到底怎么回事！
            if not s_hits and not e_hits:
                analyzer.visualize_task(start, end, tid, "./debug_images", robot_radius)

    print("="*70)
    print(f"✅ Analysis complete! Collision maps saved to: {output_dir}/")

if __name__ == "__main__":
    main()