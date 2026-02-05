#!/usr/bin/env python3
"""
碰撞检测方法对比测试脚本

对比：
1. 旧方法：圆形包围盒（忽略旋转）
2. 新方法：OBB矩形包围盒（考虑旋转）

展示改进效果
"""

import json
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
from pathlib import Path


class CircleCollision:
    """旧的圆形碰撞检测"""

    def __init__(self, objects):
        self.objects = objects

    def check_collision(self, x, y, robot_radius):
        hits = []
        for obj in self.objects:
            threshold = obj['radius'] + robot_radius
            dist_sq = (x - obj['x'])**2 + (y - obj['y'])**2
            if dist_sq < threshold**2:
                hits.append(obj['id'])
        return hits


class OBBCollision:
    """新的OBB碰撞检测"""

    def __init__(self, objects):
        self.objects = objects

    def check_collision(self, x, y, robot_radius):
        point = np.array([x, y])
        hits = []

        for obj in self.objects:
            if self._check_point_obb_collision(point, robot_radius, obj):
                hits.append(obj['id'])
        return hits

    def _check_point_obb_collision(self, point, robot_radius, obj):
        """OBB碰撞检测"""
        center = obj['center']
        half_extents = obj['half_extents']
        angle = obj['angle']

        # 转换到局部坐标系
        translated = point - center
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        local_x = translated[0] * cos_a - translated[1] * sin_a
        local_y = translated[0] * sin_a + translated[1] * cos_a

        # 扩展边界
        expanded = half_extents + robot_radius

        # 检查
        if abs(local_x) <= expanded[0] and abs(local_y) <= expanded[1]:
            return True
        return False


def load_sample_scene():
    """加载示例场景数据"""
    scene_file = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101/Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_room_isaacsim.json"
    assets_file = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"

    with open(scene_file) as f:
        scene_data = json.load(f)

    with open(assets_file) as f:
        assets_data = json.load(f)
        assets_db = {a['id']: a for a in assets_data['assets']}

    circle_objects = []
    obb_objects = []

    for obj in scene_data.get('objects', []):
        obj_id = obj.get('id', 'unknown')

        # 跳过房间
        if obj_id == 'LaboratoryRoom':
            continue

        pos_raw = obj.get('position', [0, 0, 0])
        if isinstance(pos_raw, dict):
            pos = np.array([pos_raw.get('x', 0), pos_raw.get('y', 0)])
        else:
            pos = np.array([pos_raw[0], pos_raw[1]])

        # 旋转
        rot_raw = obj.get('rotation', {'x': 0, 'y': 0, 'z': 0})
        if isinstance(rot_raw, dict):
            angle_deg = rot_raw.get('z', 0)
        else:
            angle_deg = 0
        angle_rad = math.radians(angle_deg)

        # 资产信息 - 修复：通过物体名称查找资产
        # 场景文件中没有assetId字段，需要通过物体名称匹配
        asset = None

        # 方法1：直接匹配ID
        if obj_id in assets_db:
            asset = assets_db[obj_id]

        # 方法2：尝试模糊匹配（忽略大小写）
        else:
            for asset_id, asset_data in assets_db.items():
                if obj_id.lower() == asset_id.lower():
                    asset = asset_data
                    break

        # 如果找不到资产信息，跳过该物体
        if asset is None:
            continue

        bbox = asset.get('geometry', {}).get('bbox', {})
        if not bbox:
            continue

        short_edge = bbox.get('short', 0.5)
        long_edge = bbox.get('long', 0.5)

        # 旧方法：圆形（使用长边作为直径）
        radius = max(short_edge, long_edge) / 2.0
        circle_objects.append({
            'id': obj_id,
            'x': pos[0],
            'y': pos[1],
            'radius': radius
        })

        # 新方法：OBB矩形
        half_extents = np.array([
            long_edge / 2.0,
            short_edge / 2.0
        ])

        # 计算顶点
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
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

        world_corners = local_corners @ rotation_matrix.T + pos

        obb_objects.append({
            'id': obj_id,
            'center': pos,
            'half_extents': half_extents,
            'angle': angle_rad,
            'vertices': world_corners,
            'short_edge': short_edge,
            'long_edge': long_edge
        })

    return circle_objects, obb_objects


def visualize_comparison(circle_objects, obb_objects):
    """可视化对比"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))

    # 测试点（机器人位置）
    test_points = [
        (2.5, 4.5),  # 靠近ExperimentalPlatform
        (5.3, 7.7),  # 靠近GloveBox
        (0.42, 5.5), # 靠近ReagentCabinet
    ]

    robot_radius = 0.3

    # ========================================
    # 左图：旧方法（圆形）
    # ========================================
    circle_collision = CircleCollision(circle_objects)

    for obj in circle_objects:
        # 绘制圆形
        circle = Circle(
            (obj['x'], obj['y']),
            obj['radius'],
            facecolor='lightblue',
            edgecolor='blue',
            alpha=0.3
        )
        ax1.add_patch(circle)

        # 标注
        ax1.text(obj['x'], obj['y'], obj['id'], fontsize=7, ha='center')

    # 测试点
    for i, (x, y) in enumerate(test_points):
        hits = circle_collision.check_collision(x, y, robot_radius)
        color = 'red' if hits else 'green'
        marker = 'x' if hits else 'o'

        # 机器人圆
        robot_circle = Circle((x, y), robot_radius, facecolor=color, alpha=0.5)
        ax1.add_patch(robot_circle)

        # 中心点
        ax1.plot(x, y, marker, markersize=15, color=color, markeredgewidth=3)
        ax1.text(x, y - 0.5, f"点{i+1}\n{hits if hits else '无碰撞'}", ha='center', fontsize=9)

    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('旧方法：圆形包围盒（忽略旋转）\n可能产生假阳性碰撞', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')

    # ========================================
    # 右图：新方法（OBB矩形）
    # ========================================
    obb_collision = OBBCollision(obb_objects)

    for obj in obb_objects:
        # 绘制OBB
        vertices = obj['vertices']
        poly = Polygon(
            vertices,
            closed=True,
            facecolor='lightgreen',
            edgecolor='green',
            alpha=0.3
        )
        ax2.add_patch(poly)

        # 标注
        center = obj['center']
        ax2.text(center[0], center[1], obj['id'], fontsize=7, ha='center')

        # 绘制朝向箭头
        angle = obj['angle']
        arrow_len = obj['long_edge'] / 3
        dx = arrow_len * math.cos(angle)
        dy = arrow_len * math.sin(angle)
        ax2.arrow(
            center[0], center[1],
            dx, dy,
            head_width=0.1,
            head_length=0.1,
            fc='green',
            ec='green',
            alpha=0.5
        )

    # 测试点
    for i, (x, y) in enumerate(test_points):
        hits = obb_collision.check_collision(x, y, robot_radius)
        color = 'red' if hits else 'green'
        marker = 'x' if hits else 'o'

        # 机器人圆
        robot_circle = Circle((x, y), robot_radius, facecolor=color, alpha=0.5)
        ax2.add_patch(robot_circle)

        # 中心点
        ax2.plot(x, y, marker, markersize=15, color=color, markeredgewidth=3)
        ax2.text(x, y - 0.5, f"点{i+1}\n{hits if hits else '无碰撞'}", ha='center', fontsize=9)

    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('新方法：OBB矩形包围盒（考虑旋转）\n精确碰撞检测，减少假阳性', fontsize=14, fontweight='bold')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')

    # 总标题
    fig.suptitle('碰撞检测方法对比（机器人半径=0.3m）', fontsize=16, fontweight='bold')

    plt.tight_layout()
    output_path = '/tmp/collision_comparison.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ 对比图保存到: {output_path}")

    # 打印统计信息
    print("\n" + "="*70)
    print("碰撞检测统计")
    print("="*70)

    for i, (x, y) in enumerate(test_points):
        circle_hits = circle_collision.check_collision(x, y, robot_radius)
        obb_hits = obb_collision.check_collision(x, y, robot_radius)

        print(f"\n测试点 {i+1}: ({x:.2f}, {y:.2f})")
        print(f"  圆形方法: {len(circle_hits)} 个碰撞 - {circle_hits if circle_hits else '无'}")
        print(f"  OBB方法:  {len(obb_hits)} 个碰撞 - {obb_hits if obb_hits else '无'}")

        if len(circle_hits) > len(obb_hits):
            print(f"  ✅ 改进: OBB方法减少了 {len(circle_hits) - len(obb_hits)} 个假阳性")
        elif len(circle_hits) < len(obb_hits):
            print(f"  ⚠️  注意: OBB方法检测到更多碰撞（可能更精确）")


if __name__ == "__main__":
    print("加载场景数据...")
    circle_objects, obb_objects = load_sample_scene()

    print(f"物体数量: {len(circle_objects)}")

    print("\n生成对比图...")
    visualize_comparison(circle_objects, obb_objects)

    print("\n完成！")
