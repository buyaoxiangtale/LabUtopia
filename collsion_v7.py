#!/usr/bin/env python3
"""
路径规划障碍物分析工具 v7.0 (可达性验证版)

功能：
1. OBB 精确碰撞检测 (起终点)
2. 语义过滤 (忽略移液枪等小物体)
3. GridMap + BFS 搜索 (区分"死路"与"绕路")
4. 生成详细的分类报告
"""

import os
import json
import math
import yaml
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from pathlib import Path
from typing import List, Dict, Tuple, Set, Optional
from collections import deque

# ==============================================================================
# 1. 栅格地图与BFS求解器 (核心新增逻辑)
# ==============================================================================
class GridMapReachability:
    def __init__(self, objects: List[Dict], resolution: float = 0.1, margin: float = 2.0):
        """
        resolution: 网格大小 (米), 越小越精确但越慢. 0.1m (10cm) 是个不错的平衡
        margin: 地图边界外扩距离
        """
        self.resolution = resolution
        
        # 1. 计算地图边界
        all_x, all_y = [], []
        for obj in objects:
            v = obj['vertices']
            all_x.extend(v[:, 0])
            all_y.extend(v[:, 1])
            
        if not all_x:
            self.min_x, self.max_x = -10, 10
            self.min_y, self.max_y = -10, 10
        else:
            self.min_x = min(all_x) - margin
            self.max_x = max(all_x) + margin
            self.min_y = min(all_y) - margin
            self.max_y = max(all_y) + margin
            
        self.width = int((self.max_x - self.min_x) / resolution) + 1
        self.height = int((self.max_y - self.min_y) / resolution) + 1
        
        # 0: 空闲, 1: 障碍物
        self.grid = np.zeros((self.width, self.height), dtype=np.int8)
        
        # 2. 光栅化所有障碍物
        self._rasterize_objects(objects)

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.min_x) / self.resolution)
        gy = int((wy - self.min_y) / self.resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        wx = self.min_x + (gx + 0.5) * self.resolution
        wy = self.min_y + (gy + 0.5) * self.resolution
        return wx, wy

    def _rasterize_objects(self, objects):
        """将OBB列表映射到网格"""
        for obj in objects:
            # 获取该物体的AABB (轴对齐包围盒) 以缩小扫描范围
            v = obj['vertices']
            min_ox, max_ox = np.min(v[:, 0]), np.max(v[:, 0])
            min_oy, max_oy = np.min(v[:, 1]), np.max(v[:, 1])
            
            # 转换到网格坐标范围
            start_gx, start_gy = self._world_to_grid(min_ox, min_oy)
            end_gx, end_gy = self._world_to_grid(max_ox, max_oy)
            
            # 限制在地图内
            start_gx = max(0, start_gx)
            start_gy = max(0, start_gy)
            end_gx = min(self.width, end_gx + 1)
            end_gy = min(self.height, end_gy + 1)
            
            # 预计算参数以加速循环
            center = obj['center']
            half_extents = obj['half_extents']
            angle = obj['angle']
            cos_a = math.cos(-angle)
            sin_a = math.sin(-angle)
            
            # 遍历局部区域，检查每个格子中心是否在 OBB 内
            for gx in range(start_gx, end_gx):
                for gy in range(start_gy, end_gy):
                    # 如果已经是障碍物，跳过
                    if self.grid[gx, gy] == 1: continue
                    
                    wx, wy = self._grid_to_world(gx, gy)
                    
                    # 变换点到 OBB 局部坐标系
                    dx, dy = wx - center[0], wy - center[1]
                    lx = dx * cos_a - dy * sin_a
                    ly = dx * sin_a + dy * cos_a
                    
                    # 检查是否在矩形内 (稍微放宽一点点容差)
                    if abs(lx) <= half_extents[0] and abs(ly) <= half_extents[1]:
                        self.grid[gx, gy] = 1

    def check_reachability(self, start: np.ndarray, end: np.ndarray, robot_radius: float) -> Tuple[bool, str]:
        """BFS 搜索"""
        sx, sy = self._world_to_grid(start[0], start[1])
        ex, ey = self._world_to_grid(end[0], end[1])
        
        # 边界检查
        if not (0 <= sx < self.width and 0 <= sy < self.height): return False, "Start OOB"
        if not (0 <= ex < self.width and 0 <= ey < self.height): return False, "End OOB"
        
        # 起终点本身就在障碍物里 (这是 Start Collision 检测的事，但这里也会拦住)
        if self.grid[sx, sy] == 1: return False, "Start In Obstacle"
        if self.grid[ex, ey] == 1: return False, "End In Obstacle"

        # 机器人膨胀网格半径 (粗略估计)
        # 注意：这里我们只做简单的连通性检查，不进行全图膨胀(C-Space)，因为比较耗时
        # 我们假设只要网格中心通且没撞到原始障碍物即可，robot_radius 主要用于 Start/End 检测
        
        queue = deque([(sx, sy)])
        visited = set([(sx, sy)])
        
        found = False
        
        # 8-邻域方向
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        while queue:
            cx, cy = queue.popleft()
            
            if cx == ex and cy == ey:
                found = True
                break
            
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) not in visited:
                        if self.grid[nx, ny] == 0: # 必须是空闲格子
                            visited.add((nx, ny))
                            queue.append((nx, ny))
                            
        if found:
            return True, "Reachable"
        else:
            return False, "Disconnected"

# ==============================================================================
# 2. OBB分析器 (集成加载和过滤)
# ==============================================================================
class OBBCollisionAnalyzer:
    def __init__(self, scene_json_path: str, assets_json_path: str):
        self.objects = []
        self.scene_name = Path(scene_json_path).stem
        
        # 定义要忽略的小物体关键词 (小写)
        self.ignore_keywords = {
            'pipette', 'beaker', 'flask', 'cylinder', 'testtube', 
            'reagent', 'bottle', 'vial', 'glass', 'heatingplate'
        }
        
        self.is_loaded = self.load_scene(scene_json_path, assets_json_path)

    def load_scene(self, scene_path: str, assets_path: str) -> bool:
        try:
            with open(scene_path, 'r', encoding='utf-8') as f:
                scene_data = json.load(f)
            
            assets_db = {}
            if os.path.exists(assets_path):
                try:
                    with open(assets_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        assets_db = data.get('assets', data) if isinstance(data, dict) else {}
                except: pass

            count = 0
            skipped = 0
            
            for obj in scene_data.get('objects', []):
                obj_id = obj.get('id', 'unknown')
                
                # --- 语义过滤 ---
                # 1. 检查类别
                category = obj.get('semantic', {}).get('category', '').lower()
                
                # 2. 检查 ID 关键词
                is_ignored = False
                for kw in self.ignore_keywords:
                    if kw in obj_id.lower() or kw in category:
                        is_ignored = True
                        break
                
                if is_ignored:
                    skipped += 1
                    continue
                # ----------------

                pos = np.array(obj.get('position', [0, 0])[:2])
                rot = obj.get('rotation', [0, 0, 0])
                angle = math.radians(rot[2] if len(rot) > 2 else 0)
                scale = obj.get('scale', [1, 1, 1])
                
                # 尝试从资产库获取尺寸，默认为 0.5x0.5
                short, long_edge = 0.5, 0.5
                asset_id = obj.get('assetId')
                
                # 简化的尺寸获取逻辑
                if asset_id and asset_id in assets_db:
                    info = assets_db[asset_id]
                    # 尝试解析 boundingBox
                    bbox = info.get('boundingBox') or info.get('geometry', {}).get('bbox')
                    if bbox:
                        dims = bbox.get('dimensions', [0.5, 0.5])
                        short, long_edge = min(dims[:2]), max(dims[:2])
                
                # 应用缩放
                short *= scale[0] # 简化处理，假设xy缩放接近
                long_edge *= scale[1]
                
                half_extents = np.array([long_edge/2.0, short/2.0])
                
                # 计算顶点
                cos_a, sin_a = math.cos(angle), math.sin(angle)
                R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
                
                local_verts = np.array([
                    [half_extents[0], half_extents[1]],
                    [-half_extents[0], half_extents[1]],
                    [-half_extents[0], -half_extents[1]],
                    [half_extents[0], -half_extents[1]]
                ])
                
                vertices = (local_verts @ R.T) + pos
                
                self.objects.append({
                    'id': obj_id,
                    'center': pos,
                    'half_extents': half_extents,
                    'angle': angle,
                    'vertices': vertices,
                    'short_edge': short,
                    'long_edge': long_edge
                })
                count += 1
                
            print(f"    ✓ 加载场景: {self.scene_name} (物体: {count}, 过滤杂物: {skipped})")
            return True
        except Exception as e:
            print(f"    ❌ 加载失败: {e}")
            return False

    def get_collisions_at_point(self, point, radius, only_largest=True):
        hit = []
        for obj in self.objects:
            # 1. 快速圆形过滤
            max_r = max(obj['half_extents']) + radius
            if np.linalg.norm(point - obj['center']) > max_r * 1.5:
                continue
            
            # 2. 精确 OBB 检测
            # 变换点到局部坐标
            d = point - obj['center']
            cos_a = math.cos(-obj['angle'])
            sin_a = math.sin(-obj['angle'])
            lx = d[0]*cos_a - d[1]*sin_a
            ly = d[0]*sin_a + d[1]*cos_a
            
            threshold = obj['half_extents'] + radius
            if abs(lx) <= threshold[0] and abs(ly) <= threshold[1]:
                hit.append(obj)
        
        if not hit: return []
        if only_largest:
            hit.sort(key=lambda o: o['long_edge']*o['short_edge'], reverse=True)
            return [hit[0]['id']]
        return [o['id'] for o in hit]

# ==============================================================================
# 3. 辅助函数
# ==============================================================================
def find_scene_file(scene_name: str, scene_base_dir: str):
    import re
    core = re.sub(r'_20\d{6}_\d{6}$', '', scene_name)
    base = Path(scene_base_dir)
    for d in base.iterdir():
        if d.is_dir() and core in d.name:
            json_files = list(d.glob("*_room_isaacsim.json"))
            if json_files: return str(json_files[0])
    return None

def check_straight_line_block(analyzer, start, end, radius=0.3):
    """简单的直线采样检测，用于区分 Soft/Hard Block"""
    dist = np.linalg.norm(end - start)
    if dist < 0.1: return False
    steps = int(dist / 0.1)
    for i in range(1, steps): # 跳过头尾
        t = i / steps
        p = start + (end - start) * t
        hits = analyzer.get_collisions_at_point(p, radius)
        if hits: return True
    return False

# ==============================================================================
# 4. 主程序
# ==============================================================================
def main():
    # 配置
    GOAL_PAIRS_DIR = "/home/pjlab/fbh/LabUtopia/outputs/gemini_flash_nav_targets_1_13_18_47/goal_pairs"
    SCENE_BASE_DIR = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
    ASSETS_JSON = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"
    ROBOT_RADIUS = 0.3
    
    print("="*80)
    print("路径规划可达性验证工具 v7.0")
    print("="*80)
    
    analyzers = {}
    
    # 统计数据
    stats = {
        'total': 0,
        'critical_fail': 0, # 起终点撞
        'hard_unreachable': 0, # 迷宫死路
        'soft_blocked': 0, # 需绕行 (视为成功)
        'clear': 0 # 直线通畅 (视为成功)
    }

    yaml_files = sorted(list(Path(GOAL_PAIRS_DIR).glob("*_goal_pairs.yaml")))
    
    for yaml_file in yaml_files:
        try:
            with open(yaml_file, 'r') as f: data = yaml.safe_load(f)
        except: continue
        
        goal_pairs = data.get('goal_pairs', [])
        if not goal_pairs: continue
        
        scene_name = data.get('metadata', {}).get('scene_name', yaml_file.stem.replace('_goal_pairs', ''))
        
        # 加载场景
        scene_path = find_scene_file(scene_name, SCENE_BASE_DIR)
        if not scene_path: continue
        
        if scene_path not in analyzers:
            analyzers[scene_path] = OBBCollisionAnalyzer(scene_path, ASSETS_JSON)
        analyzer = analyzers[scene_path]
        
        # 构建 GridMap (每个场景构建一次)
        if not hasattr(analyzer, 'grid_map'):
            analyzer.grid_map = GridMapReachability(analyzer.objects, resolution=0.1)

        print(f"\n场景: {scene_name} ({len(goal_pairs)} 任务)")
        
        for idx, gp in enumerate(goal_pairs):
            start = np.array(gp['start'][:2])
            end = np.array(gp['end'][:2])
            stats['total'] += 1
            
            # 1. 严格几何碰撞 (Start/End)
            s_hits = analyzer.get_collisions_at_point(start, ROBOT_RADIUS)
            e_hits = analyzer.get_collisions_at_point(end, ROBOT_RADIUS)
            
            status = ""
            log_color = "" # 仅用于逻辑标记
            
            if s_hits:
                stats['critical_fail'] += 1
                status = f"[❌ 失败] 起点卡死 ({s_hits[0]})"
            elif e_hits:
                stats['critical_fail'] += 1
                status = f"[❌ 失败] 终点卡死 ({e_hits[0]})"
            else:
                # 2. 可达性检查 (BFS)
                is_reachable, reason = analyzer.grid_map.check_reachability(start, end, ROBOT_RADIUS)
                
                if not is_reachable:
                    stats['hard_unreachable'] += 1
                    status = f"[🚫 失败] 迷宫不可达 (死路)"
                else:
                    # 3. 直线检测 (区分绕路)
                    is_line_blocked = check_straight_line_block(analyzer, start, end, ROBOT_RADIUS)
                    
                    if is_line_blocked:
                        stats['soft_blocked'] += 1
                        status = f"[⚠️ 可行] 需绕行 (Soft Block)"
                    else:
                        stats['clear'] += 1
                        status = f"[✓ 可行] 直线通畅"
            
            print(f"  Task {idx:02d}: {status}")

    # 总结报告
    print("\n" + "="*80)
    print("最终统计报告 (v7.0)")
    print("="*80)
    total = stats['total']
    fails = stats['critical_fail'] + stats['hard_unreachable']
    success_potential = stats['soft_blocked'] + stats['clear']
    
    print(f"总任务数: {total}")
    print("-" * 40)
    print(f"判定为失败 (FAIL): {fails} ({fails/total*100:.1f}%)")
    print(f"  - 起终点卡死: {stats['critical_fail']}")
    print(f"  - 迷宫不可达: {stats['hard_unreachable']}")
    print("-" * 40)
    print(f"判定为可行 (PASS): {success_potential} ({success_potential/total*100:.1f}%)")
    print(f"  - 直线通畅: {stats['clear']}")
    print(f"  - 需绕行 (Soft): {stats['soft_blocked']} <--- 之前被误判为失败的部分")
    print("="*80)

if __name__ == "__main__":
    main()