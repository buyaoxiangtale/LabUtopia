# #!/usr/bin/env python3
# """
# 路径规划障碍物分析工具 v3.1 (修复场景加载版)
# 修复内容：
# 1. 增强了 find_scene_file 的鲁棒性，使用 glob 模糊匹配 JSON 文件
# 2. 增加了更详细的错误调试信息
# """

# import os
# import json
# import sys
# import math
# import traceback
# from pathlib import Path
# from typing import List, Dict, Set, Tuple, Optional
# from collections import defaultdict

# # ==============================================================================
# # 1. 简易碰撞分析器
# # ==============================================================================
# class SimpleCollisionAnalyzer:
#     def __init__(self, scene_json_path: str, assets_json_path: str, default_inflation: float = 0.3):
#         self.objects = []
#         self.default_inflation = default_inflation
#         # 记录是否加载成功
#         self.is_loaded = self.load_scene(scene_json_path, assets_json_path)

#     def load_scene(self, scene_path: str, assets_path: str) -> bool:
#         """加载场景和资产尺寸"""
#         try:
#             if not os.path.exists(scene_path):
#                 print(f"  ❌ 文件不存在: {scene_path}")
#                 return False

#             with open(scene_path, 'r', encoding='utf-8') as f:
#                 scene_data = json.load(f)
            
#             # 检查基本结构
#             if not isinstance(scene_data, dict):
#                 print(f"  ❌ JSON格式错误: 根节点不是字典 (是 {type(scene_data)})")
#                 return False
                
#             obj_list = scene_data.get('objects', [])
#             if not obj_list:
#                 # 尝试另一种常见的Isaac Sim结构 (有时候在该层级下)
#                 if 'graph' in scene_data: # 某些版本结构不同
#                      print(f"  ⚠️  注意: 检测到不同的JSON结构，尝试解析...")
            
#             # 加载资产库
#             assets_db = {}
#             if os.path.exists(assets_path):
#                 try:
#                     with open(assets_path, 'r', encoding='utf-8') as f:
#                         assets_data = json.load(f)
#                         # 处理新的 assets 格式：{"$schema": "...", "assets": [...]}
#                         if isinstance(assets_data, dict) and 'assets' in assets_data:
#                             if isinstance(assets_data['assets'], list):
#                                 # 将数组转换为字典，以 assetId 为键
#                                 for asset in assets_data['assets']:
#                                     if 'id' in asset:
#                                         assets_db[asset['id']] = asset
#                             elif isinstance(assets_data['assets'], dict):
#                                 assets_db = assets_data['assets']
#                         else:
#                             # 旧格式，直接使用
#                             assets_db = assets_data
#                 except Exception as e:
#                     print(f"  ⚠️  Assets 文件加载失败: {e}")
#                     assets_db = {}

#             count = 0
#             for obj in obj_list:
#                 # 排除地板和墙壁 (通常很大，会干扰检测)
#                 obj_id = obj.get('id', 'unknown')
                
#                 pos_raw = obj.get('position', [0, 0, 0])
#                 scale = obj.get('scale', [1, 1, 1])

#                 # 处理 position 字段的不同格式
#                 if isinstance(pos_raw, dict):
#                     # 新格式: {"x": 1.0, "y": 2.0, "z": 3.0}
#                     pos = [pos_raw.get('x', 0), pos_raw.get('y', 0), pos_raw.get('z', 0)]
#                 elif isinstance(pos_raw, list):
#                     # 旧格式: [1.0, 2.0, 3.0]
#                     pos = pos_raw
#                 else:
#                     pos = [0, 0, 0]

#                 asset_id = obj.get('assetId')
#                 bbox_radius = 0.5 
                
#                 if asset_id and asset_id in assets_db:
#                     asset_info = assets_db[asset_id]
#                     # 尝试多种可能的 bounding box 字段结构
#                     bbox = None
#                     if 'geometry' in asset_info and 'bbox' in asset_info['geometry']:
#                         bbox = asset_info['geometry']['bbox']
#                     elif 'boundingBox' in asset_info:
#                         bbox = asset_info['boundingBox']

#                     if bbox:
#                         # 新格式：使用 short/long 字段
#                         if 'short' in bbox and 'long' in bbox:
#                             dims = [bbox['short'], bbox['long'], bbox.get('height', 1)]
#                         # 旧格式：使用 dimensions 字段
#                         elif 'dimensions' in bbox:
#                             dims = bbox['dimensions']
#                         else:
#                             dims = [1, 1, 1]
#                         bbox_radius = max(dims[0], dims[1]) * max(scale[0], scale[1]) / 2.0
#                     else:
#                         bbox_radius = 0.5  # 默认半径
                
#                 self.objects.append({
#                     'id': obj_id,
#                     'x': pos[0],
#                     'y': pos[1],
#                     'radius': bbox_radius,
#                 })
#                 count += 1
            
#             print(f"  ✓ 成功加载场景: {Path(scene_path).name} (包含 {count} 个物体)")
#             return True
                
#         except Exception as e:
#             print(f"  ❌ 场景加载异常: {e}")
#             # 打印更详细的错误以便调试
#             # traceback.print_exc()
#             return False

#     def get_collisions_at_point(self, x: float, y: float, robot_radius: float) -> List[str]:
#         if not self.objects:
#             return []
            
#         hit_objects = []
#         for obj in self.objects:
#             threshold = obj['radius'] + robot_radius
#             # 平方距离比较
#             if (x - obj['x'])**2 + (y - obj['y'])**2 < threshold**2:
#                 hit_objects.append(obj['id'])
#         return hit_objects

# # ==============================================================================
# # 2. 核心分析逻辑
# # ==============================================================================

# def analyze_choke_points(analyzer, start, end, robot_radius=0.3):
#     if not analyzer.is_loaded or not analyzer.objects:
#         return [] # 没加载地图，无法分析

#     dist = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
#     if dist < 0.1: return []
    
#     step_size = 0.1
#     steps = int(dist / step_size)
#     choke_events = []
#     last_collision_set = set()
    
#     # 掐头去尾 0.5m
#     start_step = int(0.5 / step_size)
#     end_step = steps - int(0.5 / step_size)
    
#     if start_step >= end_step: return []

#     for i in range(start_step, end_step):
#         t = i / steps
#         rx = start[0] + (end[0] - start[0]) * t
#         ry = start[1] + (end[1] - start[1]) * t
        
#         hit_ids = analyzer.get_collisions_at_point(rx, ry, robot_radius)
#         current_set = set(hit_ids)
        
#         if hit_ids and current_set != last_collision_set:
#             event_type = "narrow_gap" if len(hit_ids) > 1 else "single_block"
#             choke_events.append({
#                 'dist': f"{dist * t:.1f}m",
#                 'type': event_type,
#                 'objects': hit_ids
#             })
#         last_collision_set = current_set
        
#     return choke_events

# # ==============================================================================
# # 3. 主流程 (修复路径查找)
# # ==============================================================================

# def find_scene_file(scene_name: str, scene_base_dir: str):
#     """
#     更智能的文件查找：
#     1. 找到对应的文件夹
#     2. 在文件夹里找任何以 _room_isaacsim.json 结尾的文件
#     """
#     import re
#     # 移除 _pair 后缀和可能的日期后缀，获取核心场景名
#     # 例如: Hydrolysis_..._pair -> Hydrolysis_...
#     core_name = re.sub(r'_pair$', '', scene_name)
#     core_name = re.sub(r'_20\d{6}_\d{6}$', '', core_name) 
    
#     base = Path(scene_base_dir)
    
#     # 1. 查找包含核心名称的目录
#     # 我们搜索 *CoreName* 这样的目录
#     candidate_dirs = [d for d in base.iterdir() if d.is_dir() and core_name in d.name]
    
#     for d in candidate_dirs:
#         # 2. 在目录中查找 JSON 文件
#         # 不再去猜文件名，而是找该目录下唯一的那个场景描述文件
#         json_files = list(d.glob("*_room_isaacsim.json"))
        
#         if json_files:
#             # 找到了！直接返回第一个匹配项
#             return str(json_files[0])
            
#     return None

# def main():
#     base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
#     scene_base_dir = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
#     assets_json = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"
#     robot_radius = 0.3
    
#     if len(sys.argv) > 1: base_dir = sys.argv[1]
    
#     print(f"分析目录: {base_dir}")
#     print("-" * 50)
    
#     # 加载结果
#     results = []
#     for f in Path(base_dir).rglob("*_all_results.json"):
#         try:
#             with open(f) as fp: results.extend(json.load(fp))
#         except: pass
            
#     failed_tasks = [t for t in results if not t.get('is_success', False)]
#     print(f"失败任务数: {len(failed_tasks)}")

#     # 分组
#     tasks_by_scene = defaultdict(list)
#     for t in failed_tasks:
#         tid = t.get('task_id', 'unknown')
#         s_name = '_'.join(tid.split('_')[:-1]) if '_' in tid else tid
#         tasks_by_scene[s_name].append(t)
        
#     analyzers = {}

#     for scene_name, tasks in tasks_by_scene.items():
#         print(f"\n场景: {scene_name}")
        
#         # 查找场景文件
#         scene_path = find_scene_file(scene_name, scene_base_dir)
#         if not scene_path:
#             print(f"  ❌ 找不到场景文件夹/文件 (搜索名称: {scene_name})")
#             continue
            
#         # 缓存分析器
#         if scene_path not in analyzers:
#             analyzers[scene_path] = SimpleCollisionAnalyzer(scene_path, assets_json)
#         analyzer = analyzers[scene_path]
        
#         # 如果加载失败，跳过该场景分析
#         if not analyzer.is_loaded:
#             print(f"  ❌ 跳过分析 (地图加载失败)")
#             continue
            
#         # 分析任务
#         for task in tasks:
#             start = task.get('start')
#             end = task.get('end')
#             tid = task.get('task_id')[-3:]
            
#             s_hits = analyzer.get_collisions_at_point(start[0], start[1], robot_radius)
#             e_hits = analyzer.get_collisions_at_point(end[0], end[1], robot_radius)
#             chokes = analyze_choke_points(analyzer, start, end, robot_radius)
            
#             print(f"  任务 {tid}:")
#             if s_hits or e_hits:
#                 if s_hits: print(f"    ❌ 起点重合: {s_hits}")
#                 if e_hits: print(f"    ❌ 终点重合: {e_hits}")
#             elif chokes:
#                 print(f"    ⚠️  路径阻隔 ({len(chokes)}处):")
#                 for c in chokes:
#                     print(f"      - {c['dist']} 处被 {c['objects']} ({'夹击' if c['type']=='narrow_gap' else '挡住'})")
#             else:
#                 print(f"    ❓ 奇怪，路径确实通畅 (可能原因: 局部极小值/算法参数)")

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
"""
路径规划障碍物分析工具 v3.2 (只返回最大碰撞物体版)
修改内容：
1. get_collisions_at_point 增加 only_largest 参数
2. 修复起点终点检测逻辑，优先报告体积最大的障碍物
"""

import os
import json
import sys
import math
import traceback
from pathlib import Path
from typing import List, Dict, Set, Tuple, Optional
from collections import defaultdict

# ==============================================================================
# 1. 简易碰撞分析器
# ==============================================================================
class SimpleCollisionAnalyzer:
    def __init__(self, scene_json_path: str, assets_json_path: str, default_inflation: float = 0.3):
        # objects 存储格式: {'id': str, 'x': float, 'y': float, 'radius': float}
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
                pos_raw = obj.get('position', [0, 0, 0])
                scale = obj.get('scale', [1, 1, 1])

                if isinstance(pos_raw, dict):
                    pos = [pos_raw.get('x', 0), pos_raw.get('y', 0), pos_raw.get('z', 0)]
                elif isinstance(pos_raw, list):
                    pos = pos_raw
                else:
                    pos = [0, 0, 0]

                asset_id = obj.get('assetId')
                bbox_radius = 0.5 
                
                if asset_id and asset_id in assets_db:
                    asset_info = assets_db[asset_id]
                    bbox = None
                    if 'geometry' in asset_info and 'bbox' in asset_info['geometry']:
                        bbox = asset_info['geometry']['bbox']
                    elif 'boundingBox' in asset_info:
                        bbox = asset_info['boundingBox']

                    if bbox:
                        if 'short' in bbox and 'long' in bbox:
                            dims = [bbox['short'], bbox['long'], bbox.get('height', 1)]
                        elif 'dimensions' in bbox:
                            dims = bbox['dimensions']
                        else:
                            dims = [1, 1, 1]
                        # 计算半径逻辑：取长宽最大值的一半，再乘以缩放系数
                        bbox_radius = max(dims[0], dims[1]) * max(scale[0], scale[1]) / 2.0
                    else:
                        bbox_radius = 0.5
                
                self.objects.append({
                    'id': obj_id,
                    'x': pos[0],
                    'y': pos[1],
                    'radius': bbox_radius,
                })
                count += 1
            
            print(f"  ✓ 成功加载场景: {Path(scene_path).name} (包含 {count} 个物体)")
            return True
                
        except Exception as e:
            print(f"  ❌ 场景加载异常: {e}")
            return False

    def get_collisions_at_point(self, x: float, y: float, robot_radius: float, only_largest: bool = False) -> List[str]:
        """
        检测某一点的碰撞。
        Args:
            only_largest: 如果为 True，只返回半径最大的那个物体 ID。
        """
        if not self.objects:
            return []
            
        hit_objects = [] # 存储完整的对象字典以便排序
        
        for obj in self.objects:
            # 简单的圆形碰撞检测阈值
            threshold = obj['radius'] + robot_radius
            # 平方距离比较 (避免开方，性能稍好)
            dist_sq = (x - obj['x'])**2 + (y - obj['y'])**2
            
            if dist_sq < threshold**2:
                hit_objects.append(obj)
        
        if not hit_objects:
            return []

        if only_largest:
            # 按 radius 降序排序，取第一个
            hit_objects.sort(key=lambda o: o['radius'], reverse=True)
            return [hit_objects[0]['id']]
        else:
            # 返回所有 ID
            return [o['id'] for o in hit_objects]

# ==============================================================================
# 2. 核心分析逻辑
# ==============================================================================

def analyze_choke_points(analyzer, start, end, robot_radius=0.3):
    if not analyzer.is_loaded or not analyzer.objects:
        return []

    dist = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    if dist < 0.1: return []
    
    step_size = 0.1
    steps = int(dist / step_size)
    choke_events = []
    last_collision_set = set()
    
    start_step = int(0.5 / step_size)
    end_step = steps - int(0.5 / step_size)
    
    if start_step >= end_step: return []

    for i in range(start_step, end_step):
        t = i / steps
        rx = start[0] + (end[0] - start[0]) * t
        ry = start[1] + (end[1] - start[1]) * t
        
        # 这里分析路径中间，我们需要知道是否被多个物体夹住，所以 only_largest=False
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
    
    if len(sys.argv) > 1: base_dir = sys.argv[1]
    
    print(f"分析目录: {base_dir}")
    print("-" * 50)
    
    results = []
    for f in Path(base_dir).rglob("*_all_results.json"):
        try:
            with open(f) as fp: results.extend(json.load(fp))
        except: pass
            
    failed_tasks = [t for t in results if not t.get('is_success', False)]
    print(f"失败任务数: {len(failed_tasks)}")

    tasks_by_scene = defaultdict(list)
    for t in failed_tasks:
        tid = t.get('task_id', 'unknown')
        s_name = '_'.join(tid.split('_')[:-1]) if '_' in tid else tid
        tasks_by_scene[s_name].append(t)
        
    analyzers = {}

    for scene_name, tasks in tasks_by_scene.items():
        print(f"\n场景: {scene_name}")
        scene_path = find_scene_file(scene_name, scene_base_dir)
        if not scene_path:
            print(f"  ❌ 找不到场景文件夹/文件 (搜索名称: {scene_name})")
            continue
            
        if scene_path not in analyzers:
            analyzers[scene_path] = SimpleCollisionAnalyzer(scene_path, assets_json)
        analyzer = analyzers[scene_path]
        
        if not analyzer.is_loaded:
            print(f"  ❌ 跳过分析 (地图加载失败)")
            continue
            
        for task in tasks:
            start = task.get('start')
            end = task.get('end')
            tid = task.get('task_id')[-3:]
            
            # 修改点：在检查起点和终点时，启用 only_largest=True
            s_hits = analyzer.get_collisions_at_point(start[0], start[1], robot_radius, only_largest=True)
            e_hits = analyzer.get_collisions_at_point(end[0], end[1], robot_radius, only_largest=True)
            
            chokes = analyze_choke_points(analyzer, start, end, robot_radius)
            
            print(f"  任务 {tid}:")
            if s_hits or e_hits:
                if s_hits: print(f"    ❌ 起点重合: {s_hits}") # 现在这里只会打印一个最大的物体
                if e_hits: print(f"    ❌ 终点重合: {e_hits}")
            elif chokes:
                print(f"    ⚠️  路径阻隔 ({len(chokes)}处):")
                for c in chokes:
                    print(f"      - {c['dist']} 处被 {c['objects']} ({'夹击' if c['type']=='narrow_gap' else '挡住'})")
            else:
                print(f"    ❓ 奇怪，路径确实通畅")

if __name__ == "__main__":
    main()