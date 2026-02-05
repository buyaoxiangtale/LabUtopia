# #!/usr/bin/env python3
# """
# 路径规划距离统计和分类工具（增强版 - 包含障碍物分析）
# 统计所有成功和失败的路径距离信息，并进行分类分析和障碍物统计
# """

# import os
# import json
# import sys
# from pathlib import Path
# from typing import List, Dict, Tuple, Optional
# import math
# from collections import defaultdict

# # 添加项目路径
# sys.path.insert(0, str(Path(__file__).parent))


# def calculate_euclidean_distance(start: List[float], end: List[float]) -> float:
#     """计算两点之间的欧氏距离"""
#     return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)


# def load_all_results(base_dir: str) -> List[Dict]:
#     """
#     加载所有all_results.json文件
#     """
#     all_results = []

#     base_path = Path(base_dir)
#     if not base_path.exists():
#         print(f"错误: 目录不存在 {base_dir}")
#         return all_results

#     # 递归查找所有 *_all_results.json 文件
#     for result_file in base_path.rglob("*_all_results.json"):
#         try:
#             with open(result_file, 'r', encoding='utf-8') as f:
#                 results = json.load(f)
#                 if isinstance(results, list):
#                     all_results.extend(results)
#                     print(f"✓ 已加载: {result_file.name} ({len(results)} 条记录)")
#         except Exception as e:
#             print(f"✗ 加载失败: {result_file} - {e}")

#     return all_results


# def categorize_by_distance(distance: float) -> str:
#     """按距离对路径进行分类"""
#     if distance < 1.0:
#         return "极短 (< 1m)"
#     elif distance < 2.0:
#         return "短距离 (1-2m)"
#     elif distance < 4.0:
#         return "中等距离 (2-4m)"
#     elif distance < 6.0:
#         return "中长距离 (4-6m)"
#     elif distance < 8.0:
#         return "长距离 (6-8m)"
#     else:
#         return "超长距离 (≥ 8m)"


# def categorize_by_success(result: Dict) -> str:
#     """按成功状态分类"""
#     return "成功" if result.get('is_success', False) else "失败"


# def analyze_obstacles_from_failed_tasks(
#     failed_results: List[Dict],
#     scene_base_dir: str,
#     assets_json_path: str,
#     offset_radius: float = 0.3
# ) -> Dict:
#     """
#     分析失败任务中的障碍物信息

#     Args:
#         failed_results: 失败任务列表
#         scene_base_dir: 场景基础目录
#         assets_json_path: 资产库JSON路径
#         offset_radius: 障碍物膨胀半径

#     Returns:
#         {
#             'obstacle_statistics': {
#                 'start_on_obstacle': {object_id: count},
#                 'end_on_obstacle': {object_id: count},
#                 'blocking_objects': {object_id: count},
#                 'by_category': {category: count}
#             },
#             'failed_tasks_with_obstacles': [...]
#         }
#     """
#     try:
#         from utils.collision_analyzer import CollisionAnalyzer
#     except ImportError:
#         print("⚠️  警告: 无法导入 CollisionAnalyzer，跳过障碍物分析")
#         return {
#             'obstacle_statistics': {},
#             'failed_tasks_with_obstacles': []
#         }

#     obstacle_stats = {
#         'start_on_obstacle': defaultdict(int),  # 起点在障碍物上
#         'end_on_obstacle': defaultdict(int),    # 终点在障碍物上
#         'blocking_objects': defaultdict(int),   # 路径阻隔物体
#         'by_category': defaultdict(int)         # 按类型统计
#     }

#     failed_tasks_with_obstacles = []

#     # 按场景分组失败任务
#     failed_by_scene = defaultdict(list)
#     for result in failed_results:
#         task_id = result.get('task_id', 'unknown')
#         if '_' in task_id:
#             scene_name = '_'.join(task_id.split('_')[:-1])
#         else:
#             scene_name = task_id
#         failed_by_scene[scene_name].append(result)

#     # 对每个场景进行碰撞分析
#     print(f"\n{'='*80}")
#     print(f"障碍物分析")
#     print(f"{'='*80}")

#     for scene_name, tasks in failed_by_scene.items():
#         print(f"\n分析场景: {scene_name}")

#         # 查找场景文件
#         scene_json = find_scene_json(scene_name, scene_base_dir)
#         if not scene_json:
#             print(f"  ⚠️  未找到场景文件，跳过")
#             continue

#         try:
#             # 创建碰撞分析器
#             analyzer = CollisionAnalyzer(scene_json, assets_json_path, offset_radius)

#             for task in tasks:
#                 task_id = task.get('task_id', 'unknown')
#                 start = task.get('start', [0, 0])
#                 end = task.get('end', [0, 0])

#                 # 检查起点和终点的障碍物
#                 start_collisions = analyzer.find_colliding_objects(start[0], start[1])
#                 end_collisions = analyzer.find_colliding_objects(end[0], end[1])

#                 # 统计起点障碍物
#                 for obj in start_collisions:
#                     obstacle_stats['start_on_obstacle'][obj.id] += 1
#                     if obj.asset_info and 'semantic' in obj.asset_info:
#                         category = obj.asset_info['semantic'].get('category', 'Unknown')
#                         obstacle_stats['by_category'][category] += 1

#                 # 统计终点障碍物
#                 for obj in end_collisions:
#                     obstacle_stats['end_on_obstacle'][obj.id] += 1
#                     if obj.asset_info and 'semantic' in obj.asset_info:
#                         category = obj.asset_info['semantic'].get('category', 'Unknown')
#                         obstacle_stats['by_category'][category] += 1

#                 # 保存障碍物信息到任务
#                 task_with_obstacles = task.copy()
#                 task_with_obstacles['start_colliding_objects'] = [obj.id for obj in start_collisions]
#                 task_with_obstacles['end_colliding_objects'] = [obj.id for obj in end_collisions]

#                 # 如果有碰撞，记录详细物体信息
#                 if start_collisions or end_collisions:
#                     obstacle_details = {
#                         'start': [],
#                         'end': []
#                     }

#                     for obj in start_collisions:
#                         obstacle_details['start'].append({
#                             'id': obj.id,
#                             'position': (obj.position[0], obj.position[1]),
#                             'category': obj.asset_info.get('semantic', {}).get('category', 'Unknown') if obj.asset_info else 'Unknown'
#                         })

#                     for obj in end_collisions:
#                         obstacle_details['end'].append({
#                             'id': obj.id,
#                             'position': (obj.position[0], obj.position[1]),
#                             'category': obj.asset_info.get('semantic', {}).get('category', 'Unknown') if obj.asset_info else 'Unknown'
#                         })

#                     task_with_obstacles['obstacle_details'] = obstacle_details

#                 failed_tasks_with_obstacles.append(task_with_obstacles)

#                 # 打印障碍物信息
#                 if start_collisions or end_collisions:
#                     print(f"  任务 {task_id}:")
#                     if start_collisions:
#                         print(f"    起点障碍物: {', '.join([obj.id for obj in start_collisions])}")
#                     if end_collisions:
#                         print(f"    终点障碍物: {', '.join([obj.id for obj in end_collisions])}")

#         except Exception as e:
#             print(f"  ✗ 分析失败: {e}")
#             import traceback
#             traceback.print_exc()

#     return {
#         'obstacle_statistics': {
#             'start_on_obstacle': dict(obstacle_stats['start_on_obstacle']),
#             'end_on_obstacle': dict(obstacle_stats['end_on_obstacle']),
#             'blocking_objects': dict(obstacle_stats['blocking_objects']),
#             'by_category': dict(obstacle_stats['by_category'])
#         },
#         'failed_tasks_with_obstacles': failed_tasks_with_obstacles
#     }


# def find_scene_json(scene_name: str, scene_base_dir: str) -> Optional[str]:
#     """
#     查找场景JSON文件

#     Args:
#         scene_name: 场景名称
#         scene_base_dir: 场景基础目录

#     Returns:
#         场景JSON文件路径，如果未找到返回None
#     """
#     import re

#     # 去掉可能的日期后缀和 _pair 后缀
#     # 例如: Hydrolysis_of_Nitrile_to_Amide_20260112_202101_pair -> Hydrolysis_of_Nitrile_to_Amide
#     clean_name = re.sub(r'_pair$', '', scene_name)  # 去掉 _pair
#     clean_name = re.sub(r'_20\d{6}_\d{6}$', '', clean_name)  # 去掉日期后缀

#     # 搜索可能的场景目录
#     search_paths = [
#         Path(scene_base_dir),
#     ]

#     for search_path in search_paths:
#         # 匹配所有可能的日期后缀
#         scene_dirs = list(search_path.glob(f"*{clean_name}_20*"))

#         if scene_dirs:
#             scene_dir = scene_dirs[0]
#             # 尝试多个可能的文件名模式
#             possible_json_names = [
#                 f"{clean_name}_room_isaacsim.json",
#                 f"{scene_name.replace('_pair', '')}_room_isaacsim.json",
#             ]

#             for json_name in possible_json_names:
#                 scene_json = scene_dir / json_name
#                 if scene_json.exists():
#                     return str(scene_json)

#     return None


# def extract_failed_scenes(all_results: List[Dict]) -> Dict:
#     """
#     提取所有失败的场景信息

#     Returns:
#         {
#             'failed_by_scene': {scene_name: [failed_tasks]},
#             'failed_by_reason': {reason: [tasks]},
#             'all_failed_tasks': [...]
#         }
#     """
#     failed_by_scene = {}
#     failed_by_reason = {}
#     all_failed_tasks = []

#     for result in all_results:
#         if not result.get('is_success', False):
#             task_id = result.get('task_id', 'unknown')

#             # 提取场景名称
#             if '_' in task_id:
#                 scene_name = '_'.join(task_id.split('_')[:-1])  # 移除最后的序号
#             else:
#                 scene_name = task_id

#             # 按场景分组
#             if scene_name not in failed_by_scene:
#                 failed_by_scene[scene_name] = []
#             failed_by_scene[scene_name].append(result)
#             all_failed_tasks.append(result)

#             # 按失败原因分组
#             failure_reason = result.get('failure_reason', '未知原因')
#             if failure_reason not in failed_by_reason:
#                 failed_by_reason[failure_reason] = []
#             failed_by_reason[failure_reason].append(result)

#     return {
#         'failed_by_scene': failed_by_scene,
#         'failed_by_reason': failed_by_reason,
#         'all_failed_tasks': all_failed_tasks,
#         'total_failed_tasks': len(all_failed_tasks),
#         'total_failed_scenes': len(failed_by_scene)
#     }


# def print_failed_scenes_report(failed_info: Dict, obstacle_info: Optional[Dict] = None):
#     """
#     打印失败场景详细报告（包含障碍物信息）
#     """
#     if failed_info['total_failed_tasks'] == 0:
#         print("\n✓ 没有失败的任务！")
#         return

#     print("\n" + "="*80)
#     print("失败场景详情")
#     print("="*80)
#     print(f"\n总失败任务数: {failed_info['total_failed_tasks']}")
#     print(f"总失败场景数: {failed_info['total_failed_scenes']}")

#     # 按场景分组
#     print("\n按场景分组:")
#     scene_list = sorted(
#         failed_info['failed_by_scene'].items(),
#         key=lambda x: len(x[1]),
#         reverse=True
#     )

#     for i, (scene_name, failed_tasks) in enumerate(scene_list, 1):
#         print(f"  {i:2d}. {scene_name} - {len(failed_tasks)}个失败任务")

#     # 打印障碍物统计
#     if obstacle_info and obstacle_info.get('obstacle_statistics'):
#         print("\n" + "="*80)
#         print("障碍物统计")
#         print("="*80)

#         stats = obstacle_info['obstacle_statistics']

#         # 起点在障碍物上
#         if stats.get('start_on_obstacle'):
#             print("\n起点障碍物统计:")
#             sorted_obstacles = sorted(
#                 stats['start_on_obstacle'].items(),
#                 key=lambda x: x[1],
#                 reverse=True
#             )
#             for obj_id, count in sorted_obstacles:
#                 print(f"  - {obj_id}: {count}次")

#         # 终点在障碍物上
#         if stats.get('end_on_obstacle'):
#             print("\n终点障碍物统计:")
#             sorted_obstacles = sorted(
#                 stats['end_on_obstacle'].items(),
#                 key=lambda x: x[1],
#                 reverse=True
#             )
#             for obj_id, count in sorted_obstacles:
#                 print(f"  - {obj_id}: {count}次")

#         # 按类型统计
#         if stats.get('by_category'):
#             print("\n按障碍物类型统计:")
#             sorted_categories = sorted(
#                 stats['by_category'].items(),
#                 key=lambda x: x[1],
#                 reverse=True
#             )
#             for category, count in sorted_categories:
#                 print(f"  - {category}: {count}次")

#     print("\n" + "="*80)


# def save_failed_scenes_report_with_obstacles(
#     failed_info: Dict,
#     obstacle_info: Optional[Dict],
#     output_file: str
# ):
#     """
#     保存失败场景报告到JSON文件（包含障碍物信息）
#     """
#     report = {
#         'summary': {
#             'total_failed_tasks': failed_info['total_failed_tasks'],
#             'total_failed_scenes': failed_info['total_failed_scenes'],
#         },
#         'failed_by_scene': {
#             scene: {
#                 'failed_count': len(tasks),
#                 'failed_tasks': tasks
#             }
#             for scene, tasks in failed_info['failed_by_scene'].items()
#         },
#         'failed_by_reason': {
#             reason: {
#                 'count': len(tasks),
#                 'tasks': tasks
#             }
#             for reason, tasks in failed_info['failed_by_reason'].items()
#         },
#         'all_failed_tasks': failed_info['all_failed_tasks']
#     }

#     # 添加障碍物信息
#     if obstacle_info:
#         report['obstacle_analysis'] = obstacle_info

#     with open(output_file, 'w', encoding='utf-8') as f:
#         json.dump(report, f, indent=2, ensure_ascii=False)

#     print(f"\n失败场景报告已保存至: {output_file}")

#     if obstacle_info:
#         print(f"  包含 {len(obstacle_info.get('failed_tasks_with_obstacles', []))} 个任务的障碍物分析")


# def main():
#     # 默认路径
#     base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
#     output_file = "/home/pjlab/fbh/LabUtopia/path_distance_statistics_report_1_14_with_obstacles.json"
#     failed_scenes_file = "/home/pjlab/fbh/LabUtopia/failed_scenes_report_1_14_with_obstacles.json"

#     # 场景和资产目录
#     scene_base_dir = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
#     assets_json_path = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"

#     # 障碍物膨胀半径
#     offset_radius = 0.3

#     # 是否启用障碍物分析
#     enable_obstacle_analysis = True

#     # 允许从命令行参数指定路径
#     if len(sys.argv) > 1:
#         base_dir = sys.argv[1]
#     if len(sys.argv) > 2:
#         output_file = sys.argv[2]
#     if len(sys.argv) > 3:
#         failed_scenes_file = sys.argv[3]
#     if len(sys.argv) > 4:
#         scene_base_dir = sys.argv[4]
#     if len(sys.argv) > 5:
#         assets_json_path = sys.argv[5]
#     if len(sys.argv) > 6:
#         offset_radius = float(sys.argv[6])
#     if len(sys.argv) > 7:
#         enable_obstacle_analysis = sys.argv[6].lower() == 'true'

#     print("="*80)
#     print("路径规划距离统计分析工具（增强版 - 包含障碍物分析）")
#     print("="*80)
#     print(f"\n扫描目录: {base_dir}")
#     print(f"统计报告: {output_file}")
#     print(f"失败场景报告: {failed_scenes_file}")
#     print(f"场景目录: {scene_base_dir}")
#     print(f"资产库: {assets_json_path}")
#     print(f"膨胀半径: {offset_radius}米")
#     print(f"障碍物分析: {'启用' if enable_obstacle_analysis else '禁用'}\n")

#     # 加载所有结果
#     print("正在加载结果文件...")
#     all_results = load_all_results(base_dir)
#     print(f"\n共加载 {len(all_results)} 条路径规划记录")

#     if not all_results:
#         print("未找到任何结果文件")
#         return

#     # 提取失败场景信息
#     print("\n正在提取失败场景...")
#     failed_results = [r for r in all_results if not r.get('is_success', False)]
#     failed_info = extract_failed_scenes(all_results)

#     # 障碍物分析
#     obstacle_info = None
#     if enable_obstacle_analysis and failed_results:
#         print("\n正在进行障碍物分析...")
#         obstacle_info = analyze_obstacles_from_failed_tasks(
#             failed_results,
#             scene_base_dir,
#             assets_json_path,
#             offset_radius
#         )

#         # 使用增强的失败任务列表
#         if obstacle_info.get('failed_tasks_with_obstacles'):
#             failed_info['all_failed_tasks'] = obstacle_info['failed_tasks_with_obstacles']

#     # 打印失败场景报告
#     print_failed_scenes_report(failed_info, obstacle_info)

#     # 保存失败场景报告
#     save_failed_scenes_report_with_obstacles(failed_info, obstacle_info, failed_scenes_file)

#     print("\n" + "="*80)
#     print("✓ 分析完成！")
#     print("="*80)


# if __name__ == "__main__":
#     main()


# #   PYTHONPATH=. python3 statistics_path_distances_with_obstacles.py \
# #     outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11 \
# #     my_obstacle_report.json \
# #     my_failed_scenes.json \
# #     gemini-3-flash-preview \
# #     gemini-3-flash-preview/assets_annotated.json \
# #     0.3



# # 扫描目录: outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11
# # 统计报告: my_obstacle_report.json
# # 失败场景报告: my_failed_scenes.json
# # 场景目录: gemini-3-flash-preview
# # 资产库: gemini-3-flash-preview/assets_annotated.json
# # 膨胀半径: 0.3米
# # 障碍物分析: 启用

# # 正在加载结果文件...
# # ✓ 已加载: Hydrolysis_of_Nitrile_to_Amide_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Synthesis_of_Acid_Chloride_using_Thionyl_Chloride_20260112_202101_all_results.json (5 条记录)
# # ✓ 已加载: Synthesis_of_a_Piperazinyl-Quinoline_Derivative_20260112_202101_all_results.json (3 条记录)
# # ✓ 已加载: Fmoc_Deprotection_of_Peptide_Intermediate_20260112_202101_all_results.json (5 条记录)
# # ✓ 已加载: Deprotection_and_Cyclization_of_an_Indole_Derivati_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Urea_Synthesis_via_Isocyanate_Addition_20260112_202101_all_results.json (2 条记录)
# # ✓ 已加载: Basic_Methanolysis_of_an_Acetate_Ester_20260112_202101_all_results.json (5 条记录)
# # ✓ 已加载: Oxidation_of_Sulfide_to_Sulfoxide_20260112_202101_all_results.json (7 条记录)
# # ✓ 已加载: Preparation_of_Acid_Chloride_using_Thionyl_Chlorid_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Hydrolysis_of_Ethyl_Ester_using_Lithium_Hydroxide_20260112_202101_all_results.json (3 条记录)
# # ✓ 已加载: Boc_Deprotection_of_Benzyl-methyl-piperidin-4-yl-a_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Hydrolysis_of_Ethyl_Crotonate_using_Lithium_Hydrox_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Synthesis_of_Ethyl_2-ethoxymethylene-3-oxobutanoat_20260112_202101_all_results.json (5 条记录)
# # ✓ 已加载: Deprotection_of_Pyrrolidone_Derivative_TFA_Method_20260112_202101_all_results.json (8 条记录)
# # ✓ 已加载: Preparation_of_Ethyl_4-chloromethyl-125-trimethylp_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Synthesis_of_N-2-cyclopropylphenylcarbamothioylben_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Knoevenagel_Condensation_Protocol_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol_20260112_202101_all_results.json (3 条记录)
# # ✓ 已加载: Preparation_of_Phenylphosphonic_Dichloride_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Synthesis_of_3-4-acetyloxyphenylglutaric_anhydride_20260112_202101_all_results.json (6 条记录)
# # ✓ 已加载: Oxidation_of_Nicotine_to_Nicotinic_Acid_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Synthesis_of_Piperidine-Hydantoin_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Preparation_of_Chlorophenylisocyanate_20260112_202101_all_results.json (3 条记录)
# # ✓ 已加载: Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101_all_results.json (3 条记录)
# # ✓ 已加载: Preparation_of_13-Benzothiazol-6-sulfinic_Acid_Sod_20260112_202101_all_results.json (5 条记录)
# # ✓ 已加载: Synthesis_of_Thiazole_Derivative_via_Hantzsch_Cycl_20260112_202101_all_results.json (6 条记录)
# # ✓ 已加载: Boc_Deprotection_of_Tert-Butyl_Carbazate_20260112_202101_all_results.json (7 条记录)
# # ✓ 已加载: Synthesis_of_Thiourea_Derivative_20260112_202101_all_results.json (4 条记录)
# # ✓ 已加载: Synthesis_of_N-Benzoyl-N-benzylthiourea_20260112_202101_all_results.json (5 条记录)

# # 共加载 133 条路径规划记录

# # 正在提取失败场景...

# # 正在进行障碍物分析...

# # ================================================================================
# # 障碍物分析
# # ================================================================================

# # 分析场景: Hydrolysis_of_Nitrile_to_Amide_20260112_202101_pair
# # ✓ 加载场景: Hydrolysis_of_Nitrile_to_Amide_room_isaacsim.json
# #   物体数量: 26
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Hydrolysis_of_Nitrile_to_Amide_20260112_202101_pair_000:
# #     起点障碍物: ReagentCabinet
# #     终点障碍物: ExperimentalPlatform
# #   任务 Hydrolysis_of_Nitrile_to_Amide_20260112_202101_pair_002:
# #     起点障碍物: FumeHood
# #     终点障碍物: RotaryEvaporator
# #   任务 Hydrolysis_of_Nitrile_to_Amide_20260112_202101_pair_003:
# #     起点障碍物: RotaryEvaporator
# #     终点障碍物: ValidationPlatform

# # 分析场景: Fmoc_Deprotection_of_Peptide_Intermediate_20260112_202101_pair
# # ✓ 加载场景: Fmoc_Deprotection_of_Peptide_Intermediate_room_isaacsim.json
# #   物体数量: 21
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Fmoc_Deprotection_of_Peptide_Intermediate_20260112_202101_pair_004:
# #     起点障碍物: FumeHood
# #     终点障碍物: ExperimentalPlatform, ValidationPlatform, Chair

# # 分析场景: Urea_Synthesis_via_Isocyanate_Addition_20260112_202101_pair
# # ✓ 加载场景: Urea_Synthesis_via_Isocyanate_Addition_room_isaacsim.json
# #   物体数量: 23
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Urea_Synthesis_via_Isocyanate_Addition_20260112_202101_pair_000:
# #     起点障碍物: ReagentCabinet
# #     终点障碍物: FumeHood, ValidationPlatform

# # 分析场景: Preparation_of_Acid_Chloride_using_Thionyl_Chlorid_20260112_202101_pair
# # ✓ 加载场景: Preparation_of_Acid_Chloride_using_Thionyl_Chlorid_room_isaacsim.json
# #   物体数量: 22
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Preparation_of_Acid_Chloride_using_Thionyl_Chlorid_20260112_202101_pair_000:
# #     起点障碍物: ReagentCabinet
# #     终点障碍物: FumeHood, ExperimentalPlatform
# #   任务 Preparation_of_Acid_Chloride_using_Thionyl_Chlorid_20260112_202101_pair_001:
# #     起点障碍物: FumeHood, ExperimentalPlatform
# #     终点障碍物: FumeHood, ExperimentalPlatform

# # 分析场景: Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_pair
# # ✓ 加载场景: Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__room_isaacsim.json
# #   物体数量: 19
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_pair_000:
# #     起点障碍物: ReagentCabinet
# #     终点障碍物: ValidationPlatform, ExperimentalPlatform
# #   任务 Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_pair_001:
# #     起点障碍物: ValidationPlatform, ExperimentalPlatform
# #     终点障碍物: FumeHood, ValidationPlatform
# #   任务 Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_pair_002:
# #     起点障碍物: FumeHood, ValidationPlatform
# #     终点障碍物: RotaryEvaporator
# #   任务 Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_pair_003:
# #     起点障碍物: RotaryEvaporator
# #     终点障碍物: FumeHood, ValidationPlatform

# # 分析场景: Synthesis_of_N-2-cyclopropylphenylcarbamothioylben_20260112_202101_pair
# # ✓ 加载场景: Synthesis_of_N-2-cyclopropylphenylcarbamothioylben_room_isaacsim.json
# #   物体数量: 21
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Synthesis_of_N-2-cyclopropylphenylcarbamothioylben_20260112_202101_pair_000:
# #     起点障碍物: ReagentCabinet
# #     终点障碍物: FumeHood, ExperimentalPlatform

# # 分析场景: Synthesis_of_Thiourea_Derivative_20260112_202101_pair
# # ✓ 加载场景: Synthesis_of_Thiourea_Derivative_room_isaacsim.json
# #   物体数量: 23
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Synthesis_of_Thiourea_Derivative_20260112_202101_pair_000:
# #     起点障碍物: ReagentCabinet
# #     终点障碍物: FumeHood

# # 分析场景: Synthesis_of_N-Benzoyl-N-benzylthiourea_20260112_202101_pair
# # ✓ 加载场景: Synthesis_of_N-Benzoyl-N-benzylthiourea_room_isaacsim.json
# #   物体数量: 25
# #   膨胀半径: 0.3 米
# #   排除基础设施: 是
# #   任务 Synthesis_of_N-Benzoyl-N-benzylthiourea_20260112_202101_pair_004:
# #     起点障碍物: ExperimentalPlatform, Chair
# #     终点障碍物: ExperimentalPlatform, ValidationPlatform

# # ================================================================================
# # 失败场景详情
# # ================================================================================

# # 总失败任务数: 14
# # 总失败场景数: 8

# # 按场景分组:
# #    1. Preparation_of_5-Chloro-2-methoxybenzoyl_Chloride__20260112_202101_pair - 4个失败任务
# #    2. Hydrolysis_of_Nitrile_to_Amide_20260112_202101_pair - 3个失败任务
# #    3. Preparation_of_Acid_Chloride_using_Thionyl_Chlorid_20260112_202101_pair - 2个失败任务
# #    4. Fmoc_Deprotection_of_Peptide_Intermediate_20260112_202101_pair - 1个失败任务
# #    5. Urea_Synthesis_via_Isocyanate_Addition_20260112_202101_pair - 1个失败任务
# #    6. Synthesis_of_N-2-cyclopropylphenylcarbamothioylben_20260112_202101_pair - 1个失败任务
# #    7. Synthesis_of_Thiourea_Derivative_20260112_202101_pair - 1个失败任务
# #    8. Synthesis_of_N-Benzoyl-N-benzylthiourea_20260112_202101_pair - 1个失败任务

# # ================================================================================
# # 障碍物统计
# # ================================================================================

# # 起点障碍物统计:
# #   - ReagentCabinet: 6次
# #   - FumeHood: 4次
# #   - ExperimentalPlatform: 3次
# #   - RotaryEvaporator: 2次
# #   - ValidationPlatform: 2次
# #   - Chair: 1次

# # 终点障碍物统计:
# #   - ExperimentalPlatform: 7次
# #   - ValidationPlatform: 7次
# #   - FumeHood: 7次
# #   - RotaryEvaporator: 2次
# #   - Chair: 1次

# # 按障碍物类型统计:
# #   - furniture: 21次
# #   - safety_equipment: 11次
# #   - storage_furniture: 6次
# #   - evaporation_equipment: 4次

# # ================================================================================

# # 失败场景报告已保存至: my_failed_scenes.json
# #   包含 14 个任务的障碍物分析

# # ================================================================================
# # ✓ 分析完成！

#!/usr/bin/env python3
"""
路径规划距离统计和分类工具（增强版 v2.0）
功能：
1. 统计路径距离分布
2. 引入对照组分析：对比成功vs失败任务的障碍物碰撞率
3. 修正统计偏差（去重、分离起终点）
"""

import os
import json
import sys
from pathlib import Path
from typing import List, Dict, Tuple, Optional, Set
import math
from collections import defaultdict, Counter

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))


def calculate_euclidean_distance(start: List[float], end: List[float]) -> float:
    """计算两点之间的欧氏距离"""
    return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)


def load_all_results(base_dir: str) -> List[Dict]:
    """加载所有all_results.json文件"""
    all_results = []
    base_path = Path(base_dir)
    if not base_path.exists():
        print(f"错误: 目录不存在 {base_dir}")
        return all_results

    for result_file in base_path.rglob("*_all_results.json"):
        try:
            with open(result_file, 'r', encoding='utf-8') as f:
                results = json.load(f)
                if isinstance(results, list):
                    all_results.extend(results)
                    print(f"✓ 已加载: {result_file.name} ({len(results)} 条记录)")
        except Exception as e:
            print(f"✗ 加载失败: {result_file} - {e}")
    return all_results


def find_scene_json(scene_name: str, scene_base_dir: str) -> Optional[str]:
    """查找场景JSON文件"""
    import re
    clean_name = re.sub(r'_pair$', '', scene_name)
    clean_name = re.sub(r'_20\d{6}_\d{6}$', '', clean_name)
    
    search_paths = [Path(scene_base_dir)]
    
    for search_path in search_paths:
        scene_dirs = list(search_path.glob(f"*{clean_name}_20*"))
        if scene_dirs:
            scene_dir = scene_dirs[0]
            possible_names = [
                f"{clean_name}_room_isaacsim.json",
                f"{scene_name.replace('_pair', '')}_room_isaacsim.json",
            ]
            for json_name in possible_names:
                scene_json = scene_dir / json_name
                if scene_json.exists():
                    return str(scene_json)
    return None


def perform_comprehensive_analysis(
    all_results: List[Dict],
    scene_base_dir: str,
    assets_json_path: str,
    offset_radius: float = 0.3
) -> Dict:
    """
    执行综合分析（包含成功和失败的任务）
    """
    try:
        from utils.collision_analyzer import CollisionAnalyzer
    except ImportError:
        print("⚠️  警告: 无法导入 CollisionAnalyzer，跳过障碍物分析")
        return {}

    # 初始化统计容器
    stats = {
        'success': {
            'total': 0,
            'start_hits': Counter(),      # 起点碰撞物体计数
            'end_hits': Counter(),        # 终点碰撞物体计数
            'start_categories': Counter(),# 起点碰撞类别计数
            'end_categories': Counter()   # 终点碰撞类别计数
        },
        'failure': {
            'total': 0,
            'start_hits': Counter(),
            'end_hits': Counter(),
            'start_categories': Counter(),
            'end_categories': Counter()
        }
    }

    annotated_tasks = []

    # 按场景分组以优化加载
    tasks_by_scene = defaultdict(list)
    for res in all_results:
        task_id = res.get('task_id', 'unknown')
        scene_name = '_'.join(task_id.split('_')[:-1]) if '_' in task_id else task_id
        tasks_by_scene[scene_name].append(res)

    print(f"\n{'='*80}")
    print(f"全量障碍物分析 (对照组分析)")
    print(f"{'='*80}")

    for scene_name, tasks in tasks_by_scene.items():
        scene_json = find_scene_json(scene_name, scene_base_dir)
        if not scene_json:
            continue

        try:
            # 每个场景只加载一次分析器
            analyzer = CollisionAnalyzer(scene_json, assets_json_path, offset_radius)
            
            for task in tasks:
                is_success = task.get('is_success', False)
                group = 'success' if is_success else 'failure'
                stats[group]['total'] += 1

                start = task.get('start', [0, 0])
                end = task.get('end', [0, 0])

                # 碰撞检测
                start_collisions = analyzer.find_colliding_objects(start[0], start[1])
                end_collisions = analyzer.find_colliding_objects(end[0], end[1])

                # === 关键修正：去重统计 ===
                # 1. 获取唯一的物体ID和类别 (set comprehension)
                s_ids = {obj.id for obj in start_collisions}
                e_ids = {obj.id for obj in end_collisions}
                
                s_cats = {obj.asset_info['semantic'].get('category', 'Unknown') 
                          for obj in start_collisions if obj.asset_info}
                e_cats = {obj.asset_info['semantic'].get('category', 'Unknown') 
                          for obj in end_collisions if obj.asset_info}

                # 2. 更新统计 (每个任务对每个物体/类别只贡献1次)
                stats[group]['start_hits'].update(s_ids)
                stats[group]['end_hits'].update(e_ids)
                stats[group]['start_categories'].update(s_cats)
                stats[group]['end_categories'].update(e_cats)

                # 3. 记录到任务详情 (仅针对失败任务或需要调试时)
                task_copy = task.copy()
                if start_collisions or end_collisions:
                    task_copy['collision_info'] = {
                        'start_objects': list(s_ids),
                        'end_objects': list(e_ids),
                        'start_categories': list(s_cats),
                        'end_categories': list(e_cats)
                    }
                annotated_tasks.append(task_copy)

        except Exception as e:
            print(f"  ✗ 场景 {scene_name} 分析出错: {e}")

    return {
        'stats': stats,
        'annotated_tasks': annotated_tasks
    }


def print_comparative_report(analysis_data: Dict):
    """打印对比分析报告"""
    if not analysis_data:
        return

    stats = analysis_data['stats']
    s_total = stats['success']['total']
    f_total = stats['failure']['total']

    if s_total == 0 and f_total == 0:
        return

    print("\n" + "="*80)
    print("障碍物影响对比分析 (成功组 vs 失败组)")
    print("="*80)
    print(f"样本概况: 成功任务 {s_total} 个 | 失败任务 {f_total} 个")

    def print_top_k(title, success_counter, failure_counter, k=5):
        print(f"\n>>> {title}")
        print(f"{'名称':<30} | {'失败组碰撞率 (Count)':<25} | {'成功组碰撞率 (Count)':<25} | {'风险倍率':<10}")
        print("-" * 100)
        
        # 合并所有的key
        all_keys = set(success_counter.keys()) | set(failure_counter.keys())
        
        # 计算风险倍率并排序
        data = []
        for key in all_keys:
            f_count = failure_counter[key]
            s_count = success_counter[key]
            
            f_rate = (f_count / f_total * 100) if f_total > 0 else 0
            s_rate = (s_count / s_total * 100) if s_total > 0 else 0
            
            # 风险倍率：失败组概率 / 成功组概率
            risk_ratio = f_rate / s_rate if s_rate > 0 else (99.9 if f_rate > 0 else 0)
            
            data.append({
                'key': key,
                'f_count': f_count,
                's_count': s_count,
                'f_rate': f_rate,
                's_rate': s_rate,
                'risk': risk_ratio
            })
        
        # 按失败组碰撞率排序
        data.sort(key=lambda x: x['f_rate'], reverse=True)
        
        for item in data[:k]:
            risk_str = f"{item['risk']:.1f}x" if item['s_rate'] > 0 else "INF"
            print(f"{item['key']:<30} | {item['f_rate']:5.1f}% ({item['f_count']:3d})       | {item['s_rate']:5.1f}% ({item['s_count']:3d})       | {risk_str}")

    # 1. 终点障碍物 (最可能是导致不可达的原因)
    print_top_k("终点障碍物 Top 5 (End Objects)", 
                stats['success']['end_hits'], 
                stats['failure']['end_hits'])

    # 2. 起点障碍物
    print_top_k("起点障碍物 Top 5 (Start Objects)", 
                stats['success']['start_hits'], 
                stats['failure']['start_hits'])

    # 3. 终点类别
    print_top_k("终点物体类别分布 (End Categories)", 
                stats['success']['end_categories'], 
                stats['failure']['end_categories'])


def save_full_report(failed_tasks: List[Dict], analysis_data: Dict, output_file: str):
    """保存完整JSON报告"""
    report = {
        'summary': {
            'generated_at': str(os.path.getmtime(__file__)),
            'total_analyzed': 0
        },
        'obstacle_analysis': analysis_data.get('stats', {}),
        'failed_tasks_details': failed_tasks
    }
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\n完整报告已保存至: {output_file}")


def main():
    # 默认配置
    base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
    output_file = "./comprehensive_report.json"
    scene_base_dir = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
    assets_json_path = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"
    offset_radius = 0.3

    # CLI参数支持
    if len(sys.argv) > 1: base_dir = sys.argv[1]
    if len(sys.argv) > 2: output_file = sys.argv[2]
    
    print("="*80)
    print("路径规划综合分析工具 v2.0")
    print("="*80)

    # 1. 加载数据
    all_results = load_all_results(base_dir)
    if not all_results: return

    # 2. 提取失败任务用于展示
    failed_tasks = [r for r in all_results if not r.get('is_success', False)]
    print(f"\n数据总览: 总任务 {len(all_results)} | 失败 {len(failed_tasks)} | 成功 {len(all_results) - len(failed_tasks)}")

    # 3. 执行全量碰撞分析
    analysis_data = perform_comprehensive_analysis(
        all_results, scene_base_dir, assets_json_path, offset_radius
    )

    # 4. 打印对比报告
    if analysis_data:
        print_comparative_report(analysis_data)
        
        # 将分析结果中的详细信息合并回失败任务列表
        # 创建一个映射方便查找
        annotated_map = {
            t['task_id']: t.get('collision_info', {}) 
            for t in analysis_data.get('annotated_tasks', [])
        }
        
        for task in failed_tasks:
            tid = task.get('task_id')
            if tid in annotated_map:
                task['collision_analysis'] = annotated_map[tid]

    # 5. 保存
    save_full_report(failed_tasks, analysis_data, output_file)

if __name__ == "__main__":
    main()