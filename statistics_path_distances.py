#!/usr/bin/env python3
"""
路径规划距离统计和分类工具
统计所有成功和失败的路径距离信息，并进行分类分析
"""

import os
import json
import sys
from pathlib import Path
from typing import List, Dict, Tuple
import math


def calculate_euclidean_distance(start: List[float], end: List[float]) -> float:
    """计算两点之间的欧氏距离"""
    return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)


def load_all_results(base_dir: str) -> List[Dict]:
    """
    加载所有all_results.json文件
    """
    all_results = []

    base_path = Path(base_dir)
    if not base_path.exists():
        print(f"错误: 目录不存在 {base_dir}")
        return all_results

    # 递归查找所有 *_all_results.json 文件
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


def categorize_by_distance(distance: float) -> str:
    """按距离对路径进行分类"""
    if distance < 1.0:
        return "极短 (< 1m)"
    elif distance < 2.0:
        return "短距离 (1-2m)"
    elif distance < 4.0:
        return "中等距离 (2-4m)"
    elif distance < 6.0:
        return "中长距离 (4-6m)"
    elif distance < 8.0:
        return "长距离 (6-8m)"
    else:
        return "超长距离 (≥ 8m)"


def categorize_by_success(result: Dict) -> str:
    """按成功状态分类"""
    return "成功" if result.get('is_success', False) else "失败"


def extract_failed_scenes(all_results: List[Dict]) -> Dict:
    """
    提取所有失败的场景信息

    Returns:
        {
            'failed_by_scene': {scene_name: [failed_tasks]},
            'failed_by_reason': {reason: [tasks]},
            'all_failed_tasks': [...]
        }
    """
    failed_by_scene = {}
    failed_by_reason = {}
    all_failed_tasks = []

    for result in all_results:
        if not result.get('is_success', False):
            task_id = result.get('task_id', 'unknown')

            # 提取场景名称
            if '_' in task_id:
                scene_name = '_'.join(task_id.split('_')[:-1])  # 移除最后的序号
            else:
                scene_name = task_id

            # 按场景分组
            if scene_name not in failed_by_scene:
                failed_by_scene[scene_name] = []
            failed_by_scene[scene_name].append(result)
            all_failed_tasks.append(result)

            # 按失败原因分组
            failure_reason = result.get('failure_reason', '未知原因')
            if failure_reason not in failed_by_reason:
                failed_by_reason[failure_reason] = []
            failed_by_reason[failure_reason].append(result)

    return {
        'failed_by_scene': failed_by_scene,
        'failed_by_reason': failed_by_reason,
        'all_failed_tasks': all_failed_tasks,
        'total_failed_tasks': len(all_failed_tasks),
        'total_failed_scenes': len(failed_by_scene)
    }


def print_failed_scenes_report(failed_info: Dict):
    """
    打印失败场景详细报告
    """
    if failed_info['total_failed_tasks'] == 0:
        print("\n✓ 没有失败的任务！")
        return

    print("\n" + "="*80)
    print("失败场景详细报告")
    print("="*80)

    print(f"\n总失败任务数: {failed_info['total_failed_tasks']}")
    print(f"涉及场景数: {failed_info['total_failed_scenes']}")

    # 1. 按场景分组
    print("\n" + "-"*80)
    print("【按场景分组】")
    print("-"*80)

    # 按失败数量排序
    sorted_scenes = sorted(
        failed_info['failed_by_scene'].items(),
        key=lambda x: len(x[1]),
        reverse=True
    )

    for i, (scene_name, failed_tasks) in enumerate(sorted_scenes, 1):
        print(f"\n{i}. 场景: {scene_name}")
        print(f"   失败任务数: {len(failed_tasks)}")

        # 显示每个失败任务的详细信息
        for j, task in enumerate(failed_tasks[:5], 1):  # 最多显示5个
            task_id = task.get('task_id', 'unknown')
            failure_reason = task.get('failure_reason', '未知')
            start = task.get('start', [0, 0])
            end = task.get('end', [0, 0])

            print(f"   任务{j}: {task_id}")
            print(f"     失败原因: {failure_reason}")
            print(f"     起点: ({start[0]:.2f}, {start[1]:.2f})")
            print(f"     终点: ({end[0]:.2f}, {end[1]:.2f})")

        if len(failed_tasks) > 5:
            print(f"   ... 还有 {len(failed_tasks) - 5} 个失败任务")

    # 2. 按失败原因分组
    print("\n" + "-"*80)
    print("【按失败原因分组】")
    print("-"*80)

    sorted_reasons = sorted(
        failed_info['failed_by_reason'].items(),
        key=lambda x: len(x[1]),
        reverse=True
    )

    for i, (reason, tasks) in enumerate(sorted_reasons, 1):
        print(f"\n{i}. {reason}")
        print(f"   任务数: {len(tasks)}")

        # 显示相关场景
        scenes = set()
        for task in tasks:
            task_id = task.get('task_id', '')
            if '_' in task_id:
                scene = '_'.join(task_id.split('_')[:-1])
                scenes.add(scene)

        print(f"   涉及场景数: {len(scenes)}")
        if len(scenes) <= 5:
            print(f"   场景列表: {', '.join(list(scenes))}")
        else:
            print(f"   部分场景: {', '.join(list(scenes)[:5])}...")

    # 3. 失败场景汇总列表
    print("\n" + "-"*80)
    print("【失败场景汇总列表】")
    print("-"*80)

    print(f"\n失败场景名称列表（共{failed_info['total_failed_scenes']}个）:")
    for i, scene_name in enumerate(sorted(failed_info['failed_by_scene'].keys()), 1):
        failed_count = len(failed_info['failed_by_scene'][scene_name])
        print(f"  {i:2d}. {scene_name} - {failed_count}个失败任务")

    print("\n" + "="*80)


def save_failed_scenes_report(failed_info: Dict, output_file: str):
    """
    保存失败场景报告到JSON文件
    """
    report = {
        'summary': {
            'total_failed_tasks': failed_info['total_failed_tasks'],
            'total_failed_scenes': failed_info['total_failed_scenes'],
        },
        'failed_by_scene': {
            scene: {
                'failed_count': len(tasks),
                'failed_tasks': tasks
            }
            for scene, tasks in failed_info['failed_by_scene'].items()
        },
        'failed_by_reason': {
            reason: {
                'count': len(tasks),
                'tasks': tasks
            }
            for reason, tasks in failed_info['failed_by_reason'].items()
        },
        'all_failed_tasks': failed_info['all_failed_tasks']
    }

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n失败场景报告已保存至: {output_file}")


def analyze_results(all_results: List[Dict]) -> Dict:
    """
    分析所有路径规划结果
    """
    if not all_results:
        print("警告: 没有找到任何路径规划结果")
        return {}

    analysis = {
        'total_paths': len(all_results),
        'successful_paths': 0,
        'failed_paths': 0,
        'distance_categories': {},
        'success_by_distance': {},
        'failed_by_distance': {},
        'all_distances': [],
        'successful_distances': [],
        'failed_distances': [],
        'waypoint_stats': {
            'total_waypoints': 0,
            'avg_waypoints_per_path': 0,
            'min_waypoints': float('inf'),
            'max_waypoints': 0
        },
        'distance_stats': {
            'total_distance': 0.0,
            'avg_distance': 0.0,
            'min_distance': float('inf'),
            'max_distance': 0.0,
            'median_distance': 0.0
        },
        'euclidean_distance_stats': {
            'total_euclidean': 0.0,
            'avg_euclidean': 0.0,
            'path_efficiency_ratio': 0.0  # 实际路径/欧氏距离的比率
        }
    }

    # 统计每个结果
    for result in all_results:
        is_success = result.get('is_success', False)
        total_distance = result.get('total_distance', 0.0)
        num_waypoints = result.get('num_waypoints', 0)
        start = result.get('start', [0, 0])
        end = result.get('end', [0, 0])

        # 计算欧氏距离（直线距离）用于对比
        euclidean_distance = calculate_euclidean_distance(start, end)

        # 成功/失败统计
        if is_success:
            analysis['successful_paths'] += 1
            analysis['successful_distances'].append(total_distance)
        else:
            analysis['failed_paths'] += 1
            analysis['failed_distances'].append(total_distance)

        # 距离分类 - 使用实际路径长度而不是欧式距离
        # 对于失败的任务，total_distance为0，需要特殊处理
        if total_distance > 0:
            distance_category = categorize_by_distance(total_distance)
        else:
            # 失败的任务使用欧式距离分类
            distance_category = categorize_by_distance(euclidean_distance)
        if distance_category not in analysis['distance_categories']:
            analysis['distance_categories'][distance_category] = {
                'count': 0,
                'success_count': 0,
                'failed_count': 0,
                'actual_path_distances': [],     # 实际路径长度
                'euclidean_distances': []          # 欧式距离
            }

        analysis['distance_categories'][distance_category]['count'] += 1

        # 保存实际路径长度和欧式距离
        analysis['distance_categories'][distance_category]['actual_path_distances'].append(total_distance)
        analysis['distance_categories'][distance_category]['euclidean_distances'].append(euclidean_distance)

        if is_success:
            analysis['distance_categories'][distance_category]['success_count'] += 1
        else:
            analysis['distance_categories'][distance_category]['failed_count'] += 1

        # 航点统计
        analysis['waypoint_stats']['total_waypoints'] += num_waypoints
        if num_waypoints < analysis['waypoint_stats']['min_waypoints']:
            analysis['waypoint_stats']['min_waypoints'] = num_waypoints
        if num_waypoints > analysis['waypoint_stats']['max_waypoints']:
            analysis['waypoint_stats']['max_waypoints'] = num_waypoints

        # 距离统计
        analysis['all_distances'].append(total_distance)
        analysis['distance_stats']['total_distance'] += total_distance

        if total_distance < analysis['distance_stats']['min_distance']:
            analysis['distance_stats']['min_distance'] = total_distance
        if total_distance > analysis['distance_stats']['max_distance']:
            analysis['distance_stats']['max_distance'] = total_distance

        # 欧氏距离统计
        euclidean_dist = calculate_euclidean_distance(start, end)
        analysis['euclidean_distance_stats']['total_euclidean'] += euclidean_dist

    # 计算平均值
    if len(all_distances := analysis['all_distances']) > 0:
        analysis['distance_stats']['avg_distance'] = (
            analysis['distance_stats']['total_distance'] / len(all_distances)
        )

        # 计算中位数
        sorted_distances = sorted(all_distances)
        n = len(sorted_distances)
        if n % 2 == 0:
            analysis['distance_stats']['median_distance'] = (
                sorted_distances[n//2 - 1] + sorted_distances[n//2]
            ) / 2
        else:
            analysis['distance_stats']['median_distance'] = sorted_distances[n//2]

        # 路径效率比率 (实际路径距离 / 欧氏距离)
        if analysis['euclidean_distance_stats']['total_euclidean'] > 0:
            analysis['euclidean_distance_stats']['avg_euclidean'] = (
                analysis['euclidean_distance_stats']['total_euclidean'] / n
            )
            analysis['euclidean_distance_stats']['path_efficiency_ratio'] = (
                analysis['distance_stats']['avg_distance'] /
                analysis['euclidean_distance_stats']['avg_euclidean']
            )

        # 平均航点数
        analysis['waypoint_stats']['avg_waypoints_per_path'] = (
            analysis['waypoint_stats']['total_waypoints'] / n
        )

        # 处理边界情况
        if analysis['waypoint_stats']['min_waypoints'] == float('inf'):
            analysis['waypoint_stats']['min_waypoints'] = 0
        if analysis['distance_stats']['min_distance'] == float('inf'):
            analysis['distance_stats']['min_distance'] = 0.0

    return analysis


def print_report(analysis: Dict):
    """
    打印统计报告
    """
    if not analysis:
        print("没有可用的统计数据")
        return

    print("\n" + "="*80)
    print("路径规划距离统计报告")
    print("="*80)

    # 1. 总体统计
    print("\n【总体统计】")
    print(f"  总路径数: {analysis['total_paths']}")
    print(f"  成功路径: {analysis['successful_paths']} "
          f"({analysis['successful_paths']*100/analysis['total_paths']:.1f}%)")
    print(f"  失败路径: {analysis['failed_paths']} "
          f"({analysis['failed_paths']*100/analysis['total_paths']:.1f}%)")

    # 2. 距离统计
    print("\n【距离统计】")
    print(f"  总距离: {analysis['distance_stats']['total_distance']:.2f} m")
    print(f"  平均距离: {analysis['distance_stats']['avg_distance']:.2f} m")
    print(f"  中位数距离: {analysis['distance_stats']['median_distance']:.2f} m")
    print(f"  最小距离: {analysis['distance_stats']['min_distance']:.2f} m")
    print(f"  最大距离: {analysis['distance_stats']['max_distance']:.2f} m")

    # 3. 欧氏距离对比
    print("\n【欧氏距离对比】")
    print(f"  平均欧氏距离: {analysis['euclidean_distance_stats']['avg_euclidean']:.2f} m")
    print(f"  路径效率比率: {analysis['euclidean_distance_stats']['path_efficiency_ratio']:.2f}x")
    print(f"    (实际路径距离 / 欧氏距离, 理想值为1.0)")

    # 4. 航点统计
    print("\n【航点统计】")
    print(f"  总航点数: {analysis['waypoint_stats']['total_waypoints']}")
    print(f"  平均每条路径航点数: {analysis['waypoint_stats']['avg_waypoints_per_path']:.1f}")
    print(f"  最少航点数: {analysis['waypoint_stats']['min_waypoints']}")
    print(f"  最多航点数: {analysis['waypoint_stats']['max_waypoints']}")

    # 5. 按距离分类统计（基于欧氏距离分类）
    print("\n【按距离分类统计（基于欧氏距离分类）】")
    print("  注意：距离分类基于起点到终点的直线距离，但显示的是实际路径长度")
    distance_order = [
        "极短 (< 1m)",
        "短距离 (1-2m)",
        "中等距离 (2-4m)",
        "中长距离 (4-6m)",
        "长距离 (6-8m)",
        "超长距离 (≥ 8m)"
    ]

    for category in distance_order:
        if category in analysis['distance_categories']:
            data = analysis['distance_categories'][category]

            # 计算平均实际路径长度（只统计成功的路径，失败的路径长度为0）
            actual_path_distances = data['actual_path_distances']
            successful_distances = [d for d in actual_path_distances if d > 0]
            avg_actual_dist = sum(successful_distances) / len(successful_distances) if successful_distances else 0

            # 计算平均欧氏距离（用于参考）
            euclidean_distances = data['euclidean_distances']
            avg_euclidean_dist = sum(euclidean_distances) / len(euclidean_distances) if euclidean_distances else 0

            print(f"\n  {category}:")
            print(f"    数量: {data['count']} ({data['count']*100/analysis['total_paths']:.1f}%)")
            print(f"    成功: {data['success_count']} ({data['success_count']*100/data['count']:.1f}%)")
            print(f"    失败: {data['failed_count']} ({data['failed_count']*100/data['count']:.1f}%)")
            print(f"    平均实际路径长度: {avg_actual_dist:.2f} m")
            print(f"    平均欧氏距离（参考）: {avg_euclidean_dist:.2f} m")

    # 6. 成功率与距离关系（基于欧氏距离分类）
    print("\n【成功率与距离关系（基于欧氏距离分类）】")
    for category in distance_order:
        if category in analysis['distance_categories']:
            data = analysis['distance_categories'][category]
            success_rate = data['success_count'] * 100 / data['count'] if data['count'] > 0 else 0
            print(f"  {category}: {success_rate:.1f}%")

    print("\n" + "="*80)


def save_detailed_report(analysis: Dict, output_file: str):
    """
    保存详细报告到JSON文件
    """
    report = {
        'summary': {
            'total_paths': analysis['total_paths'],
            'successful_paths': analysis['successful_paths'],
            'failed_paths': analysis['failed_paths'],
            'success_rate': analysis['successful_paths'] * 100 / analysis['total_paths']
            if analysis['total_paths'] > 0 else 0
        },
        'distance_statistics': analysis['distance_stats'],
        'euclidean_distance_comparison': analysis['euclidean_distance_stats'],
        'waypoint_statistics': analysis['waypoint_stats'],
        'categories': analysis['distance_categories']
    }

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n详细报告已保存至: {output_file}")


def main():
    # 默认路径
    base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
    # base_dir = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results/run_2026-01-06_22-31-48"
    output_file = "/home/pjlab/fbh/LabUtopia/path_distance_statistics_report_1_14.json"
    failed_scenes_file = "/home/pjlab/fbh/LabUtopia/failed_scenes_report_1_14.json"

    # 允许从命令行参数指定路径
    if len(sys.argv) > 1:
        base_dir = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    if len(sys.argv) > 3:
        failed_scenes_file = sys.argv[3]

    print("="*80)
    print("路径规划距离统计分析工具")
    print("="*80)
    print(f"\n扫描目录: {base_dir}")
    print(f"统计报告: {output_file}")
    print(f"失败场景报告: {failed_scenes_file}\n")

    # 加载所有结果
    print("正在加载结果文件...")
    all_results = load_all_results(base_dir)
    print(f"\n共加载 {len(all_results)} 条路径规划记录")

    # 分析结果
    print("\n正在分析数据...")
    analysis = analyze_results(all_results)

    # 提取失败场景信息
    print("\n正在提取失败场景...")
    failed_info = extract_failed_scenes(all_results)

    # 打印报告
    print_report(analysis)

    # 打印失败场景报告
    print_failed_scenes_report(failed_info)

    # 保存详细报告
    save_detailed_report(analysis, output_file)

    # 保存失败场景报告
    save_failed_scenes_report(failed_info, failed_scenes_file)


if __name__ == "__main__":
    main()
