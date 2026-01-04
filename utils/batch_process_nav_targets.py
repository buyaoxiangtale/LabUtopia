#!/usr/bin/env python3
"""
批量处理导航目标点工具

功能：
1. 批量处理 nav_targets.json 文件（由 auto_batch_scene_nav.py 生成）
2. 分析导航目标点的统计信息
3. 计算连续目标点之间的距离
4. 生成目标点序列报告（CSV、JSON）
5. 可选：转换为 goal_pairs 格式

使用示例：
    # 处理所有 nav_targets.json 文件
    python utils/batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/

    # 生成统计报告
    python utils/batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/ --output report.csv

    # 转换为 goal_pairs 格式
    python utils/batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/ --convert-goal-pairs --format yaml
"""

import json
import math
import argparse
import csv
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
import statistics


def load_nav_targets_file(file_path: Path) -> Optional[Dict]:
    """
    加载导航目标点文件

    Args:
        file_path: JSON文件路径

    Returns:
        导航目标点数据，如果加载失败则返回 None
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 验证格式
        if 'nav_targets' not in data:
            print(f"⚠️  {file_path.name} 缺少 'nav_targets' 字段")
            return None

        return data

    except Exception as e:
        print(f"⚠️  无法加载 {file_path.name}: {e}")
        return None


def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """计算两点之间的欧几里得距离"""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def calculate_robot_theta(rotation_z_deg: float) -> float:
    """
    根据物体旋转角度计算机器人朝向角度（弧度）
    规则：robot_theta_deg = (object_rz_deg + 180) % 360
    """
    robot_theta_deg = (rotation_z_deg + 180) % 360
    return math.radians(robot_theta_deg)


def extract_nav_targets_info(nav_data: Dict) -> List[Dict[str, Any]]:
    """
    从导航目标点数据中提取关键信息

    Args:
        nav_data: 导航目标点数据

    Returns:
        目标点信息列表
    """
    config = nav_data.get('config', {})
    targets = nav_data.get('nav_targets', [])

    scene_name = config.get('scene_name', 'Unknown')

    results = []

    for i, target in enumerate(targets):
        step_number = target.get('step_number', i + 1)
        location = target.get('location', 'Unknown')
        matched_object = target.get('matched_object', '')
        target_x = target.get('target_x', 0)
        target_y = target.get('target_y', 0)
        rotation_z = target.get('rotation_z', 0)

        # 计算机器人朝向角度
        theta = calculate_robot_theta(rotation_z)

        results.append({
            'scene_name': scene_name,
            'step_number': step_number,
            'location': location,
            'matched_object': matched_object,
            'target_x': target_x,
            'target_y': target_y,
            'theta': theta,
            'theta_deg': math.degrees(theta),
            'rotation_z': rotation_z,
            'bbox_width': target.get('bbox_width', 0),
            'bbox_depth': target.get('bbox_depth', 0),
            'offset_dx': target.get('offset_dx', 0),
            'offset_dy': target.get('offset_dy', 0)
        })

    return results


def analyze_nav_sequences(all_targets: List[Dict]) -> List[Dict[str, Any]]:
    """
    分析导航序列，计算连续点之间的距离

    Args:
        all_targets: 所有目标点信息

    Returns:
        导航序列分析结果
    """
    # 按场景分组
    scenes = {}
    for target in all_targets:
        scene = target['scene_name']
        if scene not in scenes:
            scenes[scene] = []
        scenes[scene].append(target)

    results = []

    for scene_name, targets in scenes.items():
        # 按步骤编号排序
        targets.sort(key=lambda x: x['step_number'])

        # 计算连续点之间的距离
        for i in range(len(targets)):
            current = targets[i]

            result = {
                'scene_name': scene_name,
                'step_number': current['step_number'],
                'location': current['location'],
                'matched_object': current['matched_object'],
                'position': (current['target_x'], current['target_y']),
                'theta': current['theta'],
                'theta_deg': current['theta_deg']
            }

            # 计算到下一个点的距离（如果有）
            if i < len(targets) - 1:
                next_target = targets[i + 1]
                distance = calculate_distance(
                    (current['target_x'], current['target_y']),
                    (next_target['target_x'], next_target['target_y'])
                )
                result['distance_to_next'] = distance
                result['next_location'] = next_target['location']
            else:
                result['distance_to_next'] = 0.0
                result['next_location'] = 'END'

            results.append(result)

    return results


def print_summary(all_targets: List[Dict], sequences: List[Dict]):
    """打印统计摘要"""
    if not all_targets:
        print("❌ 没有处理结果")
        return

    print("=" * 80)
    print("导航目标点统计摘要")
    print("=" * 80)

    # 总览
    scenes = set(t['scene_name'] for t in all_targets)
    print(f"\n总场景数: {len(scenes)}")
    print(f"总目标点数: {len(all_targets)}")

    # 场景统计
    print(f"\n各场景目标点数:")
    scene_counts = {}
    for target in all_targets:
        scene = target['scene_name']
        scene_counts[scene] = scene_counts.get(scene, 0) + 1

    for scene, count in sorted(scene_counts.items()):
        print(f"  {scene}: {count} 个")

    # 位置统计
    locations = {}
    for target in all_targets:
        loc = target['location']
        locations[loc] = locations.get(loc, 0) + 1

    print(f"\n位置类型分布:")
    for loc, count in sorted(locations.items(), key=lambda x: x[1], reverse=True):
        print(f"  {loc}: {count} 次")

    # 距离统计
    distances = [s['distance_to_next'] for s in sequences if s['distance_to_next'] > 0]
    if distances:
        print(f"\n连续目标点距离 (米):")
        print(f"  总计: {sum(distances):.2f}")
        print(f"  平均: {statistics.mean(distances):.2f}")
        print(f"  中位数: {statistics.median(distances):.2f}")
        print(f"  最短: {min(distances):.2f}")
        print(f"  最长: {max(distances):.2f}")
        print(f"  标准差: {statistics.stdev(distances) if len(distances) > 1 else 0:.2f}")

    # 坐标范围
    x_coords = [t['target_x'] for t in all_targets]
    y_coords = [t['target_y'] for t in all_targets]

    print(f"\n坐标范围:")
    print(f"  X: [{min(x_coords):.2f}, {max(x_coords):.2f}]")
    print(f"  Y: [{min(y_coords):.2f}, {max(y_coords):.2f}]")

    print("=" * 80)


def save_to_csv(sequences: List[Dict], output_path: Path):
    """保存结果到CSV文件"""
    if not sequences:
        return

    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)

        # 写入表头
        writer.writerow([
            'Scene Name',
            'Step',
            'Location',
            'Matched Object',
            'Target X',
            'Target Y',
            'Theta (deg)',
            'Distance to Next',
            'Next Location'
        ])

        # 写入数据
        for s in sequences:
            writer.writerow([
                s['scene_name'],
                s['step_number'],
                s['location'],
                s['matched_object'],
                f"{s['position'][0]:.3f}",
                f"{s['position'][1]:.3f}",
                f"{s['theta_deg']:.1f}",
                f"{s['distance_to_next']:.3f}",
                s['next_location']
            ])

    print(f"\n✓ CSV报告已保存到: {output_path}")


def save_to_json(all_targets: List[Dict], sequences: List[Dict], output_path: Path):
    """保存结果到JSON文件"""
    if not all_targets:
        return

    # 计算汇总统计
    scenes = set(t['scene_name'] for t in all_targets)
    distances = [s['distance_to_next'] for s in sequences if s['distance_to_next'] > 0]

    summary = {
        'total_scenes': len(scenes),
        'total_targets': len(all_targets),
        'total_distance': sum(distances) if distances else 0,
        'avg_distance': statistics.mean(distances) if distances else 0,
        'statistics': {
            'locations': {},
            'scenes': {}
        }
    }

    # 统计位置
    for target in all_targets:
        loc = target['location']
        summary['statistics']['locations'][loc] = \
            summary['statistics']['locations'].get(loc, 0) + 1

    # 统计场景
    for target in all_targets:
        scene = target['scene_name']
        summary['statistics']['scenes'][scene] = \
            summary['statistics']['scenes'].get(scene, 0) + 1

    output_data = {
        'summary': summary,
        'sequences': sequences,
        'all_targets': all_targets
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"✓ JSON报告已保存到: {output_path}")


def convert_to_goal_pairs(all_targets: List[Dict]) -> Tuple[List[Dict], Dict]:
    """
    将导航目标点转换为 goal_pairs 格式

    Args:
        all_targets: 所有目标点信息

    Returns:
        (goal_pairs, metadata)
    """
    # 按场景分组
    scenes = {}
    for target in all_targets:
        scene = target['scene_name']
        if scene not in scenes:
            scenes[scene] = []
        scenes[scene].append(target)

    all_goal_pairs = []
    metadata = {
        'total_scenes': len(scenes),
        'scenes': {}
    }

    for scene_name, targets in scenes.items():
        # 按步骤编号排序
        targets.sort(key=lambda x: x['step_number'])

        # 生成 goal_pairs
        goal_pairs = []
        for i in range(len(targets) - 1):
            current = targets[i]
            next_target = targets[i + 1]

            goal_pairs.append({
                'start': [current['target_x'], current['target_y'], current['theta']],
                'end': [next_target['target_x'], next_target['target_y'], next_target['theta']]
            })

        all_goal_pairs.extend(goal_pairs)

        metadata['scenes'][scene_name] = {
            'num_targets': len(targets),
            'num_goal_pairs': len(goal_pairs),
            'locations': [t['location'] for t in targets]
        }

    return all_goal_pairs, metadata


def save_goal_pairs(goal_pairs: List[Dict], metadata: Dict, output_path: Path, output_format: str):
    """保存 goal_pairs 到文件"""
    output_data = {
        'goal_pairs': goal_pairs,
        'metadata': metadata
    }

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if output_format == 'yaml':
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(output_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    else:  # json
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"✓ Goal pairs 已保存到: {output_path} (格式: {output_format})")


def print_top_n_sequences(sequences: List[Dict], n: int = 5, metric: str = 'distance'):
    """打印前N条导航序列"""
    if not sequences:
        return

    print(f"\n🏆 前 {n} 条导航序列（按 {metric} 排序）:")
    print("-" * 80)

    # 过滤掉距离为0的
    filtered = [s for s in sequences if s['distance_to_next'] > 0]

    # 按指定指标排序
    key_map = {
        'distance': 'distance_to_next',
        'step': 'step_number'
    }

    key = key_map.get(metric, metric)
    reverse = (key == 'distance_to_next')

    sorted_results = sorted(filtered, key=lambda x: x.get(key, 0), reverse=reverse)

    for i, s in enumerate(sorted_results[:n], 1):
        print(f"\n{i}. {s['scene_name']} - 步骤 {s['step_number']}")
        print(f"   位置: {s['location']} → {s['next_location']}")
        print(f"   坐标: ({s['position'][0]:.2f}, {s['position'][1]:.2f}) → 距离: {s['distance_to_next']:.2f}m")


def process_directory(
    input_dir: Path,
    pattern: str = "*_nav_targets.json"
) -> Tuple[List[Dict], List[Dict]]:
    """
    处理目录中的所有导航目标点文件

    Args:
        input_dir: 输入目录
        pattern: 文件匹配模式

    Returns:
        (all_targets, sequences)
    """
    results = []

    # 查找匹配的文件
    files = list(input_dir.glob(pattern))

    if not files:
        print(f"❌ 在 {input_dir} 中没有找到匹配 '{pattern}' 的文件")
        return [], []

    print(f"📁 在 {input_dir} 中找到 {len(files)} 个文件\n")

    # 处理每个文件
    for file_path in sorted(files):
        nav_data = load_nav_targets_file(file_path)

        if nav_data is None:
            continue

        targets = extract_nav_targets_info(nav_data)
        results.extend(targets)

        scene_name = nav_data.get('config', {}).get('scene_name', 'Unknown')
        print(f"✓ {scene_name}: {len(targets)} 个导航目标点")

    # 分析序列
    sequences = analyze_nav_sequences(results)

    return results, sequences


def main():
    parser = argparse.ArgumentParser(
        description="批量处理导航目标点工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 处理所有 nav_targets.json 文件
  python batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/

  # 生成 CSV 报告
  python batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/ --output report.csv

  # 转换为 goal_pairs 格式 (JSON)
  python batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/ --convert-goal-pairs

  # 转换为 goal_pairs 格式 (YAML)
  python batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/ --convert-goal-pairs --format yaml --output goal_pairs.yaml

  # 显示前10条最长序列
  python batch_process_nav_targets.py --input outputs/auto_batch_nav_targets_test/ --top 10
        """
    )

    parser.add_argument(
        '--input',
        type=str,
        required=True,
        help='输入目录路径'
    )
    parser.add_argument(
        '--pattern',
        type=str,
        default='*_nav_targets.json',
        help='文件匹配模式（默认: *_nav_targets.json）'
    )
    parser.add_argument(
        '--output',
        type=str,
        help='输出文件路径（支持 .csv 或 .json）'
    )
    parser.add_argument(
        '--top',
        type=int,
        default=5,
        help='显示前N条序列（默认: 5）'
    )
    parser.add_argument(
        '--summary-only',
        action='store_true',
        help='只显示统计摘要，不显示详细列表'
    )
    parser.add_argument(
        '--convert-goal-pairs',
        action='store_true',
        help='转换为 goal_pairs 格式'
    )
    parser.add_argument(
        '--format',
        type=str,
        choices=['json', 'yaml'],
        default='json',
        help='goal_pairs 输出格式（默认: json）'
    )

    args = parser.parse_args()

    # 检查输入目录
    input_dir = Path(args.input)
    if not input_dir.exists():
        print(f"❌ 错误: 目录不存在 {input_dir}")
        return

    if not input_dir.is_dir():
        print(f"❌ 错误: {input_dir} 不是一个目录")
        return

    print("=" * 80)
    print("批量导航目标点处理工具")
    print("=" * 80)
    print(f"\n输入目录: {input_dir}")
    print(f"文件模式: {args.pattern}")

    # 处理文件
    all_targets, sequences = process_directory(input_dir, args.pattern)

    if not all_targets:
        print("\n❌ 没有找到可处理的导航目标点文件")
        return

    # 打印摘要
    print_summary(all_targets, sequences)

    # 打印前N条
    if not args.summary_only:
        print_top_n_sequences(sequences, args.top, 'distance')

    # 保存结果
    if args.output and not args.convert_goal_pairs:
        output_path = Path(args.output)

        if output_path.suffix == '.csv':
            save_to_csv(sequences, output_path)
        elif output_path.suffix == '.json':
            save_to_json(all_targets, sequences, output_path)
        else:
            # 默认使用JSON
            save_to_json(all_targets, sequences, output_path.with_suffix('.json'))

    # 转换为 goal_pairs
    if args.convert_goal_pairs:
        goal_pairs, metadata = convert_to_goal_pairs(all_targets)

        if args.output:
            output_path = Path(args.output)
        else:
            output_path = input_dir / "goal_pairs.json"

        save_goal_pairs(goal_pairs, metadata, output_path, args.format)

        print(f"\n✓ 转换完成: 共生成 {len(goal_pairs)} 个 goal_pairs")

    print("\n✓ 处理完成!")


if __name__ == "__main__":
    main()
