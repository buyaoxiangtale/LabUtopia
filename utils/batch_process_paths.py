#!/usr/bin/env python3
"""
批量路径处理工具

功能：
1. 批量处理同一文件夹下的多个路径文件（JSON格式）
2. 提取路径信息并生成统计报告
3. 支持过滤和排序
4. 生成汇总数据（CSV、JSON）

使用示例：
    # 处理 outputs/ 文件夹下的所有路径文件
    python utils/batch_process_paths.py --input outputs/

    # 只处理匹配模式的文件
    python utils/batch_process_paths.py --input outputs/ --pattern "*_goal_*.json"

    # 按路径长度排序
    python utils/batch_process_paths.py --input outputs/ --sort-by distance

    # 输出CSV报告
    python utils/batch_process_paths.py --input outputs/ --output report.csv
"""

import json
import argparse
import csv
from pathlib import Path
from typing import List, Dict, Any, Optional
import statistics


def load_path_file(file_path: Path) -> Optional[List[Dict]]:
    """
    加载路径文件

    Args:
        file_path: JSON文件路径

    Returns:
        路径列表，如果加载失败则返回 None
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 确保返回列表
        if isinstance(data, list):
            return data
        elif isinstance(data, dict):
            return [data]
        else:
            return None

    except Exception as e:
        print(f"⚠️  无法加载 {file_path.name}: {e}")
        return None


def extract_path_info(path_data: Dict) -> Dict[str, Any]:
    """
    从单条路径数据中提取关键信息

    Args:
        path_data: 路径数据字典

    Returns:
        包含路径信息的字典
    """
    # 基本信息
    start = path_data.get('start', [0, 0])
    end = path_data.get('end', [0, 0])
    total_distance = path_data.get('total_distance', 0)
    num_waypoints = path_data.get('num_waypoints', 0)

    # 计算直线距离
    straight_distance = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5

    # 路径效率（直线距离 / 实际距离）
    efficiency = straight_distance / total_distance if total_distance > 0 else 0

    # 路径段信息
    segments = path_data.get('segments', [])
    num_segments = len(segments)

    # 计算总时间
    total_time = 0.0
    if segments:
        total_time = sum(seg.get('estimated_time', 0) for seg in segments)

    # 计算平均速度
    avg_velocity = total_distance / total_time if total_time > 0 else 0

    # 提取方向统计（如果有segments）
    directions = []
    if segments:
        for seg in segments[:10]:  # 只统计前10段
            direction_deg = seg.get('direction_deg', 0)
            directions.append(direction_deg)

    return {
        'start': start,
        'end': end,
        'total_distance': total_distance,
        'straight_distance': straight_distance,
        'efficiency': efficiency,
        'num_waypoints': num_waypoints,
        'num_segments': num_segments,
        'total_time': total_time,
        'avg_velocity': avg_velocity,
        'sample_directions': directions[:5] if directions else []
    }


def process_directory(
    input_dir: Path,
    pattern: str = "*.json",
    sort_by: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    处理目录中的所有路径文件

    Args:
        input_dir: 输入目录
        pattern: 文件匹配模式
        sort_by: 排序字段（'distance', 'time', 'waypoints' 等）

    Returns:
        处理结果列表
    """
    results = []

    # 查找匹配的文件
    files = list(input_dir.glob(pattern))

    if not files:
        print(f"❌ 在 {input_dir} 中没有找到匹配 '{pattern}' 的文件")
        return results

    print(f"📁 在 {input_dir} 中找到 {len(files)} 个文件\n")

    # 处理每个文件
    for file_path in sorted(files):
        paths = load_path_file(file_path)

        if paths is None:
            continue

        # 处理文件中的每条路径
        for i, path_data in enumerate(paths):
            info = extract_path_info(path_data)

            # 添加元数据
            info['source_file'] = file_path.name
            info['path_index'] = i

            results.append(info)

            print(f"✓ {file_path.name} [{i}] - "
                  f"长度: {info['total_distance']:.2f}m, "
                  f"点数: {info['num_waypoints']}, "
                  f"时间: {info['total_time']:.0f}s")

    # 排序
    if sort_by:
        sort_key_map = {
            'distance': 'total_distance',
            'time': 'total_time',
            'waypoints': 'num_waypoints',
            'efficiency': 'efficiency'
        }

        key = sort_key_map.get(sort_by, sort_by)

        if key == 'total_distance' or key == 'total_time' or key == 'num_waypoints':
            results.sort(key=lambda x: x[key], reverse=True)
        elif key == 'efficiency':
            results.sort(key=lambda x: x[key], reverse=False)
        else:
            try:
                results.sort(key=lambda x: x.get(key, 0))
            except:
                pass

        print(f"\n📊 已按 '{sort_by}' 排序\n")

    return results


def print_summary(results: List[Dict[str, Any]]):
    """打印统计摘要"""
    if not results:
        print("❌ 没有处理结果")
        return

    print("=" * 80)
    print("统计摘要")
    print("=" * 80)

    # 总览
    print(f"\n总路径数: {len(results)}")

    # 距离统计
    distances = [r['total_distance'] for r in results]
    print(f"\n路径长度 (米):")
    print(f"  总计: {sum(distances):.2f}")
    print(f"  平均: {statistics.mean(distances):.2f}")
    print(f"  中位数: {statistics.median(distances):.2f}")
    print(f"  最短: {min(distances):.2f}")
    print(f"  最长: {max(distances):.2f}")
    print(f"  标准差: {statistics.stdev(distances) if len(distances) > 1 else 0:.2f}")

    # 时间统计
    times = [r['total_time'] for r in results]
    print(f"\n预计时间 (秒):")
    print(f"  总计: {sum(times):.0f} ({sum(times)/60:.1f} 分钟)")
    print(f"  平均: {statistics.mean(times):.0f}")
    print(f"  最短: {min(times):.0f}")
    print(f"  最长: {max(times):.0f}")

    # 效率统计
    efficiencies = [r['efficiency'] for r in results]
    print(f"\n路径效率 (直线/实际):")
    print(f"  平均: {statistics.mean(efficiencies):.3f}")
    print(f"  最高: {max(efficiencies):.3f}")
    print(f"  最低: {min(efficiencies):.3f}")

    # 路径点统计
    waypoints = [r['num_waypoints'] for r in results]
    print(f"\n路径点数:")
    print(f"  平均: {statistics.mean(waypoints):.0f}")
    print(f"  最多: {max(waypoints)}")
    print(f"  最少: {min(waypoints)}")

    print("=" * 80)


def save_to_csv(results: List[Dict[str, Any]], output_path: Path):
    """保存结果到CSV文件"""
    if not results:
        return

    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)

        # 写入表头
        writer.writerow([
            'Source File',
            'Path Index',
            'Start (x,y)',
            'End (x,y)',
            'Total Distance (m)',
            'Straight Distance (m)',
            'Efficiency',
            'Waypoints',
            'Segments',
            'Total Time (s)',
            'Avg Velocity (m/s)'
        ])

        # 写入数据
        for r in results:
            writer.writerow([
                r['source_file'],
                r['path_index'],
                f"({r['start'][0]:.2f}, {r['start'][1]:.2f})",
                f"({r['end'][0]:.2f}, {r['end'][1]:.2f})",
                f"{r['total_distance']:.3f}",
                f"{r['straight_distance']:.3f}",
                f"{r['efficiency']:.3f}",
                r['num_waypoints'],
                r['num_segments'],
                f"{r['total_time']:.1f}",
                f"{r['avg_velocity']:.4f}"
            ])

    print(f"\n✓ CSV报告已保存到: {output_path}")


def save_to_json(results: List[Dict[str, Any]], output_path: Path):
    """保存结果到JSON文件"""
    if not results:
        return

    # 计算汇总统计
    summary = {
        'total_paths': len(results),
        'statistics': {
            'total_distance': sum(r['total_distance'] for r in results),
            'total_time': sum(r['total_time'] for r in results),
            'avg_distance': statistics.mean([r['total_distance'] for r in results]),
            'avg_time': statistics.mean([r['total_time'] for r in results]),
            'avg_efficiency': statistics.mean([r['efficiency'] for r in results])
        }
    }

    output_data = {
        'summary': summary,
        'paths': results
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"✓ JSON报告已保存到: {output_path}")


def print_top_n(results: List[Dict[str, Any]], n: int = 5, metric: str = 'total_distance'):
    """打印前N条路径"""
    if not results:
        return

    print(f"\n🏆 前 {n} 条路径（按 {metric} 排序）:")
    print("-" * 80)

    # 映射用户友好的名称到实际键名
    metric_key_map = {
        'distance': 'total_distance',
        'time': 'total_time',
        'waypoints': 'num_waypoints',
        'efficiency': 'efficiency'
    }

    # 获取实际的键名
    key = metric_key_map.get(metric, metric)

    # 按指定指标排序
    sorted_results = sorted(results, key=lambda x: x.get(key, 0), reverse=True)

    for i, r in enumerate(sorted_results[:n], 1):
        print(f"\n{i}. {r['source_file']} [{r['path_index']}]")
        print(f"   起点: ({r['start'][0]:.2f}, {r['start'][1]:.2f})")
        print(f"   终点: ({r['end'][0]:.2f}, {r['end'][1]:.2f})")
        print(f"   长度: {r['total_distance']:.2f}m, 时间: {r['total_time']:.0f}s, "
              f"点数: {r['num_waypoints']}")


# ==================== 主程序 ====================

def main():
    parser = argparse.ArgumentParser(
        description="批量路径处理工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 处理 outputs/ 文件夹下的所有 JSON 文件
  python batch_process_paths.py --input outputs/

  # 只处理包含 "goal" 的文件
  python batch_process_paths.py --input outputs/ --pattern "*goal*.json"

  # 按路径长度排序
  python batch_process_paths.py --input outputs/ --sort-by distance

  # 生成 CSV 报告
  python batch_process_paths.py --input outputs/ --output report.csv

  # 显示前10条最长路径
  python batch_process_paths.py --input outputs/ --top 10 --sort-by distance
        """
    )

    parser.add_argument(
        '--input',
        type=str,
        default='outputs/',
        help='输入目录路径（默认: outputs/）'
    )
    parser.add_argument(
        '--pattern',
        type=str,
        default='*.json',
        help='文件匹配模式（默认: *.json）'
    )
    parser.add_argument(
        '--sort-by',
        type=str,
        choices=['distance', 'time', 'waypoints', 'efficiency'],
        help='排序字段'
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
        help='显示前N条路径（默认: 5）'
    )
    parser.add_argument(
        '--summary-only',
        action='store_true',
        help='只显示统计摘要，不显示详细列表'
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
    print("批量路径处理工具")
    print("=" * 80)
    print(f"\n输入目录: {input_dir}")
    print(f"文件模式: {args.pattern}")

    # 处理文件
    results = process_directory(
        input_dir,
        args.pattern,
        args.sort_by
    )

    if not results:
        print("\n❌ 没有找到可处理的路径文件")
        return

    # 打印摘要
    print_summary(results)

    # 打印前N条
    if not args.summary_only:
        metric = args.sort_by if args.sort_by else 'distance'
        print_top_n(results, args.top, metric)

    # 保存结果
    if args.output:
        output_path = Path(args.output)

        if output_path.suffix == '.csv':
            save_to_csv(results, output_path)
        elif output_path.suffix == '.json':
            save_to_json(results, output_path)
        else:
            # 默认使用JSON
            save_to_json(results, output_path.with_suffix('.json'))

    print("\n✓ 处理完成!")


if __name__ == "__main__":
    main()
