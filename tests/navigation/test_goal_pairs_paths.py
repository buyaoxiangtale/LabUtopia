#!/usr/bin/env python3
"""
测试 level5_Navigation_smooth_12_23.yaml 中的 goal_pairs 路径信息

功能：
1. 加载任务配置文件中的 goal_pairs
2. 验证每对起终点的可行性
3. 计算并展示完整的路径信息
4. 保存路径信息到文件
"""

import sys
import yaml
import json
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))

from utils.path_planning_precompute import PathPrecomputer


def load_task_config(config_path: str) -> dict:
    """加载任务配置文件"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def test_goal_pairs_from_config(task_config_path: str):
    """测试配置文件中的 goal_pairs"""

    print("="*80)
    print("测试 goal_pairs 路径信息")
    print("="*80)

    # 1. 加载任务配置
    print(f"\n[1] 加载任务配置: {task_config_path}")
    task_config = load_task_config(task_config_path)

    # 2. 提取 navigation_config_path
    navigation_config_path = task_config['task']['navigation_config_path']
    print(f"    导航场景配置: {navigation_config_path}")

    # 3. 提取 goal_pairs
    goal_pairs = task_config['task'].get('goal_pairs', [])
    if not goal_pairs:
        print("    ⚠️  配置文件中没有 goal_pairs")
        return

    print(f"    找到 {len(goal_pairs)} 对起终点:")

    for i, pair in enumerate(goal_pairs):
        start = pair['start']
        end = pair['end']
        print(f"      [{i+1}] ({start[0]:.3f}, {start[1]:.3f}, θ={start[2]:.2f}) -> "
              f"({end[0]:.3f}, {end[1]:.3f}, θ={end[2]:.2f})")

    # 4. 初始化路径预计算器
    print(f"\n[2] 初始化路径预计算器...")
    precomputer = PathPrecomputer(navigation_config_path)

    # 5. 逐个测试每对起终点
    print(f"\n[3] 开始测试每对起终点...")
    print("="*80)

    valid_paths = []
    failed_pairs = []

    for i, pair in enumerate(goal_pairs):
        start = pair['start']
        end = pair['end']

        # 提取 x, y 坐标（忽略角度）
        start_xy = [start[0], start[1]]
        end_xy = [end[0], end[1]]

        print(f"\n第 {i+1} 对起终点:")
        print(f"  起点: ({start_xy[0]:.3f}, {start_xy[1]:.3f}), θ={start[2]:.2f}")
        print(f"  终点: ({end_xy[0]:.3f}, {end_xy[1]:.3f}), θ={end[2]:.2f}")
        print(f"  正在规划路径...")

        # 规划路径
        path_info = precomputer.plan_single_path(start_xy, end_xy)

        if path_info:
            print(f"  ✓ 规划成功！")
            print(f"    路径长度: {path_info['total_distance']:.3f} 米")
            print(f"    路径点数: {path_info['num_waypoints']}")

            # 计算预计时间
            total_time = sum(seg['estimated_time'] for seg in path_info['segments'])
            avg_velocity = path_info['total_distance'] / total_time if total_time > 0 else 0

            print(f"    预计时间: {total_time:.2f} 秒")
            print(f"    平均速度: {avg_velocity:.4f} m/s")

            # 显示前3段和最后3段信息
            print(f"    路径段示例:")
            for seg in path_info['segments'][:3]:
                print(f"      段{seg['segment_idx']}: {seg['distance']:.3f}m, "
                      f"方向{seg['direction_deg']:.1f}°, "
                      f"速度{seg['velocity_magnitude']:.4f}m/s")
            if len(path_info['segments']) > 3:
                print(f"      ... (共 {len(path_info['segments'])} 段)")

            # 保存路径信息
            path_info['goal_pair_idx'] = i
            path_info['start_theta'] = start[2]
            path_info['end_theta'] = end[2]
            valid_paths.append(path_info)
        else:
            print(f"  ✗ 规划失败 - 无法找到有效路径")
            failed_pairs.append({
                'idx': i,
                'start': start,
                'end': end
            })

    # 6. 汇总统计
    print("\n" + "="*80)
    print("测试结果汇总")
    print("="*80)
    print(f"总测试数: {len(goal_pairs)}")
    print(f"成功: {len(valid_paths)}")
    print(f"失败: {len(failed_pairs)}")

    if valid_paths:
        distances = [p['total_distance'] for p in valid_paths]
        waypoints_counts = [p['num_waypoints'] for p in valid_paths]
        total_times = [sum(seg['estimated_time'] for seg in p['segments']) for p in valid_paths]

        print(f"\n成功路径统计:")
        print(f"  总长度:")
        print(f"    平均: {sum(distances)/len(distances):.3f} 米")
        print(f"    最短: {min(distances):.3f} 米")
        print(f"    最长: {max(distances):.3f} 米")
        print(f"  路径点数:")
        print(f"    平均: {sum(waypoints_counts)/len(waypoints_counts):.1f}")
        print(f"    最少: {min(waypoints_counts)}")
        print(f"    最多: {max(waypoints_counts)}")
        print(f"  预计时间:")
        print(f"    总计: {sum(total_times):.2f} 秒 ({sum(total_times)/60:.2f} 分钟)")
        print(f"    平均: {sum(total_times)/len(total_times):.2f} 秒")

    if failed_pairs:
        print(f"\n失败的起终点对:")
        for pair in failed_pairs:
            print(f"  [{pair['idx']+1}] {pair['start']} -> {pair['end']}")

    # 7. 保存到文件
    if valid_paths:
        output_file = "outputs/level5_goal_pairs_paths.json"
        precomputer.save_paths_to_file(valid_paths, output_file)

        # 额外保存详细报告
        report_file = "outputs/level5_goal_pairs_report.txt"
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("="*80 + "\n")
            f.write("Goal Pairs 路径测试详细报告\n")
            f.write("="*80 + "\n\n")

            f.write(f"配置文件: {task_config_path}\n")
            f.write(f"导航场景配置: {navigation_config_path}\n\n")

            f.write(f"测试结果: {len(valid_paths)}/{len(goal_pairs)} 对成功\n\n")

            for i, path_info in enumerate(valid_paths):
                f.write("="*80 + "\n")
                f.write(f"路径 {i+1}\n")
                f.write("="*80 + "\n")
                f.write(f"起点: ({path_info['start'][0]:.3f}, {path_info['start'][1]:.3f}), θ={path_info['start_theta']:.2f}\n")
                f.write(f"终点: ({path_info['end'][0]:.3f}, {path_info['end'][1]:.3f}), θ={path_info['end_theta']:.2f}\n")
                f.write(f"路径长度: {path_info['total_distance']:.3f} 米\n")
                f.write(f"路径点数: {path_info['num_waypoints']}\n")

                total_time = sum(seg['estimated_time'] for seg in path_info['segments'])
                f.write(f"预计时间: {total_time:.2f} 秒\n\n")

                f.write("路径段详情:\n")
                for seg in path_info['segments']:
                    f.write(f"  段{seg['segment_idx']:3d}: {seg['distance']:6.3f}m, "
                           f"方向{seg['direction_deg']:6.1f}°, "
                           f"速度{seg['velocity_magnitude']:7.4f}m/s, "
                           f"时间{seg['estimated_time']:6.2f}s\n")
                f.write("\n")

        print(f"\n✓ 路径信息已保存到: {output_file}")
        print(f"✓ 详细报告已保存到: {report_file}")

    return valid_paths, failed_pairs


if __name__ == "__main__":
    # 测试配置文件
    task_config_path = "config/level5_Navigation_smooth_12_23.yaml"

    valid_paths, failed_pairs = test_goal_pairs_from_config(task_config_path)

    if valid_paths:
        print("\n" + "="*80)
        print("✓ 测试完成！所有路径信息已保存。")
        print("="*80)
    else:
        print("\n" + "="*80)
        print("⚠️  所有路径规划失败，请检查配置。")
        print("="*80)
