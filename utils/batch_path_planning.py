#!/usr/bin/env python3
"""
批量路径规划工具：支持多组起终点对和对应的占用地图

功能特点：
1. 支持批量处理多组起终点对
2. 每组起终点对可配置独立的占用地图
3. 自动处理失败情况并提供详细报告
4. 支持并行处理提升效率
5. 生成统计报告和可视化结果
"""

import numpy as np
import yaml
import json
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
from concurrent.futures import ProcessPoolExecutor, as_completed
import matplotlib.pyplot as plt
from utils.a_star import (
    plan_navigation_path,
    load_grid,
    real_to_grid,
    grid_to_real,
    calculate_path_distance,
    save_path_image
)


class BatchPathPlanner:
    """批量路径规划器"""

    def __init__(self, config_file: str = None):
        """
        初始化批量规划器

        Args:
            config_file: 批量任务配置文件路径（JSON/YAML）
        """
        self.tasks = []
        self.results = []

        if config_file:
            self.load_tasks_from_file(config_file)

    def load_tasks_from_file(self, config_file: str):
        """
        从配置文件加载批量任务

        支持的配置格式（JSON 示例）:
        {
            "tasks": [
                {
                    "task_id": "task_001",
                    "start": [1.0, 1.0],
                    "end": [8.0, 8.0],
                    "barrier_image_path": "path/to/barrier1.png",
                    "x_bounds": [0.0, 10.0],
                    "y_bounds": [0.0, 10.0],
                    "offset_radius": 0.3
                },
                ...
            ]
        }
        """
        config_path = Path(config_file)

        if config_path.suffix in ['.yaml', '.yml']:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
        else:  # JSON
            with open(config_file, 'r') as f:
                config = json.load(f)

        self.tasks = config.get('tasks', [])
        print(f"✓ 已加载 {len(self.tasks)} 个路径规划任务")

    def add_task(
        self,
        task_id: str,
        start: List[float],
        end: List[float],
        barrier_image_path: str,
        x_bounds: List[float],
        y_bounds: List[float],
        offset_radius: float = 0.3
    ):
        """
        添加单个规划任务

        Args:
            task_id: 任务唯一标识符
            start: 起点坐标 [x, y]
            end: 终点坐标 [x, y]
            barrier_image_path: 占用地图图像路径
            x_bounds: x轴边界 [min, max]
            y_bounds: y轴边界 [min, max]
            offset_radius: 障碍物膨胀半径（米）
        """
        task = {
            "task_id": task_id,
            "start": start,
            "end": end,
            "barrier_image_path": barrier_image_path,
            "x_bounds": x_bounds,
            "y_bounds": y_bounds,
            "offset_radius": offset_radius
        }
        self.tasks.append(task)

    def add_tasks_from_yaml_assets(self, yaml_path: str, goal_pairs: List[Tuple[List[float], List[float]]]):
        """
        从 YAML 场景配置文件批量添加任务（所有任务共用同一个占用地图）

        Args:
            yaml_path: 导航场景配置文件路径（如 navigation_assets_fbh.yaml）
            goal_pairs: 起终点对列表，如 [([1.0, 1.0], [8.0, 8.0]), ...]
        """
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
            nav_scene = config['assets'][0]

        base_task_id = Path(yaml_path).stem

        for i, (start, end) in enumerate(goal_pairs):
            task_id = f"{base_task_id}_pair_{i:03d}"
            self.add_task(
                task_id=task_id,
                start=start,
                end=end,
                barrier_image_path=nav_scene['barrier_image_path'],
                x_bounds=nav_scene['x_bounds'],
                y_bounds=nav_scene['y_bounds'],
                offset_radius=nav_scene.get('offset_radius', 0.3)
            )

        print(f"✓ 已添加 {len(goal_pairs)} 个任务（共享占用地图: {nav_scene['barrier_image_path']}）")

    def plan_single_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        """
        规划单个任务

        Args:
            task: 任务配置字典

        Returns:
            包含规划结果的字典
        """
        task_id = task['task_id']

        # 构建 task_info（与 plan_navigation_path 兼容的格式）
        task_info = {
            'asset': {
                'barrier_image_path': task['barrier_image_path'],
                'x_bounds': task['x_bounds'],
                'y_bounds': task['y_bounds'],
                'offset_radius': task['offset_radius']
            },
            'start': task['start'],
            'end': task['end']
        }

        # 执行路径规划
        path_result = plan_navigation_path(task_info)

        if path_result is None:
            return {
                'task_id': task_id,
                'success': False,
                'error': 'Path planning failed - no valid path found'
            }

        real_path, path_grid, total_distance = path_result

        # 添加方向信息（生成 waypoints）
        waypoints = []
        for i in range(len(real_path)):
            x, y, _ = real_path[i]
            if i < len(real_path) - 1:
                next_x, next_y, _ = real_path[i + 1]
                theta = np.arctan2(next_y - y, next_x - x)
            else:
                theta = waypoints[-1][2] if waypoints else 0.0
            waypoints.append([x, y, theta])

        # 计算路径段详细信息
        segments = self._calculate_segment_info(waypoints)

        return {
            'task_id': task_id,
            'success': True,
            'start': task['start'],
            'end': task['end'],
            'barrier_image_path': task['barrier_image_path'],
            'waypoints': waypoints,
            'total_distance': total_distance,
            'num_waypoints': len(waypoints),
            'segments': segments,
            'path_grid': path_grid  # 用于可视化
        }

    def _calculate_segment_info(self, waypoints: List[List[float]]) -> List[Dict[str, Any]]:
        """计算路径段的详细信息"""
        segments = []
        max_linear_speed = 0.02

        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            distance = np.sqrt(dx**2 + dy**2)
            direction = np.arctan2(dy, dx)
            direction_deg = np.degrees(direction)

            # 速度大小（与控制器逻辑一致）
            velocity_magnitude = min(distance * 0.2, max_linear_speed)

            segments.append({
                "segment_idx": i,
                "from": [p1[0], p1[1]],
                "to": [p2[0], p2[1]],
                "distance": distance,
                "direction": direction,
                "direction_deg": direction_deg,
                "velocity_magnitude": velocity_magnitude,
                "estimated_time": distance / velocity_magnitude if velocity_magnitude > 0 else 0
            })

        return segments

    def run_batch(
        self,
        parallel: bool = False,
        max_workers: int = 4,
        verbose: bool = True
    ) -> List[Dict[str, Any]]:
        """
        批量执行路径规划

        Args:
            parallel: 是否使用并行处理
            max_workers: 并行工作进程数
            verbose: 是否显示详细输出

        Returns:
            规划结果列表
        """
        print(f"\n{'='*70}")
        print(f"开始批量路径规划")
        print(f"任务数量: {len(self.tasks)}")
        print(f"并行模式: {'启用' if parallel else '禁用'}")
        print(f"{'='*70}\n")

        self.results = []

        if parallel:
            # 并行处理
            with ProcessPoolExecutor(max_workers=max_workers) as executor:
                future_to_task = {
                    executor.submit(self.plan_single_task, task): task
                    for task in self.tasks
                }

                for future in as_completed(future_to_task):
                    task = future_to_task[future]
                    try:
                        result = future.result()
                        self.results.append(result)

                        if verbose:
                            status = "✓" if result['success'] else "✗"
                            print(f"{status} {result['task_id']}: ", end="")
                            if result['success']:
                                print(f"成功 - 长度: {result['total_distance']:.2f}m")
                            else:
                                print(f"失败 - {result['error']}")

                    except Exception as e:
                        print(f"✗ {task['task_id']}: 异常 - {str(e)}")
                        self.results.append({
                            'task_id': task['task_id'],
                            'success': False,
                            'error': f'Exception: {str(e)}'
                        })
        else:
            # 串行处理
            for i, task in enumerate(self.tasks, 1):
                result = self.plan_single_task(task)
                self.results.append(result)

                if verbose:
                    status = "✓" if result['success'] else "✗"
                    print(f"[{i}/{len(self.tasks)}] {status} {result['task_id']}: ", end="")
                    if result['success']:
                        print(f"成功 - 长度: {result['total_distance']:.2f}m, "
                              f"路径点: {result['num_waypoints']}")
                    else:
                        print(f"失败 - {result['error']}")

        # 按原始任务顺序排序结果
        task_id_to_index = {task['task_id']: i for i, task in enumerate(self.tasks)}
        self.results.sort(key=lambda r: task_id_to_index.get(r['task_id'], float('inf')))

        return self.results

    def save_results(self, output_dir: str = "outputs/batch_planning"):
        """
        保存规划结果

        Args:
            output_dir: 输出目录路径
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # 1. 保存完整结果 JSON
        results_file = output_path / "batch_results.json"
        serializable_results = []
        for result in self.results:
            serializable_result = result.copy()
            # 移除不可序列化的 path_grid
            if 'path_grid' in serializable_result:
                del serializable_result['path_grid']
            serializable_results.append(serializable_result)

        with open(results_file, 'w') as f:
            json.dump(serializable_results, f, indent=2)
        print(f"✓ 完整结果已保存到: {results_file}")

        # 2. 保存成功和失败的 task_id 列表
        successful_tasks = [r['task_id'] for r in self.results if r['success']]
        failed_tasks = [r['task_id'] for r in self.results if not r['success']]

        with open(output_path / "successful_tasks.txt", 'w') as f:
            f.write('\n'.join(successful_tasks))

        with open(output_path / "failed_tasks.txt", 'w') as f:
            f.write('\n'.join(failed_tasks))

        print(f"✓ 任务列表已保存:")
        print(f"  - 成功: {len(successful_tasks)} 个 ({output_path / 'successful_tasks.txt'})")
        print(f"  - 失败: {len(failed_tasks)} 个 ({output_path / 'failed_tasks.txt'})")

    def generate_summary_report(self, output_dir: str = "outputs/batch_planning"):
        """
        生成统计报告和可视化

        Args:
            output_dir: 输出目录路径
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # 统计信息
        successful_results = [r for r in self.results if r['success']]
        failed_results = [r for r in self.results if not r['success']]

        print(f"\n{'='*70}")
        print(f"批量规划统计报告")
        print(f"{'='*70}")
        print(f"总任务数: {len(self.results)}")
        print(f"成功: {len(successful_results)} ({len(successful_results)/len(self.results)*100:.1f}%)")
        print(f"失败: {len(failed_results)} ({len(failed_results)/len(self.results)*100:.1f}%)")

        if successful_results:
            distances = [r['total_distance'] for r in successful_results]
            num_waypoints = [r['num_waypoints'] for r in successful_results]

            print(f"\n路径长度统计:")
            print(f"  平均: {np.mean(distances):.2f} 米")
            print(f"  最短: {np.min(distances):.2f} 米")
            print(f"  最长: {np.max(distances):.2f} 米")
            print(f"  标准差: {np.std(distances):.2f} 米")

            print(f"\n路径点数量统计:")
            print(f"  平均: {np.mean(num_waypoints):.1f}")
            print(f"  最少: {np.min(num_waypoints)}")
            print(f"  最多: {np.max(num_waypoints)}")

            # 可视化统计图
            self._plot_statistics(distances, num_waypoints, output_path)

        if failed_results:
            print(f"\n失败任务:")
            for r in failed_results[:10]:  # 只显示前10个
                print(f"  - {r['task_id']}: {r['error']}")
            if len(failed_results) > 10:
                print(f"  ... 还有 {len(failed_results) - 10} 个失败任务")

        print(f"{'='*70}\n")

        # 保存报告到文件
        report_file = output_path / "summary_report.txt"
        with open(report_file, 'w') as f:
            f.write(f"批量路径规划统计报告\n")
            f.write(f"{'='*70}\n")
            f.write(f"总任务数: {len(self.results)}\n")
            f.write(f"成功: {len(successful_results)} ({len(successful_results)/len(self.results)*100:.1f}%)\n")
            f.write(f"失败: {len(failed_results)} ({len(failed_results)/len(self.results)*100:.1f}%)\n")

            if successful_results:
                f.write(f"\n路径长度统计:\n")
                f.write(f"  平均: {np.mean(distances):.2f} 米\n")
                f.write(f"  最短: {np.min(distances):.2f} 米\n")
                f.write(f"  最长: {np.max(distances):.2f} 米\n")
                f.write(f"  标准差: {np.std(distances):.2f} 米\n")

        print(f"✓ 统计报告已保存到: {report_file}")

    def _plot_statistics(self, distances: List[float], num_waypoints: List[int], output_path: Path):
        """生成统计可视化图表"""
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        # 路径长度分布
        axes[0].hist(distances, bins=20, edgecolor='black', alpha=0.7)
        axes[0].set_xlabel('Path Length (m)')
        axes[0].set_ylabel('Frequency')
        axes[0].set_title('Path Length Distribution')
        axes[0].grid(True, alpha=0.3)

        # 路径点数量分布
        axes[1].hist(num_waypoints, bins=20, edgecolor='black', alpha=0.7, color='orange')
        axes[1].set_xlabel('Number of Waypoints')
        axes[1].set_ylabel('Frequency')
        axes[1].set_title('Waypoint Count Distribution')
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(output_path / "statistics_plots.png", dpi=150)
        plt.close()
        print(f"✓ 统计图表已保存到: {output_path / 'statistics_plots.png'}")

    def visualize_paths(self, output_dir: str = "outputs/batch_planning", max_paths: int = 20):
        """
        可视化前 N 条路径

        Args:
            output_dir: 输出目录路径
            max_paths: 最多可视化路径数量
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        successful_results = [r for r in self.results if r['success']][:max_paths]

        for result in successful_results:
            # 加载占用地图
            grid, W, H = load_grid(result['barrier_image_path'])

            # 可视化路径
            task_id = result['task_id'].replace('/', '_')
            save_path = output_path / f"{task_id}_path.png"

            save_path_image(grid, result['path_grid'], save_path=str(save_path))

        print(f"✓ 已保存 {len(successful_results)} 个路径可视化图像到: {output_path}")

    def export_to_csv(self, output_path: str = "outputs/batch_planning/results.csv"):
        """
        导出结果到 CSV 格式

        Args:
            output_path: 输出 CSV 文件路径
        """
        import csv

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'task_id', 'success', 'start_x', 'start_y', 'end_x', 'end_y',
                'total_distance', 'num_waypoints', 'barrier_image_path'
            ])

            for result in self.results:
                if result['success']:
                    writer.writerow([
                        result['task_id'],
                        'Yes',
                        result['start'][0], result['start'][1],
                        result['end'][0], result['end'][1],
                        f"{result['total_distance']:.4f}",
                        result['num_waypoints'],
                        result['barrier_image_path']
                    ])
                else:
                    writer.writerow([
                        result['task_id'],
                        'No',
                        '', '', '', '',
                        '',
                        '',
                        result.get('barrier_image_path', '')
                    ])

        print(f"✓ CSV 结果已保存到: {output_path}")


# ==================== 使用示例 ====================
if __name__ == "__main__":
    print("="*70)
    print("批量路径规划工具 - 使用示例")
    print("="*70)

    # ===== 方式1: 从配置文件加载任务 =====
    print("\n【方式1】从配置文件加载任务\n")

    planner = BatchPathPlanner()

    # 假设有一个 JSON 配置文件
    # planner = BatchPathPlanner("config/batch_tasks.json")

    # ===== 方式2: 手动添加任务（每个任务独立的占用地图）=====
    print("【方式2】手动添加任务（每个任务独立的占用地图）\n")

    planner.add_task(
        task_id="scene1_task_001",
        start=[1.0, 1.0],
        end=[8.0, 8.0],
        barrier_image_path="data/navigation_scenes/barrier_image_level5_1.png",
        x_bounds=[0.0, 10.0],
        y_bounds=[0.0, 10.0],
        offset_radius=0.3
    )

    planner.add_task(
        task_id="scene2_task_001",
        start=[2.0, 2.0],
        end=[7.0, 7.0],
        barrier_image_path="data/navigation_scenes/barrier_image_level5_2.png",
        x_bounds=[0.0, 10.0],
        y_bounds=[0.0, 10.0],
        offset_radius=0.3
    )

    # ===== 方式3: 从 YAML 场景配置批量添加（共享占用地图）=====
    print("【方式3】从 YAML 场景配置批量添加（共享占用地图）\n")

    # 定义多组起终点
    goal_pairs = [
        ([1.0, 1.0], [8.0, 8.0]),
        ([2.0, 2.0], [7.0, 7.0]),
        ([1.5, 1.5], [6.0, 8.0]),
        ([3.0, 1.0], [6.0, 9.0]),
        ([1.0, 3.0], [9.0, 7.0]),
    ]

    planner.add_tasks_from_yaml_assets(
        yaml_path="config/navigation/navigation_assets_fbh.yaml",
        goal_pairs=goal_pairs
    )

    # ===== 执行批量规划 =====
    print("\n" + "="*70)
    print("执行批量路径规划")
    print("="*70 + "\n")

    results = planner.run_batch(parallel=False, verbose=True)

    # ===== 保存结果 =====
    print("\n" + "="*70)
    print("保存结果")
    print("="*70 + "\n")

    planner.save_results(output_dir="outputs/batch_planning_demo")

    # ===== 生成统计报告 =====
    planner.generate_summary_report(output_dir="outputs/batch_planning_demo")

    # ===== 可视化路径 =====
    planner.visualize_paths(output_dir="outputs/batch_planning_demo", max_paths=10)

    # ===== 导出 CSV =====
    planner.export_to_csv(output_path="outputs/batch_planning_demo/results.csv")

    print("\n" + "="*70)
    print("批量规划完成！")
    print("="*70)
