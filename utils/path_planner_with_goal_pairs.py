#!/usr/bin/env python3
"""
使用goal_pairs.yaml和障碍物图像进行批量路径规划

功能：
1. 从goal_pairs.yaml读取起终点对
2. 使用指定的障碍物图像进行A*路径规划
3. 生成完整的waypoints（包含角度信息）
4. 支持自动或手动指定场景边界
5. 输出规划结果和可视化
"""

import numpy as np
import yaml
import json
from pathlib import Path
from typing import List, Dict, Any, Tuple, Optional
import matplotlib.pyplot as plt
from PIL import Image

from utils.a_star import (
    plan_navigation_path,
    load_grid,
    save_path_image,
    calculate_path_distance
)


class ScenePathPlanner:
    """场景路径规划器"""

    def __init__(
        self,
        barrier_image_path: str,
        x_bounds: Optional[List[float]] = None,
        y_bounds: Optional[List[float]] = None,
        offset_radius: float = 0.6
    ):
        """
        初始化路径规划器

        Args:
            barrier_image_path: 障碍物图像路径
            x_bounds: X轴边界 [min, max]，如果为None则自动估算
            y_bounds: Y轴边界 [min, max]，如果为None则自动估算
            offset_radius: 障碍物膨胀半径（米）
        """
        self.barrier_image_path = barrier_image_path
        self.offset_radius = offset_radius

        # 加载障碍物地图
        self.grid, self.W, self.H = load_grid(barrier_image_path)
        print(f"✓ 加载障碍物地图: {barrier_image_path}")
        print(f"  尺寸: {self.W} x {self.H} 像素")

        # 设置边界
        if x_bounds is None or y_bounds is None:
            x_bounds, y_bounds = self._estimate_bounds_from_image()

        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        print(f"  X边界: {x_bounds}")
        print(f"  Y边界: {y_bounds}")
        print(f"  分辨率: {(x_bounds[1]-x_bounds[0])/self.W:.4f} 米/像素")

    def _estimate_bounds_from_image(self) -> Tuple[List[float], List[float]]:
        """
        从图像尺寸估算边界

        默认假设场景为正方形，边长为10米
        """
        # 计算图像宽高比
        aspect_ratio = self.W / self.H

        # 假设场景为10米x10米（可根据实际情况调整）
        default_size = 10.0

        if aspect_ratio > 1:
            # 宽图像
            x_bounds = [0, default_size * aspect_ratio]
            y_bounds = [0, default_size]
        else:
            # 高图像
            x_bounds = [0, default_size]
            y_bounds = [0, default_size / aspect_ratio]

        print(f"⚠ 自动估算边界（建议手动指定）:")
        return x_bounds, y_bounds

    def plan_single_pair(
        self,
        start: List[float],
        end: List[float],
        task_id: str = "unknown"
    ) -> Optional[Dict[str, Any]]:
        """
        规划单组起终点对的路径

        Args:
            start: 起点坐标 [x, y] 或 [x, y, z]
            end: 终点坐标 [x, y] 或 [x, y, z]
            task_id: 任务ID

        Returns:
            规划结果字典，失败时返回None
        """
        # 只取x, y坐标
        start_xy = [start[0], start[1]]
        end_xy = [end[0], end[1]]

        # 构建task_info
        task_info = {
            'asset': {
                'barrier_image_path': self.barrier_image_path,
                'x_bounds': self.x_bounds,
                'y_bounds': self.y_bounds,
                'offset_radius': self.offset_radius
            },
            'start': start_xy,
            'end': end_xy
        }

        # 执行路径规划
        path_result = plan_navigation_path(task_info)
        
        if path_result is None:
            print(f"✗ {task_id}: 路径规划失败 - 起点 {start_xy} → 终点 {end_xy}")
            # 生成失败可视化
            output_path = Path(self.output_dir) / f"{task_id}_debug.png"
            self._visualize_failure(task_info, output_path)
            return None

        real_path, path_grid, total_distance = path_result

        # 生成waypoints（添加角度信息）
        waypoints = []
        for i, (x, y, _) in enumerate(real_path):
            if i < len(real_path) - 1:
                nx, ny, _ = real_path[i + 1]
                theta = np.arctan2(ny - y, nx - x)
            else:
                theta = waypoints[-1][2] if waypoints else 0.0
            waypoints.append([x, y, theta])

        print(f"✓ {task_id}: 成功 - 距离 {total_distance:.2f}m, 路径点 {len(waypoints)}个")

        return {
            'task_id': task_id,
            'start': start_xy,
            'end': end_xy,
            'waypoints': waypoints,
            'total_distance': total_distance,
            'num_waypoints': len(waypoints),
            'path_grid': path_grid
        }

    def plan_from_goal_pairs_yaml(
        self,
        yaml_path: str,
        visualize: bool = True,
        output_dir: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        从goal_pairs.yaml文件批量规划路径

        Args:
            yaml_path: goal_pairs.yaml文件路径
            visualize: 是否生成可视化
            output_dir: 输出目录

        Returns:
            规划结果列表
        """
        # 加载goal_pairs
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        goal_pairs = config.get('goal_pairs', [])
        metadata = config.get('metadata', {})

        # 尝试从metadata获取offset_radius
        if 'config' in metadata and 'offset_radius' in metadata['config']:
            self.offset_radius = metadata['config']['offset_radius']
            print(f"✓ 使用配置中的offset_radius: {self.offset_radius}")

        scene_name = metadata.get('scene_name', 'unknown_scene')

        print(f"\n{'='*70}")
        print(f"开始批量路径规划")
        print(f"场景: {scene_name}")
        print(f"起终点对数量: {len(goal_pairs)}")
        print(f"{'='*70}\n")

        results = []
        successful = 0
        failed = 0

        for i, pair in enumerate(goal_pairs):
            start = pair['start']
            end = pair['end']
            task_id = f"{scene_name}_pair_{i:03d}"

            result = self.plan_single_pair(start, end, task_id)

            if result is not None:
                results.append(result)
                successful += 1

                # 可视化路径
                if visualize:
                    self._visualize_single_path(result, output_dir, task_id)
            else:
                failed += 1

        # 保存结果
        if output_dir:
            self._save_results(results, scene_name, output_dir)

        # 打印统计
        print(f"\n{'='*70}")
        print(f"规划完成")
        print(f"成功: {successful}/{len(goal_pairs)} ({successful/len(goal_pairs)*100:.1f}%)")
        print(f"失败: {failed}/{len(goal_pairs)} ({failed/len(goal_pairs)*100:.1f}%)")

        if successful > 0:
            distances = [r['total_distance'] for r in results]
            print(f"\n路径长度统计:")
            print(f"  平均: {np.mean(distances):.2f} 米")
            print(f"  最短: {np.min(distances):.2f} 米")
            print(f"  最长: {np.max(distances):.2f} 米")

        print(f"{'='*70}\n")

        return results

    def _visualize_single_path(
        self,
        result: Dict[str, Any],
        output_dir: Optional[str],
        task_id: str
    ):
        """可视化单条路径"""
        if output_dir is None:
            return

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        save_path = output_path / f"{task_id}_path.png"
        save_path_image(self.grid, result['path_grid'], save_path=str(save_path))

    def _save_results(
        self,
        results: List[Dict[str, Any]],
        scene_name: str,
        output_dir: str
    ):
        """保存规划结果"""
        output_path = Path(output_dir)

        # 1. 保存完整JSON结果
        json_results = []
        for result in results:
            json_result = result.copy()
            # 移除不可序列化的path_grid
            if 'path_grid' in json_result:
                del json_result['path_grid']
            json_results.append(json_result)

        results_file = output_path / f"{scene_name}_planning_results.json"
        with open(results_file, 'w') as f:
            json.dump(json_results, f, indent=2)
        print(f"✓ 结果已保存: {results_file}")

        # 2. 保存waypoints（用于后续导航任务）
        waypoints_list = [r['waypoints'] for r in results]
        waypoints_file = output_path / f"{scene_name}_waypoints.json"
        with open(waypoints_file, 'w') as f:
            json.dump(waypoints_list, f, indent=2)
        print(f"✓ Waypoints已保存: {waypoints_file}")

        # 3. 生成导航任务配置（可直接用于main.py）
        self._generate_nav_config(results, scene_name, output_path)

    def _generate_nav_config(
        self,
        results: List[Dict[str, Any]],
        scene_name: str,
        output_path: Path
    ):
        """生成导航任务配置文件"""
        # 提取goal_pairs，确保所有值都是标准Python类型
        goal_pairs = []
        for r in results:
            # 转换为标准列表，避免numpy对象
            start = [float(r['start'][0]), float(r['start'][1]), 0.0]
            end = [float(r['end'][0]), float(r['end'][1]), float(r['waypoints'][-1][2])]
            goal_pairs.append({
                'start': start,
                'end': end
            })

        nav_config = {
            'goal_pairs': goal_pairs,
            'metadata': {
                'scene_name': scene_name,
                'num_goal_pairs': len(goal_pairs),
                'barrier_image_path': self.barrier_image_path,
                'x_bounds': [float(self.x_bounds[0]), float(self.x_bounds[1])],
                'y_bounds': [float(self.y_bounds[0]), float(self.y_bounds[1])],
                'offset_radius': float(self.offset_radius)
            }
        }

        config_file = output_path / f"{scene_name}_nav_config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(nav_config, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        print(f"✓ 导航配置已保存: {config_file}")


# ==================== 命令行使用 ====================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='使用goal_pairs.yaml进行路径规划')
    parser.add_argument('--barrier_image', type=str, required=True,
                       help='障碍物图像路径')
    parser.add_argument('--goal_pairs_yaml', type=str, required=True,
                       help='goal_pairs.yaml文件路径')
    parser.add_argument('--x_bounds', type=float, nargs=2, default=None,
                       help='X轴边界 (如: 0 10)')
    parser.add_argument('--y_bounds', type=float, nargs=2, default=None,
                       help='Y轴边界 (如: 0 10)')
    parser.add_argument('--offset_radius', type=float, default=0.6,
                       help='障碍物膨胀半径（米）, 默认0.6')
    parser.add_argument('--output_dir', type=str,
                       default='outputs/path_planning_results',
                       help='输出目录')
    parser.add_argument('--no_visualize', action='store_true',
                       help='不生成可视化图像')

    args = parser.parse_args()

    # 创建规划器
    planner = ScenePathPlanner(
        barrier_image_path=args.barrier_image,
        x_bounds=args.x_bounds,
        y_bounds=args.y_bounds,
        offset_radius=args.offset_radius
    )

    # 执行规划
    results = planner.plan_from_goal_pairs_yaml(
        yaml_path=args.goal_pairs_yaml,
        visualize=not args.no_visualize,
        output_dir=args.output_dir
    )

    print(f"\n✓ 规划完成！结果保存在: {args.output_dir}")
