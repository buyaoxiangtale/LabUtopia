#!/usr/bin/env python3
"""
路径预计算工具：在场景加载前完成所有路径规划和信息计算

使用场景：
- 在不启动 Isaac Sim 的情况下预先计算路径
- 验证起终点的可行性
- 生成路径统计数据
- 批量生成导航任务
"""

import numpy as np
import yaml
import json
from typing import List, Dict, Any, Tuple, Optional
from utils.a_star import (
    plan_navigation_path,
    load_grid,
    real_to_grid,
    calculate_path_distance
)


class PathPrecomputer:
    """路径预计算器"""

    def __init__(self, config_path: str):
        """
        初始化预计算器

        Args:
            config_path: 导航场景配置文件路径 (YAML)
        """
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            self.nav_scene = config['assets'][0]

        # 加载障碍物网格（不需要加载3D场景！）
        self.grid, self.W, self.H = load_grid(self.nav_scene['barrier_image_path'])

        print(f"✓ 已加载场景配置: {self.nav_scene['name']}")
        print(f"  网格尺寸: {self.W}x{self.H}")
        print(f"  边界: x{self.nav_scene['x_bounds']}, y{self.nav_scene['y_bounds']}")

    def plan_single_path(
        self,
        start: List[float],
        end: List[float]
    ) -> Optional[Dict[str, Any]]:
        """
        规划单条路径并返回完整信息

        Args:
            start: 起点坐标 [x, y]
            end: 终点坐标 [x, y]

        Returns:
            包含完整路径信息的字典，如果规划失败则返回 None
        """
        task_info = {
            "asset": self.nav_scene,
            "start": start,
            "end": end
        }

        path_result = plan_navigation_path(task_info)

        if path_result is None:
            return None

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
            "start": start,
            "end": end,
            "waypoints": waypoints,
            "total_distance": total_distance,
            "num_waypoints": len(waypoints),
            "segments": segments,
            "path_grid": path_grid  # 网格坐标路径（用于可视化）
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

    def generate_random_paths(
        self,
        num_paths: int = 10,
        min_distance: float = 2.0,
        max_attempts: int = 100
    ) -> List[Dict[str, Any]]:
        """
        随机生成多条路径

        Args:
            num_paths: 要生成的路径数量
            min_distance: 最小路径长度（米）
            max_attempts: 每条路径的最大尝试次数

        Returns:
            成功规划的路径列表
        """
        valid_paths = []
        attempts = 0

        print(f"\n开始随机生成 {num_paths} 条路径（最小长度: {min_distance}m）...")

        while len(valid_paths) < num_paths and attempts < max_attempts:
            # 随机生成起点和终点
            start = self._generate_random_point()
            end = self._generate_random_point()

            # 检查是否在自由空间
            if not self._is_free_space(start) or not self._is_free_space(end):
                attempts += 1
                continue

            # 规划路径
            path_info = self.plan_single_path(start, end)

            if path_info is not None and path_info['total_distance'] >= min_distance:
                valid_paths.append(path_info)
                print(f"  [{len(valid_paths)}/{num_paths}] ✓ 路径长度: {path_info['total_distance']:.2f}m, "
                      f"路径点: {path_info['num_waypoints']}")
            else:
                attempts += 1

        print(f"\n✓ 成功生成 {len(valid_paths)} 条路径")
        return valid_paths

    def _generate_random_point(self) -> List[float]:
        """生成场景内的随机点"""
        x = np.random.uniform(self.nav_scene['x_bounds'][0], self.nav_scene['x_bounds'][1])
        y = np.random.uniform(self.nav_scene['y_bounds'][0], self.nav_scene['y_bounds'][1])
        return [x, y]

    def _is_free_space(self, point: List[float]) -> bool:
        """检查点是否在自由空间中"""
        grid_pos = real_to_grid(
            point[0], point[1],
            self.nav_scene['x_bounds'],
            self.nav_scene['y_bounds'],
            (self.W, self.H)
        )
        return self.grid[grid_pos[0]][grid_pos[1]] == 0

    def save_paths_to_file(self, paths: List[Dict[str, Any]], output_path: str):
        """
        保存路径信息到文件

        Args:
            paths: 路径信息列表
            output_path: 输出文件路径（.json）
        """
        # 转换为可序列化的格式
        serializable_paths = []
        for path in paths:
            serializable_paths.append({
                "start": path["start"],
                "end": path["end"],
                "waypoints": path["waypoints"],
                "total_distance": round(path["total_distance"], 4),
                "num_waypoints": path["num_waypoints"],
                "segments": path["segments"]
            })

        with open(output_path, 'w') as f:
            json.dump(serializable_paths, f, indent=2)

        print(f"✓ 路径信息已保存到: {output_path}")

    def print_path_summary(self, path_info: Dict[str, Any]):
        """打印路径摘要信息"""
        print(f"\n{'='*60}")
        print(f"路径摘要")
        print(f"{'='*60}")
        print(f"起点: ({path_info['start'][0]:.2f}, {path_info['start'][1]:.2f})")
        print(f"终点: ({path_info['end'][0]:.2f}, {path_info['end'][1]:.2f})")
        print(f"总长度: {path_info['total_distance']:.3f} 米")
        print(f"路径点数量: {path_info['num_waypoints']}")
        print(f"\n路径段信息 (前5段):")
        print(f"{'序号':<6} {'距离(m)':<10} {'方向(°)':<10} {'速度(m/s)':<12} {'预计时间(s)':<12}")
        print(f"{'-'*60}")

        for seg in path_info['segments'][:5]:
            print(f"{seg['segment_idx']:<6} "
                  f"{seg['distance']:<10.3f} "
                  f"{seg['direction_deg']:<10.1f} "
                  f"{seg['velocity_magnitude']:<12.4f} "
                  f"{seg['estimated_time']:<12.2f}")

        if len(path_info['segments']) > 5:
            print(f"... (共 {len(path_info['segments'])} 段)")

        # 统计信息
        total_time = sum(seg['estimated_time'] for seg in path_info['segments'])
        avg_velocity = path_info['total_distance'] / total_time if total_time > 0 else 0

        print(f"\n统计信息:")
        print(f"  预计总时间: {total_time:.2f} 秒")
        print(f"  平均速度: {avg_velocity:.4f} m/s")
        print(f"{'='*60}\n")


# ==================== 使用示例 ====================
if __name__ == "__main__":

    # 示例1: 规划单条指定路径
    print("="*60)
    print("示例1: 规划单条指定路径")
    print("="*60)

    precomputer = PathPrecomputer("config/navigation/navigation_assets_fbh.yaml")

    # 指定起点和终点
    start_point = [1.0, 1.0]
    end_point = [8.0, 8.0]

    path_info = precomputer.plan_single_path(start_point, end_point)

    if path_info:
        precomputer.print_path_summary(path_info)
        precomputer.save_paths_to_file([path_info], "outputs/single_path.json")
    else:
        print("❌ 路径规划失败")

    # 示例2: 随机生成多条路径
    print("\n" + "="*60)
    print("示例2: 随机生成多条路径")
    print("="*60)

    random_paths = precomputer.generate_random_paths(
        num_paths=5,
        min_distance=3.0
    )

    if random_paths:
        # 保存所有路径
        precomputer.save_paths_to_file(random_paths, "outputs/random_paths.json")

        # 打印统计信息
        distances = [p['total_distance'] for p in random_paths]
        print(f"\n生成路径统计:")
        print(f"  数量: {len(random_paths)}")
        print(f"  平均长度: {np.mean(distances):.2f} 米")
        print(f"  最短: {np.min(distances):.2f} 米")
        print(f"  最长: {np.max(distances):.2f} 米")
        print(f"  标准差: {np.std(distances):.2f} 米")

    # 示例3: 批量验证预设的起终点对
    print("\n" + "="*60)
    print("示例3: 批量验证预设起终点对")
    print("="*60)

    preset_goal_pairs = [
        ([1.0, 1.0], [8.0, 8.0]),
        ([2.0, 2.0], [7.0, 7.0]),
        ([1.5, 1.5], [6.0, 8.0]),
    ]

    valid_paths = []
    for i, (start, end) in enumerate(preset_goal_pairs):
        print(f"\n验证第 {i+1} 对起终点:")
        print(f"  起点: {start}, 终点: {end}")

        path_info = precomputer.plan_single_path(start, end)
        if path_info:
            print(f"  ✓ 成功 - 长度: {path_info['total_distance']:.2f}m")
            valid_paths.append(path_info)
        else:
            print(f"  ✗ 失败 - 无法找到路径")

    print(f"\n验证完成: {len(valid_paths)}/{len(preset_goal_pairs)} 对起终点有效")
