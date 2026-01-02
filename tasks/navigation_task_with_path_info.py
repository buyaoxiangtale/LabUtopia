"""
增强版导航任务：记录和使用完整的路径信息

相比原版 (navigation_task.py) 的改进：
1. 保存并使用路径总长度 (total_distance)
2. 计算并保存路径段详细信息 (segments)
3. 提供路径统计和预估时间
4. 支持从预计算的路径文件加载
"""

import numpy as np
import yaml
import json
from typing import Dict, Any, List, Optional
from .base_task import BaseTask
from utils.a_star import plan_navigation_path, real_to_grid, load_grid
from isaacsim.core.utils.rotations import quat_to_euler_angles


class NavigationTaskWithPathInfo(BaseTask):
    """
    带完整路径信息的导航任务

    新增属性：
        total_distance: 路径总长度（米）
        segment_velocities: 路径段速度信息列表
        estimated_time: 预计完成时间（秒）
    """

    def __init__(self, cfg, world, stage, robot):
        self.navigation_assets = []
        self.grid = None
        self.W, self.H = None, None

        # 路径信息
        self.current_start = None
        self.current_end = None
        self.current_path = None

        # 新增：完整路径信息
        self.total_distance = 0.0  # 路径总长度
        self.segment_velocities = []  # 路径段详细信息
        self.estimated_time = 0.0  # 预计完成时间

        super().__init__(cfg, world, stage, robot)

    def setup_objects(self) -> None:
        """加载导航场景配置"""
        super().setup_objects()

        if hasattr(self.cfg.task, 'navigation_config_path'):
            with open(self.cfg.task.navigation_config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.navigation_assets = config.get('assets', [])

        if self.navigation_assets:
            nav_scene = self.navigation_assets[0]
            self.grid, self.W, self.H = load_grid(nav_scene['barrier_image_path'])

    def reset(self) -> None:
        """重置任务，生成新的导航路径"""
        super().reset()
        self.robot.initialize()

        if self.navigation_assets and len(self.navigation_assets) > 0:
            success = self._generate_navigation_task()
            if not success:
                print("Warning: Unable to generate a valid navigation path")

    def _generate_navigation_task(self) -> bool:
        """
        生成导航任务（完整版，包含路径信息）

        Returns:
            bool: 是否成功生成路径
        """
        nav_scene = self.navigation_assets[0]
        max_attempts = 100

        for _ in range(max_attempts):
            # 随机生成起点和终点
            start_point, end_point = self._generate_random_points(
                nav_scene['x_bounds'],
                nav_scene['y_bounds'],
                self.grid
            )

            if start_point is None or end_point is None:
                continue

            # 规划路径
            task_info = {
                "asset": nav_scene,
                "start": start_point,
                "end": end_point
            }

            path_result = plan_navigation_path(task_info)

            if path_result is not None:
                # ✅ 改进：正确使用返回值
                real_path, path_grid, total_distance = path_result

                self.current_start = start_point
                self.current_end = end_point

                # 添加方向信息
                waypoints = []
                for i in range(len(real_path)):
                    x, y, _ = real_path[i]
                    if i < len(real_path) - 1:
                        next_x, next_y, _ = real_path[i + 1]
                        theta = np.arctan2(next_y - y, next_x - x)
                    else:
                        theta = waypoints[-1][2] if waypoints else 0.0
                    waypoints.append([x, y, theta])

                self.current_path = waypoints

                # ✅ 新增：保存路径长度
                self.total_distance = total_distance

                # ✅ 新增：计算路径段详细信息
                self.segment_velocities = self._calculate_segment_velocities()

                # ✅ 新增：计算预计完成时间
                self.estimated_time = sum(seg['estimated_time'] for seg in self.segment_velocities)

                # 设置机器人初始位置
                initial_position = np.array([start_point[0], start_point[1], 0.0])
                self.robot.set_world_pose(position=initial_position)

                # 打印路径信息
                self._print_path_info()

                return True

        return False

    def _calculate_segment_velocities(self) -> List[Dict[str, Any]]:
        """
        计算路径段的速度信息

        Returns:
            路径段信息列表，包含距离、方向、速度、预计时间等
        """
        if self.current_path is None or len(self.current_path) < 2:
            return []

        segments = []
        max_linear_speed = 0.02  # 与控制器默认值一致

        for i in range(len(self.current_path) - 1):
            p1 = self.current_path[i]
            p2 = self.current_path[i + 1]

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            distance = np.sqrt(dx**2 + dy**2)

            direction = np.arctan2(dy, dx)
            direction_deg = np.degrees(direction)

            # 速度大小（与控制器逻辑一致）
            velocity_magnitude = min(distance * 0.2, max_linear_speed)

            # 预计时间
            estimated_time = distance / velocity_magnitude if velocity_magnitude > 0 else 0

            segments.append({
                "segment_idx": i,
                "from": [p1[0], p1[1]],
                "to": [p2[0], p2[1]],
                "distance": distance,
                "direction": direction,
                "direction_deg": direction_deg,
                "velocity_magnitude": velocity_magnitude,
                "estimated_time": estimated_time
            })

        return segments

    def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
        """生成随机起点和终点"""
        W = len(grid[0])
        H = len(grid)

        for _ in range(attempts):
            start_x = np.random.uniform(x_bounds[0], x_bounds[1])
            start_y = np.random.uniform(y_bounds[0], y_bounds[1])
            end_x = np.random.uniform(x_bounds[0], x_bounds[1])
            end_y = np.random.uniform(y_bounds[0], y_bounds[1])

            i_start, j_start = real_to_grid(
                start_x, start_y, x_bounds, y_bounds, (W, H)
            )
            i_end, j_end = real_to_grid(
                end_x, end_y, x_bounds, y_bounds, (W, H)
            )

            if grid[i_start][j_start] == 0 and grid[i_end][j_end] == 0:
                return [start_x, start_y], [end_x, end_y]

        return None, None

    def _print_path_info(self):
        """打印路径信息摘要"""
        print(f"\n{'='*60}")
        print(f"导航任务生成成功")
        print(f"{'='*60}")
        print(f"起点: ({self.current_start[0]:.2f}, {self.current_start[1]:.2f})")
        print(f"终点: ({self.current_end[0]:.2f}, {self.current_end[1]:.2f})")
        print(f"路径总长度: {self.total_distance:.3f} 米")
        print(f"路径点数量: {len(self.current_path)}")
        print(f"预计完成时间: {self.estimated_time:.2f} 秒")
        print(f"平均速度: {self.total_distance / self.estimated_time:.4f} m/s" if self.estimated_time > 0 else "平均速度: N/A")
        print(f"{'='*60}\n")

    def step(self) -> Dict[str, Any]:
        """
        执行仿真步骤

        Returns:
            包含完整路径信息的状态字典
        """
        self.frame_idx += 1

        if not self.check_frame_limits():
            return None

        # 获取机器人当前位置和朝向
        position, orientation = self.robot.get_world_pose()
        euler_angles = quat_to_euler_angles(orientation, extrinsic=False)
        current_pose = np.array([position[0], position[1], euler_angles[2]])

        # 获取相机数据
        camera_data, display_data = self.get_camera_data()

        state = {
            'current_pose': current_pose,
            'start_point': self.current_start,
            'end_point': self.current_end,
            'waypoints': self.current_path,

            # ✅ 新增：完整路径信息
            'total_distance': self.total_distance,
            'segment_velocities': self.segment_velocities,
            'estimated_time': self.estimated_time,
            'num_waypoints': len(self.current_path) if self.current_path else 0,

            'camera_data': camera_data,
            'camera_display': display_data,
            'done': self.reset_needed,
            'frame_idx': self.frame_idx
        }

        return state

    def load_path_from_file(self, path_file: str) -> bool:
        """
        从预计算的路径文件加载路径

        Args:
            path_file: JSON 格式的路径文件

        Returns:
            bool: 是否成功加载
        """
        try:
            with open(path_file, 'r') as f:
                data = json.load(f)

            # 支持单条路径或多条路径
            if isinstance(data, list):
                if len(data) == 0:
                    return False
                path_data = data[0]  # 使用第一条路径
            else:
                path_data = data

            # 设置路径信息
            self.current_start = path_data['start']
            self.current_end = path_data['end']
            self.current_path = path_data['waypoints']
            self.total_distance = path_data['total_distance']
            self.num_waypoints = path_data['num_waypoints']

            # 如果有路径段信息，也加载
            if 'segments' in path_data:
                self.segment_velocities = path_data['segments']
                self.estimated_time = sum(seg['estimated_time'] for seg in self.segment_velocities)

            # 设置机器人初始位置
            initial_position = np.array([self.current_start[0], self.current_start[1], 0.0])
            self.robot.set_world_pose(position=initial_position)

            print(f"✓ 从文件加载路径: {path_file}")
            self._print_path_info()

            return True

        except Exception as e:
            print(f"❌ 加载路径文件失败: {e}")
            return False

    def get_path_summary(self) -> Dict[str, Any]:
        """
        获取路径摘要信息

        Returns:
            包含关键路径统计信息的字典
        """
        return {
            "start": self.current_start,
            "end": self.current_end,
            "total_distance": round(self.total_distance, 3),
            "num_waypoints": len(self.current_path) if self.current_path else 0,
            "estimated_time": round(self.estimated_time, 2),
            "avg_velocity": round(self.total_distance / self.estimated_time, 4) if self.estimated_time > 0 else 0,
            "num_segments": len(self.segment_velocities)
        }

    def on_task_complete(self, success: bool) -> None:
        """任务完成回调"""
        self.reset_needed = True
