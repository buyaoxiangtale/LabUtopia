import numpy as np
import yaml
from typing import Dict, Any, List, Tuple, Optional

from .base_task import BaseTask
from utils.a_star import plan_navigation_path, real_to_grid, load_grid
from isaacsim.core.utils.rotations import quat_to_euler_angles


class NavigationTaskNewCPCP(BaseTask):
    """
    顺序式导航任务：
    - 支持外部传入多个有序起终点对（队列）
    - 每次到达后自动切换到下一个起终点
    - 若没有提供队列则回退为随机起终点 + A* 规划（与旧版一致）
    
    配置/注入方式（两种二选一）：
    1) Hydra 配置传入 cfg.task.goal_pairs:
        goal_pairs:
          - start: [x1, y1]
            end:   [x2, y2]
          - [ [x3, y3], [x4, y4] ]   # 也接受这种二元列表形式
    2) 运行时调用 append_goal_pairs / set_goal_pairs 追加或重置队列。
    """

    def __init__(self, cfg, world, stage, robot):
        self.navigation_assets: List[dict] = []
        self.grid = None
        self.current_start: Optional[list] = None
        self.current_end: Optional[list] = None
        self.current_path: Optional[List[List[float]]] = None

        # 有序起终点队列
        self.goal_pairs: List[Tuple[list, list]] = []
        self.current_goal_idx: int = 0

        super().__init__(cfg, world, stage, robot)

        # 从配置预加载队列（如果有）
        if hasattr(self.cfg, "task") and hasattr(self.cfg.task, "goal_pairs"):
            self.set_goal_pairs(self.cfg.task.goal_pairs)

    # ---------- 场景与地图 ----------
    def setup_objects(self) -> None:
        super().setup_objects()

        if hasattr(self.cfg.task, "navigation_config_path"):
            with open(self.cfg.task.navigation_config_path, "r") as f:
                config = yaml.safe_load(f)
                self.navigation_assets = config.get("assets", [])

        if self.navigation_assets:
            nav_scene = self.navigation_assets[0]
            self.grid, self.W, self.H = load_grid(nav_scene["barrier_image_path"])

    # ---------- 任务流程 ----------
    def reset(self) -> None:
        super().reset()
        self.robot.initialize()

        if not self.navigation_assets:
            return

        # 优先使用外部队列；若为空则回退随机生成
        if self._has_remaining_goals():
            success = self._apply_next_goal_until_success()
            if not success:
                print("Warning: 提供的起终点均规划失败，回退为随机任务。")
                self._generate_random_navigation_task()
        else:
            self._generate_random_navigation_task()

    def _calculate_segment_velocities(self) -> List[Dict[str, Any]]:
        """
        计算路径中每个段的速度大小和方向信息

        Returns:
            List[Dict]: 每个路径段的速度信息
                - segment_idx: 段索引
                - start_point: 起点坐标
                - end_point: 终点坐标
                - distance: 段长度
                - direction: 方向角度（弧度）
                - direction_deg: 方向角度（度）
                - velocity_magnitude: 速度大小（基于距离比例）
        """
        if self.current_path is None or len(self.current_path) < 2:
            return []

        velocities = []
        # 使用与控制器相同的速度计算逻辑
        max_linear_speed = 0.02  # 与控制器默认值一致

        for i in range(len(self.current_path) - 1):
            start_point = self.current_path[i]
            end_point = self.current_path[i + 1]

            # 计算距离
            dx = end_point[0] - start_point[0]
            dy = end_point[1] - start_point[1]
            distance = np.sqrt(dx**2 + dy**2)

            # 计算方向
            direction = np.arctan2(dy, dx)
            direction_deg = np.degrees(direction)

            # 计算速度大小（与控制器逻辑一致）
            # speed = min(distance * 0.2, max_linear_speed)
            velocity_magnitude = min(distance * 0.2, max_linear_speed)

            segment_info = {
                "segment_idx": i,
                "start_point": [start_point[0], start_point[1]],
                "end_point": [end_point[0], end_point[1]],
                "distance": distance,
                "direction": direction,
                "direction_deg": direction_deg,
                "velocity_magnitude": velocity_magnitude,
                "velocity_vector": [
                    velocity_magnitude * np.cos(direction),
                    velocity_magnitude * np.sin(direction)
                ]
            }
            velocities.append(segment_info)

        return velocities

    def get_current_segment_velocity(self, current_position: Optional[List[float]] = None) -> Optional[Dict[str, Any]]:
        """
        获取当前正在执行的路径段速度信息

        Args:
            current_position: 当前机器人位置 [x, y]，如果不提供则使用第一个段

        Returns:
            Optional[Dict]: 当前段的速度信息，如果没有路径则返回None
        """
        if self.current_path is None or len(self.current_path) < 2:
            return None

        velocities = self._calculate_segment_velocities()
        if not velocities:
            return None

        if current_position is None:
            # 如果没有提供当前位置，返回第一个段
            return velocities[0]

        # 根据当前位置找到最近的路径段
        min_distance = float('inf')
        current_segment = None

        for vel in velocities:
            # 计算当前位置到段起点的距离
            start_point = vel['start_point']
            dist_to_start = np.sqrt(
                (current_position[0] - start_point[0])**2 +
                (current_position[1] - start_point[1])**2
            )

            if dist_to_start < min_distance:
                min_distance = dist_to_start
                current_segment = vel

        return current_segment

    def print_velocity_info(self) -> None:
        """
        打印当前路径的详细速度信息
        """
        velocities = self._calculate_segment_velocities()
        if not velocities:
            print("⚠️  没有可用的路径速度信息")
            return

        print(f"\n🚀 路径段速度信息 (共 {len(velocities)} 段):")
        print("=" * 80)

        for vel in velocities:
            print(f"段 {vel['segment_idx']+1}: "
                  f"[{vel['start_point'][0]:.3f}, {vel['start_point'][1]:.3f}] → "
                  f"[{vel['end_point'][0]:.3f}, {vel['end_point'][1]:.3f}]")
            print(f"    距离: {vel['distance']:.3f}m | "
                  f"方向: {vel['direction_deg']:.1f}° | "
                  f"速度: {vel['velocity_magnitude']*100:.1f}cm/s")
            print(f"    速度向量: [{vel['velocity_vector'][0]*100:.1f}, {vel['velocity_vector'][1]*100:.1f}] cm/s")
            print()

        # 统计信息
        total_distance = sum(v['distance'] for v in velocities)
        avg_velocity = sum(v['velocity_magnitude'] for v in velocities) / len(velocities)
        print(f"📊 路径统计:")
        print(f"   总距离: {total_distance:.3f}m")
        print(f"   平均速度: {avg_velocity*100:.1f}cm/s")
        print(f"   段数: {len(velocities)}")

    def step(self) -> Dict[str, Any]:
        self.frame_idx += 1
        if not self.check_frame_limits():
            return None

        position, orientation = self.robot.get_world_pose()
        yaw = quat_to_euler_angles(orientation, extrinsic=False)[2]
        current_pose = np.array([position[0], position[1], yaw])

        camera_data, display_data = self.get_camera_data()

        # 计算路径段速度信息
        segment_velocities = self._calculate_segment_velocities()

        # 获取当前正在执行的段速度信息
        current_segment_velocity = self.get_current_segment_velocity(
            current_position=[current_pose[0], current_pose[1]]
        )

        state = {
            "current_pose": current_pose,
            "start_point": self.current_start,
            "end_point": self.current_end,
            "waypoints": self.current_path,
            "segment_velocities": segment_velocities,  # 所有路径段速度信息
            "current_segment_velocity": current_segment_velocity,  # 当前段速度信息
            "camera_data": camera_data,
            "camera_display": display_data,
            "done": self.reset_needed,
            "frame_idx": self.frame_idx,
        }
        return state

    def on_task_complete(self, success: bool) -> None:
        """到达后切换到下一个起终点。"""
        self.reset_needed = True
        if success and self._has_remaining_goals():
            # 任务完成后推进队列指针
            self.current_goal_idx += 1

    # ---------- 队列操作 ----------
    def set_goal_pairs(self, pairs: List[Any]) -> None:
        """重置并加载新的起终点队列。"""
        self.goal_pairs = self._normalize_pairs(pairs)
        self.current_goal_idx = 0

        # 输出队列信息
        if len(self.goal_pairs) > 0:
            print(f"\n🚀 加载导航任务队列（共 {len(self.goal_pairs)} 个任务）:")
            for i, (start, end) in enumerate(self.goal_pairs):
                connection_info = ""
                if i > 0:
                    prev_end = self.goal_pairs[i-1][1]
                    if np.linalg.norm(np.array(prev_end) - np.array(start)) < 0.1:
                        connection_info = " 🔗 (与前任务连接)"
                    else:
                        connection_info = " ⏸️  (独立任务)"
                print(f"  任务 {i+1}: 起点 {start} -> 终点 {end}{connection_info}")
            print()
        else:
            print("Warning: goal_pairs 解析后为空，请检查配置格式")

    def append_goal_pairs(self, pairs: List[Any]) -> None:
        """在队列尾部追加起终点。"""
        self.goal_pairs.extend(self._normalize_pairs(pairs))

    def _normalize_pairs(self, pairs: List[Any]) -> List[Tuple[list, list]]:
        """接受 dict 或二元列表形式，统一为 (start,end)。
        
        支持 OmegaConf DictConfig 类型（Hydra 配置系统使用）。
        """
        normalized = []
        for item in pairs:
            # 处理 OmegaConf DictConfig 或普通 dict
            # 使用 hasattr 检查是否有 'get' 方法或 'start' 属性
            if hasattr(item, 'get') and ('start' in item or hasattr(item, 'start')):
                try:
                    start = item.get('start') if hasattr(item, 'get') else item['start']
                    end = item.get('end') if hasattr(item, 'get') else item['end']
                    # 转换为普通列表
                    normalized.append((list(start), list(end)))
                except (KeyError, AttributeError) as e:
                    print(f"Warning: 无法解析字典项 {item}: {e}")
            # 处理列表/元组格式
            elif isinstance(item, (list, tuple)) and len(item) == 2:
                if all(isinstance(p, (list, tuple)) for p in item):
                    normalized.append((list(item[0]), list(item[1])))
        return normalized

    def _has_remaining_goals(self) -> bool:
        return self.current_goal_idx < len(self.goal_pairs)

    def _apply_next_goal_until_success(self) -> bool:
        """尝试从当前指针开始依次规划，返回是否成功设置路径。"""
        nav_scene = self.navigation_assets[0]
        idx = self.current_goal_idx
        while idx < len(self.goal_pairs):
            start_point, end_point = self.goal_pairs[idx]
            if self._plan_and_set_path(nav_scene, start_point, end_point):
                # 更新指针为已使用的索引
                self.current_goal_idx = idx
                return True
            idx += 1
        return False

    # ---------- 路径规划与生成 ----------
    def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list) -> bool:
        """给定起终点，调用 A* 规划并设置路径与机器人初始位姿。"""
        task_info = {"asset": nav_scene, "start": start_point, "end": end_point}
        path_result = plan_navigation_path(task_info)
        if path_result is None:
            print(f"Warning: 路径规划失败，起点 {start_point} 终点 {end_point}")
            return False

        merged_path_real, _, _ = path_result
        waypoints = []
        for i, (x, y, _) in enumerate(merged_path_real):
            if i < len(merged_path_real) - 1:
                nx, ny, _ = merged_path_real[i + 1]
                theta = np.arctan2(ny - y, nx - x)
            else:
                theta = waypoints[-1][2] if waypoints else 0.0
            waypoints.append([x, y, theta])

        self.current_start = start_point
        self.current_end = end_point
        self.current_path = waypoints

        # 输出当前执行任务信息
        task_num = self.current_goal_idx + 1
        total_tasks = len(self.goal_pairs)
        print(f"\n📍 执行任务 {task_num}/{total_tasks}:")
        print(f"   起点: [{start_point[0]:.2f}, {start_point[1]:.2f}]")
        print(f"   终点: [{end_point[0]:.2f}, {end_point[1]:.2f}]")
        print(f"   路径点数: {len(waypoints)}")
        print(f"   预计距离: {np.sqrt((end_point[0]-start_point[0])**2 + (end_point[1]-start_point[1])**2):.2f} m")

        # 显示路径段速度信息
        self.print_velocity_info()

        initial_position = np.array([start_point[0], start_point[1], 0.0])
        self.robot.set_world_pose(position=initial_position)
        return True

    def _generate_random_navigation_task(self) -> bool:
        """保持旧版行为：随机起终点 + A* 规划。"""
        nav_scene = self.navigation_assets[0]
        max_attempts = 100

        for _ in range(max_attempts):
            start_point, end_point = self._generate_random_points(
                nav_scene["x_bounds"], nav_scene["y_bounds"], self.grid
            )
            if start_point is None or end_point is None:
                continue
            if self._plan_and_set_path(nav_scene, start_point, end_point):
                return True

        print("Warning: 无法生成有效随机路径。")
        return False

    def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
        W = len(grid[0])
        H = len(grid)

        for _ in range(attempts):
            start_x = np.random.uniform(x_bounds[0], x_bounds[1])
            start_y = np.random.uniform(y_bounds[0], y_bounds[1])
            end_x = np.random.uniform(x_bounds[0], x_bounds[1])
            end_y = np.random.uniform(y_bounds[0], y_bounds[1])

            i_start, j_start = real_to_grid(start_x, start_y, x_bounds, y_bounds, (W, H))
            i_end, j_end = real_to_grid(end_x, end_y, x_bounds, y_bounds, (W, H))

            if grid[i_start][j_start] == 0 and grid[i_end][j_end] == 0:
                return [start_x, start_y], [end_x, end_y]

        return None, None
