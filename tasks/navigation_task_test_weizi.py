# # # # # import numpy as np
# # # # # import yaml
# # # # # from typing import Dict, Any, List, Tuple, Optional

# # # # # from .base_task import BaseTask
# # # # # from utils.a_star import plan_navigation_path, real_to_grid, load_grid
# # # # # from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat


# # # # # class NavigationTaskTestWeizi(BaseTask):
# # # # #     """
# # # # #     顺序式导航任务：
# # # # #     - 支持外部传入多个有序起终点对（队列）
# # # # #     - 每次到达后自动切换到下一个起终点
# # # # #     - 若没有提供队列则回退为随机起终点 + A* 规划（与旧版一致）
    
# # # # #     配置/注入方式（两种二选一）：
# # # # #     1) Hydra 配置传入 cfg.task.goal_pairs:
# # # # #         goal_pairs:
# # # # #           - start: [x1, y1]
# # # # #             end:   [x2, y2]
# # # # #           - [ [x3, y3], [x4, y4] ]   # 也接受这种二元列表形式
# # # # #     2) 运行时调用 append_goal_pairs / set_goal_pairs 追加或重置队列。
# # # # #     """

# # # # #     def __init__(self, cfg, world, stage, robot):
# # # # #         self.navigation_assets: List[dict] = []
# # # # #         self.grid = None
# # # # #         self.current_start: Optional[list] = None
# # # # #         self.current_end: Optional[list] = None
# # # # #         self.current_path: Optional[List[List[float]]] = None

# # # # #         # 有序起终点队列
# # # # #         self.goal_pairs: List[Tuple[list, list]] = []
# # # # #         self.current_goal_idx: int = 0

# # # # #         super().__init__(cfg, world, stage, robot)

# # # # #         # 从配置预加载队列（如果有）
# # # # #         if hasattr(self.cfg, "task") and hasattr(self.cfg.task, "goal_pairs"):
# # # # #             self.set_goal_pairs(self.cfg.task.goal_pairs)

# # # # #     # ---------- 场景与地图 ----------
# # # # #     def setup_objects(self) -> None:
# # # # #         super().setup_objects()

# # # # #         if hasattr(self.cfg.task, "navigation_config_path"):
# # # # #             with open(self.cfg.task.navigation_config_path, "r") as f:
# # # # #                 config = yaml.safe_load(f)
# # # # #                 self.navigation_assets = config.get("assets", [])

# # # # #         if self.navigation_assets:
# # # # #             nav_scene = self.navigation_assets[0]
# # # # #             self.grid, self.W, self.H = load_grid(nav_scene["barrier_image_path"])

# # # # #     # ---------- 任务流程 ----------
# # # # #     def reset(self) -> None:
# # # # #         super().reset()
# # # # #         self.robot.initialize()

# # # # #         if not self.navigation_assets:
# # # # #             return

# # # # #         # 优先使用外部队列；若为空则回退随机生成
# # # # #         if self._has_remaining_goals():
# # # # #             success = self._apply_next_goal_until_success()
# # # # #             if not success:
# # # # #                 print("Warning: 提供的起终点均规划失败，回退为随机任务。")
# # # # #                 self._generate_random_navigation_task()
# # # # #         else:
# # # # #             self._generate_random_navigation_task()

# # # # #     def _calculate_segment_velocities(self) -> List[Dict[str, Any]]:
# # # # #         """
# # # # #         计算路径中每个段的速度大小和方向信息

# # # # #         Returns:
# # # # #             List[Dict]: 每个路径段的速度信息
# # # # #                 - segment_idx: 段索引
# # # # #                 - start_point: 起点坐标
# # # # #                 - end_point: 终点坐标
# # # # #                 - distance: 段长度
# # # # #                 - direction: 方向角度（弧度）
# # # # #                 - direction_deg: 方向角度（度）
# # # # #                 - velocity_magnitude: 速度大小（基于距离比例）
# # # # #         """
# # # # #         if self.current_path is None or len(self.current_path) < 2:
# # # # #             return []

# # # # #         velocities = []
# # # # #         # 使用与控制器相同的速度计算逻辑
# # # # #         max_linear_speed = 0.02  # 与控制器默认值一致

# # # # #         for i in range(len(self.current_path) - 1):
# # # # #             start_point = self.current_path[i]
# # # # #             end_point = self.current_path[i + 1]

# # # # #             # 计算距离
# # # # #             dx = end_point[0] - start_point[0]
# # # # #             dy = end_point[1] - start_point[1]
# # # # #             distance = np.sqrt(dx**2 + dy**2)

# # # # #             # 计算方向
# # # # #             direction = np.arctan2(dy, dx)
# # # # #             direction_deg = np.degrees(direction)

# # # # #             # 计算速度大小（与控制器逻辑一致）
# # # # #             # speed = min(distance * 0.2, max_linear_speed)
# # # # #             velocity_magnitude = min(distance * 0.2, max_linear_speed)

# # # # #             segment_info = {
# # # # #                 "segment_idx": i,
# # # # #                 "start_point": [start_point[0], start_point[1]],
# # # # #                 "end_point": [end_point[0], end_point[1]],
# # # # #                 "distance": distance,
# # # # #                 "direction": direction,
# # # # #                 "direction_deg": direction_deg,
# # # # #                 "velocity_magnitude": velocity_magnitude,
# # # # #                 "velocity_vector": [
# # # # #                     velocity_magnitude * np.cos(direction),
# # # # #                     velocity_magnitude * np.sin(direction)
# # # # #                 ]
# # # # #             }
# # # # #             velocities.append(segment_info)

# # # # #         return velocities

# # # # #     def get_current_segment_velocity(self, current_position: Optional[List[float]] = None) -> Optional[Dict[str, Any]]:
# # # # #         """
# # # # #         获取当前正在执行的路径段速度信息

# # # # #         Args:
# # # # #             current_position: 当前机器人位置 [x, y]，如果不提供则使用第一个段

# # # # #         Returns:
# # # # #             Optional[Dict]: 当前段的速度信息，如果没有路径则返回None
# # # # #         """
# # # # #         if self.current_path is None or len(self.current_path) < 2:
# # # # #             return None

# # # # #         velocities = self._calculate_segment_velocities()
# # # # #         if not velocities:
# # # # #             return None

# # # # #         if current_position is None:
# # # # #             # 如果没有提供当前位置，返回第一个段
# # # # #             return velocities[0]

# # # # #         # 根据当前位置找到最近的路径段
# # # # #         min_distance = float('inf')
# # # # #         current_segment = None

# # # # #         for vel in velocities:
# # # # #             # 计算当前位置到段起点的距离
# # # # #             start_point = vel['start_point']
# # # # #             dist_to_start = np.sqrt(
# # # # #                 (current_position[0] - start_point[0])**2 +
# # # # #                 (current_position[1] - start_point[1])**2
# # # # #             )

# # # # #             if dist_to_start < min_distance:
# # # # #                 min_distance = dist_to_start
# # # # #                 current_segment = vel

# # # # #         return current_segment

# # # # #     def print_velocity_info(self) -> None:
# # # # #         """
# # # # #         打印当前路径的详细速度信息
# # # # #         """
# # # # #         velocities = self._calculate_segment_velocities()
# # # # #         if not velocities:
# # # # #             print("⚠️  没有可用的路径速度信息")
# # # # #             return

# # # # #         print(f"\n🚀 路径段速度信息 (共 {len(velocities)} 段):")
# # # # #         print("=" * 80)

# # # # #         for vel in velocities:
# # # # #             print(f"段 {vel['segment_idx']+1}: "
# # # # #                   f"[{vel['start_point'][0]:.3f}, {vel['start_point'][1]:.3f}] → "
# # # # #                   f"[{vel['end_point'][0]:.3f}, {vel['end_point'][1]:.3f}]")
# # # # #             print(f"    距离: {vel['distance']:.3f}m | "
# # # # #                   f"方向: {vel['direction_deg']:.1f}° | "
# # # # #                   f"速度: {vel['velocity_magnitude']*100:.1f}cm/s")
# # # # #             print(f"    速度向量: [{vel['velocity_vector'][0]*100:.1f}, {vel['velocity_vector'][1]*100:.1f}] cm/s")
# # # # #             print()

# # # # #         # 统计信息
# # # # #         total_distance = sum(v['distance'] for v in velocities)
# # # # #         avg_velocity = sum(v['velocity_magnitude'] for v in velocities) / len(velocities)
# # # # #         print(f"📊 路径统计:")
# # # # #         print(f"   总距离: {total_distance:.3f}m")
# # # # #         print(f"   平均速度: {avg_velocity*100:.1f}cm/s")
# # # # #         print(f"   段数: {len(velocities)}")

# # # # #     def step(self) -> Dict[str, Any]:
# # # # #         self.frame_idx += 1
# # # # #         if not self.check_frame_limits():
# # # # #             return None

# # # # #         position, orientation = self.robot.get_world_pose()
# # # # #         yaw = quat_to_euler_angles(orientation, extrinsic=False)[2]
# # # # #         current_pose = np.array([position[0], position[1], yaw])

# # # # #         camera_data, display_data = self.get_camera_data()

# # # # #         # 计算路径段速度信息
# # # # #         segment_velocities = self._calculate_segment_velocities()

# # # # #         # 获取当前正在执行的段速度信息
# # # # #         current_segment_velocity = self.get_current_segment_velocity(
# # # # #             current_position=[current_pose[0], current_pose[1]]
# # # # #         )

# # # # #         state = {
# # # # #             "current_pose": current_pose,
# # # # #             "start_point": self.current_start,
# # # # #             "end_point": self.current_end,
# # # # #             "waypoints": self.current_path,
# # # # #             "segment_velocities": segment_velocities,  # 所有路径段速度信息
# # # # #             "current_segment_velocity": current_segment_velocity,  # 当前段速度信息
# # # # #             "camera_data": camera_data,
# # # # #             "camera_display": display_data,
# # # # #             "done": self.reset_needed,
# # # # #             "frame_idx": self.frame_idx,
# # # # #         }
# # # # #         return state

# # # # #     def on_task_complete(self, success: bool) -> None:
# # # # #         """到达后切换到下一个起终点。"""
# # # # #         self.reset_needed = True
# # # # #         if success and self._has_remaining_goals():
# # # # #             # 任务完成后推进队列指针
# # # # #             self.current_goal_idx += 1

# # # # #     # ---------- 队列操作 ----------
# # # # #     def set_goal_pairs(self, pairs: List[Any]) -> None:
# # # # #         """重置并加载新的起终点队列。"""
# # # # #         self.goal_pairs = self._normalize_pairs(pairs)
# # # # #         self.current_goal_idx = 0

# # # # #         # 输出队列信息
# # # # #         if len(self.goal_pairs) > 0:
# # # # #             print(f"\n🚀 加载导航任务队列（共 {len(self.goal_pairs)} 个任务）:")
# # # # #             for i, (start, end) in enumerate(self.goal_pairs):
# # # # #                 connection_info = ""
# # # # #                 if i > 0:
# # # # #                     prev_end = self.goal_pairs[i-1][1]
# # # # #                     if np.linalg.norm(np.array(prev_end[:2]) - np.array(start[:2])) < 0.1:
# # # # #                         connection_info = " 🔗 (与前任务连接)"
# # # # #                     else:
# # # # #                         connection_info = " ⏸️  (独立任务)"

# # # # #                 # 格式化显示位姿信息
# # # # #                 start_str = f"[{start[0]:.2f}, {start[1]:.2f}"
# # # # #                 if len(start) > 2:
# # # # #                     start_str += f", {start[2]*180/np.pi:.1f}°]"
# # # # #                 else:
# # # # #                     start_str += "]"

# # # # #                 end_str = f"[{end[0]:.2f}, {end[1]:.2f}"
# # # # #                 if len(end) > 2:
# # # # #                     end_str += f", {end[2]*180/np.pi:.1f}°]"
# # # # #                 else:
# # # # #                     end_str += "]"

# # # # #                 print(f"  任务 {i+1}: 起点 {start_str} -> 终点 {end_str}{connection_info}")
# # # # #             print()
# # # # #         else:
# # # # #             print("Warning: goal_pairs 解析后为空，请检查配置格式")

# # # # #     def append_goal_pairs(self, pairs: List[Any]) -> None:
# # # # #         """在队列尾部追加起终点。"""
# # # # #         self.goal_pairs.extend(self._normalize_pairs(pairs))

# # # # #     def _normalize_pairs(self, pairs: List[Any]) -> List[Tuple[list, list]]:
# # # # #         """接受 dict 或二元列表形式，统一为 (start,end)。

# # # # #         支持 OmegaConf DictConfig 类型（Hydra 配置系统使用）。
# # # # #         支持带角度信息的坐标格式：[x, y, theta] 或 [x, y]
# # # # #         """
# # # # #         normalized = []
# # # # #         for item in pairs:
# # # # #             # 处理 OmegaConf DictConfig 或普通 dict
# # # # #             # 使用 hasattr 检查是否有 'get' 方法或 'start' 属性
# # # # #             if hasattr(item, 'get') and ('start' in item or hasattr(item, 'start')):
# # # # #                 try:
# # # # #                     start = item.get('start') if hasattr(item, 'get') else item['start']
# # # # #                     end = item.get('end') if hasattr(item, 'get') else item['end']

# # # # #                     # 确保start和end是3元素（x, y, theta）或2元素（x, y）
# # # # #                     start = self._ensure_pose_format(list(start))
# # # # #                     end = self._ensure_pose_format(list(end))

# # # # #                     normalized.append((start, end))
# # # # #                 except (KeyError, AttributeError) as e:
# # # # #                     print(f"Warning: 无法解析字典项 {item}: {e}")
# # # # #             # 处理列表/元组格式
# # # # #             elif isinstance(item, (list, tuple)) and len(item) == 2:
# # # # #                 if all(isinstance(p, (list, tuple)) for p in item):
# # # # #                     start = self._ensure_pose_format(list(item[0]))
# # # # #                     end = self._ensure_pose_format(list(item[1]))
# # # # #                     normalized.append((start, end))
# # # # #         return normalized

# # # # #     def _ensure_pose_format(self, pose: list) -> list:
# # # # #         """
# # # # #         确保位姿格式为 [x, y, theta]，如果只有 [x, y] 则添加默认角度 0.0

# # # # #         Args:
# # # # #             pose: 输入的位姿列表

# # # # #         Returns:
# # # # #             标准化的3元素位姿列表 [x, y, theta]
# # # # #         """
# # # # #         if len(pose) == 2:
# # # # #             # 如果只有坐标，添加默认角度
# # # # #             return [pose[0], pose[1], 0.0]
# # # # #         elif len(pose) == 3:
# # # # #             # 已经是完整格式
# # # # #             return pose
# # # # #         else:
# # # # #             raise ValueError(f"位姿格式错误，应为 [x, y] 或 [x, y, theta]，得到: {pose}")

# # # # #     def _has_remaining_goals(self) -> bool:
# # # # #         return self.current_goal_idx < len(self.goal_pairs)

# # # # #     def _apply_next_goal_until_success(self) -> bool:
# # # # #         """尝试从当前指针开始依次规划，返回是否成功设置路径。"""
# # # # #         nav_scene = self.navigation_assets[0]
# # # # #         idx = self.current_goal_idx
# # # # #         while idx < len(self.goal_pairs):
# # # # #             start_point, end_point = self.goal_pairs[idx]
# # # # #             if self._plan_and_set_path(nav_scene, start_point, end_point):
# # # # #                 # 更新指针为已使用的索引
# # # # #                 self.current_goal_idx = idx
# # # # #                 return True
# # # # #             idx += 1
# # # # #         return False

# # # # #     # ---------- 路径规划与生成 ----------
# # # # #     def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list) -> bool:
# # # # #         """给定起终点，调用 A* 规划并设置路径与机器人初始位姿。"""
# # # # #         # 只使用坐标信息进行路径规划，忽略角度（与 navigation_task_new_cp_cp.py 保持一致）
# # # # #         start_coords = start_point[:2]  # [x, y]
# # # # #         end_coords = end_point[:2]      # [x, y]

# # # # #         task_info = {"asset": nav_scene, "start": start_coords, "end": end_coords}
# # # # #         path_result = plan_navigation_path(task_info)
# # # # #         if path_result is None:
# # # # #             print(f"Warning: 路径规划失败，起点 {start_coords} 终点 {end_coords}")
# # # # #             return False

# # # # #         merged_path_real, _ = path_result
# # # # #         waypoints = []
# # # # #         for i, (x, y, _) in enumerate(merged_path_real):
# # # # #             if i < len(merged_path_real) - 1:
# # # # #                 nx, ny, _ = merged_path_real[i + 1]
# # # # #                 theta = np.arctan2(ny - y, nx - x)
# # # # #             else:
# # # # #                 theta = waypoints[-1][2] if waypoints else 0.0
# # # # #             waypoints.append([x, y, theta])

# # # # #         self.current_start = start_point  # 保存完整位姿信息
# # # # #         self.current_end = end_point      # 保存完整位姿信息
# # # # #         self.current_path = waypoints

# # # # #         # 输出当前执行任务信息
# # # # #         task_num = self.current_goal_idx + 1
# # # # #         total_tasks = len(self.goal_pairs)

# # # # #         # 格式化显示位姿信息
# # # # #         start_display = f"[{start_point[0]:.2f}, {start_point[1]:.2f}"
# # # # #         if len(start_point) > 2:
# # # # #             start_display += f", {start_point[2]*180/np.pi:.1f}°]"
# # # # #         else:
# # # # #             start_display += "]"

# # # # #         end_display = f"[{end_point[0]:.2f}, {end_point[1]:.2f}"
# # # # #         if len(end_point) > 2:
# # # # #             end_display += f", {end_point[2]*180/np.pi:.1f}°]"
# # # # #         else:
# # # # #             end_display += "]"

# # # # #         print(f"\n📍 执行任务 {task_num}/{total_tasks}:")
# # # # #         print(f"   起点: {start_display}")
# # # # #         print(f"   终点: {end_display}")
# # # # #         print(f"   路径点数: {len(waypoints)}")
# # # # #         print(f"   预计距离: {np.sqrt((end_point[0]-start_point[0])**2 + (end_point[1]-start_point[1])**2):.2f} m")

# # # # #         # 显示路径段速度信息
# # # # #         self.print_velocity_info()

# # # # #         # 设置机器人初始位姿
# # # # #         # 与 navigation_task_new_cp_cp.py 保持一致：只设置位置，不设置朝向
# # # # #         # 让机器人使用路径规划计算出的朝向（waypoints[0][2]）
# # # # #         initial_position = np.array([start_point[0], start_point[1], 0.0])
        
# # # # #         # 使用路径第一个点的朝向，确保与路径规划方向一致
# # # # #         # 这样机器人会朝向路径的下一个点，而不是配置中可能不准确的角度
# # # # #         initial_yaw = waypoints[0][2] if waypoints else 0.0
        
# # # # #         # 将欧拉角 [roll, pitch, yaw] 转换为四元数
# # # # #         # 对于地面机器人，roll 和 pitch 通常为 0，只有 yaw（绕 Z 轴旋转）
# # # # #         initial_orientation = euler_angles_to_quat(np.array([0.0, 0.0, initial_yaw]))
        
# # # # #         self.robot.set_world_pose(
# # # # #             position=initial_position,
# # # # #             orientation=initial_orientation
# # # # #         )
# # # # #         return True

# # # # #     def _generate_random_navigation_task(self) -> bool:
# # # # #         """保持旧版行为：随机起终点 + A* 规划。"""
# # # # #         nav_scene = self.navigation_assets[0]
# # # # #         max_attempts = 100

# # # # #         for _ in range(max_attempts):
# # # # #             start_point, end_point = self._generate_random_points(
# # # # #                 nav_scene["x_bounds"], nav_scene["y_bounds"], self.grid
# # # # #             )
# # # # #             if start_point is None or end_point is None:
# # # # #                 continue
# # # # #             if self._plan_and_set_path(nav_scene, start_point, end_point):
# # # # #                 return True

# # # # #         print("Warning: 无法生成有效随机路径。")
# # # # #         return False

# # # # #     def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
# # # # #         W = len(grid[0])
# # # # #         H = len(grid)

# # # # #         for _ in range(attempts):
# # # # #             start_x = np.random.uniform(x_bounds[0], x_bounds[1])
# # # # #             start_y = np.random.uniform(y_bounds[0], y_bounds[1])
# # # # #             end_x = np.random.uniform(x_bounds[0], x_bounds[1])
# # # # #             end_y = np.random.uniform(y_bounds[0], y_bounds[1])

# # # # #             i_start, j_start = real_to_grid(start_x, start_y, x_bounds, y_bounds, (W, H))
# # # # #             i_end, j_end = real_to_grid(end_x, end_y, x_bounds, y_bounds, (W, H))

# # # # #             if grid[i_start][j_start] == 0 and grid[i_end][j_end] == 0:
# # # # #                 return [start_x, start_y], [end_x, end_y]

# # # # #         return None, None
# # # # import numpy as np
# # # # import yaml
# # # # from typing import Dict, Any, List, Tuple, Optional

# # # # from .base_task import BaseTask
# # # # from utils.a_star import plan_navigation_path, real_to_grid, load_grid

# # # # # 兼容性导入：处理不同 Isaac Sim 版本路径差异
# # # # try:
# # # #     from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
# # # # except ImportError:
# # # #     from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat


# # # # class NavigationTaskTestWeizi(BaseTask):
# # # #     """
# # # #     顺序式导航任务 (修复增强版)：
# # # #     - 修复了Z轴穿模问题
# # # #     - 修复了初始朝向覆盖用户配置的问题
# # # #     - 修正了观测数据的速度上限
# # # #     """

# # # #     def __init__(self, cfg, world, stage, robot):
# # # #         self.navigation_assets: List[dict] = []
# # # #         self.grid = None
# # # #         self.current_start: Optional[list] = None
# # # #         self.current_end: Optional[list] = None
# # # #         self.current_path: Optional[List[List[float]]] = None

# # # #         # 有序起终点队列
# # # #         self.goal_pairs: List[Tuple[list, list]] = []
# # # #         self.current_goal_idx: int = 0

# # # #         super().__init__(cfg, world, stage, robot)

# # # #         # 从配置预加载队列（如果有）
# # # #         if hasattr(self.cfg, "task") and hasattr(self.cfg.task, "goal_pairs"):
# # # #             self.set_goal_pairs(self.cfg.task.goal_pairs)

# # # #     # ---------- 场景与地图 ----------
# # # #     def setup_objects(self) -> None:
# # # #         super().setup_objects()

# # # #         if hasattr(self.cfg.task, "navigation_config_path"):
# # # #             with open(self.cfg.task.navigation_config_path, "r") as f:
# # # #                 config = yaml.safe_load(f)
# # # #                 self.navigation_assets = config.get("assets", [])

# # # #         if self.navigation_assets:
# # # #             nav_scene = self.navigation_assets[0]
# # # #             self.grid, self.W, self.H = load_grid(nav_scene["barrier_image_path"])

# # # #     # ---------- 任务流程 ----------
# # # #     def reset(self) -> None:
# # # #         super().reset()
# # # #         self.robot.initialize()

# # # #         if not self.navigation_assets:
# # # #             return

# # # #         # 优先使用外部队列；若为空则回退随机生成
# # # #         if self._has_remaining_goals():
# # # #             success = self._apply_next_goal_until_success()
# # # #             if not success:
# # # #                 print("Warning: 提供的起终点均规划失败，回退为随机任务。")
# # # #                 self._generate_random_navigation_task()
# # # #         else:
# # # #             self._generate_random_navigation_task()
# # # #         if self.world:
# # # #             self.world.set(render=False)

# # # #     def _calculate_segment_velocities(self) -> List[Dict[str, Any]]:
# # # #         """
# # # #         计算路径中每个段的速度大小和方向信息
# # # #         """
# # # #         if self.current_path is None or len(self.current_path) < 2:
# # # #             return []

# # # #         velocities = []
# # # #         # 【修改点 3】修正速度上限，与控制器保持一致 (1.0 m/s)，而不是 0.02
# # # #         max_linear_speed = 1.0 

# # # #         for i in range(len(self.current_path) - 1):
# # # #             start_point = self.current_path[i]
# # # #             end_point = self.current_path[i + 1]

# # # #             # 计算距离
# # # #             dx = end_point[0] - start_point[0]
# # # #             dy = end_point[1] - start_point[1]
# # # #             distance = np.sqrt(dx**2 + dy**2)

# # # #             # 计算方向
# # # #             direction = np.arctan2(dy, dx)
# # # #             direction_deg = np.degrees(direction)

# # # #             # 计算速度大小（逻辑与控制器类似，但上限已修正）
# # # #             # 注意：这里仅作为观测信息提供给Agent，实际控制由Controller决定
# # # #             velocity_magnitude = min(distance * 2.0, max_linear_speed) # 稍微调高比例系数

# # # #             segment_info = {
# # # #                 "segment_idx": i,
# # # #                 "start_point": [start_point[0], start_point[1]],
# # # #                 "end_point": [end_point[0], end_point[1]],
# # # #                 "distance": distance,
# # # #                 "direction": direction,
# # # #                 "direction_deg": direction_deg,
# # # #                 "velocity_magnitude": velocity_magnitude,
# # # #                 "velocity_vector": [
# # # #                     velocity_magnitude * np.cos(direction),
# # # #                     velocity_magnitude * np.sin(direction)
# # # #                 ]
# # # #             }
# # # #             velocities.append(segment_info)

# # # #         return velocities

# # # #     def get_current_segment_velocity(self, current_position: Optional[List[float]] = None) -> Optional[Dict[str, Any]]:
# # # #         """获取当前正在执行的路径段速度信息"""
# # # #         if self.current_path is None or len(self.current_path) < 2:
# # # #             return None

# # # #         velocities = self._calculate_segment_velocities()
# # # #         if not velocities:
# # # #             return None

# # # #         if current_position is None:
# # # #             return velocities[0]

# # # #         # 根据当前位置找到最近的路径段
# # # #         min_distance = float('inf')
# # # #         current_segment = None

# # # #         for vel in velocities:
# # # #             start_point = vel['start_point']
# # # #             dist_to_start = np.sqrt(
# # # #                 (current_position[0] - start_point[0])**2 +
# # # #                 (current_position[1] - start_point[1])**2
# # # #             )

# # # #             if dist_to_start < min_distance:
# # # #                 min_distance = dist_to_start
# # # #                 current_segment = vel

# # # #         return current_segment

# # # #     def print_velocity_info(self) -> None:
# # # #         """打印当前路径的详细速度信息"""
# # # #         velocities = self._calculate_segment_velocities()
# # # #         if not velocities:
# # # #             print("⚠️  没有可用的路径速度信息")
# # # #             return

# # # #         print(f"\n🚀 路径段速度信息 (共 {len(velocities)} 段):")
# # # #         print("=" * 80)

# # # #         for vel in velocities:
# # # #             print(f"段 {vel['segment_idx']+1}: "
# # # #                   f"[{vel['start_point'][0]:.3f}, {vel['start_point'][1]:.3f}] → "
# # # #                   f"[{vel['end_point'][0]:.3f}, {vel['end_point'][1]:.3f}]")
# # # #             print(f"    距离: {vel['distance']:.3f}m | "
# # # #                   f"方向: {vel['direction_deg']:.1f}° | "
# # # #                   f"参考速度: {vel['velocity_magnitude']*100:.1f}cm/s")
# # # #             print()

# # # #     def step(self) -> Dict[str, Any]:
# # # #         self.frame_idx += 1
# # # #         if not self.check_frame_limits():
# # # #             return None

# # # #         position, orientation = self.robot.get_world_pose()
# # # #         yaw = quat_to_euler_angles(orientation, extrinsic=False)[2]
# # # #         current_pose = np.array([position[0], position[1], yaw])

# # # #         camera_data, display_data = self.get_camera_data()

# # # #         segment_velocities = self._calculate_segment_velocities()
# # # #         current_segment_velocity = self.get_current_segment_velocity(
# # # #             current_position=[current_pose[0], current_pose[1]]
# # # #         )

# # # #         state = {
# # # #             "current_pose": current_pose,
# # # #             "start_point": self.current_start,
# # # #             "end_point": self.current_end,
# # # #             "waypoints": self.current_path,
# # # #             "segment_velocities": segment_velocities,
# # # #             "current_segment_velocity": current_segment_velocity,
# # # #             "camera_data": camera_data,
# # # #             "camera_display": display_data,
# # # #             "done": self.reset_needed,
# # # #             "frame_idx": self.frame_idx,
# # # #         }
# # # #         return state

# # # #     def on_task_complete(self, success: bool) -> None:
# # # #         """到达后切换到下一个起终点"""
# # # #         self.reset_needed = True
# # # #         if success and self._has_remaining_goals():
# # # #             self.current_goal_idx += 1

# # # #     # ---------- 队列操作 ----------
# # # #     def set_goal_pairs(self, pairs: List[Any]) -> None:
# # # #         """重置并加载新的起终点队列"""
# # # #         self.goal_pairs = self._normalize_pairs(pairs)
# # # #         self.current_goal_idx = 0

# # # #         if len(self.goal_pairs) > 0:
# # # #             print(f"\n🚀 加载导航任务队列（共 {len(self.goal_pairs)} 个任务）:")
# # # #             for i, (start, end) in enumerate(self.goal_pairs):
# # # #                 start_str = f"[{start[0]:.2f}, {start[1]:.2f}"
# # # #                 start_str += f", {start[2]*180/np.pi:.1f}°]" if len(start) > 2 else "]"
# # # #                 print(f"  任务 {i+1}: 起点 {start_str} -> 终点 ...")
# # # #             print()
# # # #         else:
# # # #             print("Warning: goal_pairs 解析后为空，请检查配置格式")

# # # #     def append_goal_pairs(self, pairs: List[Any]) -> None:
# # # #         self.goal_pairs.extend(self._normalize_pairs(pairs))

# # # #     def _normalize_pairs(self, pairs: List[Any]) -> List[Tuple[list, list]]:
# # # #         normalized = []
# # # #         for item in pairs:
# # # #             if hasattr(item, 'get') and ('start' in item or hasattr(item, 'start')):
# # # #                 try:
# # # #                     start = item.get('start') if hasattr(item, 'get') else item['start']
# # # #                     end = item.get('end') if hasattr(item, 'get') else item['end']
# # # #                     start = self._ensure_pose_format(list(start))
# # # #                     end = self._ensure_pose_format(list(end))
# # # #                     normalized.append((start, end))
# # # #                 except (KeyError, AttributeError) as e:
# # # #                     print(f"Warning: 无法解析字典项 {item}: {e}")
# # # #             elif isinstance(item, (list, tuple)) and len(item) == 2:
# # # #                 if all(isinstance(p, (list, tuple)) for p in item):
# # # #                     start = self._ensure_pose_format(list(item[0]))
# # # #                     end = self._ensure_pose_format(list(item[1]))
# # # #                     normalized.append((start, end))
# # # #         return normalized

# # # #     def _ensure_pose_format(self, pose: list) -> list:
# # # #         if len(pose) == 2:
# # # #             return [pose[0], pose[1], 0.0]
# # # #         elif len(pose) == 3:
# # # #             return pose
# # # #         else:
# # # #             raise ValueError(f"位姿格式错误，应为 [x, y] 或 [x, y, theta]，得到: {pose}")

# # # #     def _has_remaining_goals(self) -> bool:
# # # #         return self.current_goal_idx < len(self.goal_pairs)

# # # #     def _apply_next_goal_until_success(self) -> bool:
# # # #         nav_scene = self.navigation_assets[0]
# # # #         idx = self.current_goal_idx
# # # #         while idx < len(self.goal_pairs):
# # # #             start_point, end_point = self.goal_pairs[idx]
# # # #             if self._plan_and_set_path(nav_scene, start_point, end_point):
# # # #                 self.current_goal_idx = idx
# # # #                 return True
# # # #             idx += 1
# # # #         return False

# # # #     # ---------- 路径规划与生成 (核心修复区域) ----------
# # # #     def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list) -> bool:
# # # #         """给定起终点，调用 A* 规划并设置路径与机器人初始位姿。"""
# # # #         # 只使用坐标信息进行路径规划
# # # #         start_coords = start_point[:2]
# # # #         end_coords = end_point[:2]

# # # #         task_info = {"asset": nav_scene, "start": start_coords, "end": end_coords}
# # # #         path_result = plan_navigation_path(task_info)
# # # #         if path_result is None:
# # # #             print(f"Warning: 路径规划失败，起点 {start_coords} 终点 {end_coords}")
# # # #             return False

# # # #         merged_path_real, _ = path_result
# # # #         waypoints = []
# # # #         for i, (x, y, _) in enumerate(merged_path_real):
# # # #             if i < len(merged_path_real) - 1:
# # # #                 nx, ny, _ = merged_path_real[i + 1]
# # # #                 theta = np.arctan2(ny - y, nx - x)
# # # #             else:
# # # #                 theta = waypoints[-1][2] if waypoints else 0.0
# # # #             waypoints.append([x, y, theta])

# # # #         self.current_start = start_point
# # # #         self.current_end = end_point
# # # #         self.current_path = waypoints

# # # #         # 输出信息
# # # #         task_num = self.current_goal_idx + 1
# # # #         print(f"\n📍 执行任务 {task_num}/{len(self.goal_pairs)}:")
# # # #         print(f"   路径点数: {len(waypoints)}")
# # # #         self.print_velocity_info()

# # # #         # 【修改点 1】修复Z轴高度，防止穿模
# # # #         # 从 0.0 抬升至 0.05
# # # #         initial_position = np.array([start_point[0], start_point[1], 0.05])

# # # #         # 【修改点 2】修复朝向逻辑冲突
# # # #         # 优先使用配置中的角度，如果配置只有[x,y]，则使用路径切线
# # # #         if len(start_point) > 2 and abs(start_point[2]) > 1e-6: # 简单的非零检查，或者直接相信用户输入长度
# # # #             # 注意：_ensure_pose_format 会把长度补齐为3，所以这里需要根据原始输入判断
# # # #             # 但既然我们已经在 set_goal_pairs 统一格式化了，
# # # #             # 我们可以假设：如果任务是用户配的，我们就信 start_point[2]
# # # #             # 这里的 start_point 已经是 self.goal_pairs 里的值，已经是3维的了
# # # #             # 所以我们可以直接使用它。
# # # #             # 为了更智能一点，如果用户配置的是 [x, y]，_ensure_pose_format 补的是 0.0
# # # #             # 这可能会导致默认朝向 0 度。
# # # #             # 更好的做法是：在 normalize 时标记是否是用户提供的角度，或者简单起见，
# # # #             # 这里我们信任 start_point[2]，除非它是我们在 normalize 里补的默认值。
# # # #             # 鉴于之前逻辑问题是“完全忽略用户角度”，现在我们改为“完全信任当前start_point的角度”。
# # # #             initial_yaw = start_point[2]
# # # #             print(f"   -> 设置初始朝向 (用户指定): {initial_yaw*180/np.pi:.1f}°")
# # # #         else:
# # # #             # 如果你希望在用户未指定角度时自动对齐，需要在 normalize 阶段保留原始信息
# # # #             # 或者在这里对比 0.0。
# # # #             # 现阶段最稳妥的修复是：使用路径切线方向作为默认，
# # # #             # 但为了修复你的“测试转身”需求，我们信任 start_point。
# # # #             # 实际上，A*规划出来的路径点并没有包含初始点的旋转。
# # # #             # 如果你想测试转身，必须在 yaml 里写明角度。
# # # #             initial_yaw = start_point[2] 

# # # #         # 如果你确实想保留自动对齐功能给那些没写角度的任务：
# # # #         # 可以对比 path 的切线。这里为了解决你的问题，我们暂且认为 start_point 包含了意图。
# # # #         # 如果 start_point 是 normalize 补的 0，那机器人就朝 0。

# # # #         initial_orientation = euler_angles_to_quat(np.array([0.0, 0.0, initial_yaw]))

# # # #         self.robot.set_world_pose(
# # # #             position=initial_position,
# # # #             orientation=initial_orientation
# # # #         )

# # # #         # self.robot.set_joint_positions(np.array([start_point[0], start_point[1], initial_yaw]))

# # # #         # 【修改点 4】重置物理速度，防止残余动量导致第一帧弹跳
# # # #         if hasattr(self.robot, "set_linear_velocity"):
# # # #              self.robot.set_linear_velocity(np.zeros(3))
# # # #         if hasattr(self.robot, "set_angular_velocity"):
# # # #              self.robot.set_angular_velocity(np.zeros(3))
        
# # # #         self.set_joint_to_default()

# # # #         return True

# # # #     def set_joint_to_default(self) -> None:
# # # #         """
# # # #         将机器人关节（机械臂等）设置到默认导航位置，避免遮挡视线或碰撞
# # # #         """
# # # #         if hasattr(self.cfg.task, "default_joint_positions"):
# # # #             default_joints = np.array(self.cfg.task.default_joint_positions)
# # # #             # 使用 set_joint_positions 直接设置关节状态（非 PD 控制，瞬时到达）
# # # #             self.robot.set_joint_positions(default_joints)
# # # #         else:
# # # #             # 如果配置中没写，可以根据具体机器人模型硬编码一个安全姿势
# # # #             # 假设前3个是底座(x,y,yaw)，后面是臂
# # # #             num_dof = self.robot.num_dof
# # # #             if num_dof > 3:
# # # #                 current_joints = self.robot.get_joint_positions()
# # # #                 # 保持前三个（底座坐标）不变，后面全部归零或设置特定角度
# # # #                 target_joints = np.zeros(num_dof)
# # # #                 target_joints[:3] = current_joints[:3] 
# # # #                 self.robot.set_joint_positions(target_joints)

# # # #     def _generate_random_navigation_task(self) -> bool:
# # # #         nav_scene = self.navigation_assets[0]
# # # #         max_attempts = 100

# # # #         for _ in range(max_attempts):
# # # #             start_point, end_point = self._generate_random_points(
# # # #                 nav_scene["x_bounds"], nav_scene["y_bounds"], self.grid
# # # #             )
# # # #             if start_point is None or end_point is None:
# # # #                 continue
# # # #             # 随机生成的点没有角度，我们在这里补上路径切线角度逻辑
# # # #             # 由于 _plan_and_set_path 现在信任传入的 start_point[2]
# # # #             # 我们需要在调用前处理一下，或者让 plan 函数内部支持自动对齐
# # # #             # 简单做法：随机任务让 plan 函数自己决定角度
# # # #             # 这里我们传入带 0 角度的点，然后在 plan 里被执行
# # # #             if self._plan_and_set_path(nav_scene, start_point + [0.0], end_point + [0.0]):
# # # #                 return True

# # # #         print("Warning: 无法生成有效随机路径。")
# # # #         return False

# # # #     def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
# # # #         W = len(grid[0])
# # # #         H = len(grid)

# # # #         for _ in range(attempts):
# # # #             start_x = np.random.uniform(x_bounds[0], x_bounds[1])
# # # #             start_y = np.random.uniform(y_bounds[0], y_bounds[1])
# # # #             end_x = np.random.uniform(x_bounds[0], x_bounds[1])
# # # #             end_y = np.random.uniform(y_bounds[0], y_bounds[1])

# # # #             i_start, j_start = real_to_grid(start_x, start_y, x_bounds, y_bounds, (W, H))
# # # #             i_end, j_end = real_to_grid(end_x, end_y, x_bounds, y_bounds, (W, H))

# # # #             if grid[i_start][j_start] == 0 and grid[i_end][j_end] == 0:
# # # #                 # 返回纯坐标列表
# # # #                 return [start_x, start_y], [end_x, end_y]

# # # #         return None, None

# # # import numpy as np
# # # import yaml
# # # from typing import Dict, Any, List, Tuple, Optional

# # # from .base_task import BaseTask
# # # from utils.a_star import plan_navigation_path, real_to_grid, load_grid

# # # # 兼容性导入：处理不同 Isaac Sim 版本路径差异
# # # try:
# # #     from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
# # # except ImportError:
# # #     from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat


# # # class NavigationTaskTestWeizi(BaseTask):
# # #     """
# # #     顺序式导航任务 (修复增强版)：
# # #     - 修复了 World 对象 set 方法报错问题
# # #     - 修复了关节重置覆盖底座位置的问题
# # #     - 修复了 Z 轴穿模与残余动量问题
# # #     """

# # #     def __init__(self, cfg, world, stage, robot):
# # #         self.navigation_assets: List[dict] = []
# # #         self.grid = None
# # #         self.current_start: Optional[list] = None
# # #         self.current_end: Optional[list] = None
# # #         self.current_path: Optional[List[List[float]]] = None

# # #         # 有序起终点队列
# # #         self.goal_pairs: List[Tuple[list, list]] = []
# # #         self.current_goal_idx: int = 0

# # #         super().__init__(cfg, world, stage, robot)

# # #         # 从配置预加载队列（如果有）
# # #         if hasattr(self.cfg, "task") and hasattr(self.cfg.task, "goal_pairs"):
# # #             self.set_goal_pairs(self.cfg.task.goal_pairs)

# # #     # ---------- 场景与地图 ----------
# # #     def setup_objects(self) -> None:
# # #         super().setup_objects()

# # #         if hasattr(self.cfg.task, "navigation_config_path"):
# # #             with open(self.cfg.task.navigation_config_path, "r") as f:
# # #                 config = yaml.safe_load(f)
# # #                 self.navigation_assets = config.get("assets", [])

# # #         if self.navigation_assets:
# # #             nav_scene = self.navigation_assets[0]
# # #             self.grid, self.W, self.H = load_grid(nav_scene["barrier_image_path"])

# # #     # ---------- 任务流程 ----------
# # #     def reset(self) -> None:
# # #         super().reset()
# # #         self.robot.initialize()

# # #         if not self.navigation_assets:
# # #             return

# # #         # 优先使用外部队列；若为空则回退随机生成
# # #         if self._has_remaining_goals():
# # #             success = self._apply_next_goal_until_success()
# # #             if not success:
# # #                 print("Warning: 提供的起终点均规划失败，回退为随机任务。")
# # #                 self._generate_random_navigation_task()
# # #         else:
# # #             self._generate_random_navigation_task()
        
# # #         # 【核心修复】：使用 step() 驱动仿真步进，不能使用 set()
# # #         if self.world:
# # #             self.world.step(render=False)

# # #     def _calculate_segment_velocities(self) -> List[Dict[str, Any]]:
# # #         if self.current_path is None or len(self.current_path) < 2:
# # #             return []

# # #         velocities = []
# # #         max_linear_speed = 1.0  # 修正速度上限

# # #         for i in range(len(self.current_path) - 1):
# # #             start_point = self.current_path[i]
# # #             end_point = self.current_path[i + 1]

# # #             dx = end_point[0] - start_point[0]
# # #             dy = end_point[1] - start_point[1]
# # #             distance = np.sqrt(dx**2 + dy**2)

# # #             direction = np.arctan2(dy, dx)
# # #             direction_deg = np.degrees(direction)

# # #             velocity_magnitude = min(distance * 2.0, max_linear_speed) 

# # #             segment_info = {
# # #                 "segment_idx": i,
# # #                 "start_point": [start_point[0], start_point[1]],
# # #                 "end_point": [end_point[0], end_point[1]],
# # #                 "distance": distance,
# # #                 "direction": direction,
# # #                 "direction_deg": direction_deg,
# # #                 "velocity_magnitude": velocity_magnitude,
# # #                 "velocity_vector": [
# # #                     velocity_magnitude * np.cos(direction),
# # #                     velocity_magnitude * np.sin(direction)
# # #                 ]
# # #             }
# # #             velocities.append(segment_info)

# # #         return velocities

# # #     def get_current_segment_velocity(self, current_position: Optional[List[float]] = None) -> Optional[Dict[str, Any]]:
# # #         if self.current_path is None or len(self.current_path) < 2:
# # #             return None

# # #         velocities = self._calculate_segment_velocities()
# # #         if not velocities or current_position is None:
# # #             return velocities[0] if velocities else None

# # #         min_distance = float('inf')
# # #         current_segment = None

# # #         for vel in velocities:
# # #             start_point = vel['start_point']
# # #             dist_to_start = np.sqrt((current_position[0] - start_point[0])**2 + (current_position[1] - start_point[1])**2)
# # #             if dist_to_start < min_distance:
# # #                 min_distance = dist_to_start
# # #                 current_segment = vel
# # #         return current_segment

# # #     def step(self) -> Dict[str, Any]:
# # #         self.frame_idx += 1
# # #         if not self.check_frame_limits():
# # #             return None

# # #         position, orientation = self.robot.get_world_pose()
# # #         yaw = quat_to_euler_angles(orientation, extrinsic=False)[2]
# # #         current_pose = np.array([position[0], position[1], yaw])

# # #         camera_data, display_data = self.get_camera_data()
# # #         segment_velocities = self._calculate_segment_velocities()
# # #         current_segment_velocity = self.get_current_segment_velocity(current_position=[current_pose[0], current_pose[1]])

# # #         state = {
# # #             "current_pose": current_pose,
# # #             "start_point": self.current_start,
# # #             "end_point": self.current_end,
# # #             "waypoints": self.current_path,
# # #             "segment_velocities": segment_velocities,
# # #             "current_segment_velocity": current_segment_velocity,
# # #             "camera_data": camera_data,
# # #             "camera_display": display_data,
# # #             "done": self.reset_needed,
# # #             "frame_idx": self.frame_idx,
# # #         }
# # #         return state

# # #     def on_task_complete(self, success: bool) -> None:
# # #         self.reset_needed = True
# # #         if success and self._has_remaining_goals():
# # #             self.current_goal_idx += 1

# # #     # ---------- 队列操作 ----------
# # #     def set_goal_pairs(self, pairs: List[Any]) -> None:
# # #         self.goal_pairs = self._normalize_pairs(pairs)
# # #         self.current_goal_idx = 0

# # #     def _normalize_pairs(self, pairs: List[Any]) -> List[Tuple[list, list]]:
# # #         normalized = []
# # #         for item in pairs:
# # #             if hasattr(item, 'get') and ('start' in item or hasattr(item, 'start')):
# # #                 start = item.get('start') if hasattr(item, 'get') else item['start']
# # #                 end = item.get('end') if hasattr(item, 'get') else item['end']
# # #                 normalized.append((self._ensure_pose_format(list(start)), self._ensure_pose_format(list(end))))
# # #             elif isinstance(item, (list, tuple)) and len(item) == 2:
# # #                 normalized.append((self._ensure_pose_format(list(item[0])), self._ensure_pose_format(list(item[1]))))
# # #         return normalized

# # #     def _ensure_pose_format(self, pose: list) -> list:
# # #         if len(pose) == 2: return [pose[0], pose[1], 0.0]
# # #         if len(pose) == 3: return pose
# # #         raise ValueError(f"位姿格式错误: {pose}")

# # #     def _has_remaining_goals(self) -> bool:
# # #         return self.current_goal_idx < len(self.goal_pairs)

# # #     def _apply_next_goal_until_success(self) -> bool:
# # #         nav_scene = self.navigation_assets[0]
# # #         idx = self.current_goal_idx
# # #         while idx < len(self.goal_pairs):
# # #             start_point, end_point = self.goal_pairs[idx]
# # #             if self._plan_and_set_path(nav_scene, start_point, end_point):
# # #                 self.current_goal_idx = idx
# # #                 return True
# # #             idx += 1
# # #         return False

# # #     # ---------- 路径规划与生成 ----------
# # #     def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list) -> bool:
# # #         start_coords, end_coords = start_point[:2], end_point[:2]
# # #         task_info = {"asset": nav_scene, "start": start_coords, "end": end_coords}
# # #         path_result = plan_navigation_path(task_info)
        
# # #         if path_result is None:
# # #             return False

# # #         merged_path_real, _ = path_result
# # #         waypoints = []
# # #         for i, (x, y, _) in enumerate(merged_path_real):
# # #             if i < len(merged_path_real) - 1:
# # #                 nx, ny, _ = merged_path_real[i + 1]
# # #                 theta = np.arctan2(ny - y, nx - x)
# # #             else:
# # #                 theta = waypoints[-1][2] if waypoints else 0.0
# # #             waypoints.append([x, y, theta])

# # #         self.current_start, self.current_end, self.current_path = start_point, end_point, waypoints

# # #         # 【修复】：设置 Z 轴高度为 0.05 避免地面积压，设置初始朝向
# # #         initial_position = np.array([start_point[0], start_point[1], 0.05])
# # #         initial_yaw = start_point[2]
# # #         initial_orientation = euler_angles_to_quat(np.array([0.0, 0.0, initial_yaw]))

# # #         self.robot.set_world_pose(position=initial_position, orientation=initial_orientation)

# # #         # 【修复】：重置残余物理动量
# # #         if hasattr(self.robot, "set_linear_velocity"):
# # #              self.robot.set_linear_velocity(np.zeros(3))
# # #         if hasattr(self.robot, "set_angular_velocity"):
# # #              self.robot.set_angular_velocity(np.zeros(3))
        
# # #         # 【修复】：设置关节姿态时保护底座坐标
# # #         self.set_joint_to_default()
# # #         return True

# # #     def set_joint_to_default(self) -> None:
# # #         """
# # #         重置机械臂关节，但保留底座当前的位姿
# # #         """
# # #         num_dof = self.robot.num_dof
# # #         if hasattr(self.cfg.task, "default_joint_positions"):
# # #             default_joints = np.array(self.cfg.task.default_joint_positions)
# # #             self.robot.set_joint_positions(default_joints)
# # #         elif num_dof > 3:
# # #             # 关键：获取当前已通过 set_world_pose 确定的底座关节位置
# # #             current_joints = self.robot.get_joint_positions()
# # #             target_joints = np.zeros(num_dof)
# # #             target_joints[:3] = current_joints[:3] # 保护 x, y, yaw
# # #             target_joints[3:] = 0.0 # 其余机械臂关节归零
# # #             self.robot.set_joint_positions(target_joints)

# # #     def _generate_random_navigation_task(self) -> bool:
# # #         nav_scene = self.navigation_assets[0]
# # #         for _ in range(100):
# # #             start_point, end_point = self._generate_random_points(nav_scene["x_bounds"], nav_scene["y_bounds"], self.grid)
# # #             if start_point and self._plan_and_set_path(nav_scene, start_point + [0.0], end_point + [0.0]):
# # #                 return True
# # #         return False

# # #     def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
# # #         W, H = len(grid[0]), len(grid)
# # #         for _ in range(attempts):
# # #             sx, sy = np.random.uniform(x_bounds[0], x_bounds[1]), np.random.uniform(y_bounds[0], y_bounds[1])
# # #             ex, ey = np.random.uniform(x_bounds[0], x_bounds[1]), np.random.uniform(y_bounds[0], y_bounds[1])
# # #             i_s, j_s = real_to_grid(sx, sy, x_bounds, y_bounds, (W, H))
# # #             i_e, j_e = real_to_grid(ex, ey, x_bounds, y_bounds, (W, H))
# # #             if grid[i_s][j_s] == 0 and grid[i_e][j_e] == 0:
# # #                 return [sx, sy], [ex, ey]
# # #         return None, None

# import numpy as np
# import yaml
# from typing import Dict, Any, List, Tuple, Optional

# from .base_task import BaseTask
# from utils.a_star import plan_navigation_path, real_to_grid, load_grid

# # 兼容性导入
# try:
#     from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
# except ImportError:
#     from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat


# class NavigationTaskTestWeizi(BaseTask):
#     """
#     顺序式导航任务 (逻辑修正版)：
#     - 使用 set_joint_positions 代替 set_world_pose，确保底座与关节状态同步
#     - 统一在 _plan_and_set_path 中完成底座与机械臂姿态的初始化
#     """

#     def __init__(self, cfg, world, stage, robot):
#         self.navigation_assets: List[dict] = []
#         self.grid = None
#         self.current_start: Optional[list] = None
#         self.current_end: Optional[list] = None
#         self.current_path: Optional[List[List[float]]] = None

#         self.goal_pairs: List[Tuple[list, list]] = []
#         self.current_goal_idx: int = 0

#         super().__init__(cfg, world, stage, robot)

#         if hasattr(self.cfg, "task") and hasattr(self.cfg.task, "goal_pairs"):
#             self.set_goal_pairs(self.cfg.task.goal_pairs)

#     def setup_objects(self) -> None:
#         super().setup_objects()
#         if hasattr(self.cfg.task, "navigation_config_path"):
#             with open(self.cfg.task.navigation_config_path, "r") as f:
#                 config = yaml.safe_load(f)
#                 self.navigation_assets = config.get("assets", [])

#         if self.navigation_assets:
#             nav_scene = self.navigation_assets[0]
#             self.grid, self.W, self.H = load_grid(nav_scene["barrier_image_path"])

#     def reset(self) -> None:
#         super().reset()
#         self.robot.initialize()

#         if not self.navigation_assets:
#             return

#         if self._has_remaining_goals():
#             success = self._apply_next_goal_until_success()
#             if not success:
#                 print("Warning: 提供的起终点均规划失败，回退为随机任务。")
#                 self._generate_random_navigation_task()
#         else:
#             self._generate_random_navigation_task()

#         if self.world:
#             self.world.step(render=False)

#     def step(self) -> Dict[str, Any]:
#         self.frame_idx += 1
#         if not self.check_frame_limits():
#             return None

#         position, orientation = self.robot.get_world_pose()
#         yaw = quat_to_euler_angles(orientation, extrinsic=False)[2]
#         current_pose = np.array([position[0], position[1], yaw])

#         camera_data, display_data = self.get_camera_data()

#         state = {
#             "current_pose": current_pose,
#             "start_point": self.current_start,
#             "end_point": self.current_end,
#             "waypoints": self.current_path,
#             "camera_data": camera_data,
#             "camera_display": display_data,
#             "done": self.reset_needed,
#             "frame_idx": self.frame_idx,
#         }
#         return state

#     def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list) -> bool:
#         start_coords, end_coords = start_point[:2], end_point[:2]
#         task_info = {"asset": nav_scene, "start": start_coords, "end": end_coords}
#         path_result = plan_navigation_path(task_info)

#         if path_result is None:
#             return False

#         merged_path_real, _ = path_result
#         waypoints = []
#         for i, (x, y, _) in enumerate(merged_path_real):
#             if i < len(merged_path_real) - 1:
#                 nx, ny, _ = merged_path_real[i + 1]
#                 theta = np.arctan2(ny - y, nx - x)
#             else:
#                 theta = waypoints[-1][2] if waypoints else 0.0
#             waypoints.append([x, y, theta])

#         self.current_start, self.current_end, self.current_path = start_point, end_point, waypoints

#         # --- 核心逻辑修改：使用 set_joint_positions 统一初始化 ---
#         num_dof = self.robot.num_dof
#         target_joints = np.zeros(num_dof)

#         # 1. 设置底座关节 (Joint 0: x, Joint 1: y, Joint 2: yaw)
#         target_joints[0] = start_point[0]
#         target_joints[1] = start_point[1]
#         target_joints[2] = start_point[2]

#         # 2. 设置机械臂关节 (Joint 3+)
#         if hasattr(self.cfg.task, "default_joint_positions"):
#             default_positions = np.array(self.cfg.task.default_joint_positions)
#             # 如果配置提供了全量关节(含底座)，则直接使用
#             if len(default_positions) == num_dof:
#                 target_joints = default_positions
#                 target_joints[0:3] = [start_point[0], start_point[1], start_point[2]] # 强制覆盖为当前任务起点
#             else:
#                 # 如果只提供了机械臂部分，则拼接到 3 号索引之后
#                 arm_dof = min(len(default_positions), num_dof - 3)
#                 target_joints[3:3+arm_dof] = default_positions[:arm_dof]

#         # 执行关节空间初始化 (这会同时更新机器人的物理位置和关节状态)
#         self.robot.set_joint_positions(target_joints)

#         # 3. 清除物理速度
#         if hasattr(self.robot, "set_linear_velocity"):
#              self.robot.set_linear_velocity(np.zeros(3))
#         if hasattr(self.robot, "set_angular_velocity"):
#              self.robot.set_angular_velocity(np.zeros(3))

#         print(f"📍 机器人关节已重置: Base([{target_joints[0]:.2f}, {target_joints[1]:.2f}, {target_joints[2]:.2f}]), Arm({target_joints[3:]})")
#         return True

#     def set_goal_pairs(self, pairs: List[Any]) -> None:
#         self.goal_pairs = self._normalize_pairs(pairs)
#         self.current_goal_idx = 0

#     def _normalize_pairs(self, pairs: List[Any]) -> List[Tuple[list, list]]:
#         normalized = []
#         for item in pairs:
#             if hasattr(item, 'get') and ('start' in item or hasattr(item, 'start')):
#                 start = item.get('start') if hasattr(item, 'get') else item['start']
#                 end = item.get('end') if hasattr(item, 'get') else item['end']
#                 normalized.append((self._ensure_pose_format(list(start)), self._ensure_pose_format(list(end))))
#             elif isinstance(item, (list, tuple)) and len(item) == 2:
#                 normalized.append((self._ensure_pose_format(list(item[0])), self._ensure_pose_format(list(item[1]))))
#         return normalized

#     def _ensure_pose_format(self, pose: list) -> list:
#         if len(pose) == 2: return [pose[0], pose[1], 0.0]
#         if len(pose) == 3: return pose
#         raise ValueError(f"位姿格式错误: {pose}")

#     def _has_remaining_goals(self) -> bool:
#         return self.current_goal_idx < len(self.goal_pairs)

#     def _apply_next_goal_until_success(self) -> bool:
#         nav_scene = self.navigation_assets[0]
#         idx = self.current_goal_idx
#         while idx < len(self.goal_pairs):
#             start_point, end_point = self.goal_pairs[idx]
#             if self._plan_and_set_path(nav_scene, start_point, end_point):
#                 self.current_goal_idx = idx
#                 return True
#             idx += 1
#         return False

#     def _generate_random_navigation_task(self) -> bool:
#         nav_scene = self.navigation_assets[0]
#         for _ in range(100):
#             start_point, end_point = self._generate_random_points(nav_scene["x_bounds"], nav_scene["y_bounds"], self.grid)
#             if start_point and self._plan_and_set_path(nav_scene, start_point + [0.0], end_point + [0.0]):
#                 return True
#         return False

#     def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
#         W, H = len(grid[0]), len(grid)
#         for _ in range(attempts):
#             sx, sy = np.random.uniform(x_bounds[0], x_bounds[1]), np.random.uniform(y_bounds[0], y_bounds[1])
#             ex, ey = np.random.uniform(x_bounds[0], x_bounds[1]), np.random.uniform(y_bounds[0], y_bounds[1])
#             i_s, j_s = real_to_grid(sx, sy, x_bounds, y_bounds, (W, H))
#             i_e, j_e = real_to_grid(ex, ey, x_bounds, y_bounds, (W, H))
#             if grid[i_s][j_s] == 0 and grid[i_e][j_e] == 0:
#                 return [sx, sy], [ex, ey]
#         return None, None


import numpy as np
import yaml
from typing import Dict, Any, List, Tuple, Optional

# 导入可视化标记相关 API
from omni.isaac.core.objects import VisualCylinder
from .base_task import BaseTask
from utils.a_star import plan_navigation_path, real_to_grid, load_grid

# 兼容性导入
try:
    from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
except ImportError:
    from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat


class NavigationTaskTestWeizi(BaseTask):
    """
    顺序式导航任务 (全量规范版)：
    - 规范化起点初始化：同步底座与机械臂关节状态
    - 规范化终点位姿：支持终点最终朝向设定，并在场景中可视化终点
    - 路径末端修正：强制路径终点朝向为用户设定的最终朝向
    """

    def __init__(self, cfg, world, stage, robot):
        self.navigation_assets: List[dict] = []
        self.grid = None
        self.current_start: Optional[list] = None
        self.current_end: Optional[list] = None
        self.current_path: Optional[List[List[float]]] = None

        # 终点可视化指示器
        self.goal_marker = None

        self.goal_pairs: List[Tuple[list, list]] = []
        self.current_goal_idx: int = 0

        super().__init__(cfg, world, stage, robot)

        if hasattr(self.cfg, "task") and hasattr(self.cfg.task, "goal_pairs"):
            self.set_goal_pairs(self.cfg.task.goal_pairs)

    def setup_objects(self) -> None:
        """初始化场景对象，并加入终点位姿可视化标记"""
        super().setup_objects()

        # 1. 创建可视化终点标记 (红色扁平圆柱表示位置)
        self.goal_marker = VisualCylinder(
            prim_path="/World/NavigationGoal",
            name="navigation_goal_marker",
            position=np.array([0, 0, 0.01]),
            radius=0.2,
            height=0.02,
            color=np.array([1.0, 0, 0]), # 红色
        )

        if hasattr(self.cfg.task, "navigation_config_path"):
            with open(self.cfg.task.navigation_config_path, "r") as f:
                config = yaml.safe_load(f)
                self.navigation_assets = config.get("assets", [])

        if self.navigation_assets:
            nav_scene = self.navigation_assets[0]
            self.grid, self.W, self.H = load_grid(nav_scene["barrier_image_path"])

    def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list) -> bool:
        """
        核心修复：规范化起点与终点的最终位姿
        """
        start_coords, end_coords = start_point[:2], end_point[:2]
        task_info = {"asset": nav_scene, "start": start_coords, "end": end_coords}
        path_result = plan_navigation_path(task_info)

        if path_result is None:
            return False

        merged_path_real, _ ,_ = path_result
        
        # 检查路径是否有效（plan_navigation_path 失败时返回 (None, debug_path, 0.0)）
        if merged_path_real is None:
            return False
        waypoints = []
        for i, (x, y, _) in enumerate(merged_path_real):
            if i < len(merged_path_real) - 1:
                nx, ny, _ = merged_path_real[i + 1]
                theta = np.arctan2(ny - y, nx - x)
            else:
                # --- 修复点 1：确认终点最终位姿 ---
                # 如果用户在配置中指定了终点角度，则路径最后一步强制转向该角度
                theta = end_point[2] 
            waypoints.append([x, y, theta])

        self.current_start, self.current_end, self.current_path = start_point, end_point, waypoints

        # --- 修复点 2：可视化终点最终位姿 ---
        if self.goal_marker:
            # 将标记移动到终点坐标，并设置终点要求的朝向
            goal_quat = euler_angles_to_quat(np.array([0.0, 0.0, end_point[2]]))
            self.goal_marker.set_world_pose(
                position=np.array([end_point[0], end_point[1], 0.01]),
                orientation=goal_quat
            )

        # --- 修复点 3：规范化起点初始化 (关节同步) ---
        num_dof = self.robot.num_dof
        target_joints = np.zeros(num_dof)

        # 设置底座关节 (Joint 0: x, Joint 1: y, Joint 2: yaw)
        target_joints[0] = start_point[0]
        target_joints[1] = start_point[1]
        target_joints[2] = start_point[2]

        # 设置机械臂关节 (Joint 3+)
        if hasattr(self.cfg.task, "default_joint_positions"):
            default_positions = np.array(self.cfg.task.default_joint_positions)
            if len(default_positions) == num_dof:
                target_joints = default_positions
                target_joints[0:3] = [start_point[0], start_point[1], start_point[2]]
            else:
                arm_dof = min(len(default_positions), num_dof - 3)
                target_joints[3:3+arm_dof] = default_positions[:arm_dof]

        self.robot.set_joint_positions(target_joints)

        # 清除物理速度，防止残余动量
        if hasattr(self.robot, "set_linear_velocity"):
             self.robot.set_linear_velocity(np.zeros(3))
        if hasattr(self.robot, "set_angular_velocity"):
             self.robot.set_angular_velocity(np.zeros(3))

        print(f"📍 任务初始化 | 终点位姿确认: [{end_point[0]:.2f}, {end_point[1]:.2f}, {end_point[2]*180/np.pi:.1f}°]")
        return True

    def set_goal_pairs(self, pairs: List[Any]) -> None:
        self.goal_pairs = self._normalize_pairs(pairs)
        self.current_goal_idx = 0

    def _normalize_pairs(self, pairs: List[Any]) -> List[Tuple[list, list]]:
        normalized = []
        for item in pairs:
            if hasattr(item, 'get') and ('start' in item or hasattr(item, 'start')):
                start = item.get('start') if hasattr(item, 'get') else item['start']
                end = item.get('end') if hasattr(item, 'get') else item['end']
                normalized.append((self._ensure_pose_format(list(start)), self._ensure_pose_format(list(end))))
            elif isinstance(item, (list, tuple)) and len(item) == 2:
                # 兼容 [[x,y,theta], [x,y,theta]] 格式
                normalized.append((self._ensure_pose_format(list(item[0])), self._ensure_pose_format(list(item[1]))))
        return normalized

    def _ensure_pose_format(self, pose: list) -> list:
        """确保位姿格式统一为 [x, y, theta]"""
        if len(pose) == 2: return [pose[0], pose[1], 0.0]
        if len(pose) == 3: return pose
        raise ValueError(f"位姿格式错误: {pose}")

    def reset(self) -> None:
        super().reset()
        self.robot.initialize()
        if not self.navigation_assets: return

        if self._has_remaining_goals():
            success = self._apply_next_goal_until_success()
            if not success: self._generate_random_navigation_task()
        else:
            self._generate_random_navigation_task()

        if self.world: self.world.step(render=False)

    def step(self) -> Dict[str, Any]:
        self.frame_idx += 1
        if not self.check_frame_limits(): return None

        position, orientation = self.robot.get_world_pose()
        yaw = quat_to_euler_angles(orientation, extrinsic=False)[2]
        current_pose = np.array([position[0], position[1], yaw])
        camera_data, display_data = self.get_camera_data()

        state = {
            "current_pose": current_pose,
            "start_point": self.current_start,
            "end_point": self.current_end,
            "waypoints": self.current_path,
            "camera_data": camera_data,
            "camera_display": display_data,
            "done": self.reset_needed,
            "frame_idx": self.frame_idx,
        }
        return state

    # ---------- 辅助方法 ----------
    def _has_remaining_goals(self) -> bool:
        return self.current_goal_idx < len(self.goal_pairs)

    # def _apply_next_goal_until_success(self) -> bool:
    #     nav_scene = self.navigation_assets[0]
    #     idx = self.current_goal_idx
    #     while idx < len(self.goal_pairs):
    #         start_point, end_point = self.goal_pairs[idx]
    #         if self._plan_and_set_path(nav_scene, start_point, end_point):
    #             self.current_goal_idx = idx
    #             return True
    #         idx += 1
    #     return False

    def _apply_next_goal_until_success(self) -> bool:
        nav_scene = self.navigation_assets[0]
        
        # 从当前的索引开始尝试
        while self.current_goal_idx < len(self.goal_pairs):
            start_point, end_point = self.goal_pairs[self.current_goal_idx]
            
            # 尝试规划路径
            if self._plan_and_set_path(nav_scene, start_point, end_point):
                # --- 关键修正点 ---
                # 路径规划成功，将索引移向下一个任务，以便下次 reset 时执行新任务
                self.current_goal_idx += 1 
                print(f"✅ 成功加载任务对，后续将执行任务索引: {self.current_goal_idx}")
                return True
            
            # 如果当前任务规划失败（例如起点或终点在障碍物内），跳过它尝试下一个
            print(f"⚠️ 任务索引 {self.current_goal_idx} 规划失败，尝试跳过...")
            self.current_goal_idx += 1
            
        return False

    def _generate_random_navigation_task(self) -> bool:
        nav_scene = self.navigation_assets[0]
        for _ in range(100):
            start_point, end_point = self._generate_random_points(nav_scene["x_bounds"], nav_scene["y_bounds"], self.grid)
            if start_point:
                # 随机任务也赋予随机的终点朝向
                rand_start = start_point + [np.random.uniform(-np.pi, np.pi)]
                rand_end = end_point + [np.random.uniform(-np.pi, np.pi)]
                if self._plan_and_set_path(nav_scene, rand_start, rand_end):
                    return True
        return False

    def _generate_random_points(self, x_bounds, y_bounds, grid, attempts=100):
        W, H = len(grid[0]), len(grid)
        for _ in range(attempts):
            sx, sy = np.random.uniform(x_bounds[0], x_bounds[1]), np.random.uniform(y_bounds[0], y_bounds[1])
            ex, ey = np.random.uniform(x_bounds[0], x_bounds[1]), np.random.uniform(y_bounds[0], y_bounds[1])
            i_s, j_s = real_to_grid(sx, sy, x_bounds, y_bounds, (W, H))
            i_e, j_e = real_to_grid(ex, ey, x_bounds, y_bounds, (W, H))
            if grid[i_s][j_s] == 0 and grid[i_e][j_e] == 0:
                return [sx, sy], [ex, ey]
        return None, None