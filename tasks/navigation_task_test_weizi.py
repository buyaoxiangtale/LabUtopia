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
        
        # 上一次机器人的位置，用于检测是否到达目标
        self.last_robot_pose: Optional[np.ndarray] = None
        self.path_completed: bool = False

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

    def _plan_and_set_path(self, nav_scene: dict, start_point: list, end_point: list, set_robot_pose: bool = True) -> bool:
        """
        核心修复：规范化起点与终点的最终位姿
        
        Args:
            nav_scene: 导航场景配置
            start_point: 起点 [x, y, theta]
            end_point: 终点 [x, y, theta]
            set_robot_pose: 是否设置机器人位姿（初始化时为True，切换目标时为False）
        """
        start_coords, end_coords = start_point[:2], end_point[:2]
        task_info = {"asset": nav_scene, "start": start_coords, "end": end_coords}
        
        print(f"🔍 路径规划: 起点 [{start_coords[0]:.2f}, {start_coords[1]:.2f}] → 终点 [{end_coords[0]:.2f}, {end_coords[1]:.2f}]")
        path_result = plan_navigation_path(task_info)

        if path_result is None:
            print(f"❌ 路径规划失败：A*算法未找到可行路径")
            return False

        merged_path_real, _, _ = path_result
        print(f"✅ A*路径规划成功，原始路径点数: {len(merged_path_real)}")
        
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
        print(f"📋 生成路径点数: {len(waypoints)}, 第一个点: [{waypoints[0][0]:.2f}, {waypoints[0][1]:.2f}], 最后一个点: [{waypoints[-1][0]:.2f}, {waypoints[-1][1]:.2f}]")

        # --- 修复点 2：可视化终点最终位姿 ---
        if self.goal_marker:
            # 将标记移动到终点坐标，并设置终点要求的朝向
            goal_quat = euler_angles_to_quat(np.array([0.0, 0.0, end_point[2]]))
            self.goal_marker.set_world_pose(
                position=np.array([end_point[0], end_point[1], 0.01]),
                orientation=goal_quat
            )

        # --- 修复点 3：规范化起点初始化 (关节同步) ---
        # 只在初始化时设置机器人位姿，切换目标时不设置（保持当前位置）
        if set_robot_pose:
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
            
            print(f"📍 任务初始化 | 机器人位姿已设置: [{start_point[0]:.2f}, {start_point[1]:.2f}, {start_point[2]*180/np.pi:.1f}°]")
        else:
            print(f"🔄 动态切换目标 | 保持当前机器人位置不变")

        print(f"📍 路径规划完成 | 终点位姿: [{end_point[0]:.2f}, {end_point[1]:.2f}, {end_point[2]*180/np.pi:.1f}°] | 路径点数: {len(waypoints)}")
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

        joint_states = self.robot.get_joint_positions()
        joint_states_array = np.array(joint_states[0:3])
        # 检查当前目标是否完成
        if self._check_goal_reached(joint_states_array):
            # 先递增索引，指向下一个目标
            self.current_goal_idx += 1
            print(f"✅ Task: 目标点 {self.current_goal_idx - 1} 已到达，当前位置: [{joint_states_array[0]:.2f}, {joint_states_array[1]:.2f}, {joint_states_array[2]*180/np.pi:.1f}°]")
            
            # 如果还有剩余目标，加载下一个
            if self._has_remaining_goals():
                print(f"🎯 Task: 加载下一个目标 (索引: {self.current_goal_idx})")
                success = self._load_next_goal()
                if success:
                    print(f"   新路径已生成，起点: [{self.current_start[0]:.2f}, {self.current_start[1]:.2f}], 终点: [{self.current_end[0]:.2f}, {self.current_end[1]:.2f}], 路径点数: {len(self.current_path)}")
                else:
                    print(f"⚠️  Task: 路径生成失败！")
            else:
                print(f"🏁 Task: 所有目标点已完成！")

        state = {
            "current_pose": current_pose,
            "start_point": self.current_start,
            "end_point": self.current_end,
            "waypoints": self.current_path,
            "camera_data": camera_data,
            "camera_display": display_data,
            "done": self.reset_needed,
            "frame_idx": self.frame_idx,
            "all_goals_done": not self._has_remaining_goals(),  # 标记是否所有任务完成
        }
        return state

    # ---------- 辅助方法 ----------
    def _has_remaining_goals(self) -> bool:
        return self.current_goal_idx < len(self.goal_pairs)
    
    def _check_goal_reached(self, current_pose: np.ndarray) -> bool:
        """
        检查机器人是否到达当前目标点
        
        Args:
            current_pose: 当前机器人位姿 [x, y, yaw]
            
        Returns:
            bool: 是否到达目标
        """
        if self.current_end is None:
            return False
        
        # 获取配置的阈值
        position_threshold = 0.08
        angle_threshold = 0.1
        if hasattr(self.cfg.task, 'position_threshold'):
            position_threshold = self.cfg.task.position_threshold
        if hasattr(self.cfg.task, 'angle_threshold'):
            angle_threshold = self.cfg.task.angle_threshold
        
        # 计算位置差异
        pos_diff = np.linalg.norm(current_pose[:2] - np.array(self.current_end[:2]))
        
        # 计算角度差异
        angle_diff = abs(current_pose[2] - self.current_end[2])
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff)  # 处理角度环绕
        
        # 如果到达目标，记录状态避免重复触发
        if pos_diff < position_threshold and angle_diff < angle_threshold:
            if not self.path_completed:
                self.path_completed = True
                return True
        else:
            self.path_completed = False
        
        return False
    
    def _load_next_goal(self) -> bool:
        """
        加载下一个目标点对，规划路径并更新当前状态
        注意：调用此方法前，current_goal_idx 应已指向要加载的目标
        
        【修复】：使用迭代替代递归，避免栈溢出
        
        Returns:
            bool: 是否成功加载
        """
        nav_scene = self.navigation_assets[0]
        
        # 使用迭代而非递归，避免栈溢出
        while self._has_remaining_goals():
            _, end_point = self.goal_pairs[self.current_goal_idx]
            
            # 使用当前机器人位置作为起点（实现连续导航）
            position = self.robot.get_joint_positions()[0:3]
            print(f"🚀 Task: 从当前位置 [{position[0]:.2f}, {position[1]:.2f}, {position[2]*180/np.pi:.1f}°] 导航到目标 {self.current_goal_idx} [{end_point[0]:.2f}, {end_point[1]:.2f}, {end_point[2]*180/np.pi:.1f}°]")
            
            # 规划从当前位置到下一个目标的路径，set_robot_pose=False 避免瞬移机器人
            if self._plan_and_set_path(nav_scene, position, end_point, set_robot_pose=False):
                self.path_completed = False
                return True
            else:
                print(f"⚠️ Task: 从当前位置到目标点 {self.current_goal_idx} 的路径规划失败，尝试跳过")
                # 路径规划失败，递增索引尝试下一个目标
                self.current_goal_idx += 1
        
        print(f"⚠️ Task: 没有更多目标点了")
        return False

    def _apply_next_goal_until_success(self) -> bool:
        nav_scene = self.navigation_assets[0]
        
        # 从当前的索引开始尝试
        temp_idx = self.current_goal_idx
        while temp_idx < len(self.goal_pairs):
            start_point, end_point = self.goal_pairs[temp_idx]
            
            # 尝试规划路径
            if self._plan_and_set_path(nav_scene, start_point, end_point):
                # --- 修正：初始化时不递增索引，索引递增在任务完成时进行 ---
                self.current_goal_idx = temp_idx
                self.path_completed = False
                print(f"✅ 成功加载初始任务对 (索引: {self.current_goal_idx})")
                return True
            
            # 如果当前任务规划失败（例如起点或终点在障碍物内），跳过它尝试下一个
            print(f"⚠️ 任务索引 {temp_idx} 规划失败，尝试跳过...")
            temp_idx += 1
            
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