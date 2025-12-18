import numpy as np
from typing import List, Tuple, Optional
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims.impl import Articulation
from isaacsim.core.utils.types import ArticulationAction


def _normalize_angle(angle: float) -> float:
    """将角度归一化到 [-π, π]"""
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return angle


class RidgebaseController:
    def __init__(
        self,
        robot_articulation: Articulation,
        max_linear_speed: float = 1.0,  # 建议在yaml中确认此值至少为 0.5 或 1.0
        max_angular_speed: float = 1.0,
        position_threshold: float = 0.15,
        angle_threshold: float = 0.15,
        dt: float = 0.2,
        final_angle: Optional[float] = None,
    ):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold
        self.dt = dt
        self.final_angle = final_angle

        # ——— 角度控制增益 ———
        self.k_p_angular = 1.5
        self.k_d_angular = 0.3

        # ——— [新增] 线性速度控制参数 ———
        self.k_p_linear = 2.0        # 距离增益：1米距离时尝试以 2m/s 速度行驶（会被 max_linear_speed 截断）
        self.min_start_speed = 0.1   # [核心修改] 最小启动速度，防止起步为0

        # ——— 速度平滑 ———
        self.vel_filter_alpha = 0.6
        self._last_cmd_vel = np.zeros(3)

        self.waypoints = None
        self.current_waypoint_idx = 0

        self._joints_subset = ArticulationSubset(
            robot_articulation,
            ["dummy_base_prismatic_x_joint", "dummy_base_prismatic_y_joint", "dummy_base_revolute_z_joint"]
        )

    def set_waypoints(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        final_angle: Optional[float] = None
    ) -> None:
        self.waypoints = np.array(waypoints, dtype=np.float64)
        self.current_waypoint_idx = 0
        if final_angle is not None:
            self.final_angle = final_angle
        self._last_cmd_vel = np.zeros(3)

    def compute_control(self, current_pose: np.ndarray) -> Tuple[float, float, float, float]:
        """
        计算控制指令
        """
        if self.waypoints is None or len(self.waypoints) == 0:
            return 0.0, 0.0, 0.0, 0.0

        # —— Step 1: 获取当前状态 ——
        # 注意：这里保留了你的原始逻辑。
        # 如果 current_pose 是机器人Base在世界下的位置，而 joint_positions 是dummy offset，
        # 那么 curr_x 计算的是 "Base位置 + Offset"。请确保这是你预期的逻辑。
        joint_positions = self._joints_subset.get_joint_positions()
        joint_velocities = self._joints_subset.get_joint_velocities()

        if joint_velocities is None or len(joint_velocities) < 3:
            joint_velocities = np.zeros(3)

        curr_x = current_pose[0] + joint_positions[0]
        curr_y = current_pose[1] + joint_positions[1]
        curr_theta = current_pose[2] + joint_positions[2]

        # —— Step 2: 获取当前目标点 ——
        target = self.waypoints[self.current_waypoint_idx]
        dx = target[0] - curr_x
        dy = target[1] - curr_y
        distance = np.hypot(dx, dy)

        # —— Step 3: 检查是否到达 ——
        reached_position = distance < self.position_threshold
        if reached_position:
            if self.current_waypoint_idx == len(self.waypoints) - 1:
                # 终点逻辑
                target_heading = self.final_angle if self.final_angle is not None else target[2]
                angle_error = _normalize_angle(target_heading - curr_theta)

                if abs(angle_error) < self.angle_threshold:
                    return 0.0, 0.0, 0.0, angle_error

                # 原地旋转
                current_omega = joint_velocities[2]
                omega = self.k_p_angular * angle_error - self.k_d_angular * current_omega
                return 0.0, 0.0, omega, angle_error
            else:
                # 切换下一个点
                self.current_waypoint_idx += 1
                target = self.waypoints[self.current_waypoint_idx]
                dx = target[0] - curr_x
                dy = target[1] - curr_y
                distance = np.hypot(dx, dy)

        # —— Step 4: 计算方向 ——
        target_angle = np.arctan2(dy, dx)
        angle_diff = _normalize_angle(target_angle - curr_theta)

        # —— Step 5: 计算线性速度 [核心修改处] ——

        # 1. 基础 P 控制：速度与距离成正比
        desired_speed = self.k_p_linear * distance

        # 2. 最小启动速度：如果没到达终点，至少保持 min_start_speed
        if distance > self.position_threshold:
            desired_speed = max(desired_speed, self.min_start_speed)

        # 3. 减速区优化：当距离非常近时（小于0.2m），允许速度平滑下降以避免过冲，但受限于 P 控制自然下降
        # 这里不需要额外的复杂逻辑，因为 k_p_linear=2.0 在 0.1m 时会输出 0.2m/s，
        # 配合 min_start_speed 可能会有点冲，所以我们做个平滑过渡：
        if distance < 0.2:
             # 近距离时，取消最小速度限制，允许它减速到0
             desired_speed = self.k_p_linear * distance

        # 4. 最大速度限制
        speed = np.clip(desired_speed, 0.0, self.max_linear_speed)

        # 5. 分解速度向量
        x_vel = speed * np.cos(target_angle)
        y_vel = speed * np.sin(target_angle)

        # —— Step 6: 计算角速度 ——
        current_omega = joint_velocities[2]
        theta_vel = (
            self.k_p_angular * angle_diff 
            - self.k_d_angular * current_omega
        )

        return x_vel, y_vel, theta_vel, angle_diff

    def get_action(self, current_pose: np.ndarray) -> Tuple[Optional[ArticulationAction], bool]:
        x_vel, y_vel, theta_vel, angle_diff = self.compute_control(current_pose)

        # —— 限幅 ——
        x_vel = np.clip(x_vel, -self.max_linear_speed, self.max_linear_speed)
        y_vel = np.clip(y_vel, -self.max_linear_speed, self.max_linear_speed)
        theta_vel = np.clip(theta_vel, -self.max_angular_speed, self.max_angular_speed)

        # —— 速度低通滤波 ——
        raw_vel = np.array([x_vel, y_vel, theta_vel])
        filtered_vel = (
            self.vel_filter_alpha * raw_vel +
            (1 - self.vel_filter_alpha) * self._last_cmd_vel
        )
        self._last_cmd_vel = filtered_vel.copy()
        x_vel, y_vel, theta_vel = filtered_vel

        # —— 积分 ——
        joint_positions = self._joints_subset.get_joint_positions()
        next_x = joint_positions[0] + x_vel * self.dt
        next_y = joint_positions[1] + y_vel * self.dt
        next_theta = joint_positions[2] + theta_vel * self.dt

        position = np.array([next_x, next_y, next_theta])
        velocity = np.array([x_vel, y_vel, theta_vel]) # 加上速度前馈

        action = self._joints_subset.make_articulation_action(
            joint_positions=position,
            joint_velocities=velocity
        )

        # —— 完成判定 ——
        done = False
        if self.waypoints is not None and self.current_waypoint_idx >= len(self.waypoints) - 1:
            joint_pos = self._joints_subset.get_joint_positions()
            # 保持坐标系计算一致性
            world_x = current_pose[0] + joint_pos[0]
            world_y = current_pose[1] + joint_pos[1]
            world_theta = current_pose[2] + joint_pos[2]

            target = self.waypoints[-1]
            dist_err = np.hypot(target[0] - world_x, target[1] - world_y)

            if self.final_angle is not None:
                angle_err = abs(_normalize_angle(self.final_angle - world_theta))
            else:
                angle_err = abs(_normalize_angle(target[2] - world_theta))

            done = (dist_err < self.position_threshold) and (angle_err < self.angle_threshold)

        return action, done

    def is_path_complete(self) -> bool:
        return self.waypoints is not None and self.current_waypoint_idx >= len(self.waypoints)