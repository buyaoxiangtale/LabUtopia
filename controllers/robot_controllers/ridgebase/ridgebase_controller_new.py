import numpy as np
from typing import List, Tuple, Optional
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims.impl import Articulation
from isaacsim.core.utils.types import ArticulationAction


class RidgebaseControllerNew:
    """
    改进的Ridgebase控制器，解决抖动问题。

    主要改进：
    1. PID控制替代简单P控制
    2. 添加速度平滑过渡
    3. 死区控制减少微小误差抖动
    4. 低通滤波平滑控制输出
    5. 角度控制稳定性改进
    """

    def __init__(
        self,
        robot_articulation: Articulation,
        max_linear_speed: float = 1.0,
        max_angular_speed: float = 1.0,
        position_threshold: float = 0.1,
        angle_threshold: float = 0.1,
        dt: float = 0.02,
        final_angle: float = None
    ):
        # 基本参数
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold
        self.dt = dt
        self.final_angle = final_angle

        # PID控制参数 - 减少抖动
        self.k_p_linear = 0.8    # 降低比例增益
        self.k_i_linear = 0.1    # 添加积分项
        self.k_d_linear = 0.05   # 添加微分项

        self.k_p_angular = 2.0   # 降低角速度比例增益
        self.k_i_angular = 0.2   # 添加角度积分项
        self.k_d_angular = 0.1   # 添加角度微分项

        # 死区参数 - 避免微小误差引起抖动
        self.linear_deadzone = 0.01  # 线速度死区
        self.angular_deadzone = 0.05  # 角速度死区

        # 低通滤波参数
        self.alpha_linear = 0.3   # 线速度滤波系数
        self.alpha_angular = 0.2  # 角速度滤波系数

        # 路径和状态
        self.waypoints = None
        self.current_waypoint_idx = 0

        # PID状态变量
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        # 滤波状态
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0

        # 关节子集
        self._joints_subset = ArticulationSubset(
            robot_articulation,
            ["dummy_base_prismatic_x_joint", "dummy_base_prismatic_y_joint", "dummy_base_revolute_z_joint"]
        )

    def reset(self):
        """重置控制器状态"""
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0

    def set_waypoints(self, waypoints: List[Tuple[float, float, float]], final_angle: Optional[float] = None) -> None:
        """设置路径点"""
        self.waypoints = np.array(waypoints)
        self.current_waypoint_idx = 0
        self.final_angle = final_angle
        self.reset()  # 重置PID状态

    def _normalize_angle(self, angle: float) -> float:
        """角度归一化到[-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _apply_deadzone(self, value: float, deadzone: float) -> float:
        """应用死区控制"""
        if abs(value) < deadzone:
            return 0.0
        return value

    def _low_pass_filter(self, new_value: float, prev_value: float, alpha: float) -> float:
        """低通滤波"""
        return alpha * new_value + (1 - alpha) * prev_value

    def _pid_control(self, error: float, prev_error: float, integral: float,
                    kp: float, ki: float, kd: float, dt: float,
                    integral_limit: float = 1.0) -> Tuple[float, float]:
        """
        PID控制计算

        Args:
            error: 当前误差
            prev_error: 上次误差
            integral: 积分项
            kp, ki, kd: PID参数
            dt: 时间步长
            integral_limit: 积分限幅

        Returns:
            tuple: (控制输出, 更新后的积分项)
        """
        # 比例项
        proportional = kp * error

        # 积分项（带限幅）
        integral_new = integral + ki * error * dt
        integral_new = np.clip(integral_new, -integral_limit, integral_limit)

        # 微分项
        derivative = kd * (error - prev_error) / dt if dt > 0 else 0.0

        # PID输出
        output = proportional + integral_new + derivative

        return output, integral_new

    def _compute_smooth_speed(self, distance: float, max_speed: float) -> float:
        """
        计算平滑过渡速度
        距离越近，速度越慢，避免突然停止
        """
        if distance < self.position_threshold:
            # 在阈值内，速度线性减小到0
            speed = (distance / self.position_threshold) * max_speed * 0.5
        elif distance < self.position_threshold * 3:
            # 在3倍阈值内，平滑过渡
            ratio = distance / (self.position_threshold * 3)
            speed = ratio * max_speed
        else:
            # 远距离，使用最大速度
            speed = max_speed

        return speed

    def compute_control(self, current_pose: np.ndarray) -> Tuple[float, float, float, float]:
        """计算控制输出（改进版本）"""
        if self.waypoints is None or self.current_waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0, 0.0, 0.0

        joint_positions = self._joints_subset.get_joint_positions()
        target = self.waypoints[self.current_waypoint_idx]

        # 更新当前位置（包含关节偏移）
        current_pose[0] += joint_positions[0]
        current_pose[1] += joint_positions[1]
        current_pose[2] += joint_positions[2]

        # 计算距离和角度误差
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        distance = np.sqrt(dx**2 + dy**2)

        target_angle = np.arctan2(dy, dx)
        angle_diff = self._normalize_angle(target_angle - current_pose[2])

        # 检查是否到达当前路径点
        if distance < self.position_threshold:
            if self.current_waypoint_idx == len(self.waypoints) - 1:
                # 到达最后一个点，执行最终角度对齐
                return self._final_orientation_control(current_pose)
            else:
                # 切换到下一个路径点
                self.current_waypoint_idx += 1
                self.reset()  # 重置PID状态
                return self.compute_control(current_pose)

        # PID控制计算线速度
        linear_error = distance
        linear_output, self.linear_integral = self._pid_control(
            linear_error, self.prev_linear_error, self.linear_integral,
            self.k_p_linear, self.k_i_linear, self.k_d_linear, self.dt
        )
        self.prev_linear_error = linear_error

        # 限制线速度并应用死区和平滑过渡
        linear_output = self._apply_deadzone(linear_output, self.linear_deadzone)
        speed = self._compute_smooth_speed(distance, self.max_linear_speed)
        linear_output = np.clip(linear_output, -speed, speed)

        # 计算线速度分量
        x_vel = linear_output * np.cos(target_angle)
        y_vel = linear_output * np.sin(target_angle)

        # PID控制计算角速度
        angular_error = angle_diff
        angular_output, self.angular_integral = self._pid_control(
            angular_error, self.prev_angular_error, self.angular_integral,
            self.k_p_angular, self.k_i_angular, self.k_d_angular, self.dt
        )
        self.prev_angular_error = angular_error

        # 限制角速度并应用死区
        angular_output = self._apply_deadzone(angular_output, self.angular_deadzone)
        theta_vel = np.clip(angular_output, -self.max_angular_speed, self.max_angular_speed)

        # 应用低通滤波平滑输出
        x_vel = self._low_pass_filter(x_vel, self.prev_linear_vel * np.cos(target_angle), self.alpha_linear)
        y_vel = self._low_pass_filter(y_vel, self.prev_linear_vel * np.sin(target_angle), self.alpha_linear)
        theta_vel = self._low_pass_filter(theta_vel, self.prev_angular_vel, self.alpha_angular)

        # 更新滤波状态
        self.prev_linear_vel = np.sqrt(x_vel**2 + y_vel**2)
        self.prev_angular_vel = theta_vel

        return x_vel, y_vel, theta_vel, angle_diff

    def _final_orientation_control(self, current_pose: np.ndarray) -> Tuple[float, float, float, float]:
        """最终角度对齐控制"""
        if self.final_angle is not None:
            final_angle_diff = self._normalize_angle(self.final_angle - current_pose[2])
        else:
            target = self.waypoints[self.current_waypoint_idx]
            final_angle_diff = self._normalize_angle(target[2] - current_pose[2])

        # 对最终角度使用更保守的PID参数
        if abs(final_angle_diff) < self.angle_threshold:
            return 0.0, 0.0, 0.0, final_angle_diff

        # 使用更小的PID参数进行最终角度对齐
        angular_output, self.angular_integral = self._pid_control(
            final_angle_diff, self.prev_angular_error, self.angular_integral,
            self.k_p_angular * 0.5, self.k_i_angular * 0.5, self.k_d_angular * 0.5, self.dt
        )

        angular_output = self._apply_deadzone(angular_output, self.angular_deadzone * 0.5)
        theta_vel = np.clip(angular_output, -self.max_angular_speed * 0.5, self.max_angular_speed * 0.5)
        theta_vel = self._low_pass_filter(theta_vel, self.prev_angular_vel, self.alpha_angular)

        self.prev_angular_vel = theta_vel
        self.prev_angular_error = final_angle_diff

        return 0.0, 0.0, theta_vel, final_angle_diff

    def get_action(self, current_pose: np.ndarray) -> Tuple[Optional[ArticulationAction], bool]:
        """获取控制动作"""
        x_vel, y_vel, theta_vel, angle_diff = self.compute_control(current_pose)

        # 确保速度在合理范围内
        x_vel = np.clip(x_vel, -self.max_linear_speed, self.max_linear_speed)
        y_vel = np.clip(y_vel, -self.max_linear_speed, self.max_linear_speed)
        theta_vel = np.clip(theta_vel, -self.max_angular_speed, self.max_angular_speed)

        # 获取当前关节位置
        joint_positions = self._joints_subset.get_joint_positions()

        # 计算下一个关节位置
        next_x = joint_positions[0] + x_vel * self.dt
        next_y = joint_positions[1] + y_vel * self.dt
        next_theta = joint_positions[2] + theta_vel * self.dt

        position = np.array([next_x, next_y, next_theta])
        action = self._joints_subset.make_articulation_action(
            joint_positions=position,
            joint_velocities=None
        )

        # 判断是否完成
        if self.final_angle is None:
            done = (self.waypoints is not None and
                    self.current_waypoint_idx == len(self.waypoints) - 1 and
                    np.sqrt(x_vel**2 + y_vel**2) < 0.01 and  # 线速度接近0
                    abs(theta_vel) < self.angle_threshold)
        else:
            done = (self.waypoints is not None and
                    self.current_waypoint_idx == len(self.waypoints) - 1 and
                    np.sqrt(x_vel**2 + y_vel**2) < 0.01 and  # 线速度接近0
                    abs(theta_vel) < self.angle_threshold and
                    abs(angle_diff) < self.angle_threshold)

        return action, done

    def is_path_complete(self) -> bool:
        """检查路径是否完成"""
        return (self.waypoints is not None and
                self.current_waypoint_idx >= len(self.waypoints))