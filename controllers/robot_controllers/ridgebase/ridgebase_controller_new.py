import numpy as np
from typing import List, Tuple, Optional
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims.impl import Articulation
from isaacsim.core.utils.types import ArticulationAction


class RidgebaseControllerNew:
    """
    高性能Ridgebase控制器，优化速度与平滑性平衡。

    主要特性：
    1. 高速PID控制（大幅提高响应速度）
    2. 最小死区控制（超高灵敏度）
    3. 优化平滑速度过渡（减少不必要限制）
    4. 自适应低通滤波（平衡响应和平滑）
    5. 快速角度控制（快速最终对齐）

    速度优化策略：
    - PID增益大幅提升：线速度2.5倍，角速度1.5倍
    - 死区极小化：线速度死区减少10倍，角速度减少5倍
    - 滤波系数优化：线速度95%响应，角速度85%响应
    - 速度过渡优化：减少过渡范围，提高近距离速度
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
        """
        初始化高性能Ridgebase控制器

        Args:
            robot_articulation: 机器人关节对象
            max_linear_speed: 最大线速度 (m/s)
            max_angular_speed: 最大角速度 (rad/s)
            position_threshold: 位置到达阈值 (m)
            angle_threshold: 角度到达阈值 (rad)
            dt: 控制时间步长 (s)
            final_angle: 最终目标角度 (rad)，可选
        """
        # 基本参数
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold
        self.dt = dt
        self.final_angle = final_angle

        # PID控制参数 - 高速度模式
        self.k_p_linear = 2.5    # 进一步提高响应速度 (从1.5到2.5)
        self.k_i_linear = 0.02   # 减少积分项 (从0.05到0.02)
        self.k_d_linear = 0.01   # 减少微分项 (从0.02到0.01)

        self.k_p_angular = 4.5   # 大幅提高转向响应 (从3.0到4.5)
        self.k_i_angular = 0.05  # 减少积分项 (从0.1到0.05)
        self.k_d_angular = 0.02  # 减少微分项 (从0.05到0.02)

        # 死区参数 - 超高响应性
        self.linear_deadzone = 0.0001  # 极小死区 (从0.001到0.0001)
        self.angular_deadzone = 0.002   # 极小死区 (从0.01到0.002)

        # 低通滤波参数 - 最小延迟
        self.alpha_linear = 0.95   # 极快响应 (从0.8到0.95)
        self.alpha_angular = 0.85  # 快速响应 (从0.6到0.85)

        # 路径和状态
        self.waypoints = None
        self.current_waypoint_idx = 0

        # ============= PID控制器状态变量 =============
        # 积分项状态（用于消除稳态误差）
        self.linear_integral = 0.0   # 线速度积分累积值
        self.angular_integral = 0.0  # 角速度积分累积值

        # 上一时刻误差（用于微分控制）
        self.prev_linear_error = 0.0  # 上一步线速度误差
        self.prev_angular_error = 0.0 # 上一步角速度误差

        # ============= 低通滤波历史状态 =============
        # 上一时刻速度输出（用于平滑控制）
        self.prev_linear_vel = 0.0   # 上一步线速度输出
        self.prev_angular_vel = 0.0  # 上一步角速度输出

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
                    integral_limit: float = 0.2) -> Tuple[float, float]:
        """
        PID控制器核心算法 - 实现比例+积分+微分控制

        PID控制原理：
        - 比例(P): 响应当前误差，决定响应速度
        - 积分(I): 累积历史误差，消除稳态误差
        - 微分(D): 预测误差变化趋势，抑制震荡

        Args:
            error: 当前误差值
            prev_error: 上一时刻误差值（用于微分计算）
            integral: 当前积分累积值
            kp: 比例增益 (决定响应速度)
            ki: 积分增益 (决定稳态误差消除能力)
            kd: 微分增益 (决定震荡抑制能力)
            dt: 时间步长
            integral_limit: 积分项限幅 (防止积分饱和)

        Returns:
            tuple: (控制输出, 更新后的积分累积值)
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
        高速版本：最小化速度限制，优先保证速度
        """
        if distance < self.position_threshold:
            # 在阈值内，保持高速度直到最后
            speed = (distance / self.position_threshold) * max_speed * 0.95  # 从0.8提高到0.95
        elif distance < self.position_threshold * 1.5:
            # 在1.5倍阈值内，快速过渡到全速
            ratio = (distance - self.position_threshold) / (self.position_threshold * 0.5)
            speed = 0.95 * max_speed + ratio * 0.05 * max_speed  # 更陡峭的过渡
        else:
            # 远距离，使用最大速度
            speed = max_speed

        return speed

    def compute_control(self, current_pose: np.ndarray) -> Tuple[float, float, float, float]:
        """
        计算控制输出 - 高性能PID控制策略

        控制流程：
        1. 路径验证和状态更新
        2. 误差计算（距离和角度）
        3. 路径点切换逻辑
        4. PID速度控制计算
        5. 多重优化处理（死区、滤波、平滑过渡）

        Args:
            current_pose: 当前机器人姿态 [x, y, theta]

        Returns:
            tuple: (x_vel, y_vel, theta_vel, angle_diff)
        """
        # ============= 1. 路径状态检查 =============
        if self.waypoints is None or self.current_waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0, 0.0, 0.0

        # ============= 2. 获取当前状态 =============
        joint_positions = self._joints_subset.get_joint_positions()
        target = self.waypoints[self.current_waypoint_idx]

        # 更新绝对位置（Isaac Sim的姿态是相对的，需要加上关节偏移）
        current_pose[0] += joint_positions[0]  # X绝对位置
        current_pose[1] += joint_positions[1]  # Y绝对位置
        current_pose[2] += joint_positions[2]  # 绝对角度

        # ============= 3. 计算控制误差 =============
        # 距离误差（用于线速度控制）
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        distance = np.sqrt(dx**2 + dy**2)

        # 角度误差（用于角速度控制）
        target_angle = np.arctan2(dy, dx)  # 目标朝向角度
        angle_diff = self._normalize_angle(target_angle - current_pose[2])

        # ============= 4. 路径点切换逻辑 =============
        if distance < self.position_threshold:
            if self.current_waypoint_idx == len(self.waypoints) - 1:
                # 到达最后一个路径点，执行最终角度对齐
                return self._final_orientation_control(current_pose)
            else:
                # 切换到下一个路径点，重置PID状态避免状态跳跃
                self.current_waypoint_idx += 1
                self.reset()  # 重置PID积分器和滤波历史
                return self.compute_control(current_pose)

        # ============= 5. PID线速度控制 =============
        # 使用距离作为误差输入，控制机器人接近目标的速度
        linear_error = distance  # 距离误差作为PID输入
        linear_output, self.linear_integral = self._pid_control(
            linear_error, self.prev_linear_error, self.linear_integral,
            self.k_p_linear, self.k_i_linear, self.k_d_linear, self.dt
        )
        self.prev_linear_error = linear_error  # 更新历史误差用于下次微分计算

        # ============= 6. 多重优化处理 =============
        # 6.1 死区控制：过滤微小误差，避免不必要的抖动
        linear_output = self._apply_deadzone(linear_output, self.linear_deadzone)

        # 6.2 平滑速度过渡：根据距离动态调整允许的最大速度
        speed = self._compute_smooth_speed(distance, self.max_linear_speed)

        # 6.3 最终速度限制
        linear_output = np.clip(linear_output, -speed, speed)

        # ============= 7. 分解线速度到XY分量 =============
        # 将标量线速度分解为X、Y方向的分量
        x_vel = linear_output * np.cos(target_angle)  # 沿目标方向的X速度
        y_vel = linear_output * np.sin(target_angle)  # 沿目标方向的Y速度

        # ============= 8. PID角速度控制 =============
        # 使用角度误差控制机器人转向
        angular_error = angle_diff  # 角度误差作为PID输入
        angular_output, self.angular_integral = self._pid_control(
            angular_error, self.prev_angular_error, self.angular_integral,
            self.k_p_angular, self.k_i_angular, self.k_d_angular, self.dt
        )
        self.prev_angular_error = angular_error  # 更新历史误差

        # ============= 9. 角速度优化处理 =============
        # 9.1 死区控制：避免微小角度误差引起过度转向
        angular_output = self._apply_deadzone(angular_output, self.angular_deadzone)

        # 9.2 速度限制
        theta_vel = np.clip(angular_output, -self.max_angular_speed, self.max_angular_speed)

        # ============= 10. 低通滤波平滑输出 =============
        # 应用低通滤波减少高频抖动，提供平滑的控制输出
        x_vel = self._low_pass_filter(
            x_vel,
            self.prev_linear_vel * np.cos(target_angle),  # 历史X分量
            self.alpha_linear
        )
        y_vel = self._low_pass_filter(
            y_vel,
            self.prev_linear_vel * np.sin(target_angle),  # 历史Y分量
            self.alpha_linear
        )
        theta_vel = self._low_pass_filter(theta_vel, self.prev_angular_vel, self.alpha_angular)

        # ============= 11. 更新滤波历史状态 =============
        # 保存当前输出用于下次滤波计算
        self.prev_linear_vel = np.sqrt(x_vel**2 + y_vel**2)  # 当前线速度大小
        self.prev_angular_vel = theta_vel  # 当前角速度

        return x_vel, y_vel, theta_vel, angle_diff

    def _final_orientation_control(self, current_pose: np.ndarray) -> Tuple[float, float, float, float]:
        """
        最终角度对齐控制 - 精确角度定位

        当机器人到达目标位置后，执行最终的角度对齐。
        这个阶段只控制角度，线速度为0。

        Args:
            current_pose: 当前机器人姿态

        Returns:
            tuple: (0, 0, theta_vel, angle_diff) - 只输出角速度
        """
        # 计算最终角度误差
        if self.final_angle is not None:
            # 使用指定的最终角度
            final_angle_diff = self._normalize_angle(self.final_angle - current_pose[2])
        else:
            # 使用路径点中的角度
            target = self.waypoints[self.current_waypoint_idx]
            final_angle_diff = self._normalize_angle(target[2] - current_pose[2])

        # 检查是否已达到角度精度要求
        if abs(final_angle_diff) < self.angle_threshold:
            return 0.0, 0.0, 0.0, final_angle_diff

        # 使用高PID参数进行最终角度对齐，快速到位
        angular_output, self.angular_integral = self._pid_control(
            final_angle_diff, self.prev_angular_error, self.angular_integral,
            self.k_p_angular * 0.9, self.k_i_angular * 0.9, self.k_d_angular * 0.9, self.dt
        )

        angular_output = self._apply_deadzone(angular_output, self.angular_deadzone * 0.3)  # 更小的死区
        theta_vel = np.clip(angular_output, -self.max_angular_speed * 0.9, self.max_angular_speed * 0.9)
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