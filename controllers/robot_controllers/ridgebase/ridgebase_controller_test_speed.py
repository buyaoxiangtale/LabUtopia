# import numpy as np
# from typing import List, Tuple, Optional
# # 注释掉Isaac Sim相关导入，用于测试模式
# # from isaacsim.core.api.articulations import ArticulationSubset
# # from isaacsim.core.prims.impl import Articulation
# # from isaacsim.core.utils.types import ArticulationAction

# class RidgebaseController:
#     """
#     原始Ridgebase控制器 - 简单的比例控制实现

#     控制策略：
#     - 使用简单的比例(P)控制计算速度
#     - 距离越近速度越慢（线性关系）
#     - 直接输出关节位置控制指令

#     优点：简单、稳定、无抖动
#     缺点：响应较慢、控制精度有限
#     """
#     def __init__(
#         self,
#         robot_articulation=None,  # 可选，用于测试模式
#         max_linear_speed: float = 1.0,
#         max_angular_speed: float = 1.0,
#         position_threshold: float = 0.1,
#         angle_threshold: float = 0.1,
#         dt: float = 0.01,
#         final_angle: float = None,
#         test_mode: bool = False  # 测试模式标志
#     ):
#         """
#         初始化Ridgebase控制器

#         Args:
#             robot_articulation: 机器人关节对象
#             max_linear_speed: 最大线速度 (m/s)
#             max_angular_speed: 最大角速度 (rad/s)
#             position_threshold: 位置到达阈值 (m)
#             angle_threshold: 角度到达阈值 (rad)
#             dt: 控制时间步长 (s)
#             final_angle: 最终目标角度 (rad)，可选
#         """
#         # 基本控制参数
#         self.max_linear_speed = max_linear_speed
#         self.max_angular_speed = max_angular_speed
#         self.position_threshold = position_threshold
#         self.angle_threshold = angle_threshold
#         self.dt = 0.02  # 固定控制周期
#         self.final_angle = final_angle
#         self.test_mode = test_mode or (robot_articulation is None)  # 自动检测测试模式

#         # 简单的比例控制增益
#         self.k_p_linear = 1      # 线速度比例增益
#         self.k_p_angular = 4     # 角速度比例增益（更高，因为角度控制需要更灵敏）

#         # 路径和状态变量
#         self.waypoints = None                    # 路径点列表
#         self.current_waypoint_idx = 0            # 当前路径点索引

#         # 创建关节子集用于控制（测试模式下跳过）
#         if not self.test_mode:
#             # Ridgebase机器人使用三个虚拟关节：X平移、Y平移、Z旋转
#             self._joints_subset = ArticulationSubset(
#                 robot_articulation,
#                 ["dummy_base_prismatic_x_joint", "dummy_base_prismatic_y_joint", "dummy_base_revolute_z_joint"]
#             )
#         else:
#             # 测试模式：创建模拟关节位置
#             self._joints_subset = None
#             self._test_joint_positions = np.array([0.0, 0.0, 0.0])  # 模拟关节位置

#     def set_waypoints(self, waypoints: List[Tuple[float, float, float]], final_angle: Optional[float] = None) -> None:
#         """
#         设置导航路径点

#         Args:
#             waypoints: 路径点列表，每个点包含(x, y, theta)
#             final_angle: 最终角度要求，可选
#         """
#         self.waypoints = np.array(waypoints)
#         self.current_waypoint_idx = 0
#         self.final_angle = final_angle  

#     def compute_control(self, current_pose: np.ndarray) -> Tuple[float, float, float]:
#         """
#         计算控制输出 - 简单比例控制策略

#         控制逻辑：
#         1. 检查路径是否完成
#         2. 计算当前位置（包含关节偏移）
#         3. 计算距离和角度误差
#         4. 根据距离判断是否到达路径点
#         5. 计算线速度和角速度

#         Args:
#             current_pose: 当前机器人姿态 [x, y, theta]

#         Returns:
#             tuple: (x_vel, y_vel, theta_vel, angle_diff)
#         """
#         # 检查路径是否有效
#         if self.waypoints is None or self.current_waypoint_idx >= len(self.waypoints):
#             return 0.0, 0.0, 0.0, 0.0

#         # 获取当前关节位置并更新姿态
#         # 这是因为Isaac Sim中的机器人姿态是相对的，需要加上关节的绝对位置
#         if not self.test_mode:
#             joint_positions = self._joints_subset.get_joint_positions()
#         else:
#             # 测试模式：使用模拟关节位置
#             joint_positions = self._test_joint_positions

#         target = self.waypoints[self.current_waypoint_idx]
#         current_pose[0] += joint_positions[0]  # X位置
#         current_pose[1] += joint_positions[1]  # Y位置
#         current_pose[2] += joint_positions[2]  # 角度

#         # 计算距离误差
#         dx = target[0] - current_pose[0]
#         dy = target[1] - current_pose[1]
#         distance = np.sqrt(dx**2 + dy**2)

#         # 计算角度误差（朝向目标的方向）
#         target_angle = np.arctan2(dy, dx)
#         angle_diff = (target_angle - current_pose[2]) % (2 * np.pi)
#         # 角度归一化到[-pi, pi]范围
#         if angle_diff > np.pi:
#             angle_diff -= 2 * np.pi
#         elif angle_diff < -np.pi:
#             angle_diff += 2 * np.pi

#         # 检查是否到达当前路径点
#         if distance < self.position_threshold:
#             if self.current_waypoint_idx == len(self.waypoints) - 1:
#                 # 到达最后一个路径点，进行最终角度对齐
#                 if self.final_angle is not None:
#                     final_angle_diff = (self.final_angle - current_pose[2]) % (2 * np.pi)
#                     if final_angle_diff > np.pi:
#                         final_angle_diff -= 2 * np.pi
#                 else:
#                     final_angle_diff = (target[2] - current_pose[2]) % (2 * np.pi)
#                     if final_angle_diff > np.pi:
#                         final_angle_diff -= 2 * np.pi

#                 if abs(final_angle_diff) < self.angle_threshold:
#                     return 0.0, 0.0, 0.0, final_angle_diff  # 完全停止
#                 return 0.0, 0.0, self.k_p_angular * final_angle_diff, final_angle_diff  # 只进行角度调整
#             else:
#                 # 切换到下一个路径点
#                 self.current_waypoint_idx += 1
#                 return self.compute_control(current_pose)

#         # 计算线速度 - 简单的距离比例控制
#         # 距离越近，速度越慢，但不超过最大速度
#         speed = min(distance * 0.2, self.max_linear_speed)
#         x_vel = speed * np.cos(target_angle)
#         y_vel = speed * np.sin(target_angle)

#         # 计算角速度 - 简单的比例控制
#         theta_vel = self.k_p_angular * angle_diff

#         return x_vel, y_vel, theta_vel, angle_diff

#     def get_action(self, current_pose: np.ndarray) -> Tuple[Optional[ArticulationAction], bool]:
#         """
#         获取控制动作 - 生成关节位置指令

#         工作流程：
#         1. 计算期望速度
#         2. 限制速度范围
#         3. 计算下一时刻关节位置
#         4. 生成关节动作
#         5. 判断任务是否完成

#         Args:
#             current_pose: 当前姿态

#         Returns:
#             tuple: (action, done) - 关节动作和完成标志
#         """
#         # 计算控制速度
#         x_vel, y_vel, theta_vel, angle_diff = self.compute_control(current_pose)

#         # 速度限制
#         x_vel = np.clip(abs(x_vel), 0, self.max_linear_speed) * np.sign(x_vel)
#         y_vel = np.clip(abs(y_vel), 0, self.max_linear_speed) * np.sign(y_vel)
#         theta_vel = np.clip(theta_vel, -self.max_angular_speed, self.max_angular_speed)

#         # 获取当前关节位置
#         if not self.test_mode:
#             joint_positions = self._joints_subset.get_joint_positions()
#         else:
#             joint_positions = self._test_joint_positions

#         # 计算下一时刻关节位置（位置控制模式）
#         next_x = joint_positions[0] + x_vel * self.dt
#         next_y = joint_positions[1] + y_vel * self.dt
#         next_theta = joint_positions[2] + theta_vel * self.dt

#         # 生成关节动作
#         position = np.array([next_x, next_y, next_theta])
#         action = self._joints_subset.make_articulation_action(
#             joint_positions=position,
#             joint_velocities=None  # 使用位置控制
#         )

#         # 判断任务完成条件
#         if self.final_angle is None:
#             # 无特定角度要求：当前是最后一个路径点且角速度很小
#             done = (self.waypoints is not None and
#                     self.current_waypoint_idx == len(self.waypoints) - 1 and
#                     abs(theta_vel) < self.angle_threshold)
#         else:
#             # 有角度要求：满足位置和角度条件
#             done = (self.waypoints is not None and
#                     self.current_waypoint_idx == len(self.waypoints) - 1 and
#                     abs(theta_vel) < self.angle_threshold and
#                     abs(angle_diff) < self.angle_threshold)

#         return action, done

#     def is_path_complete(self) -> bool:
#         """
#         检查路径是否完成

#         Returns:
#             bool: 路径是否完成
#         """
#         return (self.waypoints is not None and
#                 self.current_waypoint_idx >= len(self.waypoints))


# def test_speed_analysis():
#     """
#     测试函数：分析控制器在不同距离和角度下的速度输出
#     输出每个测试点的速度大小和方向
#     """
#     print("🧪 Ridgebase控制器速度分析测试")
#     print("=" * 60)

#     # 创建控制器实例（不需要真实机器人关节用于测试）
#     controller = RidgebaseController(
#         robot_articulation=None,  # 测试模式
#         max_linear_speed=0.02,    # 2cm/s
#         max_angular_speed=1.5,    # 1.5 rad/s
#         position_threshold=0.08,  # 8cm
#         angle_threshold=0.1       # 0.1 rad
#     )

#     # 设置测试路径点
#     waypoints = [
#         (0.1, 0.0, 0.0),    # 10cm前方
#         (0.0, 0.1, np.pi/2), # 10cm右侧，90度
#         (-0.1, 0.0, np.pi),  # 10cm后方，180度
#         (0.0, -0.1, -np.pi/2) # 10cm左侧，-90度
#     ]
#     controller.set_waypoints(waypoints)

#     print("📍 测试路径点:")
#     for i, wp in enumerate(waypoints):
#         print(f"   点{i+1}: ({wp[0]*100:.1f}cm, {wp[1]*100:.1f}cm, {wp[2]*180/np.pi:.1f}°)")
#     print()

#     print("🚀 速度分析结果:")
#     print("   距离(cm) | 角度(°) | 线速度(cm/s) | 方向(°) | 角速度(rad/s)")
#     print("   ----------|----------|--------------|----------|---------------")

#     # 测试不同距离和角度组合
#     test_distances = [0.2, 0.15, 0.1, 0.05]  # 20cm, 15cm, 10cm, 5cm
#     test_angles = [0, np.pi/4, np.pi/2, np.pi, -np.pi/2]  # 0°, 45°, 90°, 180°, -90°

#     results = []

#     for dist in test_distances:
#         for angle in test_angles:
#             # 模拟当前位置（距离目标dist，角度偏差angle）
#             current_pose = np.array([dist * np.cos(angle), dist * np.sin(angle), angle])

#             # 计算控制输出
#             x_vel, y_vel, theta_vel, angle_diff = controller.compute_control(current_pose)

#             # 计算线速度大小和方向
#             linear_speed = np.sqrt(x_vel**2 + y_vel**2)
#             if linear_speed > 0:
#                 linear_direction = np.arctan2(y_vel, x_vel) * 180 / np.pi
#             else:
#                 linear_direction = 0.0

#             # 记录结果
#             results.append({
#                 'distance': dist * 100,  # cm
#                 'angle': angle * 180 / np.pi,  # degrees
#                 'linear_speed': linear_speed * 100,  # cm/s
#                 'linear_direction': linear_direction,  # degrees
#                 'angular_speed': theta_vel  # rad/s
#             })

#             print(f"   {dist*100:8.1f} | {angle*180/np.pi:7.1f} | {linear_speed*100:11.1f}   | {linear_direction:7.1f} | {theta_vel:11.3f}")

#     print()
#     print("📊 分析总结:")
#     print(f"   测试点总数: {len(results)}")
#     print(f"   平均线速度: {np.mean([r['linear_speed'] for r in results]):.3f} cm/s")
#     print(f"   最大线速度: {np.max([r['linear_speed'] for r in results]):.3f} cm/s")
#     print(f"   平均角速度: {np.mean([abs(r['angular_speed']) for r in results]):.3f} rad/s")
#     print(f"   最大角速度: {np.max([abs(r['angular_speed']) for r in results]):.3f} rad/s")
#     # 按距离分组分析
#     print("\n📈 按距离分析:")
#     for dist in test_distances:
#         dist_results = [r for r in results if abs(r['distance'] - dist*100) < 0.1]
#         if dist_results:
#             avg_speed = np.mean([r['linear_speed'] for r in dist_results])
#             print(f"   距离{dist*100:.0f}cm: 平均线速度 {avg_speed:.2f} cm/s")

#     # 按角度分组分析
#     print("\n🎯 按角度分析:")
#     for angle in test_angles:
#         angle_results = [r for r in results if abs(r['angle'] - angle*180/np.pi) < 0.1]
#         if angle_results:
#             avg_angular_speed = np.mean([r['angular_speed'] for r in angle_results])
#             print(f"   角度{angle*180/np.pi:6.1f}°: 平均角速度 {avg_angular_speed:.3f} rad/s")
#     print("\n💡 关键发现:")
#     print("   - 线速度与距离成正比（距离越远速度越快）")
#     print("   - 角速度与角度偏差成正比（偏差越大转向越快）")
#     print("   - 控制器对角度变化更敏感（k_p_angular = 4 vs k_p_linear = 1）")
#     print("   - 在距离<8cm时会切换到角度对齐模式")


# if __name__ == "__main__":
#     test_speed_analysis()
import numpy as np
from typing import List, Tuple, Optional
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims.impl import Articulation
from isaacsim.core.utils.types import ArticulationAction

class RidgebaseController:
    """
    原始Ridgebase控制器 - 简单的比例控制实现

    控制策略：
    - 使用简单的比例(P)控制计算速度
    - 距离越近速度越慢（线性关系）
    - 直接输出关节位置控制指令

    优点：简单、稳定、无抖动
    缺点：响应较慢、控制精度有限
    """
    def __init__(
        self,
        robot_articulation: Articulation,
        max_linear_speed: float = 1.0,
        max_angular_speed: float = 1.0,
        position_threshold: float = 0.1,
        angle_threshold: float = 0.1,
        dt: float = 0.01,
        final_angle: float = None
    ):
        """
        初始化Ridgebase控制器

        Args:
            robot_articulation: 机器人关节对象
            max_linear_speed: 最大线速度 (m/s)
            max_angular_speed: 最大角速度 (rad/s)
            position_threshold: 位置到达阈值 (m)
            angle_threshold: 角度到达阈值 (rad)
            dt: 控制时间步长 (s)
            final_angle: 最终目标角度 (rad)，可选
        """
        # 基本控制参数
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold
        self.dt = 0.02  # 固定控制周期
        self.final_angle = final_angle

        # 简单的比例控制增益
        self.k_p_linear = 1      # 线速度比例增益
        self.k_p_angular = 4     # 角速度比例增益（更高，因为角度控制需要更灵敏）

        # 路径和状态变量
        self.waypoints = None                    # 路径点列表
        self.current_waypoint_idx = 0            # 当前路径点索引

        # 创建关节子集用于控制
        # Ridgebase机器人使用三个虚拟关节：X平移、Y平移、Z旋转
        self._joints_subset = ArticulationSubset(
            robot_articulation,
            ["dummy_base_prismatic_x_joint", "dummy_base_prismatic_y_joint", "dummy_base_revolute_z_joint"]
        )

    def set_waypoints(self, waypoints: List[Tuple[float, float, float]], final_angle: Optional[float] = None) -> None:
        """
        设置导航路径点

        Args:
            waypoints: 路径点列表，每个点包含(x, y, theta)
            final_angle: 最终角度要求，可选
        """
        self.waypoints = np.array(waypoints)
        self.current_waypoint_idx = 0
        self.final_angle = final_angle  

    def compute_control(self, current_pose: np.ndarray) -> Tuple[float, float, float]:
        """
        计算控制输出 - 简单比例控制策略

        控制逻辑：
        1. 检查路径是否完成
        2. 计算当前位置（包含关节偏移）
        3. 计算距离和角度误差
        4. 根据距离判断是否到达路径点
        5. 计算线速度和角速度

        Args:
            current_pose: 当前机器人姿态 [x, y, theta]

        Returns:
            tuple: (x_vel, y_vel, theta_vel, angle_diff)
        """
        # 检查路径是否有效
        if self.waypoints is None or self.current_waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0, 0.0, 0.0

        # 获取当前关节位置并更新姿态
        # 这是因为Isaac Sim中的机器人姿态是相对的，需要加上关节的绝对位置
        joint_positions = self._joints_subset.get_joint_positions()
        target = self.waypoints[self.current_waypoint_idx]
        current_pose[0] += joint_positions[0]  # X位置
        current_pose[1] += joint_positions[1]  # Y位置
        current_pose[2] += joint_positions[2]  # 角度

        # 计算距离误差
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        distance = np.sqrt(dx**2 + dy**2)

        # 计算角度误差（朝向目标的方向）
        target_angle = np.arctan2(dy, dx)
        angle_diff = (target_angle - current_pose[2]) % (2 * np.pi)
        # 角度归一化到[-pi, pi]范围
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        # 检查是否到达当前路径点
        if distance < self.position_threshold:
            if self.current_waypoint_idx == len(self.waypoints) - 1:
                # 到达最后一个路径点，进行最终角度对齐
                if self.final_angle is not None:
                    final_angle_diff = (self.final_angle - current_pose[2]) % (2 * np.pi)
                    if final_angle_diff > np.pi:
                        final_angle_diff -= 2 * np.pi
                else:
                    final_angle_diff = (target[2] - current_pose[2]) % (2 * np.pi)
                    if final_angle_diff > np.pi:
                        final_angle_diff -= 2 * np.pi

                if abs(final_angle_diff) < self.angle_threshold:
                    return 0.0, 0.0, 0.0, final_angle_diff  # 完全停止
                return 0.0, 0.0, self.k_p_angular * final_angle_diff, final_angle_diff  # 只进行角度调整
            else:
                # 切换到下一个路径点
                self.current_waypoint_idx += 1
                return self.compute_control(current_pose)

        # 计算线速度 - 简单的距离比例控制
        # 距离越近，速度越慢，但不超过最大速度
        speed = min(distance * 0.2, self.max_linear_speed)
        x_vel = speed * np.cos(target_angle)
        y_vel = speed * np.sin(target_angle)

        # 计算角速度 - 简单的比例控制
        theta_vel = self.k_p_angular * angle_diff

        return x_vel, y_vel, theta_vel, angle_diff

    def get_action(self, current_pose: np.ndarray) -> Tuple[Optional[ArticulationAction], bool]:
        """
        获取控制动作 - 生成关节位置指令

        工作流程：
        1. 计算期望速度
        2. 限制速度范围
        3. 计算下一时刻关节位置
        4. 生成关节动作
        5. 判断任务是否完成

        Args:
            current_pose: 当前姿态

        Returns:
            tuple: (action, done) - 关节动作和完成标志
        """
        # 计算控制速度
        x_vel, y_vel, theta_vel, angle_diff = self.compute_control(current_pose)

        # 速度限制
        x_vel = np.clip(abs(x_vel), 0, self.max_linear_speed) * np.sign(x_vel)
        y_vel = np.clip(abs(y_vel), 0, self.max_linear_speed) * np.sign(y_vel)
        theta_vel = np.clip(theta_vel, -self.max_angular_speed, self.max_angular_speed)

        # 获取当前关节位置
        joint_positions = self._joints_subset.get_joint_positions()

        # 计算下一时刻关节位置（位置控制模式）
        next_x = joint_positions[0] + x_vel * self.dt
        next_y = joint_positions[1] + y_vel * self.dt
        next_theta = joint_positions[2] + theta_vel * self.dt

        # 生成关节动作
        position = np.array([next_x, next_y, next_theta])
        action = self._joints_subset.make_articulation_action(
            joint_positions=position,
            joint_velocities=None  # 使用位置控制
        )

        # 判断任务完成条件
        if self.final_angle is None:
            # 无特定角度要求：当前是最后一个路径点且角速度很小
            done = (self.waypoints is not None and
                    self.current_waypoint_idx == len(self.waypoints) - 1 and
                    abs(theta_vel) < self.angle_threshold)
        else:
            # 有角度要求：满足位置和角度条件
            done = (self.waypoints is not None and
                    self.current_waypoint_idx == len(self.waypoints) - 1 and
                    abs(theta_vel) < self.angle_threshold and
                    abs(angle_diff) < self.angle_threshold)

        return action, done

    def is_path_complete(self) -> bool:
        """
        检查路径是否完成

        Returns:
            bool: 路径是否完成
        """
        return (self.waypoints is not None and
                self.current_waypoint_idx >= len(self.waypoints))
