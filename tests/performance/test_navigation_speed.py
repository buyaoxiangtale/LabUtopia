#!/usr/bin/env python3
"""
测试改进后导航控制器的速度性能
"""

import numpy as np
import time
from controllers.robot_controllers.ridgebase.ridgebase_controller_new import RidgebaseControllerNew

def test_controller_speed():
    """测试控制器速度响应"""
    print("🧪 测试改进后导航控制器速度性能")
    print("=" * 50)

    # 创建控制器实例（模拟参数）
    controller = RidgebaseControllerNew(
        robot_articulation=None,  # 测试模式，不需要真实关节
        max_linear_speed=0.02,    # 2cm/s
        max_angular_speed=1.5,    # 1.5 rad/s
        position_threshold=0.08,  # 8cm
        angle_threshold=0.1       # 0.1 rad
    )

    # 测试场景：距离目标10cm，角度偏差0.2 rad
    test_distance = 0.1  # 10cm
    test_angle_diff = 0.2  # 0.2 rad

    print(f"📍 测试条件:")
    print(f"   距离目标: {test_distance*100:.1f}cm")
    print(f"   角度偏差: {test_angle_diff:.3f}rad")
    print(f"   最大线速度: {controller.max_linear_speed*100:.1f}cm/s")
    print(f"   最大角速度: {controller.max_angular_speed:.1f}rad/s")
    print()

    # 模拟控制循环
    current_pose = np.array([0.0, 0.0, 0.0])  # 起始姿态
    target = np.array([test_distance, 0.0, 0.0])  # 目标位置

    print("🚀 控制响应测试:")
    print("   步骤 | 距离(cm) | 线速度(cm/s) | 角速度(rad/s)")
    print("   ------|----------|--------------|---------------")

    for step in range(10):
        # 计算距离和角度
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        distance = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        angle_diff = controller._normalize_angle(target_angle - current_pose[2])

        # 获取控制输出
        x_vel, y_vel, theta_vel, _ = controller.compute_control(current_pose)

        # 计算线速度大小
        linear_speed = np.sqrt(x_vel**2 + y_vel**2)

        print(f"   {step+1:2d}   | {distance*100:8.1f} | {linear_speed*100:10.1f}   | {theta_vel:11.3f}")

        # 更新位置（简化模拟）
        current_pose[0] += x_vel * controller.dt
        current_pose[1] += y_vel * controller.dt
        current_pose[2] += theta_vel * controller.dt

        if distance < controller.position_threshold:
            break

    print()
    print("✅ 控制器参数配置:")
    print(f"   PID线速度: P={controller.k_p_linear}, I={controller.k_i_linear}, D={controller.k_d_linear}")
    print(f"   PID角速度: P={controller.k_p_angular}, I={controller.k_i_angular}, D={controller.k_d_angular}")
    print(f"   死区: 线={controller.linear_deadzone}, 角={controller.angular_deadzone}")
    print(f"   滤波: 线={controller.alpha_linear}, 角={controller.alpha_angular}")
    print()
    print("💡 预期改进:")
    print("   - 更高的初始响应速度")
    print("   - 更快的接近目标速度")
    print("   - 保持一定的平滑性以减少抖动")

if __name__ == "__main__":
    test_controller_speed()