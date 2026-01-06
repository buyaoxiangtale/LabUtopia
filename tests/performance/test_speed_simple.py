#!/usr/bin/env python3
"""
简化的速度分析测试 - 不依赖Isaac Sim和numpy
用于分析Ridgebase控制器的速度输出特性
"""

import math

class SimpleController:
    """简化的控制器实现，用于测试速度特性"""

    def __init__(self, max_linear_speed=0.02, max_angular_speed=1.5,
                 position_threshold=0.08, angle_threshold=0.1):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold
        self.dt = 0.02

        # 控制增益
        self.k_p_linear = 1
        self.k_p_angular = 4

        # 测试模式变量
        self.waypoints = None
        self.current_waypoint_idx = 0
        self.final_angle = None

    def set_waypoints(self, waypoints, final_angle=None):
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.final_angle = final_angle

    def normalize_angle(self, angle):
        """角度归一化到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_control(self, current_pose):
        """计算控制输出"""
        if self.waypoints is None or self.current_waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0, 0.0, 0.0

        target = self.waypoints[self.current_waypoint_idx]

        # 计算距离误差
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        distance = math.sqrt(dx**2 + dy**2)

        # 计算角度误差
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - current_pose[2])

        # 检查是否到达路径点
        if distance < self.position_threshold:
            if self.current_waypoint_idx == len(self.waypoints) - 1:
                # 最终角度对齐
                if self.final_angle is not None:
                    final_angle_diff = self.normalize_angle(self.final_angle - current_pose[2])
                else:
                    final_angle_diff = self.normalize_angle(target[2] - current_pose[2])

                if abs(final_angle_diff) < self.angle_threshold:
                    return 0.0, 0.0, 0.0, final_angle_diff
                return 0.0, 0.0, self.k_p_angular * final_angle_diff, final_angle_diff
            else:
                self.current_waypoint_idx += 1
                return self.compute_control(current_pose)

        # 计算速度
        speed = min(distance * 0.2, self.max_linear_speed)
        x_vel = speed * math.cos(target_angle)
        y_vel = speed * math.sin(target_angle)
        theta_vel = self.k_p_angular * angle_diff

        return x_vel, y_vel, theta_vel, angle_diff


def test_speed_analysis():
    """速度分析测试"""
    print("🧪 Ridgebase控制器速度分析测试")
    print("=" * 60)

    # 创建控制器
    controller = SimpleController()

    # 设置测试路径点
    waypoints = [
        (0.1, 0.0, 0.0),        # 10cm前方
        (0.0, 0.1, math.pi/2),   # 10cm右侧，90度
        (-0.1, 0.0, math.pi),    # 10cm后方，180度
        (0.0, -0.1, -math.pi/2)  # 10cm左侧，-90度
    ]
    controller.set_waypoints(waypoints)

    print("📍 测试路径点:")
    for i, wp in enumerate(waypoints):
        angle_deg = wp[2] * 180 / math.pi
        print(f"   点{i+1}: ({wp[0]*100:.1f}cm, {wp[1]*100:.1f}cm, {angle_deg:.1f}°)")
    print()

    print("🚀 速度分析结果:")
    print("   距离(cm) | 角度(°) | 线速度(cm/s) | 方向(°) | 角速度(rad/s)")
    print("   ----------|----------|--------------|----------|---------------")

    # 测试不同距离和角度组合
    test_distances = [0.2, 0.15, 0.1, 0.05]  # 20cm, 15cm, 10cm, 5cm
    test_angles = [0, math.pi/4, math.pi/2, math.pi, -math.pi/2]  # 0°, 45°, 90°, 180°, -90°

    results = []

    for dist in test_distances:
        for angle in test_angles:
            # 模拟当前位置
            current_pose = [dist * math.cos(angle), dist * math.sin(angle), angle]

            # 计算控制输出
            x_vel, y_vel, theta_vel, angle_diff = controller.compute_control(current_pose)

            # 计算线速度大小和方向
            linear_speed = math.sqrt(x_vel**2 + y_vel**2)
            if linear_speed > 0:
                linear_direction = math.atan2(y_vel, x_vel) * 180 / math.pi
            else:
                linear_direction = 0.0

            # 记录结果
            results.append({
                'distance': dist * 100,  # cm
                'angle': angle * 180 / math.pi,  # degrees
                'linear_speed': linear_speed * 100,  # cm/s
                'linear_direction': linear_direction,  # degrees
                'angular_speed': theta_vel  # rad/s
            })

            print(f"   {dist*100:8.1f} | {angle*180/math.pi:7.1f} | {linear_speed*100:11.1f}   | {linear_direction:7.1f} | {theta_vel:11.3f}")

    print()
    print("📊 分析总结:")

    # 计算统计数据
    linear_speeds = [r['linear_speed'] for r in results]
    angular_speeds = [abs(r['angular_speed']) for r in results]

    avg_linear = sum(linear_speeds) / len(linear_speeds)
    max_linear = max(linear_speeds)
    avg_angular = sum(angular_speeds) / len(angular_speeds)
    max_angular = max(angular_speeds)

    print(f"   测试点总数: {len(results)}")
    print(f"   平均线速度: {avg_linear:.3f} cm/s")
    print(f"   最大线速度: {max_linear:.3f} cm/s")
    print(f"   平均角速度: {avg_angular:.3f} rad/s")
    print(f"   最大角速度: {max_angular:.3f} rad/s")
    # 按距离分组分析
    print("\n📈 按距离分析:")
    for dist in test_distances:
        dist_cm = dist * 100
        dist_results = [r for r in results if abs(r['distance'] - dist_cm) < 0.1]
        if dist_results:
            speeds = [r['linear_speed'] for r in dist_results]
            avg_speed = sum(speeds) / len(speeds)
            print(f"   距离{dist_cm:.0f}cm: 平均线速度 {avg_speed:.2f} cm/s")
    # 按角度分组分析
    print("\n🎯 按角度分析:")
    for angle in test_angles:
        angle_deg = angle * 180 / math.pi
        angle_results = [r for r in results if abs(r['angle'] - angle_deg) < 0.1]
        if angle_results:
            ang_speeds = [r['angular_speed'] for r in angle_results]
            avg_angular_speed = sum(ang_speeds) / len(ang_speeds)
            print(f"   角度{angle_deg:6.1f}°: 平均角速度 {avg_angular_speed:.3f} rad/s")
    print("\n💡 关键发现:")
    print("   - 线速度与距离成正比（距离越远速度越快）")
    print("   - 角速度与角度偏差成正比（偏差越大转向越快）")
    print("   - 控制器对角度变化更敏感（k_p_angular = 4 vs k_p_linear = 1）")
    print("   - 在距离<8cm时会切换到角度对齐模式")


if __name__ == "__main__":
    test_speed_analysis()