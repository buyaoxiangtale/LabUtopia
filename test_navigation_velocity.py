#!/usr/bin/env python3
"""
测试导航任务的速度信息功能
演示如何获取每个路径段的速度大小和方向
"""

import numpy as np

# 模拟导航任务类（简化版本）
class MockNavigationTask:
    def __init__(self):
        # 模拟路径数据
        self.current_path = [
            [0.0, 0.0, 0.0],      # 起点
            [1.0, 0.0, 0.0],      # 沿X轴移动
            [1.0, 1.0, 1.57],     # 转弯向上
            [2.0, 1.0, 0.0],      # 继续向前
            [2.0, 2.0, 1.57]      # 到达终点
        ]
        self.current_start = [0.0, 0.0]
        self.current_end = [2.0, 2.0]

    def _calculate_segment_velocities(self):
        """计算路径中每个段的速度大小和方向信息"""
        if self.current_path is None or len(self.current_path) < 2:
            return []

        velocities = []
        max_linear_speed = 0.02  # 默认最大线速度

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

    def print_velocity_info(self):
        """打印当前路径的详细速度信息"""
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

def test_velocity_features():
    """测试速度信息功能"""
    print("🧪 导航任务速度信息功能测试")
    print("=" * 50)

    # 创建模拟任务
    task = MockNavigationTask()

    # 显示路径信息
    print("📍 模拟路径:")
    print(f"   起点: {task.current_start}")
    print(f"   终点: {task.current_end}")
    print(f"   路径点数: {len(task.current_path)}")

    # 显示速度信息
    task.print_velocity_info()

    # 模拟状态获取
    print("\n🔄 模拟状态信息:")
    mock_state = {
        "current_pose": np.array([0.5, 0.0, 0.0]),  # 在第一段中间
        "segment_velocities": task._calculate_segment_velocities(),
    }

    print(f"   当前位置: [{mock_state['current_pose'][0]:.1f}, {mock_state['current_pose'][1]:.1f}]")
    print(f"   路径段数: {len(mock_state['segment_velocities'])}")

    # 显示第一段的速度信息
    if mock_state['segment_velocities']:
        first_segment = mock_state['segment_velocities'][0]
        print(f"\n📍 第一段速度信息:")
        print(f"   起点: {first_segment['start_point']}")
        print(f"   终点: {first_segment['end_point']}")
        print(f"   距离: {first_segment['distance']:.3f}m")
        print(f"   方向: {first_segment['direction_deg']:.1f}°")
        print(f"   速度大小: {first_segment['velocity_magnitude']*100:.1f}cm/s")
        print(f"   速度向量: [{first_segment['velocity_vector'][0]*100:.1f}, {first_segment['velocity_vector'][1]*100:.1f}] cm/s")

if __name__ == "__main__":
    test_velocity_features()
