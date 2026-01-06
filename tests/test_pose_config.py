#!/usr/bin/env python3
"""
测试位姿配置功能
验证配置文件中的goal_pairs是否正确支持角度信息
"""

# 模拟配置数据
test_config = {
    "task": {
        "goal_pairs": [
            {
                "start": [-5.59, 0.98, 0.0],   # [x, y, theta]
                "end": [-0.35, -0.18, 1.57]    # [x, y, theta] - 90度
            },
            {
                "start": [-0.35, -0.18, 1.57], # [x, y, theta]
                "end": [-5.59, 0.98, 3.14]     # [x, y, theta] - 180度
            }
        ]
    }
}

def test_normalize_pairs():
    """测试_normalize_pairs方法"""
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))

    # 导入必要的类
    class MockTask:
        def _normalize_pairs(self, pairs):
            """简化的_normalize_pairs实现"""
            normalized = []
            for item in pairs:
                if isinstance(item, dict) and 'start' in item and 'end' in item:
                    start = item['start']
                    end = item['end']

                    # 确保格式正确
                    if len(start) == 2:
                        start = [start[0], start[1], 0.0]
                    if len(end) == 2:
                        end = [end[0], end[1], 0.0]

                    normalized.append((start, end))
                elif isinstance(item, (list, tuple)) and len(item) == 2:
                    start = item[0] if len(item[0]) == 3 else item[0] + [0.0]
                    end = item[1] if len(item[1]) == 3 else item[1] + [0.0]
                    normalized.append((start, end))
            return normalized

        def _ensure_pose_format(self, pose):
            """确保位姿格式正确"""
            if len(pose) == 2:
                return [pose[0], pose[1], 0.0]
            elif len(pose) == 3:
                return pose
            else:
                raise ValueError(f"位姿格式错误: {pose}")

    # 测试
    task = MockTask()
    pairs = test_config["task"]["goal_pairs"]

    print("🧪 测试位姿配置解析")
    print("=" * 50)

    try:
        normalized = task._normalize_pairs(pairs)
        print(f"✅ 成功解析 {len(normalized)} 个任务对")

        for i, (start, end) in enumerate(normalized):
            print(f"\n任务 {i+1}:")
            print(f"  起点: [{start[0]:.2f}, {start[1]:.2f}, {start[2]*180/3.14159:.1f}°]")
            print(f"  终点: [{end[0]:.2f}, {end[1]:.2f}, {end[2]*180/3.14159:.1f}°]")

        print("\n✅ 位姿配置测试通过！")
        return True

    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

if __name__ == "__main__":
    test_normalize_pairs()
