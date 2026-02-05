#!/usr/bin/env python3
"""
验证 ParquetFormatCollector 修复是否正确
"""

import numpy as np
import pandas as pd
from pathlib import Path
from data_collectors.parquet_format_collector import ParquetFormatCollector


def test_parquet_format():
    """测试 Parquet 格式输出"""
    print("=" * 80)
    print("测试 ParquetFormatCollector 修复")
    print("=" * 80)

    # 创建临时收集器
    test_dir = Path('/tmp/test_parquet_fix')
    test_dir.mkdir(parents=True, exist_ok=True)

    collector = ParquetFormatCollector(
        save_dir=str(test_dir),
        camera_configs=[{
            'name': 'observation',
            'image_type': 'rgb+depth',
            'resolution': [256, 256]
        }],
        save_images=False,
        save_videos=False
    )

    # 模拟相机内参
    intrinsic = np.array([
        [122.166, 0.0, 128.0],
        [0.0, 122.166, 128.0],
        [0.0, 0.0, 1.0]
    ])
    collector.set_camera_intrinsics({'observation': intrinsic})

    # 模拟轨迹数据（相机位姿）
    num_steps = 10
    for i in range(num_steps):
        # 模拟相机移动（x 方向移动）
        pose = np.eye(4)
        pose[0, 3] = i * 0.1  # x 位置
        pose[1, 3] = i * 0.05  # y 位置

        # 缓存数据
        collector.cache_step(
            camera_images={},  # 空字典，因为我们不保存图像
            camera_poses={'observation': pose},
            joint_angles=np.array([0.0, 0.0, 0.0])
        )

    # 写入数据
    final_positions = np.array([0.0, 0.0, 0.0])
    collector.write_cached_data(final_positions)

    # 验证输出
    parquet_path = test_dir / 'trajectory_000000' / 'data' / 'chunk-000' / 'episode_000000.parquet'

    if not parquet_path.exists():
        print(f"❌ 失败: Parquet 文件未生成: {parquet_path}")
        return False

    df = pd.read_parquet(parquet_path)

    print(f"\n✓ Parquet 文件已生成: {parquet_path}")
    print(f"  Shape: {df.shape}")
    print(f"  列名: {list(df.columns)}")

    # 检查列名
    print(f"\n检查列名:")
    has_correct_intrinsic = 'observation.camera_intrinsic' in df.columns
    has_nested_intrinsic = 'observation.observation.camera_intrinsic' in df.columns

    if has_correct_intrinsic:
        print(f"  ✅ 正确的列名: observation.camera_intrinsic")
    else:
        print(f"  ❌ 缺少正确的列名: observation.camera_intrinsic")

    if has_nested_intrinsic:
        print(f"  ❌ 错误的嵌套列名: observation.observation.camera_intrinsic")
    else:
        print(f"  ✅ 没有嵌套列名")

    # 检查 action 数据
    print(f"\n检查 Action 数据:")
    is_identity = False  # 初始化变量

    if 'action' in df.columns:
        actions = df['action'].tolist()
        print(f"  Action 数量: {len(actions)}")

        if len(actions) > 0:
            first_action = actions[0]
            second_action = actions[1] if len(actions) > 1 else None
            print(f"  第1帧 Action: {first_action}")

            if second_action is not None:
                print(f"  第2帧 Action: {second_action}")

                # 比较第1帧和第2帧的位置部分
                # 4x4 变换矩阵的位置在最后一列: [tx, ty, tz, 1]
                # row[0][3] = tx, row[1][3] = ty, row[2][3] = tz, row[3][3] = 1
                first_tx = first_action[0][3] if isinstance(first_action[0][3], (int, float)) else first_action[0][3].item()
                first_ty = first_action[1][3] if isinstance(first_action[1][3], (int, float)) else first_action[1][3].item()
                first_tz = first_action[2][3] if isinstance(first_action[2][3], (int, float)) else first_action[2][3].item()

                second_tx = second_action[0][3] if isinstance(second_action[0][3], (int, float)) else second_action[0][3].item()
                second_ty = second_action[1][3] if isinstance(second_action[1][3], (int, float)) else second_action[1][3].item()
                second_tz = second_action[2][3] if isinstance(second_action[2][3], (int, float)) else second_action[2][3].item()

                first_pos = np.array([first_tx, first_ty, first_tz])
                second_pos = np.array([second_tx, second_ty, second_tz])

                are_different = not np.allclose(first_pos, second_pos)
                print(f"  第1帧 ≠ 第2帧: {are_different}")

                if are_different:
                    print(f"  ✅ Action 包含真实轨迹（帧与帧之间不同）")
                    print(f"     位置变化: {first_pos} → {second_pos}")
                else:
                    print(f"  ❌ Action 所有帧相同（可能是单位矩阵）")

                # 检查第1帧是否是单位矩阵（仅检查位置部分）
                is_identity = np.allclose(first_pos, np.array([0., 0., 0.]))

                if is_identity:
                    print(f"  ⚠️  第1帧是单位矩阵，但帧与帧之间有变化")

            # 检查 action 的数据结构
            if isinstance(first_action, list) or isinstance(first_action, np.ndarray):
                action_matrix = np.array(first_action)
                print(f"  Action 形状: {action_matrix.shape}")

                if len(actions) == 1:
                    # 只有1帧的情况
                    is_identity = np.allclose(action_matrix, np.eye(4))
                    if not is_identity:
                        print(f"  ✅ Action 不是单位矩阵（包含真实轨迹）")
                    else:
                        print(f"  ❌ Action 是单位矩阵（未使用真实轨迹）")
            else:
                print(f"  ⚠️  Action 数据类型未知: {type(first_action)}")

    # 清理
    import shutil
    shutil.rmtree(test_dir)

    print(f"\n✓ 测试完成，临时文件已清理")

    # 测试通过条件：
    # 1. 列名正确（没有嵌套的 observation）
    # 2. Action 数据包含真实轨迹（帧与帧之间不同）
    # 注意：第1帧可以是单位矩阵（起点在原点），只要后续帧有运动即可
    test_passed = has_correct_intrinsic and not has_nested_intrinsic

    # 如果有多帧，检查帧与帧之间是否不同
    if 'action' in df.columns and len(df) > 1:
        actions = df['action'].tolist()
        first_action = actions[0]
        second_action = actions[1]

        # 提取位置并比较
        first_pos = np.array([first_action[0][3], first_action[1][3], first_action[2][3]])
        second_pos = np.array([second_action[0][3], second_action[1][3], second_action[2][3]])

        frames_are_different = not np.allclose(first_pos, second_pos)
        test_passed = test_passed and frames_are_different

    return test_passed


if __name__ == '__main__':
    success = test_parquet_format()
    if success:
        print(f"\n🎉 所有测试通过！修复已生效。")
    else:
        print(f"\n❌ 测试失败！请检查代码。")
