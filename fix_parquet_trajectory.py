#!/usr/bin/env python3
"""
Parquet 轨迹数据修复工具

功能：
1. 统计现有 parquet 文件的数据问题
2. 修复列名（去掉嵌套的 observation）
3. 修复 action 数据（从单位矩阵改为真实轨迹）
"""

import pandas as pd
import numpy as np
from pathlib import Path
import json
import shutil


def analyze_parquet_file(parquet_path: Path):
    """分析单个 parquet 文件"""
    print(f"\n分析文件: {parquet_path}")

    df = pd.read_parquet(parquet_path)

    print(f"  Shape: {df.shape}")
    print(f"  列名: {list(df.columns)}")

    # 检查 action
    is_identity = False
    if 'action' in df.columns:
        actions = df['action'].tolist()
        if len(actions) > 0:
            first_action = actions[0]
            print(f"  Action 形状: {np.array(first_action).shape}")
            print(f"  第1帧 Action: {first_action}")

            # 检查是否是单位矩阵
            try:
                action_matrix = np.array(first_action).flatten()
                identity_matrix = np.eye(4).flatten()
                is_identity = np.allclose(action_matrix, identity_matrix)
                print(f"  Action 是单位矩阵: {is_identity}")
            except:
                print(f"  Action 是单位矩阵: True (默认)")
                is_identity = True

            if is_identity:
                print(f"  ❌ 问题: Action 是单位矩阵，不是真实轨迹")

    # 检查列名
    has_nested_observation = 'observation.observation.camera_intrinsic' in df.columns
    if has_nested_observation:
        print(f"  ❌ 问题: 列名嵌套 observation.observation.camera_intrinsic")
    else:
        print(f"  ✅ 列名正确")

    return df.shape[0], has_nested_observation


def fix_parquet_file(
    parquet_path: Path,
    output_path: Path,
    camera_trajectory: np.ndarray  # (N, 4, 4) 真实轨迹
):
    """
    修复单个 parquet 文件

    Args:
        parquet_path: 原始 parquet 文件路径
        output_path: 修复后的输出路径
        camera_trajectory: 真实的相机轨迹数据 (N, 4, 4)
    """
    df = pd.read_parquet(parquet_path)

    # 1. 修复列名：去掉嵌套的 observation
    column_mapping = {
        'observation.observation.camera_intrinsic': 'observation.camera_intrinsic',
        'observation.observation.camera_extrinsic': 'observation.camera_extrinsic',
    }

    for old_col, new_col in column_mapping.items():
        if old_col in df.columns:
            df.rename(columns={old_col: new_col}, inplace=True)
            print(f"  ✓ 重命名列: {old_col} -> {new_col}")

    # 2. 修复 action 数据
    if 'action' in df.columns and camera_trajectory is not None:
        num_frames = len(df)

        if len(camera_trajectory) != num_frames:
            print(f"  ⚠️  警告: 轨迹长度 {len(camera_trajectory)} != 帧数 {num_frames}")
            # 调整轨迹长度
            if len(camera_trajectory) > num_frames:
                camera_trajectory = camera_trajectory[:num_frames]
            else:
                # 重复最后一个位姿
                last_pose = camera_trajectory[-1]
                padding = [last_pose] * (num_frames - len(camera_trajectory))
                camera_trajectory = np.vstack([camera_trajectory] + padding)
                camera_trajectory = camera_trajectory.reshape(num_frames, 4, 4)

        # 将轨迹写入 action 列
        df['action'] = [camera_trajectory[i].tolist() for i in range(num_frames)]
        print(f"  ✓ 修复 action 数据: 从单位矩阵改为真实轨迹 (N={num_frames})")

    # 3. 保存修复后的文件
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(output_path, index=False, compression='snappy')
    print(f"  ✓ 保存修复后的文件: {output_path}")


def batch_analyze_parquet_files(root_dir: Path):
    """批量分析所有 parquet 文件"""
    print("=" * 80)
    print("批量分析 Parquet 文件")
    print("=" * 80)

    parquet_files = list(root_dir.glob("**/episode_*.parquet"))

    if not parquet_files:
        print(f"❌ 未找到 parquet 文件在: {root_dir}")
        return

    print(f"找到 {len(parquet_files)} 个 parquet 文件\n")

    total_frames = 0
    files_with_issues = []

    for parquet_path in parquet_files:
        num_frames, has_issue = analyze_parquet_file(parquet_path)
        total_frames += num_frames
        if has_issue:
            files_with_issues.append(parquet_path)

    print("\n" + "=" * 80)
    print("统计汇总")
    print("=" * 80)
    print(f"总文件数: {len(parquet_files)}")
    print(f"总帧数: {total_frames}")
    print(f"有问题的文件数: {len(files_with_issues)}")

    if files_with_issues:
        print(f"\n需要修复的文件:")
        for f in files_with_issues[:5]:  # 只显示前5个
            print(f"  - {f}")
        if len(files_with_issues) > 5:
            print(f"  ... 和其他 {len(files_with_issues) - 5} 个文件")

    return files_with_issues


def generate_mock_trajectory(num_frames: int) -> np.ndarray:
    """
    生成模拟轨迹数据用于测试

    真实场景中，应该从实际的相机位姿数据中获取轨迹
    """
    # 模拟一条简单的直线轨迹
    trajectory = np.zeros((num_frames, 4, 4))
    trajectory[:, :, :] = np.eye(4)[:, :, :]

    # 添加一些位移（模拟机器人移动）
    for i in range(num_frames):
        trajectory[i, 0, 3] = i * 0.01  # x 方向移动
        trajectory[i, 1, 3] = i * 0.005  # y 方向移动

    return trajectory


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="Parquet 轨迹数据修复工具")
    parser.add_argument('--root-dir', type=str, required=True, help='数据集根目录')
    parser.add_argument('--mode', type=str, choices=['analyze', 'fix'], default='analyze',
                       help='模式: analyze（分析）或 fix（修复）')
    parser.add_argument('--use-mock-trajectory', action='store_true',
                       help='使用模拟轨迹进行修复（仅用于测试）')

    args = parser.parse_args()

    root_dir = Path(args.root_dir)

    if not root_dir.exists():
        print(f"❌ 目录不存在: {root_dir}")
        return

    if args.mode == 'analyze':
        # 分析模式
        files_with_issues = batch_analyze_parquet_files(root_dir)

        if files_with_issues:
            print(f"\n💡 下一步: 运行修复命令")
            print(f"   python fix_parquet_trajectory.py --root-dir {root_dir} --mode fix")

    elif args.mode == 'fix':
        # 修复模式
        parquet_files = list(root_dir.glob("**/episode_*.parquet"))

        print("=" * 80)
        print("修复 Parquet 文件")
        print("=" * 80)

        for parquet_path in parquet_files:
            print(f"\n处理文件: {parquet_path}")

            # 获取真实轨迹（这里使用模拟轨迹，实际应该从数据源获取）
            df = pd.read_parquet(parquet_path)
            num_frames = len(df)

            if args.use_mock_trajectory:
                print(f"  ⚠️  使用模拟轨迹（仅用于测试）")
                camera_trajectory = generate_mock_trajectory(num_frames)
            else:
                print(f"  ❌ 错误: 未提供真实轨迹数据")
                print(f"  提示: 请使用 --use-mock-trajectory 测试，或修改代码以加载真实轨迹")
                print(f"  真实轨迹应该从相机位姿数据中获取")
                return

            # 修复文件（保存到新位置）
            output_path = parquet_path.parent / f"{parquet_path.stem}_fixed.parquet"
            fix_parquet_file(parquet_path, output_path, camera_trajectory)


if __name__ == '__main__':
    main()
