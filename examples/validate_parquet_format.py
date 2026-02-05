#!/usr/bin/env python3
"""
验证 Parquet 数据格式的脚本

检查生成的 Parquet 文件是否符合 LeRobot 标准格式
"""

import pandas as pd
import numpy as np
from pathlib import Path
import sys


def validate_parquet_file(parquet_path: str):
    """
    验证单个 Parquet 文件的格式

    Args:
        parquet_path: Parquet 文件路径
    """
    print(f"\n{'='*70}")
    print(f"验证文件: {parquet_path}")
    print(f"{'='*70}")

    # 读取文件
    try:
        df = pd.read_parquet(parquet_path)
    except Exception as e:
        print(f"❌ 无法读取文件: {e}")
        return False

    # 基本信息
    print(f"\n✅ 文件读取成功")
    print(f"  形状: {df.shape}")
    print(f"  列数: {len(df.columns)}")
    print(f"  行数: {len(df)}")

    # 检查必需的列
    required_columns = {
        'episode_index': 'Episode 索引',
        'observation.state': '机器人状态',
        'action': '相机轨迹（action）',
    }

    print(f"\n检查必需列:")
    all_present = True
    for col, desc in required_columns.items():
        if col in df.columns:
            print(f"  ✅ {col}: {desc}")
        else:
            print(f"  ❌ {col}: {desc} - 缺失!")
            all_present = False

    if not all_present:
        print(f"\n❌ 缺少必需的列")
        return False

    # 检查相机数据列
    print(f"\n检查相机数据列:")
    camera_cols = [col for col in df.columns if col.startswith('observation.')]
    if camera_cols:
        print(f"  找到 {len(camera_cols)} 个相机相关列:")
        for col in camera_cols:
            print(f"    - {col}")
    else:
        print(f"  ⚠️  没有找到相机数据列")

    # 验证数据类型和形状
    print(f"\n验证数据内容:")

    # Episode index
    episode_idx = df['episode_index'].iloc[0]
    print(f"  episode_index: {episode_idx} (type: {type(episode_idx).__name__})")

    # Observation state
    state = df['observation.state'].iloc[0]
    print(f"  observation.state: {state} (type: {type(state).__name__}, len: {len(state) if hasattr(state, '__len__') else 'N/A'})")

    # Action (相机轨迹)
    action = df['action'].iloc[0]
    print(f"  action[0]: shape={np.array(action).shape} (应该是 4x4)")

    if len(df) > 1:
        action_last = df['action'].iloc[-1]
        print(f"  action[-1]: shape={np.array(action_last).shape}")

    # 检查相机内参（如果存在）
    if 'observation.observation.camera_intrinsic' in df.columns:
        intrinsic = df['observation.observation.camera_intrinsic'].iloc[0]
        if intrinsic is not None:
            intrinsic_arr = np.array(intrinsic)
            print(f"  observation.observation.camera_intrinsic: shape={intrinsic_arr.shape} (应该是 3x3)")
            if intrinsic_arr.shape == (3, 3):
                print(f"    ✅ 形状正确")
            else:
                print(f"    ❌ 形状错误，期望 (3, 3)")
        else:
            print(f"  observation.observation.camera_intrinsic: None (第一帧不应为 None)")
    else:
        print(f"  ⚠️  observation.observation.camera_intrinsic: 不存在")

    # 检查相机外参（如果存在）
    if 'observation.observation.camera_extrinsic' in df.columns:
        extrinsic = df['observation.observation.camera_extrinsic'].iloc[0]
        if extrinsic is not None:
            extrinsic_arr = np.array(extrinsic)
            print(f"  observation.observation.camera_extrinsic: shape={extrinsic_arr.shape} (应该是 4x4)")
            if extrinsic_arr.shape == (4, 4):
                print(f"    ✅ 形状正确")
            else:
                print(f"    ❌ 形状错误，期望 (4, 4)")
        else:
            print(f"  observation.observation.camera_extrinsic: None (第一帧不应为 None)")

    # 显示前几行
    print(f"\n前 3 行数据预览:")
    print(df.head(3).to_string())

    print(f"\n列名列表:")
    for i, col in enumerate(df.columns, 1):
        print(f"  {i:2d}. {col}")

    return True


def find_latest_parquet(data_dir: str):
    """
    查找最新的 Parquet 文件

    Args:
        data_dir: 数据目录
    """
    data_path = Path(data_dir)

    # 查找所有 parquet 文件
    parquet_files = list(data_path.glob("**/*.parquet"))

    if not parquet_files:
        print(f"❌ 在 {data_dir} 中没有找到 Parquet 文件")
        return None

    # 按修改时间排序，返回最新的
    parquet_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return parquet_files[0]


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='验证 Parquet 数据格式')
    parser.add_argument('--path', type=str, default='outputs/collect',
                       help='数据目录路径（默认: outputs/collect）')
    parser.add_argument('--file', type=str, default=None,
                       help='直接指定 Parquet 文件路径')

    args = parser.parse_args()

    if args.file:
        # 验证指定的文件
        validate_parquet_file(args.file)
    else:
        # 查找并验证最新的文件
        parquet_file = find_latest_parquet(args.path)

        if parquet_file:
            validate_parquet_file(str(parquet_file))
        else:
            print(f"\n❌ 未找到可验证的文件")
            sys.exit(1)


if __name__ == '__main__':
    main()
