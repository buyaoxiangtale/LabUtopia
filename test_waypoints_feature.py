#!/usr/bin/env python3
"""
测试waypoints和base_pose保存功能

这个脚本会检查HDF5文件中是否包含waypoints和base_pose字段
"""
import h5py
import numpy as np
import sys
from pathlib import Path

def check_episode_file(episode_path: str):
    """检查单个episode文件是否包含waypoints和base_pose"""
    print(f"\n{'='*80}")
    print(f"检查文件: {Path(episode_path).name}")
    print('='*80)

    try:
        with h5py.File(episode_path, 'r') as f:
            print("\n【包含的数据集】")
            datasets = list(f.keys())
            for i, key in enumerate(datasets, 1):
                dataset = f[key]
                print(f"{i}. {key}")
                print(f"   形状: {dataset.shape}")
                print(f"   数据类型: {dataset.dtype}")

            # 检查waypoints
            if 'waypoints' in datasets:
                print("\n" + "="*80)
                print("✅ 成功！发现 waypoints 数据")
                print("="*80)
                waypoints = f['waypoints'][:]
                print(f"路径点数量: {len(waypoints)}")
                print(f"路径点形状: {waypoints.shape}")
                print(f"\n前5个路径点:")
                for i, wp in enumerate(waypoints[:5]):
                    print(f"  {i+1}. x={wp[0]:.3f}, y={wp[1]:.3f}, theta={wp[2]:.3f}")
                if len(waypoints) > 5:
                    print(f"  ... 还有 {len(waypoints)-5} 个路径点")
            else:
                print("\n" + "="*80)
                print("❌ 未发现 waypoints 数据")
                print("="*80)

            # 检查base_pose
            if 'base_pose' in datasets:
                print("\n" + "="*80)
                print("✅ 成功！发现 base_pose 数据")
                print("="*80)
                base_pose = f['base_pose'][:]
                print(f"轨迹步数: {len(base_pose)}")
                print(f"轨迹形状: {base_pose.shape}")
                print(f"\n前5步位置:")
                for i, pose in enumerate(base_pose[:5]):
                    print(f"  {i+1}. x={pose[0]:.3f}, y={pose[1]:.3f}, theta={pose[2]:.3f}")
                if len(base_pose) > 5:
                    print(f"  ... 还有 {len(base_pose)-5} 步")

                # 计算轨迹总长度
                if len(base_pose) > 1:
                    distances = np.sqrt(np.diff(base_pose[:, 0])**2 + np.diff(base_pose[:, 1])**2)
                    total_distance = np.sum(distances)
                    print(f"\n轨迹总长度: {total_distance:.3f} 米")
            else:
                print("\n" + "="*80)
                print("❌ 未发现 base_pose 数据")
                print("="*80)

            # 检查其他关键字段
            print("\n" + "="*80)
            print("【其他数据字段】")
            print("="*80)
            if 'agent_pose' in datasets:
                print("✅ agent_pose (机器人关节角度)")
            if 'actions' in datasets:
                print("✅ actions (动作指令)")
            if 'language_instruction' in datasets:
                instruction = f['language_instruction'][()]
                print(f"✅ language_instruction: \"{instruction}\"")

            # 相机数据
            camera_keys = [k for k in datasets if 'camera' in k.lower()]
            if camera_keys:
                print(f"\n✅ 相机数据 ({len(camera_keys)} 个相机)")
                for cam in camera_keys:
                    print(f"   - {cam}")

    except Exception as e:
        print(f"\n❌ 读取文件时出错: {e}")
        import traceback
        traceback.print_exc()
        return False

    return True


def find_latest_episode(base_dir: str):
    """查找最新的episode文件"""
    base_path = Path(base_dir)
    if not base_path.exists():
        print(f"错误: 目录不存在 {base_dir}")
        return None

    # 查找所有episode文件
    episode_files = sorted(base_path.rglob("episode_*.h5"))
    if not episode_files:
        print(f"未找到episode文件在 {base_dir}")
        return None

    # 返回最新的文件
    return str(episode_files[-1])


def main():
    # 默认测试路径
    test_dirs = [
        "/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.11/21.21.14_level5_Navigation_smooth_1_11/dataset",
        "/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.11/20.45.15_level5_Navigation_smooth_1_4/dataset",
    ]

    print("="*80)
    print("Waypoints和Base_pose保存功能测试")
    print("="*80)

    # 如果提供了命令行参数，使用指定的目录或文件
    if len(sys.argv) > 1:
        test_path = sys.argv[1]
        if Path(test_path).is_file() and test_path.endswith('.h5'):
            # 直接指定了文件
            check_episode_file(test_path)
        elif Path(test_path).is_dir():
            # 指定了目录
            episode_file = find_latest_episode(test_path)
            if episode_file:
                check_episode_file(episode_file)
        else:
            print(f"错误: 路径不存在或不是episode文件 {test_path}")
    else:
        # 使用默认路径测试
        found = False
        for test_dir in test_dirs:
            episode_file = find_latest_episode(test_dir)
            if episode_file:
                check_episode_file(episode_file)
                found = True
                break

        if not found:
            print("\n❌ 在默认路径中未找到episode文件")
            print("请指定一个episode文件路径:")
            print("  python test_waypoints_feature.py /path/to/episode_0000.h5")
            print("或指定一个包含episode的目录:")
            print("  python test_waypoints_feature.py /path/to/dataset/")


if __name__ == "__main__":
    main()
