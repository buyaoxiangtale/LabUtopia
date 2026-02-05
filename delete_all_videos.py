#!/usr/bin/env python3
"""
删除所有视频文件脚本
删除 observation.video.depth 和 observation.video.rgb 目录中的视频
"""

import os
import sys
import shutil
from pathlib import Path


def delete_video_files(base_dir, dry_run=False):
    """
    删除所有视频文件（RGB 和 Depth）

    Args:
        base_dir: 基础目录路径
        dry_run: 如果为 True，只显示会做什么，不实际删除
    """
    base_path = Path(base_dir)

    if not base_path.exists():
        print(f"❌ 错误：目录不存在: {base_dir}")
        return

    # 查找所有视频目录
    depth_dirs = list(base_path.rglob("observation.video.depth"))
    rgb_dirs = list(base_path.rglob("observation.video.rgb"))

    total_dirs = len(depth_dirs) + len(rgb_dirs)
    total_files = 0
    deleted_files = 0
    failed_deletes = 0
    empty_dirs = 0

    print("🗑️  开始删除视频文件...")
    print(f"📁 目标目录: {base_dir}")
    print("🎯 删除目标:")
    print("   • observation.video.depth/*.mp4 (深度视频)")
    print("   • observation.video.rgb/*.mp4 (RGB视频)")
    if dry_run:
        print("⚠️  DRY RUN 模式：不会实际删除文件")
    print("================================================================")
    print("")

    # 处理深度视频目录
    print("🔵 处理深度视频目录 (observation.video.depth)")
    print("")

    for idx, depth_dir in enumerate(depth_dirs, 1):
        print(f"📂 [{idx}/{len(depth_dirs)}] 处理深度视频目录:")
        print(f"    路径: {depth_dir}")

        # 查找视频文件
        video_files = list(depth_dir.glob("*.mp4"))

        if not video_files:
            print(f"    ℹ️  目录为空，无文件需要删除")
            empty_dirs += 1
            print("")
            continue

        for video_file in video_files:
            total_files += 1
            print(f"    📹 删除文件: {video_file.name}")

            if not dry_run:
                try:
                    video_file.unlink()
                    print(f"      ✅ 删除成功")
                    deleted_files += 1
                except Exception as e:
                    print(f"      ❌ 删除失败: {e}")
                    failed_deletes += 1
            else:
                print(f"      🔍 [DRY RUN] 将会删除")
                deleted_files += 1

        print("")

    # 处理RGB视频目录
    print("🟢 处理RGB视频目录 (observation.video.rgb)")
    print("")

    for idx, rgb_dir in enumerate(rgb_dirs, 1):
        print(f"📂 [{idx}/{len(rgb_dirs)}] 处理RGB视频目录:")
        print(f"    路径: {rgb_dir}")

        # 查找视频文件
        video_files = list(rgb_dir.glob("*.mp4"))

        if not video_files:
            print(f"    ℹ️  目录为空，无文件需要删除")
            empty_dirs += 1
            print("")
            continue

        for video_file in video_files:
            total_files += 1
            print(f"    📹 删除文件: {video_file.name}")

            if not dry_run:
                try:
                    video_file.unlink()
                    print(f"      ✅ 删除成功")
                    deleted_files += 1
                except Exception as e:
                    print(f"      ❌ 删除失败: {e}")
                    failed_deletes += 1
            else:
                print(f"      🔍 [DRY RUN] 将会删除")
                deleted_files += 1

        print("")

    print("================================================================")
    print("📊 删除完成！统计信息：")
    print(f"  • 处理目录数: {total_dirs}")
    print(f"  • 空目录数: {empty_dirs}")
    print(f"  • 总共找到: {total_files} 个视频文件")
    print(f"  • 成功删除: {deleted_files} 个文件")
    if dry_run:
        print(f"  • (DRY RUN 模式，未实际删除)")
    print(f"  • 删除失败: {failed_deletes} 个文件")
    print("================================================================")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='删除 observation.video.depth 和 observation.video.rgb 中的所有视频文件'
    )
    parser.add_argument(
        'directory',
        nargs='?',
        default='/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.20/18.32.23_level5_Navigation_Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop',
        help='包含视频文件的基础目录'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='模拟运行，不实际删除文件'
    )

    args = parser.parse_args()

    delete_video_files(args.directory, dry_run=args.dry_run)


if __name__ == '__main__':
    main()
