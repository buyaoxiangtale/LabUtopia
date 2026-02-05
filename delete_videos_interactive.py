#!/usr/bin/env python3
"""
删除视频文件脚本（支持选择视频类型）
可删除 observation.video.depth、observation.video.trajectory 等目录中的视频
"""

import os
import sys
import argparse
from pathlib import Path


def delete_videos_by_type(base_dir, video_types=None, dry_run=False):
    """
    按类型删除视频文件

    Args:
        base_dir: 基础目录路径
        video_types: 要删除的视频类型列表，如 ['depth', 'trajectory', 'rgb']
                    如果为 None，则删除所有视频
        dry_run: 如果为 True，只显示会做什么，不实际删除
    """
    base_path = Path(base_dir)

    if not base_path.exists():
        print(f"❌ 错误：目录不存在: {base_dir}")
        return

    # 查找所有视频目录
    if video_types is None:
        # 删除所有视频
        video_pattern = "observation.video.*"
        video_dirs = list(base_path.rglob("observation.video.*"))
    else:
        # 只删除指定类型的视频
        video_dirs = []
        for vtype in video_types:
            pattern = f"observation.video.{vtype}"
            video_dirs.extend(base_path.rglob(pattern))

    if not video_dirs:
        print("⚠️  未找到匹配的视频目录")
        return

    total_files = 0
    deleted_files = 0
    failed_deletes = 0
    empty_dirs = 0

    print("🗑️  开始删除视频文件...")
    print(f"📁 目标目录: {base_dir}")
    print("🎯 删除目标:")
    if video_types is None:
        print("   • 所有 observation.video.* 目录")
    else:
        for vtype in video_types:
            print(f"   • observation.video.{vtype}")
    if dry_run:
        print("⚠️  DRY RUN 模式：不会实际删除文件")
    print("================================================================")
    print("")

    # 处理每个视频目录
    for idx, video_dir in enumerate(video_dirs, 1):
        dir_name = video_dir.name
        print(f"📂 [{idx}/{len(video_dirs)}] 处理目录: {dir_name}")
        print(f"    路径: {video_dir}")

        # 查找视频文件
        video_files = list(video_dir.glob("*.mp4"))

        if not video_files:
            print(f"    ℹ️  目录为空，无文件需要删除")
            empty_dirs += 1
            print("")
            continue

        for video_file in video_files:
            total_files += 1
            file_size = video_file.stat().st_size / (1024 * 1024)  # MB
            print(f"    📹 [{total_files}] {video_file.name} ({file_size:.1f} MB)")

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
    print(f"  • 处理目录数: {len(video_dirs)}")
    print(f"  • 空目录数: {empty_dirs}")
    print(f"  • 总共找到: {total_files} 个视频文件")
    print(f"  • 成功删除: {deleted_files} 个文件")
    if dry_run:
        print(f"  • (DRY RUN 模式，未实际删除)")
    print(f"  • 删除失败: {failed_deletes} 个文件")
    print("================================================================")


def main():
    parser = argparse.ArgumentParser(
        description='删除指定类型的视频文件（depth/trajectory/rgb）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例用法:
  # 只删除深度视频
  python3 delete_videos_interactive.py --types depth

  # 删除深度视频和轨迹视频
  python3 delete_videos_interactive.py --types depth trajectory

  # 删除所有视频
  python3 delete_videos_interactive.py --all

  # 预览会删除什么（不实际删除）
  python3 delete_videos_interactive.py --types depth --dry-run
        '''
    )

    parser.add_argument(
        'directory',
        nargs='?',
        default='/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.20/18.32.23_level5_Navigation_Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop',
        help='包含视频文件的基础目录'
    )

    parser.add_argument(
        '--types',
        nargs='+',
        choices=['depth', 'trajectory', 'rgb'],
        help='要删除的视频类型（可选择多个）'
    )

    parser.add_argument(
        '--all',
        action='store_true',
        help='删除所有类型的视频'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='模拟运行，不实际删除文件'
    )

    args = parser.parse_args()

    # 确定要删除的视频类型
    if args.all:
        video_types = None
    elif args.types:
        video_types = args.types
    else:
        # 默认只删除深度视频
        video_types = ['depth']

    delete_videos_by_type(args.directory, video_types=video_types, dry_run=args.dry_run)


if __name__ == '__main__':
    main()
