#!/usr/bin/env python3
"""
批量重命名点云文件脚本
将所有 episode_*_with_trajectory.ply 重命名为 path.ply
"""

import os
import sys
from pathlib import Path


def _resolve_input_dirs(inputs, search_root: str) -> list[Path]:
    """
    将用户输入的目录参数解析为实际存在的目录列表。

    支持两种输入：
    1) 绝对/相对路径：如果路径存在且是目录，直接使用
    2) 目录名（例如 21.20.55_xxx）：会在 search_root 下递归查找同名目录
       - 若找到 1 个匹配：使用该目录
       - 若找到 0 个匹配：报错
       - 若找到多个匹配：报错并提示候选项，避免误操作
    """
    resolved: list[Path] = []
    root = Path(search_root)

    for raw in inputs:
        s = str(raw).strip()
        if not s:
            continue

        # 兼容用户从文档里拷贝过来的 @前缀
        if s.startswith("@"):
            s = s[1:]

        p = Path(s)
        if p.exists() and p.is_dir():
            resolved.append(p)
            continue

        # 当作“目录名”在 root 下查找
        if not root.exists():
            raise FileNotFoundError(f"search_root 不存在: {search_root}")
        matches = [m for m in root.rglob(s) if m.is_dir() and m.name == s]
        if len(matches) == 1:
            resolved.append(matches[0])
        elif len(matches) == 0:
            raise FileNotFoundError(f"未找到目录: '{s}'（在 {search_root} 下也未找到同名目录）")
        else:
            cand = "\n".join([f"  - {str(m)}" for m in sorted(matches)])
            raise RuntimeError(f"目录名 '{s}' 在 {search_root} 下匹配到多个目录，"
                               f"为避免误操作请改用完整路径：\n{cand}")

    return resolved


def rename_ply_files(base_dir: str | Path, dry_run: bool = False):
    """
    批量重命名点云文件

    Args:
        base_dir: 基础目录路径
        dry_run: 如果为 True，只显示会做什么，不实际执行重命名
    """
    base_path = Path(base_dir)

    if not base_path.exists():
        print(f"❌ 错误：目录不存在: {base_dir}")
        return
    if not base_path.is_dir():
        print(f"❌ 错误：不是目录: {base_dir}")
        return

    # 查找所有 .ply 文件
    ply_files = list(base_path.rglob("*_with_trajectory.ply"))

    if not ply_files:
        print(f"⚠️  未找到匹配的点云文件")
        return

    total = len(ply_files)
    renamed = 0
    skipped = 0
    failed = 0

    print("🚀 开始批量重命名点云文件...")
    print(f"📁 目标目录: {base_dir}")
    if dry_run:
        print("⚠️  DRY RUN 模式：不会实际修改文件")
    print("================================================================")
    print("")

    for idx, ply_file in enumerate(ply_files, 1):
        file_dir = ply_file.parent
        target_file = file_dir / "path.ply"

        print(f"📋 [{idx:03d}/{total}] 处理文件...")
        print(f"    源文件: {ply_file.name}")
        print(f"    目标: path.ply")

        # 检查目标文件是否已存在
        if target_file.exists():
            print(f"    ⚠️  跳过: 目标文件已存在")
            skipped += 1
            print("")
            continue

        # 重命名文件
        if not dry_run:
            try:
                ply_file.rename(target_file)
                print(f"    ✅ 重命名成功")
                renamed += 1
            except Exception as e:
                print(f"    ❌ 重命名失败: {e}")
                failed += 1
        else:
            print(f"    🔍 [DRY RUN] 将会重命名")
            renamed += 1

        print("")

    print("================================================================")
    print("📊 操作完成！统计信息：")
    print(f"  • 总共找到: {total} 个 .ply 文件")
    print(f"  • 成功重命名: {renamed} 个文件")
    print(f"  • 跳过: {skipped} 个文件（目标已存在）")
    print(f"  • 失败: {failed} 个文件")
    print("================================================================")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='批量重命名点云文件：episode_*_with_trajectory.ply -> path.ply'
    )
    parser.add_argument(
        'directories',
        nargs='*',
        help='一个或多个目录（可给完整路径；也可只给目录名，将在 --search-root 下自动查找）'
    )
    parser.add_argument(
        '--search-root',
        default='/home/pjlab/fbh/LabUtopia/outputs/collect',
        help='当 directories 里提供的是“目录名”时，在该根目录下递归查找匹配目录'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='模拟运行，不实际修改文件'
    )

    args = parser.parse_args()

    # 兼容旧用法：不传参数时给一个示例路径（只提示，不自动执行）
    if not args.directories:
        print("❌ 你没有传入任何目录。示例：")
        print("   python3 rename_ply_files.py /path/to/run_dir_1 /path/to/run_dir_2")
        print("   或：python3 rename_ply_files.py 21.20.55_xxx 22.14.56_xxx --search-root /home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.21")
        sys.exit(2)

    try:
        dirs = _resolve_input_dirs(args.directories, search_root=args.search_root)
    except Exception as e:
        print(f"❌ 参数解析失败: {e}")
        sys.exit(2)

    # 多目录批量执行
    total_dirs = len(dirs)
    print(f"✅ 共解析到 {total_dirs} 个目录，将逐个处理：")
    for d in dirs:
        print(f"  - {d}")
    print("")

    for idx, d in enumerate(dirs, 1):
        print(f"\n===== [{idx}/{total_dirs}] 处理目录: {d} =====")
        rename_ply_files(d, dry_run=args.dry_run)


if __name__ == '__main__':
    main()
