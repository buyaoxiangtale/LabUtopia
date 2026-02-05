#!/usr/bin/env python3
"""
批量修改 Parquet 里“固定相机外参 camera_extrinsic”的平移向量。

目标：
- 在指定 run_dir（例如 outputs/collect/2026.01.21/22.14.56_xxx）下递归查找
  trajectory_*/data/chunk-000/*.parquet
- 找到所有列名以 'camera_extrinsic' 结尾的列（例如 'observation.camera_extrinsic'，
  或 'observation.<cam>.camera_extrinsic'），把其第一行(通常只有第一行有值)的 4x4 矩阵平移项
  改成 (tx, ty, tz)，保留原有旋转部分。

注意：
- 该脚本需要 pandas + (pyarrow 或 fastparquet) 才能读写 parquet。
- 如果你的系统 python 没有这些依赖，请在 IsaacSim 自带 python 环境里运行，
  或在 venv 里安装：pip install -U pandas pyarrow
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, List, Tuple


def _resolve_input_dirs(inputs: Iterable[str], search_root: str) -> List[Path]:
    """
    将输入解析为实际存在的目录列表。

    支持：
    - 直接给完整路径（存在且为目录）
    - 只给目录名（例如 22.14.56_xxx），会在 search_root 下递归查找同名目录
    - 允许 '@' 前缀（会自动去掉）
    """
    resolved: List[Path] = []
    root = Path(search_root)

    for raw in inputs:
        s = str(raw).strip()
        if not s:
            continue
        if s.startswith("@"):
            s = s[1:]

        p = Path(s)
        if p.exists() and p.is_dir():
            resolved.append(p)
            continue

        if not root.exists():
            raise FileNotFoundError(f"search_root 不存在: {search_root}")

        matches = [m for m in root.rglob(s) if m.is_dir() and m.name == s]
        if len(matches) == 1:
            resolved.append(matches[0])
        elif len(matches) == 0:
            raise FileNotFoundError(f"未找到目录: '{s}'（在 {search_root} 下也未找到同名目录）")
        else:
            cand = "\n".join([f"  - {str(m)}" for m in sorted(matches)])
            raise RuntimeError(
                f"目录名 '{s}' 在 {search_root} 下匹配到多个目录，为避免误操作请改用完整路径：\n{cand}"
            )

    return resolved


def _find_parquet_files(run_dir: Path) -> List[Path]:
    # 兼容你的目录结构：trajectory_XXXXXX/data/chunk-000/episode_XXXXXX.parquet
    return sorted(run_dir.glob("trajectory_*/data/chunk-000/*.parquet"))


def _as_4x4_list(x) -> Tuple[List[List[float]], bool]:
    """
    将 object 转为 4x4 list[list[float]]。
    返回 (matrix, ok)
    """
    if x is None:
        return [], False

    # 有些引擎会把 list 存成字符串（例如 "[[...],[...]]"）
    if isinstance(x, str):
        xs = x.strip()
        if xs.startswith("[") and xs.endswith("]"):
            try:
                parsed = json.loads(xs)
                return _as_4x4_list(parsed)
            except Exception:
                return [], False

    # 最常见：python list
    if isinstance(x, list):
        try:
            # 允许行是 list/tuple/np.ndarray（pandas 读取后经常会出现）
            if len(x) == 4 and all(hasattr(r, "__len__") and len(r) == 4 for r in x):
                out = []
                for r in x:
                    out.append([float(v) for v in list(r)])
                return out, True
        except Exception:
            return [], False
        return [], False

    # numpy array（如果 pandas 读出来是 ndarray）
    try:
        import numpy as np  # noqa

        if isinstance(x, np.ndarray):
            # 常见：二维数组 (4,4)
            if x.shape == (4, 4):
                return x.astype(float).tolist(), True

            # 你这批数据的形态：object 数组 shape=(4,)，每个元素是一行长度为4的 array
            # repr: array([array([...]), array([...]), array([...]), array([...])], dtype=object)
            if x.shape == (4,) and x.dtype == object:
                try:
                    rows = []
                    for r in x.tolist():
                        rr = list(r)
                        if len(rr) != 4:
                            return [], False
                        rows.append([float(v) for v in rr])
                    if len(rows) == 4:
                        return rows, True
                except Exception:
                    return [], False
    except Exception:
        pass

    return [], False


def _update_translation(mat4: List[List[float]], tx: float, ty: float, tz: float) -> List[List[float]]:
    mat4[0][3] = float(tx)
    mat4[1][3] = float(ty)
    mat4[2][3] = float(tz)
    return mat4


def process_parquet_file(
    parquet_path: Path,
    tx: float,
    ty: float,
    tz: float,
    dry_run: bool,
    debug: bool = False,
    debug_max: int = 1,
) -> Tuple[int, int]:
    """
    返回 (updated_cols, skipped_cols)
    """
    try:
        import pandas as pd  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "缺少依赖 pandas。请在包含 pandas+pyarrow 的环境运行（例如 IsaacSim python），"
            "或安装：pip install -U pandas pyarrow"
        ) from e

    df = pd.read_parquet(parquet_path)

    extrinsic_cols = [c for c in df.columns if str(c).endswith("camera_extrinsic")]
    if not extrinsic_cols:
        return 0, 0

    updated = 0
    skipped = 0
    debug_left = debug_max

    for col in extrinsic_cols:
        v0 = df[col].iloc[0] if len(df) > 0 else None
        mat, ok = _as_4x4_list(v0)
        if not ok:
            skipped += 1
            if debug and debug_left > 0:
                print(f"    [DEBUG] {parquet_path.name} col='{col}': type={type(v0)} repr={repr(v0)[:400]}")
                debug_left -= 1
            continue

        new_mat = _update_translation(mat, tx=tx, ty=ty, tz=tz)
        # 只改第一行（其他行通常为 None）
        df.at[df.index[0], col] = new_mat
        updated += 1

    if updated > 0 and not dry_run:
        # 可选：写回同名文件（覆盖）
        df.to_parquet(parquet_path, index=False)

    return updated, skipped


def main() -> int:
    parser = argparse.ArgumentParser(
        description="批量修改 run_dir 下所有 parquet 的 camera_extrinsic 平移向量为 (tx,ty,tz)。"
    )
    parser.add_argument(
        "directories",
        nargs="*",
        help="一个或多个 run_dir 路径/目录名（可带 @ 前缀）。目录名会在 --search-root 下查找。",
    )
    parser.add_argument(
        "--paths-file",
        default=None,
        help="一个 txt 文件，里面每行一个 run_dir（可写绝对路径/目录名；允许 @ 前缀；空行会忽略）。",
    )
    parser.add_argument(
        "--search-root",
        default="/home/pjlab/fbh/LabUtopia/outputs/collect",
        help="当 directories 里提供的是目录名时，在该根目录下递归查找匹配目录。",
    )
    parser.add_argument("--tx", type=float, default=0.56)
    parser.add_argument("--ty", type=float, default=0.0)
    parser.add_argument("--tz", type=float, default=0.1)
    parser.add_argument("--dry-run", action="store_true", help="只打印将要修改的文件/列，不写回。")
    parser.add_argument("--debug", action="store_true", help="打印解析失败的样例（每个 parquet 最多打印 1 条）。")
    parser.add_argument("--debug-max", type=int, default=1, help="每个 parquet 最多打印多少条 debug 样例。")
    args = parser.parse_args()

    # 合并来自命令行和 paths-file 的目录输入
    dir_inputs: List[str] = []
    if args.paths_file:
        pf = Path(args.paths_file)
        if not pf.exists():
            print(f"❌ paths-file 不存在: {pf}")
            return 2
        lines = pf.read_text(encoding="utf-8", errors="ignore").splitlines()
        for ln in lines:
            s = ln.strip()
            if not s:
                continue
            if s.startswith("#"):
                continue
            dir_inputs.append(s)
    if args.directories:
        dir_inputs.extend(list(args.directories))

    if not dir_inputs:
        print("❌ 你没有提供任何目录。你可以：")
        print("  - 直接在命令行传入：python3 update_parquet_camera_extrinsic_translation.py /abs/run1 /abs/run2 ...")
        print("  - 或用 --paths-file：python3 update_parquet_camera_extrinsic_translation.py --paths-file runs.txt")
        return 2

    try:
        run_dirs = _resolve_input_dirs(dir_inputs, search_root=args.search_root)
    except Exception as e:
        print(f"❌ 参数解析失败: {e}")
        return 2

    total_files = 0
    total_updated_cols = 0
    total_skipped_cols = 0

    print(f"✅ 将把 camera_extrinsic 平移改为 (tx,ty,tz)=({args.tx},{args.ty},{args.tz})")
    if args.dry_run:
        print("⚠️  DRY RUN：不会写回 parquet")
    print("")

    for run_dir in run_dirs:
        parquet_files = _find_parquet_files(run_dir)
        print(f"📁 Run dir: {run_dir}")
        print(f"  - parquet files: {len(parquet_files)}")
        if not parquet_files:
            print("")
            continue

        for p in parquet_files:
            total_files += 1
            try:
                updated, skipped = process_parquet_file(
                    p,
                    tx=args.tx,
                    ty=args.ty,
                    tz=args.tz,
                    dry_run=args.dry_run,
                    debug=args.debug,
                    debug_max=args.debug_max,
                )
                if updated > 0 or skipped > 0:
                    print(f"  ✓ {p.relative_to(run_dir)}  (updated_cols={updated}, skipped_cols={skipped})")
                total_updated_cols += updated
                total_skipped_cols += skipped
            except Exception as e:
                print(f"  ❌ {p}: {e}")
        print("")

    print("================================================================")
    print("📊 完成统计：")
    print(f"  • 处理 parquet 文件数: {total_files}")
    print(f"  • 成功更新的 camera_extrinsic 列数: {total_updated_cols}")
    print(f"  • 跳过（非4x4/为空）的 camera_extrinsic 列数: {total_skipped_cols}")
    print("================================================================")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


