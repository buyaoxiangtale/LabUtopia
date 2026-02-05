#!/usr/bin/env python3
"""
将“可视化深度图”(uint8 灰度或伪彩色 jet/viridis BGR) 批量转换为“单通道 uint16 深度图”。

目标格式：
- 单通道 PNG（CV_16U / PIL I;16）
- 像素值 = round(depth_m * scale)，默认 scale=10000
  因此可表示 0~6.5535m（65535/10000）

重要说明（务必读）：
1) 如果输入是伪彩色（jet/viridis），这是“近似反解”：
   我们用 LUT（0..255 的 colormap 颜色表）做最近邻反查得到 u8，再按固定范围映射回米制深度。
   由于 colormap 不是严格可逆、且可能有压缩/抖动/色彩空间变化，结果会有少量误差。
2) 如果你能重新采集，强烈建议直接保存原始深度：depth_normalize=false（那就是严格的 uint16 毫米/或自定义比例）。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np
import cv2


def build_colormap_lut(colormap: str) -> np.ndarray:
    """返回 shape=(256,3) 的 BGR LUT（uint8）。"""
    x = np.arange(256, dtype=np.uint8).reshape(256, 1)
    if colormap == "jet":
        lut = cv2.applyColorMap(x, cv2.COLORMAP_JET)
    elif colormap == "viridis":
        lut = cv2.applyColorMap(x, cv2.COLORMAP_VIRIDIS)
    else:
        raise ValueError(f"Unsupported colormap for LUT: {colormap}")
    return lut.reshape(256, 3)  # BGR


def infer_u8_from_bgr(
    bgr: np.ndarray,
    lut_bgr: np.ndarray,
    chunk: int = 20000,
) -> np.ndarray:
    """
    将 BGR 伪彩色图近似反解成 uint8 深度索引（0..255）。
    用 LUT 最近邻匹配，chunked 避免占用过多内存。
    """
    h, w, _ = bgr.shape
    pixels = bgr.reshape(-1, 3).astype(np.int16)  # N,3
    lut = lut_bgr.astype(np.int16)                # 256,3

    out = np.empty((pixels.shape[0],), dtype=np.uint8)
    for s in range(0, pixels.shape[0], chunk):
        p = pixels[s : s + chunk]  # M,3
        # (M,256,3) -> (M,256)
        diff = p[:, None, :] - lut[None, :, :]
        dist2 = np.sum(diff * diff, axis=2)
        out[s : s + chunk] = np.argmin(dist2, axis=1).astype(np.uint8)
    return out.reshape(h, w)


def u8_to_depth_u16(
    u8: np.ndarray,
    vis_min_m: float,
    vis_max_m: float,
    scale: float,
    *,
    max_depth_m: Optional[float] = None,
    drop_far: bool = False,
    invert: bool = False,
    invert_keep_zero: bool = True,
) -> np.ndarray:
    """
    将 0..255 映射回米制深度 depth_m，再乘 scale，输出 uint16。

    - vis_min_m / vis_max_m：可视化映射的物理范围（米）
    - scale：输出单位比例（默认 10000，即 0.1mm）
    - max_depth_m + drop_far：
        若设置 max_depth_m 且 drop_far=True，则把 depth_m > max_depth_m 的像素“去掉”
        （输出置 65535，即白色），而不是饱和到 65535/或置 0。
    """
    u8f = u8.astype(np.float32)
    depth_m = vis_min_m + (u8f / 255.0) * (vis_max_m - vis_min_m)
    if max_depth_m is not None and drop_far:
        keep = depth_m <= float(max_depth_m)
    else:
        keep = None

    depth_scaled = np.clip(depth_m * scale, 0.0, 65535.0)
    depth_u16 = np.rint(depth_scaled).astype(np.uint16)
    if keep is not None:
        depth_u16 = depth_u16.copy()
        # 需求：> max_depth_m 的区域设置为“白色”（uint16 最大值）
        depth_u16[~keep] = np.uint16(65535)

    # 可选：反转显示（近白远黑）
    # 默认保持无效值 0 不变（仍然是黑色 mask）
    if invert:
        if invert_keep_zero:
            mask0 = depth_u16 == 0
            depth_u16 = (65535 - depth_u16).astype(np.uint16)
            depth_u16[mask0] = 0
        else:
            depth_u16 = (65535 - depth_u16).astype(np.uint16)
    return depth_u16


def convert_one_png(
    in_path: Path,
    out_path: Path,
    *,
    mode: str,
    colormap: str,
    vis_min_m: float,
    vis_max_m: float,
    scale: float,
    max_depth_m: Optional[float],
    drop_far: bool,
    invert: bool,
    invert_keep_zero: bool,
    lut_bgr: Optional[np.ndarray],
    chunk: int,
) -> Tuple[bool, str]:
    """
    mode:
      - auto: 根据图像 shape 推断（2D->gray, 3D->pseudo）
      - gray: 输入是灰度（可能是 1ch 或 3ch 且三通道相等）
      - pseudo: 输入是伪彩色（BGR）
    """
    img = cv2.imread(str(in_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        return False, "cv2.imread failed"

    # 已经是 uint16 单通道：直接复制/跳过
    if img.dtype == np.uint16 and img.ndim == 2:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if out_path != in_path:
            cv2.imwrite(str(out_path), img)
        return True, "already_u16"

    # 解析输入为 u8 index
    if mode == "auto":
        # auto 策略：
        # - 2D: 灰度
        # - 3D 且三通道完全一致：灰度（很多保存流程会把灰度存成 3 通道）
        # - 其他 3D: 伪彩色
        if img.ndim == 2:
            mode_eff = "gray"
        elif img.ndim == 3 and img.shape[2] == 3 and np.array_equal(img[..., 0], img[..., 1]) and np.array_equal(img[..., 1], img[..., 2]):
            mode_eff = "gray"
        else:
            mode_eff = "pseudo"
    else:
        mode_eff = mode

    if mode_eff == "gray":
        if img.ndim == 3:
            # 若三通道一致，取任意通道
            if np.array_equal(img[..., 0], img[..., 1]) and np.array_equal(img[..., 1], img[..., 2]):
                img = img[..., 0]
            else:
                # 不是纯灰度，仍当作伪彩色处理
                mode_eff = "pseudo"
        if mode_eff == "gray":
            if img.dtype != np.uint8:
                # 非 uint8 灰度：先压到 0..255（保守）
                img_f = img.astype(np.float32)
                img_f = np.clip(img_f, 0.0, 255.0)
                u8 = img_f.astype(np.uint8)
            else:
                u8 = img
            depth_u16 = u8_to_depth_u16(
                u8,
                vis_min_m,
                vis_max_m,
                scale,
                max_depth_m=max_depth_m,
                drop_far=drop_far,
                invert=invert,
                invert_keep_zero=invert_keep_zero,
            )
            out_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_path), depth_u16)
            return True, "gray_u8_to_u16"

    # pseudo
    if img.ndim != 3 or img.shape[2] != 3:
        return False, f"expected BGR 3ch for pseudo, got shape={img.shape}, dtype={img.dtype}"
    if lut_bgr is None:
        lut_bgr = build_colormap_lut(colormap)
    u8 = infer_u8_from_bgr(img, lut_bgr, chunk=chunk)
    depth_u16 = u8_to_depth_u16(
        u8,
        vis_min_m,
        vis_max_m,
        scale,
        max_depth_m=max_depth_m,
        drop_far=drop_far,
        invert=invert,
        invert_keep_zero=invert_keep_zero,
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), depth_u16)
    return True, f"pseudo_{colormap}_to_u16"


def iter_depth_dirs(root: Path) -> Iterable[Path]:
    """递归查找所有 observation.images.depth 目录。"""
    yield from root.rglob("observation.images.depth")


def main():
    ap = argparse.ArgumentParser(
        description="批量将 observation.images.depth 下的可视化深度图转换为单通道 uint16 深度图（depth_m*10000）。"
    )
    ap.add_argument(
        "root",
        type=str,
        help="根目录（会递归查找 observation.images.depth）或直接给 observation.images.depth 目录",
    )
    ap.add_argument(
        "--mode",
        choices=["auto", "gray", "pseudo"],
        default="auto",
        help="输入深度图模式：auto(默认)/gray(灰度)/pseudo(伪彩色)",
    )
    ap.add_argument(
        "--colormap",
        choices=["jet", "viridis"],
        default="jet",
        help="当输入是伪彩色时使用的 colormap（默认 jet）",
    )
    ap.add_argument("--vis-min-m", type=float, default=0.1, help="可视化映射的最小深度（米）")
    ap.add_argument("--vis-max-m", type=float, default=10.0, help="可视化映射的最大深度（米）")
    ap.add_argument("--scale", type=float, default=10000.0, help="米->整数的比例（默认 10000）")
    ap.add_argument(
        "--max-depth-m",
        type=float,
        default=None,
        help="最大保留深度（米）。与 --drop-far 配合使用：超过该深度的像素置 0（去掉）。",
    )
    ap.add_argument(
        "--drop-far",
        action="store_true",
        help="超过 --max-depth-m 的深度像素置 65535（白色），用于把超远区域统一标成 far。",
    )
    ap.add_argument(
        "--invert",
        action="store_true",
        help="反转输出的 uint16 深度值：depth_u16 = 65535 - depth_u16（用于近白远黑显示）。",
    )
    ap.add_argument(
        "--invert-keep-zero",
        action="store_true",
        default=True,
        help="与 --invert 一起使用：保持原本的 0（无效深度）仍为 0，不参与反转（默认开启）。",
    )
    ap.add_argument(
        "--out-suffix",
        type=str,
        default="_u16",
        help="输出目录后缀：observation.images.depth{suffix}（默认 _u16）",
    )
    ap.add_argument(
        "--inplace",
        action="store_true",
        help="原地覆盖写回（危险，不推荐）。默认写到同级新目录 observation.images.depth_u16/。",
    )
    ap.add_argument("--chunk", type=int, default=20000, help="伪彩色反解时的分块大小（内存/速度折中）")
    ap.add_argument("--limit", type=int, default=0, help="最多处理多少张图片（0=不限制）")
    args = ap.parse_args()

    root = Path(args.root)
    if not root.exists():
        raise SystemExit(f"root not found: {root}")

    # 如果用户直接传了 observation.images.depth 目录，就只处理它；否则递归找所有
    depth_dirs = [root] if root.name == "observation.images.depth" else list(iter_depth_dirs(root))
    if not depth_dirs:
        raise SystemExit("no observation.images.depth directories found")

    lut_bgr = build_colormap_lut(args.colormap) if args.mode in ("pseudo",) else None

    total = 0
    ok = 0
    for d in sorted(depth_dirs):
        out_dir = d if args.inplace else d.parent / f"{d.name}{args.out_suffix}"
        pngs = sorted(d.glob("*.png"))
        if not pngs:
            continue
        for p in pngs:
            if args.limit and total >= args.limit:
                break
            total += 1
            if total % 50 == 0:
                print(f"[PROGRESS] processed {total} images...")
            out_path = p if args.inplace else (out_dir / p.name)
            success, tag = convert_one_png(
                p,
                out_path,
                mode=args.mode,
                colormap=args.colormap,
                vis_min_m=args.vis_min_m,
                vis_max_m=args.vis_max_m,
                scale=args.scale,
                max_depth_m=args.max_depth_m,
                drop_far=args.drop_far,
                invert=args.invert,
                invert_keep_zero=args.invert_keep_zero,
                lut_bgr=lut_bgr,
                chunk=args.chunk,
            )
            if success:
                ok += 1
            else:
                print(f"[FAIL] {p}: {tag}")

        print(f"[DIR] {d} -> {out_dir} (processed {len(pngs)} png)")

    print(f"DONE: ok={ok} total={total}")


if __name__ == "__main__":
    main()


