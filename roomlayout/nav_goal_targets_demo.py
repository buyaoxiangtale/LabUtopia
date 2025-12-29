"""
Demo: derive navigation targets between experimental areas (BENCH / HOOD).

需求变更：
- BENCH / HOOD 视为两个实验平台，机器人需在两平台（桌面）之间移动。
- 只计算“平台”的 bbox，不再围绕步骤中具体器皿。
- 平台对应 room_assets 中的对象：
    BENCH -> experimental_platform_experimental_platform
    HOOD  -> FumeHood_FumeHood
- 目标点生成：在平台 bbox 内/边缘，沿 -y 方向退 offset_radius（机器人半径）。
- 仅打印规划点，不控制机器人。
"""

import json
import yaml
import os
import argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent

def parse_args():
    parser = argparse.ArgumentParser(description='Navigation goal targets demo')
    parser.add_argument('--protocol', type=str,
                       default=str(ROOT / "roomlayout/protocol_Preparation_of_Sodium_Acetate,_20251111_165059.json"),
                       help='Path to protocol JSON file')
    parser.add_argument('--room-assets', type=str,
                       default=str(ROOT / "roomlayout/Preparation_of_Sodium_Acetate,_Crystallization,_and_Simple_Distillation_of_Residual_Liquor_room_isaacsim.json"),
                       help='Path to room assets JSON file')
    parser.add_argument('--asset-lib', type=str,
                       default=str(ROOT / "roomlayout/Asset.json"),
                       help='Path to asset library JSON file')
    parser.add_argument('--nav-cfg', type=str,
                       default=str(ROOT / "config/navigation/navigation_assets_fbh2.yaml"),
                       help='Path to navigation config YAML file')
    parser.add_argument('--verbose', action='store_true', default=True,
                       help='Enable verbose output')
    return parser.parse_args()

args = parse_args()

PROTOCOL = Path(args.protocol)
ROOM_ASSETS = Path(args.room_assets)
ASSET_LIB = Path(args.asset_lib)
NAV_CFG = Path(args.nav_cfg)
VERBOSE = args.verbose

# 平台映射：步骤 location -> 房间对象 assetId 前缀/完整名
# 从 protocol 中提取唯一 location 类型并映射到平台
def build_location_to_platform(protocol_data):
    locations = set()
    for step in protocol_data.get("procedure", []):
        loc = step.get("location")
        if loc:
            locations.add(loc)

    # 硬编码映射（可根据需求扩展）
    hard_mapping = {
    "BENCH": "experimental_platform_experimental_platform",
    "HOOD": "FumeHood_FumeHood",
        "FUMEHOOD": "FumeHood_FumeHood",
        "EXPERIMENTAL_DESK": "desk_desk",
        "VALIDATION_DESK": "desk_desk",  # 假设验证台也是普通桌子
}

    return {loc: hard_mapping.get(loc, f"{loc.lower()}_{loc.lower()}") for loc in locations}


def load_offset_radius():
    with open(NAV_CFG, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg["assets"][0]["offset_radius"]


def load_protocol():
    with open(PROTOCOL, "r") as f:
        return json.load(f)


LOCATION_TO_PLATFORM = build_location_to_platform(load_protocol())


def load_room_assets():
    with open(ROOM_ASSETS, "r") as f:
        return json.load(f)


def load_asset_lib():
    with open(ASSET_LIB, "r") as f:
        return json.load(f)


def grid_centers(asset_lib):
    centers = {}
    bounds = asset_lib["table"]["grid"]["id_to_bounds_m"]
    for gid, info in bounds.items():
        cx = sum(info["x_range"]) / 2
        cz = sum(info["z_range"]) / 2
        centers[int(gid)] = (cx, cz, info["name"], info["x_range"], info["z_range"])
    return centers


def _norm(s: str) -> str:
    return s.lower().replace(" ", "").replace("-", "").replace("_", "")


def pick_platform(location, room_assets):
    """
    根据 location 选择对应的平台对象：
    BENCH -> experimental_platform_experimental_platform
    HOOD  -> FumeHood_FumeHood
    """
    target_prefix = LOCATION_TO_PLATFORM.get(location.upper())
    if not target_prefix:
        return None
    objs = room_assets.get("objects", [])
    for obj in objs:
        aid = obj.get("assetId", "")
        if _norm(aid).startswith(_norm(target_prefix)):
            return obj
    return None


def build_bbox_lookup(asset_lib):
    """Map normalized asset name -> (half_x, half_z). Use max(short,long)/2 for both axes for safety."""
    lookup = {}
    for a in asset_lib.get("assets", []):
        name = a.get("name", "")
        bbox = a.get("bbox", {})
        half_long = max(bbox.get("short", 0), bbox.get("long", 0)) / 2.0
        half_short = min(bbox.get("short", 0), bbox.get("long", 0)) / 2.0
        # x 方向取短边 /2，z(y) 方向取长边 /2，更符合 “沿 y 轴副半轴” 的靠后布置
        lookup[_norm(name)] = (half_short if half_short else half_long, half_long if half_long else half_short)
    return lookup


def bbox_for_assetid(asset_id: str, asset_lib, bbox_lookup):
    """Given an assetId like Foo_Bar, return (half_x, half_z) if known."""
    if not asset_id:
        return None
    norm_full = _norm(asset_id)
    primary = asset_id.split("_")[0] if "_" in asset_id else asset_id
    # 先尝试完整 assetId，再尝试前缀，再尝试前缀模糊匹配（startswith）
    hit = bbox_lookup.get(norm_full) or bbox_lookup.get(_norm(primary))
    if hit:
        return hit
    for k, v in bbox_lookup.items():
        if k.startswith(norm_full) or norm_full.startswith(k):
            return v
    return None


def platform_target(platform, offset_radius, bbox_lookup):
    """
    仅考虑平台自身 bbox，在其 -y 方向（桌面后沿）退 offset_radius 生成目标。
    平台 bbox 用 Asset.json 的尺寸，center 取 room_assets 中的位置。
    """
    info = asset_bbox_info(platform, bbox_lookup)
    if not info:
        return None
    cx, cy = info["center"]
    hy = info["half_y"]
    # 沿 -y 退 hy+offset
    tx = cx
    tz = cy - (hy + offset_radius)
    return {
        "x": tx,
        "z": tz,
        "note": f"platform {platform.get('assetId')} back -y (hy+offset)",
        "bbox": info,
    }


def asset_bbox_info(asset, bbox_lookup):
    """Return bbox info: center(x,y), half_x, half_y, x_min/max, y_min/max."""
    pos = asset.get("position", {})
    ox, oy = pos.get("x"), pos.get("y")
    half = bbox_for_assetid(asset.get("assetId"), None, bbox_lookup)
    if ox is None or oy is None or not half:
        return None
    hx, hz = half
    return {
        "center": (ox, oy),
        "half_x": hx,
        "half_y": hz,
        "x_min": ox - hx,
        "x_max": ox + hx,
        "y_min": oy - hz,
        "y_max": oy + hz,
    }


def point_inside_any(target, room_assets, bbox_lookup):
    """Check if target (x,z) falls inside any object's footprint (axis-aligned, conservative square)."""
    objs = room_assets.get("objects", [])
    tx, tz = target
    for obj in objs:
        aid = obj.get("assetId", "")
        primary = aid.split("_")[0] if "_" in aid else aid
        half = bbox_lookup.get(_norm(primary))
        if not half:
            continue
        hx, hz = half
        pos = obj.get("position", {})
        ox, oy = pos.get("x"), pos.get("y")  # room layout uses x,y; we treat y as z
        if ox is None or oy is None:
            continue
        if abs(tx - ox) <= hx and abs(tz - oy) <= hz:
            return True
    return False


def find_clear_target(center, offset_radius, room_assets, bbox_lookup):
    """
    Try multiple offsets around grid center to avoid placing target inside an object.
    Offsets tested in order. If all fail, return None.
    """
    x, z, _, xr, zr = center

    def inside_bounds(px, pz):
        return xr[0] <= px <= xr[1] and zr[0] <= pz <= zr[1]

    attempts = []

    candidates = [
        (x, z - offset_radius, "offset -z"),
        (x + offset_radius, z, "offset +x"),
        (x - offset_radius, z, "offset -x"),
        (x, z + offset_radius, "offset +z"),
        (x + offset_radius, z - offset_radius, "offset +x -z"),
        (x - offset_radius, z - offset_radius, "offset -x -z"),
    ]
    # 若基本偏移全部撞物体，进一步在半径环上采样
    extra_radii = [offset_radius * 1.5, offset_radius * 2.0]
    angles = [0, 45, 90, 135, 180, 225, 270, 315]

    def try_pos(px, pz, note):
        if not inside_bounds(px, pz):
            attempts.append((note, "out_of_bounds"))
            return None
        if point_inside_any((px, pz), room_assets, bbox_lookup):
            attempts.append((note, "collision"))
            return None
        attempts.append((note, "ok"))
        return {"x": px, "z": pz, "note": note}

    for cx, cz, note in candidates:
        res = try_pos(cx, cz, note)
        if res:
            return res, attempts

    for r in extra_radii:
        for ang in angles:
            rad = ang * 3.1415926 / 180.0
            cx = x + r * np.cos(rad)
            cz = z + r * np.sin(rad)
            res = try_pos(cx, cz, f"ring r={r:.2f} ang={ang}")
            if res:
                return res, attempts

    return None, attempts


def suggest_target(grid_center, offset_radius):
    """
    Place target near the grid center, offset in -z to leave room in front.
    grid_center can be (x, z, name) or (x, z, name, x_range, z_range).
    """
    x, z = grid_center[0], grid_center[1]
    return {"x": x, "z": z - offset_radius, "note": "offset by safety radius along -z"}


def main():
    offset_radius = load_offset_radius()
    protocol = load_protocol()
    room_assets = load_room_assets()
    asset_lib = load_asset_lib()
    centers = grid_centers(asset_lib)
    bbox_lookup = build_bbox_lookup(asset_lib)

    steps = protocol.get("procedure", [])
    print(f"Loaded {len(steps)} steps; offset_radius = {offset_radius} m\n")

    for step in steps:
        loc = step.get("location", "BENCH").upper()
        platform = pick_platform(loc, room_assets)
        target = None
        debug_attempts = []

        print(f"Step {step.get('step_number')}: {step.get('description')}")
        print(f"  Location: {loc}")

        if platform:
            info = asset_bbox_info(platform, bbox_lookup)
            if info:
                cx, cy = info["center"]
                print(
                    f"  Platform bbox {platform.get('assetId')}: "
                    f"center=({cx:.3f},{cy:.3f}), "
                    f"hx={info['half_x']:.3f}, hy={info['half_y']:.3f}, "
                    f"x[{info['x_min']:.3f},{info['x_max']:.3f}], "
                    f"y[{info['y_min']:.3f},{info['y_max']:.3f}]"
                )
                target = platform_target(platform, offset_radius, bbox_lookup)
                if target:
                    print(f"  Nav target (x,z): ({target['x']:.3f}, {target['z']:.3f})  [{target['note']}]")
                else:
                    print("  Nav target: not assigned (missing bbox info)")
            else:
                print("  Platform bbox not found in asset lib.")
        else:
            print("  Platform not found for this location.")

        print()


if __name__ == "__main__":
    main()


#  python roomlayout/12_17/nav_goal_targets_demo_12_17.py