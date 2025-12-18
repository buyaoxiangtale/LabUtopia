"""
Demo: derive navigation targets between experimental areas.

适配 12_17 场景：
- 通过规范化匹配自动建立 protocol location -> room_assets 对象的映射
- 支持新格式 assets_annotated.json（geometry.bbox）
- 目标点生成：在平台 bbox 边缘，沿 -y 方向退 offset_radius（机器人半径）
- 仅打印规划点，不控制机器人
"""

import json
import yaml
import os
import argparse
from pathlib import Path
import numpy as np

# 12_17 文件夹在 roomlayout 下，需要往上 3 级到达项目根目录
ROOT = Path(__file__).resolve().parent.parent.parent

def parse_args():
    parser = argparse.ArgumentParser(description='Navigation goal targets demo')
    parser.add_argument('--protocol', type=str,
                       default=str(ROOT / "roomlayout/protocol_Preparation_of_Sodium_Acetate,_20251111_165059.json"),
                       help='Path to protocol JSON file')
    parser.add_argument('--room-assets', type=str,
                       default=str(ROOT / "roomlayout/Preparation_of_Sodium_Acetate,_Crystallization,_and_Simple_Distillation_of_Residual_Liquor_room_isaacsim.json"),
                       help='Path to room assets JSON file')
    parser.add_argument('--asset-lib', type=str,
                       default=str(ROOT / "roomlayout/12_17/assets_annotated.json"),
                       help='Path to asset library JSON file (supports new assets_annotated.json format)')
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


def _norm(s: str) -> str:
    """规范化字符串：去除下划线/空格/横线，转小写"""
    return s.lower().replace(" ", "").replace("-", "").replace("_", "")


# 特殊映射：某些 location 是设备名称，需要映射到其所在的平台
# 注意：LabBench 在 assets_annotated.json 中对应 ExperimentalPlatform
SPECIAL_MAPPING = {
    "rotaryevaporator": "RotaryEvaporator",  # 旋转蒸发仪在实验台上
    "labbench": "ExperimentalPlatform",  # LabBench 映射到 ExperimentalPlatform
    "bench": "ExperimentalPlatform",  # BENCH 的别名
    "hood": "FumeHood",  # HOOD 的别名
}


def build_location_to_platform(protocol_data, room_assets):
    """
    从 protocol 中提取 location，通过规范化匹配自动映射到 room_assets 中的平台对象。
    只有特殊情况（如设备名映射到所在平台）才使用硬编码。
    """
    # 收集 protocol 中所有的 location
    locations = set()
    for step in protocol_data.get("procedure", []):
        loc = step.get("location")
        if loc:
            locations.add(loc)
    
    # 获取 room_assets 中所有对象的 id
    platform_ids = [obj.get("id", "") for obj in room_assets.get("objects", [])]
    
    result = {}
    for loc in locations:
        norm_loc = _norm(loc)
        
        # 1. 先检查特殊映射
        if norm_loc in SPECIAL_MAPPING:
            result[loc] = SPECIAL_MAPPING[norm_loc]
            continue
        
        # 2. 规范化匹配：在 room_assets 中查找匹配的对象 id
        matched = None
        for pid in platform_ids:
            if _norm(pid) == norm_loc:
                matched = pid
                break
        
        # 3. 如果精确匹配失败，尝试模糊匹配（startswith）
        if not matched:
            for pid in platform_ids:
                if _norm(pid).startswith(norm_loc) or norm_loc.startswith(_norm(pid)):
                    matched = pid
                    break
        
        result[loc] = matched if matched else loc
    
    return result


def load_offset_radius():
    with open(NAV_CFG, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg["assets"][0]["offset_radius"]


def load_protocol():
    with open(PROTOCOL, "r") as f:
        return json.load(f)


def load_room_assets():
    with open(ROOM_ASSETS, "r") as f:
        return json.load(f)


def load_asset_lib():
    with open(ASSET_LIB, "r") as f:
        return json.load(f)


def grid_centers(asset_lib):
    """从 asset_lib 中提取网格中心（如果存在 table.grid 定义）"""
    centers = {}
    table = asset_lib.get("table", {})
    grid = table.get("grid", {})
    bounds = grid.get("id_to_bounds_m", {})
    for gid, info in bounds.items():
        cx = sum(info["x_range"]) / 2
        cz = sum(info["z_range"]) / 2
        centers[int(gid)] = (cx, cz, info["name"], info["x_range"], info["z_range"])
    return centers


def pick_platform(location, room_assets, location_mapping):
    """
    根据 location 选择对应的平台对象。
    使用 location_mapping 进行映射，通过规范化匹配在 room_assets 中查找。
    """
    target_id = location_mapping.get(location)
    if not target_id:
        # 尝试直接规范化匹配
        target_id = location
    
    objs = room_assets.get("objects", [])
    norm_target = _norm(target_id)
    
    for obj in objs:
        obj_id = obj.get("id", "")
        if _norm(obj_id) == norm_target:
            return obj
    
    # 模糊匹配
    for obj in objs:
        obj_id = obj.get("id", "")
        if _norm(obj_id).startswith(norm_target) or norm_target.startswith(_norm(obj_id)):
            return obj
    
    return None


def build_bbox_lookup(asset_lib):
    """
    Map normalized asset name/id -> (half_x, half_z).
    支持两种格式：
    - 旧格式: asset.bbox.{short, long}
    - 新格式 (assets_annotated.json): asset.geometry.bbox.{short, long}
    """
    lookup = {}
    for a in asset_lib.get("assets", []):
        # 获取 name 和 id 作为查找键
        name = a.get("name", "")
        asset_id = a.get("id", "")
        
        # 兼容两种 bbox 格式
        bbox = a.get("bbox", {})
        if not bbox:
            # 新格式：geometry.bbox
            geometry = a.get("geometry", {})
            bbox = geometry.get("bbox", {})
        
        if not bbox:
            continue
            
        half_long = max(bbox.get("short", 0), bbox.get("long", 0)) / 2.0
        half_short = min(bbox.get("short", 0), bbox.get("long", 0)) / 2.0
        
        # x 方向取短边 /2，z(y) 方向取长边 /2
        half_tuple = (half_short if half_short else half_long, half_long if half_long else half_short)
        
        # 同时用 name 和 id 作为键
        if name:
            lookup[_norm(name)] = half_tuple
        if asset_id:
            lookup[_norm(asset_id)] = half_tuple
    
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
        "note": f"platform {platform.get('id')} back -y (hy+offset)",
        "bbox": info,
    }


def asset_bbox_info(asset, bbox_lookup):
    """Return bbox info: center(x,y), half_x, half_y, x_min/max, y_min/max."""
    pos = asset.get("position", {})
    ox, oy = pos.get("x"), pos.get("y")
    # 兼容 id 和 assetId 两种字段名
    asset_id = asset.get("id") or asset.get("assetId")
    half = bbox_for_assetid(asset_id, None, bbox_lookup)
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
        # 兼容 id 和 assetId 两种字段名
        aid = obj.get("id") or obj.get("assetId", "")
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
    bbox_lookup = build_bbox_lookup(asset_lib)
    
    # 构建 location -> platform 映射（自动规范化匹配）
    location_mapping = build_location_to_platform(protocol, room_assets)
    if VERBOSE:
        print("Location mapping (auto-matched):")
        for loc, plat in location_mapping.items():
            print(f"  {loc} -> {plat}")
        print()
        print(f"Loaded {len(bbox_lookup)} assets into bbox_lookup")
        print()

    steps = protocol.get("procedure", [])
    print(f"Loaded {len(steps)} steps; offset_radius = {offset_radius} m\n")

    for step in steps:
        loc = step.get("location", "BENCH")
        platform = pick_platform(loc, room_assets, location_mapping)
        target = None
        debug_attempts = []

        print(f"Step {step.get('step_number')}: {step.get('description')}")
        print(f"  Location: {loc} -> {location_mapping.get(loc, loc)}")

        if platform:
            info = asset_bbox_info(platform, bbox_lookup)
            if info:
                cx, cy = info["center"]
                print(
                    f"  Platform bbox {platform.get('id')}: "
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
