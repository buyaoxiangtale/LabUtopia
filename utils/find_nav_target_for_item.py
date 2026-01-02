#!/usr/bin/env python3
"""
为实验平台上的物品生成导航目标点

功能：
1. 从房间布局中选择一个在 experimental_platform 上的物品
2. 找到平台外的导航位置，使得机器人可以面向该物品
3. 验证导航点是否在障碍物外（使用导航网格）
4. 参考现有的 nav_goal_targets_demo 逻辑

使用示例：
    python utils/find_nav_target_for_item.py --item Beaker
    python utils/find_nav_target_for_item.py --item RoundBottomFlask --visualize
"""

import json
import math
import argparse
import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np

# ==================== 路径配置 ====================
ROOT = Path(__file__).resolve().parent.parent

# 默认文件路径
DEFAULT_ROOM_LAYOUT = ROOT / "roomlayout/12_17/Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json"
DEFAULT_ASSET_LIB = ROOT / "roomlayout/12_17/assets_annotated.json"
DEFAULT_NAV_CFG = ROOT / "config/navigation/navigation_assets_12_18.yaml"

# 导航偏移半径（机器人半径 + 安全距离）
OFFSET_RADIUS = 0.6

# ==================== 工具函数 ====================

def _norm(s: str) -> str:
    """规范化字符串：去除下划线/空格/横线，转小写"""
    if not s:
        return ""
    return str(s).lower().replace(" ", "").replace("-", "").replace("_", "")


def load_json(path):
    """加载 JSON 或 YAML 文件（支持 Path 或 str）"""
    path = Path(path)
    if not path.exists():
        print(f"❌ 错误: 找不到文件 {path}")
        return {}

    with open(path, "r", encoding="utf-8") as f:
        # 根据文件扩展名选择解析器
        if path.suffix in ['.yaml', '.yml']:
            return yaml.safe_load(f)
        else:
            return json.load(f)


def build_bbox_lookup(asset_lib: dict) -> dict:
    """
    构建资产尺寸查找表
    支持 assets_annotated.json 格式：geometry -> bbox -> {short, long}
    """
    lookup = {}
    for a in asset_lib.get("assets", []):
        asset_id = a.get("id", "")
        name = a.get("name", "")

        geometry = a.get("geometry", {})
        bbox = geometry.get("bbox", {})

        if not bbox:
            continue

        short_side = bbox.get("short", 0)
        long_side = bbox.get("long", 0)

        # half_x 使用 short/2, half_y 使用 long/2
        half_tuple = (short_side / 2.0, long_side / 2.0)

        if asset_id:
            lookup[_norm(asset_id)] = half_tuple
        if name:
            lookup[_norm(name)] = half_tuple

    return lookup


def get_object_bbox(obj: dict, bbox_lookup: dict) -> Optional[Tuple[float, float, float, float]]:
    """
    获取对象的边界框信息

    Returns:
        (center_x, center_y, half_x, half_y) 或 None
    """
    pos = obj.get("position", {})
    ox, oy = pos.get("x"), pos.get("y")

    # 使用映射后的资产ID
    asset_id = get_asset_id_for_object(obj)
    half = bbox_lookup.get(_norm(asset_id))

    if not half:
        # 尝试前缀匹配
        for k, v in bbox_lookup.items():
            if _norm(asset_id).startswith(k):
                half = v
                break

    if ox is None or oy is None or not half:
        return None

    hx, hy = half
    return (ox, oy, hx, hy)


# ==================== 核心逻辑 ====================

# 特殊映射：房间布局ID -> 资产库ID
PLATFORM_ID_MAPPING = {
    "labbench": "ExperimentalPlatform",
    "experimentalplatform": "ExperimentalPlatform",
    "validationplatform": "ValidationPlatform",
    "fumehood": "FumeHood"
}


def find_experimental_platform(room_assets: dict) -> Optional[dict]:
    """找到实验平台对象"""
    for obj in room_assets.get("objects", []):
        obj_id = obj.get("id", "")
        if "platform" in obj_id.lower() or "labbench" in obj_id.lower():
            return obj
    return None


def get_asset_id_for_object(obj: dict) -> str:
    """
    获取对象在资产库中对应的ID

    处理映射关系，如 LabBench -> ExperimentalPlatform
    """
    obj_id = obj.get("id", "")
    norm_id = _norm(obj_id)

    # 检查特殊映射
    for key, value in PLATFORM_ID_MAPPING.items():
        if key in norm_id:
            return value

    # 否则返回原ID
    return obj_id


def find_item_on_platform(room_assets: dict, item_name: str) -> Optional[dict]:
    """在实验平台上找到指定物品"""
    norm_target = _norm(item_name)

    for obj in room_assets.get("objects", []):
        obj_id = obj.get("id", "")
        location = obj.get("initial_location", "")

        # 检查是否在实验平台上且ID匹配
        if location == "experimental_platform" and norm_target in _norm(obj_id):
            return obj

    return None


def list_items_on_platform(room_assets: dict) -> List[dict]:
    """列出所有在实验平台上的物品"""
    items = []
    for obj in room_assets.get("objects", []):
        if obj.get("initial_location") == "experimental_platform":
            items.append(obj)
    return items


def calculate_nav_target_for_item(
    item: dict,
    platform: dict,
    offset_radius: float,
    bbox_lookup: dict
) -> Optional[dict]:
    """
    为实验平台上的物品计算导航目标点

    策略：
    1. 找到物品在平台上的相对位置
    2. 计算从平台中心到物品的方向
    3. 在平台边缘，沿该方向向外延伸 offset_radius
    4. 确保机器人面向物品
    """
    # 获取平台信息
    platform_info = get_object_bbox(platform, bbox_lookup)
    if not platform_info:
        print("❌ 无法获取平台边界信息")
        return None

    px, py, phx, phy = platform_info
    prot_z = platform.get("rotation", {}).get("z", 0)

    # 获取物品信息
    item_pos = item.get("position", {})
    ix, iy = item_pos.get("x"), item_pos.get("y")

    # 计算从平台中心到物品的向量
    dx_item = ix - px
    dy_item = iy - py

    # 归一化方向
    dist = math.sqrt(dx_item**2 + dy_item**2)
    if dist < 0.01:
        # 物品在中心附近，使用平台的"正面"方向
        rad = math.radians(prot_z)
        dx_item = -math.sin(rad)
        dy_item = math.cos(rad)
    else:
        dx_item /= dist
        dy_item /= dist

    # 计算导航目标点（在平台边缘外）
    # 策略：从物品位置向外推到平台边缘，再加上 offset_radius
    # 这确保机器人面向物品

    # 计算从平台中心到物品的角度
    angle_to_item = math.atan2(dy_item, dx_item)

    # 平台半长（对角线方向的最大半径）
    platform_radius = math.sqrt(phx**2 + phy**2)

    # 导航点位置：沿从平台中心到物品的方向，向外延伸
    nav_dist = platform_radius + offset_radius
    nx = px + nav_dist * math.cos(angle_to_item)
    ny = py + nav_dist * math.sin(angle_to_item)

    # 机器人朝向：面向物品
    # 机器人的朝向应该指向物品，即 angle_to_item
    robot_theta = angle_to_item

    return {
        "nav_target": {
            "x": nx,
            "y": ny,
            "theta": robot_theta
        },
        "item": {
            "id": item.get("id"),
            "position": (ix, iy)
        },
        "platform": {
            "id": platform.get("id"),
            "center": (px, py),
            "half_size": (phx, phy),
            "rotation": prot_z
        },
        "offset_info": {
            "distance_from_platform_edge": offset_radius,
            "direction_to_item": math.degrees(angle_to_item)
        }
    }


def verify_nav_target_free(
    nav_target: dict,
    nav_cfg_path: Path,
    room_assets: dict
) -> Tuple[bool, str]:
    """
    验证导航目标点是否在障碍物外（使用导航网格）
    """
    # 导入导航相关模块
    import sys
    sys.path.insert(0, str(ROOT))

    from utils.a_star import load_grid, real_to_grid

    # 加载导航配置
    nav_cfg = load_json(nav_cfg_path)
    nav_scene = nav_cfg["assets"][0]

    # 加载障碍物网格
    grid, W, H = load_grid(nav_scene["barrier_image_path"])
    x_bounds = nav_scene["x_bounds"]
    y_bounds = nav_scene["y_bounds"]
    offset_radius = nav_scene["offset_radius"]

    # 检查导航点是否在网格内
    nx, ny = nav_target["x"], nav_target["y"]

    if not (x_bounds[0] <= nx <= x_bounds[1]):
        return False, f"导航点X坐标超出边界: {nx} not in {x_bounds}"
    if not (y_bounds[0] <= ny <= y_bounds[1]):
        return False, f"导航点Y坐标超出边界: {ny} not in {y_bounds}"

    # 转换为网格坐标
    grid_x, grid_y = real_to_grid(nx, ny, x_bounds, y_bounds, (W, H))

    # 检查是否在障碍物上（考虑膨胀）
    from utils.a_star import inflate_obstacles
    inflated_grid = inflate_obstacles(grid, int(offset_radius / ((x_bounds[1] - x_bounds[0]) / W)))

    is_free = inflated_grid[grid_x][grid_y] == 0

    if is_free:
        return True, "✓ 导航点在自由空间中"
    else:
        return False, f"✗ 导航点在障碍物上（网格坐标: {grid_x}, {grid_y}）"


# ==================== 主程序 ====================

def main():
    parser = argparse.ArgumentParser(
        description="为实验平台上的物品生成导航目标点"
    )
    parser.add_argument(
        "--item",
        type=str,
        help="物品ID（如 Beaker, RoundBottomFlask 等）"
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="列出所有在实验平台上的物品"
    )
    parser.add_argument(
        "--room-layout",
        type=str,
        default=str(DEFAULT_ROOM_LAYOUT),
        help="房间布局JSON文件路径"
    )
    parser.add_argument(
        "--asset-lib",
        type=str,
        default=str(DEFAULT_ASSET_LIB),
        help="资产库JSON文件路径"
    )
    parser.add_argument(
        "--nav-cfg",
        type=str,
        default=str(DEFAULT_NAV_CFG),
        help="导航配置文件路径"
    )
    parser.add_argument(
        "--offset-radius",
        type=float,
        default=OFFSET_RADIUS,
        help="导航偏移半径（米）"
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="验证导航点是否在障碍物外"
    )
    parser.add_argument(
        "--output",
        type=str,
        help="输出结果到JSON文件"
    )

    args = parser.parse_args()

    # 1. 加载数据
    print("=" * 80)
    print("实验平台物品导航目标点生成工具")
    print("=" * 80)

    room_assets = load_json(Path(args.room_layout))
    asset_lib = load_json(Path(args.asset_lib))

    if not room_assets or not asset_lib:
        print("❌ 无法加载必要的文件")
        return

    bbox_lookup = build_bbox_lookup(asset_lib)
    print(f"✓ 已加载 {len(bbox_lookup)} 个资产定义")
    print(f"✓ 导航偏移半径: {args.offset_radius} 米")

    # 2. 列出所有在实验平台上的物品
    items_on_platform = list_items_on_platform(room_assets)
    experimental_platform = find_experimental_platform(room_assets)

    if not experimental_platform:
        print("❌ 无法找到实验平台")
        return

    print(f"✓ 实验平台: {experimental_platform.get('id')}")
    print(f"  位置: {experimental_platform.get('position')}")
    print(f"  旋转: {experimental_platform.get('rotation')}")

    if args.list:
        print(f"\n在实验平台上的物品 ({len(items_on_platform)} 个):")
        for i, item in enumerate(items_on_platform, 1):
            print(f"  {i}. {item.get('id')}")
            print(f"     位置: {item.get('position')}")
        return

    # 3. 选择目标物品
    if not args.item:
        print("\n请使用 --item 指定物品ID，或使用 --list 查看所有物品")
        print("示例: python find_nav_target_for_item.py --item Beaker")
        return

    target_item = find_item_on_platform(room_assets, args.item)

    if not target_item:
        print(f"❌ 无法在实验平台上找到物品: {args.item}")
        print(f"提示: 使用 --list 查看可用物品")
        return

    print(f"\n✓ 目标物品: {target_item.get('id')}")
    print(f"  位置: {target_item.get('position')}")
    print(f"  旋转: {target_item.get('rotation')}")

    # 4. 计算导航目标点
    print("\n" + "-" * 80)
    print("计算导航目标点...")
    print("-" * 80)

    nav_result = calculate_nav_target_for_item(
        target_item,
        experimental_platform,
        args.offset_radius,
        bbox_lookup
    )

    if not nav_result:
        print("❌ 无法计算导航目标点")
        return

    # 5. 显示结果
    nav_target = nav_result["nav_target"]
    item_info = nav_result["item"]
    platform_info = nav_result["platform"]
    offset_info = nav_result["offset_info"]

    print("\n导航目标点信息:")
    print(f"  位置: X={nav_target['x']:.3f}, Y={nav_target['y']:.3f}")
    print(f"  朝向: θ={nav_target['theta']:.3f} 弧度 ({math.degrees(nav_target['theta']):.1f}°)")
    print(f"\n物品信息:")
    print(f"  ID: {item_info['id']}")
    print(f"  位置: ({item_info['position'][0]:.3f}, {item_info['position'][1]:.3f})")
    print(f"\n平台信息:")
    print(f"  ID: {platform_info['id']}")
    print(f"  中心: ({platform_info['center'][0]:.3f}, {platform_info['center'][1]:.3f})")
    print(f"  尺寸: {platform_info['half_size'][0]*2:.2f}m × {platform_info['half_size'][1]*2:.2f}m")
    print(f"  旋转: {platform_info['rotation']}°")
    print(f"\n偏移信息:")
    print(f"  平台边缘外距离: {offset_info['distance_from_platform_edge']:.2f}m")
    print(f"  相对物品方向: {offset_info['direction_to_item']:.1f}°")

    # 6. 验证导航点
    if args.verify:
        print("\n" + "-" * 80)
        print("验证导航点...")
        print("-" * 80)

        is_valid, message = verify_nav_target_free(
            nav_target,
            Path(args.nav_cfg),
            room_assets
        )

        print(message)

        if not is_valid:
            print("\n⚠️  警告: 导航点可能不可达")
            print("建议：调整偏移半径或手动微调位置")

    # 7. 输出到文件
    if args.output:
        output_data = {
            "item": {
                "id": item_info['id'],
                "position": {"x": item_info['position'][0], "y": item_info['position'][1]}
            },
            "platform": {
                "id": platform_info['id'],
                "center": {"x": platform_info['center'][0], "y": platform_info['center'][1]},
                "half_size": {"x": platform_info['half_size'][0], "y": platform_info['half_size'][1]},
                "rotation": platform_info['rotation']
            },
            "nav_target": {
                "position": {"x": nav_target['x'], "y": nav_target['y']},
                "rotation": {"z": math.degrees(nav_target['theta'])}
            },
            "offset_info": offset_info
        }

        with open(args.output, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)

        print(f"\n✓ 结果已保存到: {args.output}")

    # 8. 生成使用代码示例
    print("\n" + "=" * 80)
    print("使用示例代码:")
    print("=" * 80)
    start_x = nav_target['x']
    start_y = nav_target['y']
    start_theta = math.degrees(nav_target['theta'])
    end_x = item_info['position'][0]
    end_y = item_info['position'][1]

    print(f"""
# 在你的导航任务中使用:

goal_pairs:
  - start: [{start_x:.3f}, {start_y:.3f}, {start_theta:.2f}]
    end: [{end_x:.3f}, {end_y:.3f}, 0.0]
    # 说明: 从平台外面向 {item_info['id']}
""")


if __name__ == "__main__":
    main()
