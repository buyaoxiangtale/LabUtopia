#!/usr/bin/env python3
"""
提取导航目标点并生成 JSON/YAML 格式的 goal_pairs 文件
完全独立的实现，集成了父对象查找和 bbox 比较逻辑
"""

import json
import math
import yaml
import argparse
from pathlib import Path
from typing import Dict, Optional, Tuple, List

# ================= 路径配置 =================
ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json"
ROOM_ASSETS = ROOT / "Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json"
ASSET_LIB = ROOT / "assets_annotated.json"
OFFSET_RADIUS = 0.6

# ================= 辅助函数 =================

def _norm(s: str) -> str:
    """规范化字符串：去除下划线/空格/横线，转小写"""
    if not s: return ""
    return str(s).lower().replace(" ", "").replace("-", "").replace("_", "")

def load_json(path: Path) -> dict:
    if not path.exists():
        print(f"警告: 找不到文件 {path}")
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

# 映射逻辑：将协议中的位置映射到房间布局中的对象 ID
SPECIAL_MAPPING = {
    "rotaryevaporator": "RotaryEvaporator", # 修正：rotaryevaporator 通常指设备本身，可能在 floor 上也可能在台子上
    "labbench": "ExperimentalPlatform",
    "bench": "ExperimentalPlatform",
    "hood": "FumeHood",
    "fumehood": "FumeHood",
    "validation_platform": "ValidationPlatform",
    "reagent_cabinet": "ReagentCabinet"
}

def build_bbox_lookup(asset_lib: dict) -> Dict[str, Tuple[float, float]]:
    """
    从 assets_annotated.json 构建 bbox 查找表
    返回: normalized_name -> (half_x, half_y)
    """
    lookup = {}
    assets_list = asset_lib.get("assets", [])
    
    for a in assets_list:
        asset_id = a.get("id", "")
        name = a.get("name", "")
        
        # 从 geometry.bbox 获取尺寸
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

def get_platform_info(platform: dict, bbox_lookup: dict) -> Optional[dict]:
    """获取物体的中心、旋转、半尺寸及完整尺寸"""
    pos = platform.get("position", {})
    ox, oy = pos.get("x"), pos.get("y")
    
    # 获取资产库中的尺寸
    obj_id = platform.get("id") or platform.get("assetId")
    half = bbox_lookup.get(_norm(obj_id))
    
    # 尝试前缀匹配
    if not half and obj_id:
        norm_id = _norm(obj_id)
        for key, value in bbox_lookup.items():
            if norm_id.startswith(key) or key.startswith(norm_id):
                half = value
                break
    
    if ox is None or oy is None or not half:
        return None
        
    hx, hy = half
    return {
        "id": obj_id,
        "center": (ox, oy),
        "half_x": hx,
        "half_y": hy,
        "width": hx * 2,  # 完整宽度
        "depth": hy * 2,  # 完整深度
        "rz": platform.get("rotation", {}).get("z", 0)
    }

def calculate_nav_target(platform: dict, offset_radius: float, bbox_lookup: dict) -> Optional[dict]:
    """
    计算导航目标点
    - 0度对应 +Y, 270度对应 +X
    - 坐标计算结果采用“向上取整”逻辑，确保安全距离
    """
    info = get_platform_info(platform, bbox_lookup)
    if not info: return None
    
    cx, cy = info["center"]
    hy = info["half_y"]
    rz_deg = info["rz"]
    
    # 机器人停靠距离
    dist = hy + offset_radius
    rad = math.radians(rz_deg)
    
    # 1. 基础投影计算 (0deg=+Y, 270deg=+X)
    # 假设物体正面朝向其局部坐标系的 -Y 方向
    # 旋转后偏移量：
    # dx = -dist * sin(rad)
    # dy = -dist * cos(rad)  <-- 修正：如果是 -Y 方向，旋转 0 度应该在 (0, -dist)
    
    # 让我们重新推导一下：
    # 物体局部坐标系中，正面在 (0, -dist)
    # 旋转矩阵 R = [[cos, -sin], [sin, cos]]
    # 全局坐标偏移 = R * [0, -dist]^T
    # dx = 0*cos - (-dist)*sin = dist * sin(rad)
    # dy = 0*sin + (-dist)*cos = -dist * cos(rad)
    
    # 但是之前的代码使用的是：
    # dx = -dist * math.sin(rad)
    # dy = dist * math.cos(rad)
    # 这意味着它假设正面在 (0, dist) 即 +Y 方向
    
    # 根据 Isaac Sim 标准 (Y-forward)，物体正面通常是 +Y
    # 但很多资产（如桌子、柜子）正面可能是 -Y
    # 这里保持与原 demo 一致的逻辑（假设正面在 +Y ? 或者之前的逻辑是为了适配特定的旋转定义）
    
    # 回看 nav_goal_targets_demo_12_17.py 的逻辑：
    # dx_raw = -dist * math.sin(rad)
    # dy_raw = dist * math.cos(rad)
    # 如果 rad=0, dx=0, dy=dist -> (0, dist) -> +Y 方向
    # 如果 rad=90, dx=-dist, dy=0 -> (-dist, 0) -> -X 方向
    
    # 让我们使用通用的逻辑：
    # 假设物体需要从"正面"接近。
    # 对于大多数家具，如果 rotation=0，正面通常朝向 -Y (Isaac Sim Viewport 默认视角) 或者 +Y
    # 根据之前的注释 "0度对应 +Y"，我们沿用之前的公式
    
    dx_raw = -dist * math.sin(rad)
    dy_raw = dist * math.cos(rad)
    
    tx_raw = cx + dx_raw
    ty_raw = cy + dy_raw

    # 2. 向上取整逻辑 (往大舍入)
    precision = 1000.0
    tx = math.ceil(tx_raw * precision) / precision
    ty = math.ceil(ty_raw * precision) / precision
    
    return {
        "x": tx, "y": ty,
        "info": info
    }

def calculate_robot_theta(rz_deg):
    """
    根据物体旋转角度计算机器人朝向角度
    规则：机器人应该面对物体
    如果物体朝向 Rz，机器人应该朝向 (Rz + 180) % 360
    """
    robot_theta_deg = (rz_deg + 180) % 360
    robot_theta = math.radians(robot_theta_deg)
    return robot_theta

# ================= 核心逻辑：对象解析与父对象查找 =================

def find_object_by_id(obj_id: str, objects: list) -> Optional[dict]:
    """在 objects 列表中查找指定 id 的对象"""
    norm_id = _norm(obj_id)
    # 精确匹配
    for obj in objects:
        if _norm(obj.get("id", "")) == norm_id:
            return obj
    # 模糊匹配
    for obj in objects:
        real_id = _norm(obj.get("id", ""))
        if real_id.startswith(norm_id) or norm_id.startswith(real_id):
            return obj
    return None

def get_bbox_area(info: dict) -> float:
    """计算 bbox 面积"""
    return info["width"] * info["depth"]

def resolve_target_object(location: str, room_assets_data: dict, bbox_lookup: dict) -> Optional[dict]:
    """
    解析目标对象：
    1. 映射 location 到对象 ID
    2. 查找对象实例
    3. 检查 initial_location
    4. 如果不在 floor，查找父对象并比较 bbox
    5. 返回最终的目标对象
    """
    objects = room_assets_data.get("objects", [])
    
    # 1. 映射 location
    norm_loc = _norm(location)
    target_id = SPECIAL_MAPPING.get(norm_loc)
    
    if not target_id:
        # 尝试直接匹配
        target_id = location
    
    # 2. 查找对象实例
    # 优先查找 ID 完全匹配的
    target_obj = find_object_by_id(target_id, objects)
    
    if not target_obj:
        # 尝试查找包含该 ID 的（例如 location="ReagentCabinet" -> 找到 "ReagentCabinet"）
        # 或者 location="SodiumHydride" -> 找到 "SodiumHydride"
        pass
        
    if not target_obj:
        print(f"  警告: 无法在房间中找到对应 '{location}' (ID: {target_id}) 的对象")
        return None
        
    # 3. 检查 initial_location
    initial_loc = target_obj.get("initial_location", "").lower()
    
    # 如果 initial_location 为空或 floor，直接返回该对象
    if not initial_loc or initial_loc == "floor":
        return target_obj
        
    # 4. 查找父对象
    print(f"  发现对象 '{target_obj.get('id')}' 的 initial_location 为 '{initial_loc}'，尝试查找父对象...")
    parent_obj = find_object_by_id(initial_loc, objects)
    
    if not parent_obj:
        print(f"  警告: 找不到父对象 '{initial_loc}'，将使用原对象")
        return target_obj
        
    # 获取两个对象的 info (包含 bbox)
    child_info = get_platform_info(target_obj, bbox_lookup)
    parent_info = get_platform_info(parent_obj, bbox_lookup)
    
    if not child_info or not parent_info:
        print("  警告: 无法获取对象或父对象的尺寸信息，将使用原对象")
        return target_obj
        
    # 5. 比较 bbox 面积
    child_area = get_bbox_area(child_info)
    parent_area = get_bbox_area(parent_info)
    
    if parent_area >= child_area:
        print(f"  ✓ 切换目标: {target_obj.get('id')} -> {parent_obj.get('id')} (父对象面积更大: {parent_area:.2f} >= {child_area:.2f})")
        return parent_obj
    else:
        print(f"  ✓ 保持原目标: {target_obj.get('id')} (面积 {child_area:.2f} > 父对象 {parent_area:.2f})")
        return target_obj

# ================= 主流程 =================

def extract_nav_goals(protocol_data, room_assets_data, asset_lib_data, offset_radius=0.6):
    """
    提取所有步骤的导航目标点
    """
    bbox_lookup = build_bbox_lookup(asset_lib_data)
    steps = protocol_data.get("procedure", [])
    nav_points = []
    
    print(f"开始处理 {len(steps)} 个步骤...")
    
    for step in steps:
        step_idx = step.get("step_number")
        loc = step.get("location", "BENCH")
        
        print(f"\n步骤 {step_idx}: Location = {loc}")
        
        # 解析目标对象（包含父对象查找逻辑）
        target_instance = resolve_target_object(loc, room_assets_data, bbox_lookup)
        
        if target_instance:
            target = calculate_nav_target(target_instance, offset_radius, bbox_lookup)
            if target:
                info = target["info"]
                x = target["x"]
                y = target["y"]
                rz_deg = info["rz"]
                
                theta = calculate_robot_theta(rz_deg)
                current_point = [x, y, theta]
                
                # 去重检查
                is_duplicate = False
                if nav_points:
                    last_point = nav_points[-1]["point"]
                    tolerance = 1e-3
                    if (abs(current_point[0] - last_point[0]) < tolerance and
                        abs(current_point[1] - last_point[1]) < tolerance and
                        abs(current_point[2] - last_point[2]) < tolerance):
                        is_duplicate = True
                        print(f"  -> 跳过重复点")
                
                if not is_duplicate:
                    nav_points.append({
                        "step": step_idx,
                        "location": loc,
                        "target_id": target_instance.get("id"),
                        "point": current_point,
                        "robot_theta_deg": math.degrees(theta)
                    })
                    print(f"  -> 生成导航点: [{x:.3f}, {y:.3f}, {theta:.3f}] (目标: {target_instance.get('id')})")
            else:
                print(f"  警告: 无法计算导航目标（可能缺失尺寸信息）")
        else:
            print(f"  错误: 无法解析目标对象")
            
    return nav_points

def generate_goal_pairs(nav_points):
    """生成路径对"""
    goal_pairs = []
    if len(nav_points) < 2:
        return goal_pairs
    
    for i in range(len(nav_points) - 1):
        start_point = nav_points[i]["point"]
        end_point = nav_points[i + 1]["point"]
        
        # 简单检查是否相同
        tolerance = 1e-3
        if (abs(start_point[0] - end_point[0]) < tolerance and
            abs(start_point[1] - end_point[1]) < tolerance and
            abs(start_point[2] - end_point[2]) < tolerance):
            continue
            
        goal_pairs.append({
            "start": start_point,
            "end": end_point
        })
    return goal_pairs

def main():
    parser = argparse.ArgumentParser(description='提取导航目标点并生成 JSON/YAML 格式')
    parser.add_argument('--protocol', type=str, default=str(PROTOCOL))
    parser.add_argument('--room-assets', type=str, default=str(ROOM_ASSETS))
    parser.add_argument('--asset-lib', type=str, default=str(ASSET_LIB))
    parser.add_argument('--offset-radius', type=float, default=OFFSET_RADIUS)
    parser.add_argument('--output', type=str, default=None)
    parser.add_argument('--format', type=str, choices=['json', 'yaml'], default='json')
    
    args = parser.parse_args()
    
    protocol_data = load_json(Path(args.protocol))
    room_assets_data = load_json(Path(args.room_assets))
    asset_lib_data = load_json(Path(args.asset_lib))
    
    print("=" * 80)
    print("提取导航目标点 (集成父对象查找逻辑)")
    print("=" * 80)
    
    nav_points = extract_nav_goals(protocol_data, room_assets_data, asset_lib_data, args.offset_radius)
    
    if not nav_points:
        print("未生成任何导航点")
        return

    goal_pairs = generate_goal_pairs(nav_points)
    
    output_data = {"goal_pairs": goal_pairs}
    
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = Path(__file__).parent / f"goal_pairs.{args.format}"
        
    print("\n" + "=" * 80)
    print(f"保存结果到: {output_path}")
    
    if args.format == 'json':
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)
    else:
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)
            
    print(f"成功生成 {len(goal_pairs)} 个路径对")

if __name__ == "__main__":
    main()
