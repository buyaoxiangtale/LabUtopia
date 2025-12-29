"""
处理 initial_location 不在 floor 上的对象
- 查找对象的 initial_location 对应的父对象
- 在 room_assets 中找到父对象
- 从 assets_annotated.json 获取两个对象的 bbox
- 计算并返回更大 bbox 对应的对象 id
"""

import json
from pathlib import Path
import math
from typing import Dict, Optional, Tuple

# ================= 配置路径 =================
ROOT = Path(__file__).resolve().parent
ROOM_ASSETS_PATH = ROOT / "Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json"
ASSET_LIB_PATH = ROOT / "assets_annotated.json"
OUTPUT_PATH = ROOT / "processed_objects_mapping.json"


# ================= 辅助函数 =================

def _norm(s: str) -> str:
    """规范化字符串：去除下划线/空格/横线，转小写"""
    if not s:
        return ""
    return str(s).lower().replace(" ", "").replace("-", "").replace("_", "")


def load_json(path: Path) -> dict:
    """加载 JSON 文件"""
    if not path.exists():
        print(f"警告: 找不到文件 {path}")
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


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


def find_object_by_id(obj_id: str, objects: list) -> Optional[dict]:
    """
    在 objects 列表中查找指定 id 的对象
    支持规范化匹配和模糊匹配
    """
    norm_id = _norm(obj_id)
    
    # 精确匹配
    for obj in objects:
        obj_id_real = obj.get("id", "")
        if _norm(obj_id_real) == norm_id:
            return obj
    
    # 模糊匹配
    for obj in objects:
        obj_id_real = obj.get("id", "")
        if _norm(obj_id_real).startswith(norm_id) or norm_id.startswith(_norm(obj_id_real)):
            return obj
    
    return None


def get_bbox_area(half_x: float, half_y: float) -> float:
    """计算 bbox 面积（简化为矩形面积）"""
    return (half_x * 2) * (half_y * 2)


def get_object_bbox(obj: dict, bbox_lookup: Dict[str, Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """
    获取对象的 bbox (half_x, half_y)
    如果找不到，返回 None
    """
    obj_id = obj.get("id") or obj.get("assetId", "")
    if not obj_id:
        return None
    
    # 先尝试精确匹配
    norm_id = _norm(obj_id)
    bbox = bbox_lookup.get(norm_id)
    
    if bbox:
        return bbox
    
    # 尝试前缀匹配
    for key, value in bbox_lookup.items():
        if key.startswith(norm_id) or norm_id.startswith(key):
            return value
    
    return None


def process_nonfloor_objects(room_assets: dict, asset_lib: dict) -> dict:
    """
    处理所有 initial_location 不在 floor 上的对象
    返回映射关系: 原对象id -> 更大的对象id
    """
    objects = room_assets.get("objects", [])
    bbox_lookup = build_bbox_lookup(asset_lib)
    
    result = {}
    processed_count = 0
    
    for obj in objects:
        obj_id = obj.get("id", "")
        initial_loc = obj.get("initial_location", "")
        
        # 只处理不在 floor 上的对象
        if not initial_loc or initial_loc.lower() == "floor":
            continue
        
        # 查找父对象
        parent_obj = find_object_by_id(initial_loc, objects)
        
        if not parent_obj:
            print(f"  ⚠️  警告: 找不到父对象 '{initial_loc}' (对于对象 '{obj_id}')")
            result[obj_id] = obj_id  # 使用原对象
            continue
        
        # 获取两个对象的 bbox
        child_bbox = get_object_bbox(obj, bbox_lookup)
        parent_bbox = get_object_bbox(parent_obj, bbox_lookup)
        
        if not child_bbox or not parent_bbox:
            print(f"  ⚠️  警告: 无法获取 bbox (对象 '{obj_id}' 或 父对象 '{initial_loc}')")
            result[obj_id] = obj_id  # 使用原对象
            continue
        
        # 计算面积
        child_area = get_bbox_area(child_bbox[0], child_bbox[1])
        parent_area = get_bbox_area(parent_bbox[0], parent_bbox[1])
        
        # 选择更大的
        if parent_area >= child_area:
            larger_id = parent_obj.get("id", "")
            result[obj_id] = larger_id
            print(f"  ✓ {obj_id} (面积: {child_area:.4f}) -> {larger_id} (面积: {parent_area:.4f}) [使用父对象]")
        else:
            result[obj_id] = obj_id
            print(f"  ✓ {obj_id} (面积: {child_area:.4f}) [使用原对象，父对象面积: {parent_area:.4f}]")
        
        processed_count += 1
    
    print(f"\n处理完成: 共处理 {processed_count} 个非 floor 对象")
    return result


def update_location_mapping_with_parent(original_mapping: dict, processed_mapping: dict) -> dict:
    """
    使用处理后的映射更新原始的 location 映射
    如果 protocol 中的 location 映射到的对象是 processed_mapping 中的键，
    则替换为更大的对象 id
    """
    updated = {}
    
    for protocol_loc, platform_id in original_mapping.items():
        # 检查该 platform_id 是否需要替换
        new_platform_id = processed_mapping.get(platform_id, platform_id)
        updated[protocol_loc] = new_platform_id
        
        if new_platform_id != platform_id:
            print(f"  更新映射: {protocol_loc}: {platform_id} -> {new_platform_id}")
    
    return updated


# ================= 主程序 =================

def main():
    """主函数"""
    print("=" * 80)
    print("处理非 floor 对象 - 计算更大 bbox 的父对象")
    print("=" * 80)
    
    # 1. 加载数据
    room_assets = load_json(ROOM_ASSETS_PATH)
    asset_lib = load_json(ASSET_LIB_PATH)
    
    if not room_assets or not asset_lib:
        print("错误: 无法加载必要的数据文件")
        return
    
    print(f"\n✓ 已加载 room_assets: {ROOM_ASSETS_PATH.name}")
    print(f"✓ 已加载 asset_lib: {ASSET_LIB_PATH.name}")
    print(f"✓ room_assets 中共有 {len(room_assets.get('objects', []))} 个对象")
    print(f"✓ asset_lib 中共有 {len(asset_lib.get('assets', []))} 个资产定义\n")
    
    # 2. 处理非 floor 对象
    print("开始处理非 floor 对象...")
    print("-" * 80)
    processed_mapping = process_nonfloor_objects(room_assets, asset_lib)
    
    # 3. 保存结果
    print(f"\n保存结果到: {OUTPUT_PATH}")
    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump({
            "description": "映射: 原对象id -> 更大的对象id (针对非 floor 对象)",
            "processed_mapping": processed_mapping,
            "total_processed": len(processed_mapping)
        }, f, indent=2, ensure_ascii=False)
    
    print("✓ 保存完成")
    
    # 4. 显示统计
    print("\n" + "=" * 80)
    print("统计信息")
    print("=" * 80)
    print(f"处理的非 floor 对象数量: {len(processed_mapping)}")
    
    # 分类统计
    use_parent_count = sum(1 for k, v in processed_mapping.items() if k != v)
    use_self_count = len(processed_mapping) - use_parent_count
    
    print(f"  - 使用父对象: {use_parent_count}")
    print(f"  - 使用原对象: {use_self_count}")
    
    # 5. 显示映射表
    if processed_mapping:
        print("\n映射详情:")
        print("-" * 80)
        for child_id, parent_id in processed_mapping.items():
            status = "→" if child_id != parent_id else "="
            print(f"  {child_id:40} {status} {parent_id}")


def main_integration_test():
    """
    集成测试: 模拟 nav_goal_targets_demo_12_17.py 中的使用场景
    """
    print("\n" + "=" * 80)
    print("集成测试: 模拟 location 映射更新")
    print("=" * 80)
    
    # 加载数据
    room_assets = load_json(ROOM_ASSETS_PATH)
    asset_lib = load_json(ASSET_LIB_PATH)
    protocol_path = ROOT / "protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json"
    protocol = load_json(protocol_path)
    
    if not room_assets or not asset_lib or not protocol:
        print("错误: 无法加载必要的数据文件")
        return
    
    # 1. 处理非 floor 对象
    processed_mapping = process_nonfloor_objects(room_assets, asset_lib)
    
    # 2. 构建原始 location 映射 (简化版)
    from nav_goal_targets_demo_12_17 import build_location_to_platform
    original_mapping = build_location_to_platform(protocol, room_assets)
    
    # 3. 更新映射
    print("\n更新 location 映射...")
    print("-" * 80)
    updated_mapping = update_location_mapping_with_parent(original_mapping, processed_mapping)
    
    # 4. 保存完整映射
    output_full_path = ROOT / "complete_location_mapping.json"
    with open(output_full_path, "w", encoding="utf-8") as f:
        json.dump({
            "description": "完整的 location 映射 (已更新为使用更大的对象)",
            "original_mapping": original_mapping,
            "processed_mapping": processed_mapping,
            "updated_mapping": updated_mapping
        }, f, indent=2, ensure_ascii=False)
    
    print(f"\n✓ 完整映射已保存到: {output_full_path}")


if __name__ == "__main__":
    # 运行主程序
    main()
    
    # 可选: 运行集成测试
    print("\n是否运行集成测试? (y/n): ", end="")
    choice = input().strip().lower()
    if choice == "y":
        try:
            main_integration_test()
        except ImportError as e:
            print(f"无法导入 nav_goal_targets_demo_12_17: {e}")
            print("跳过集成测试")

