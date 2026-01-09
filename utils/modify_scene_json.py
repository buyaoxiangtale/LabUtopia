#!/usr/bin/env python3
"""
场景 JSON 修改工具

快速修改场景布局文件中的物体位置或移除物体
"""

import json
import sys
from pathlib import Path


def remove_object(
    json_path: str,
    object_id: str,
    output_path: str = None
):
    """从场景中移除物体"""
    with open(json_path, 'r') as f:
        data = json.load(f)

    original_count = len(data['objects'])
    data['objects'] = [obj for obj in data['objects'] if obj['id'] != object_id]

    if len(data['objects']) == original_count:
        print(f"❌ 物体 '{object_id}' 未找到")
        return False

    # 保存
    output_path = output_path or json_path.replace('.json', '_modified.json')
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"✓ 已移除物体 '{object_id}'")
    print(f"  保存到: {output_path}")
    return True


def move_object(
    json_path: str,
    object_id: str,
    new_x: float,
    new_y: float,
    new_z: float = None,
    output_path: str = None
):
    """移动物体到新位置"""
    with open(json_path, 'r') as f:
        data = json.load(f)

    found = False
    for obj in data['objects']:
        if obj['id'] == object_id:
            obj['position']['x'] = new_x
            obj['position']['y'] = new_y
            if new_z is not None:
                obj['position']['z'] = new_z
            found = True
            break

    if not found:
        print(f"❌ 物体 '{object_id}' 未找到")
        return False

    # 保存
    output_path = output_path or json_path.replace('.json', '_modified.json')
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"✓ 已移动物体 '{object_id}'")
    print(f"  新位置: ({new_x}, {new_y}, {new_z})")
    print(f"  保存到: {output_path}")
    return True


def list_objects(json_path: str):
    """列出场景中的所有物体"""
    with open(json_path, 'r') as f:
        data = json.load(f)

    print(f"\n场景中的物体 ({len(data['objects'])} 个):")
    print("=" * 80)

    for i, obj in enumerate(data['objects'], 1):
        pos = obj['position']
        rot = obj.get('rotation', {'x': 0, 'y': 0, 'z': 0})
        print(f"{i:3d}. {obj['id']}")
        print(f"     位置: ({pos['x']:.2f}, {pos['y']:.2f}, {pos.get('z', 0):.2f})")
        print(f"     旋转: ({rot.get('x', 0):.1f}, {rot.get('y', 0):.1f}, {rot.get('z', 0):.1f})")
        print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法:")
        print("  # 列出所有物体")
        print("  python modify_scene_json.py <scene.json> --list")
        print()
        print("  # 移除物体")
        print("  python modify_scene_json.py <scene.json> --remove <object_id> [--output output.json]")
        print()
        print("  # 移动物体")
        print("  python modify_scene_json.py <scene.json> --move <object_id> <x> <y> [z] [--output output.json]")
        print()
        print("示例:")
        print("  python modify_scene_json.py room.json --list")
        print("  python modify_scene_json.py room.json --remove FumeHood")
        print("  python modify_scene_json.py room.json --move Chair 3.0 4.0")
        print("  python modify_scene_json.py room.json --move Chair 3.0 4.0 0.5 --output room_new.json")
        sys.exit(1)

    json_path = sys.argv[1]

    # 列出物体
    if '--list' in sys.argv:
        list_objects(json_path)

    # 移除物体
    elif '--remove' in sys.argv:
        idx = sys.argv.index('--remove')
        if idx + 1 >= len(sys.argv):
            print("❌ 错误: --remove 需要指定物体 ID")
            sys.exit(1)

        object_id = sys.argv[idx + 1]
        output = None
        if '--output' in sys.argv:
            output_idx = sys.argv.index('--output')
            if output_idx + 1 < len(sys.argv):
                output = sys.argv[output_idx + 1]

        remove_object(json_path, object_id, output)

    # 移动物体
    elif '--move' in sys.argv:
        idx = sys.argv.index('--move')
        if idx + 3 >= len(sys.argv):
            print("❌ 错误: --move 需要指定物体 ID 和新位置 (x y [z])")
            sys.exit(1)

        object_id = sys.argv[idx + 1]
        x = float(sys.argv[idx + 2])
        y = float(sys.argv[idx + 3])

        z = None
        if idx + 4 < len(sys.argv) and not sys.argv[idx + 4].startswith('--'):
            z = float(sys.argv[idx + 4])

        output = None
        if '--output' in sys.argv:
            output_idx = sys.argv.index('--output')
            if output_idx + 1 < len(sys.argv):
                output = sys.argv[output_idx + 1]

        move_object(json_path, object_id, x, y, z, output)

    else:
        print("❌ 错误: 请指定操作 (--list, --remove, 或 --move)")
        sys.exit(1)
