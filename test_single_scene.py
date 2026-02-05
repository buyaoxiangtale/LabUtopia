#!/usr/bin/env python3
"""
单场景导航点生成测试脚本
用于验证 auto_batch_scene_nav.py 的功能
"""

import json
import sys
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))

from utils.auto_batch_scene_nav import (
    SceneConfig,
    MultiSceneNavGenerator,
    convert_nav_targets_to_goal_pairs,
    deduplicate_goal_pairs
)


def test_single_scene(
    scene_dir: str,
    output_file: str = None,
    offset_radius: float = 0.3,
    verbose: bool = True
):
    """
    测试单个场景的导航点生成

    Args:
        scene_dir: 场景目录路径
        output_file: 输出文件路径（可选）
        offset_radius: 机器人半径（米）
        verbose: 是否显示详细输出
    """
    scene_path = Path(scene_dir)

    if not scene_path.exists():
        print(f"❌ 错误: 场景目录不存在: {scene_dir}")
        return

    print(f"\n{'='*80}")
    print(f"单场景导航点生成测试")
    print(f"{'='*80}")
    print(f"场景目录: {scene_path}")
    print(f"机器人半径: {offset_radius}米")
    print(f"{'='*80}\n")

    # 1. 查找共享资产库
    asset_lib_path = scene_path.parent / "assets_annotated.json"

    if not asset_lib_path.exists():
        print(f"❌ 错误: 找不到资产库文件: {asset_lib_path}")
        return

    print(f"✓ 找到资产库: {asset_lib_path.name}\n")

    # 2. 查找场景文件
    room_files = list(scene_path.glob("*_room_isaacsim.json"))
    protocol_files = list(scene_path.glob("protocol*.json"))

    if not room_files:
        print(f"❌ 错误: 找不到 *_room_isaacsim.json 文件")
        return

    if not protocol_files:
        print(f"❌ 错误: 找不到 protocol*.json 文件")
        return

    room_file = room_files[0]
    protocol_file = protocol_files[0]

    print(f"✓ 房间文件: {room_file.name}")
    print(f"✓ 协议文件: {protocol_file.name}\n")

    # 3. 创建场景配置
    scene_config = SceneConfig(
        scene_name=scene_path.name,
        scene_dir=scene_path,
        room_assets_file=room_file.name,
        protocol_file=protocol_file.name,
        asset_lib_file=asset_lib_path,
        offset_radius=offset_radius
    )

    # 4. 生成导航点
    generator = MultiSceneNavGenerator([scene_config], verbose=verbose)
    nav_targets = generator.process_single_scene(scene_config)

    if not nav_targets:
        print(f"\n❌ 错误: 未能生成任何导航目标点")
        return

    # 5. 转换为 goal_pairs 格式
    goal_pairs = convert_nav_targets_to_goal_pairs(nav_targets)
    deduplicated_pairs = deduplicate_goal_pairs(goal_pairs)

    print(f"\n{'='*80}")
    print(f"结果统计")
    print(f"{'='*80}")
    print(f"导航目标点数量: {len(nav_targets)}")
    print(f"Goal Pairs 数量 (原始): {len(goal_pairs)}")
    print(f"Goal Pairs 数量 (去重后): {len(deduplicated_pairs)}")
    print(f"{'='*80}\n")

    # 6. 显示前3个导航点详情
    print(f"前3个导航点详情:")
    print(f"{'-'*80}")
    for i, nt in enumerate(nav_targets[:3], 1):
        print(f"\n导航点 {i}:")
        print(f"  步骤: {nt.step_number}")
        print(f"  位置: {nt.location}")
        print(f"  匹配对象: {nt.matched_object}")
        print(f"  目标坐标: ({nt.target_x:.3f}, {nt.target_y:.3f})")
        print(f"  对象中心: ({nt.object_center_x:.2f}, {nt.object_center_y:.2f})")
        print(f"  对象尺寸: {nt.bbox_width:.2f}m x {nt.bbox_depth:.2f}m")
        print(f"  旋转角度: {nt.rotation_z}°")
        print(f"  偏移量: dx={nt.offset_dx:.3f}, dy={nt.offset_dy:.3f}")

    # 7. 显示前3个 goal_pairs
    if deduplicated_pairs:
        print(f"\n\n前3个 Goal Pairs:")
        print(f"{'-'*80}")
        for i, gp in enumerate(deduplicated_pairs[:3], 1):
            start = gp['start']
            end = gp['end']
            print(f"\nGoal Pair {i}:")
            print(f"  起点: ({start[0]:.3f}, {start[1]:.3f}, θ={start[2]:.2f} rad)")
            print(f"  终点: ({end[0]:.3f}, {end[1]:.3f}, θ={end[2]:.2f} rad)")

    # 8. 保存结果（如果指定了输出文件）
    if output_file:
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        result = {
            "scene_name": scene_path.name,
            "config": {
                "scene_dir": str(scene_path),
                "room_assets_file": room_file.name,
                "protocol_file": protocol_file.name,
                "asset_lib_file": str(asset_lib_path),
                "offset_radius": offset_radius
            },
            "nav_targets": [
                {
                    "step_number": nt.step_number,
                    "location": nt.location,
                    "matched_object": nt.matched_object,
                    "target_x": nt.target_x,
                    "target_y": nt.target_y,
                    "offset_dx": nt.offset_dx,
                    "offset_dy": nt.offset_dy,
                    "bbox_width": nt.bbox_width,
                    "bbox_depth": nt.bbox_depth,
                    "object_center_x": nt.object_center_x,
                    "object_center_y": nt.object_center_y,
                    "rotation_z": nt.rotation_z
                }
                for nt in nav_targets
            ],
            "goal_pairs": deduplicated_pairs,
            "num_targets": len(nav_targets),
            "num_goal_pairs": len(deduplicated_pairs)
        }

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)

        print(f"\n✓ 结果已保存到: {output_path}")

    print(f"\n{'='*80}")
    print(f"✓ 测试完成！")
    print(f"{'='*80}\n")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='单场景导航点生成测试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 测试单个场景（显示输出，不保存文件）
  python3 test_single_scene.py /path/to/scene_dir

  # 测试并保存结果
  python3 test_single_scene.py /path/to/scene_dir --output test_output.json

  # 指定自定义机器人半径
  python3 test_single_scene.py /path/to/scene_dir --radius 0.5

  # 使用默认测试场景
  python3 test_single_scene.py
        """
    )

    parser.add_argument('scene_dir', nargs='?',
                       default='/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/Basic_Methanolysis_of_an_Acetate_Ester_20260112_202101',
                       help='场景目录路径（默认: Basic_Methanolysis 场景）')
    parser.add_argument('--output', '-o', type=str,
                       help='输出文件路径（JSON 格式）')
    parser.add_argument('--radius', '-r', type=float, default=0.3,
                       help='机器人半径（米），默认: 0.3米')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='静默模式，减少输出')

    args = parser.parse_args()

    test_single_scene(
        scene_dir=args.scene_dir,
        output_file=args.output,
        offset_radius=args.radius,
        verbose=not args.quiet
    )
