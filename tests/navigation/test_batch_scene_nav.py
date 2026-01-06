#!/usr/bin/env python3
"""
多场景导航点批量生成 - 测试脚本

演示如何使用 batch_scene_nav_targets.py 进行多场景处理
"""

import sys
sys.path.append('/home/pjlab/fbh/LabUtopia')

from roomlayout.batch_scene_nav_targets import (
    MultiSceneNavGenerator,
    SceneConfig
)
from pathlib import Path


def test_single_scene():
    """测试1: 处理单个场景（12_17）"""
    print("\n" + "="*80)
    print("测试1: 处理单个场景（12_17）")
    print("="*80 + "\n")

    scene = SceneConfig(
        scene_name="12_17_alkylation",
        scene_dir=Path("/home/pjlab/fbh/LabUtopia/roomlayout/12_17"),
        room_assets_file="Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json",
        protocol_file="protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json",
        asset_lib_file="assets_annotated.json",
        offset_radius=0.6
    )

    generator = MultiSceneNavGenerator([scene], verbose=True)
    results = generator.process_all_scenes()
    generator.save_results(Path("outputs/batch_nav_test_single"))

    print("\n" + "="*80)
    print("测试1完成！")
    print("="*80)


def test_multiple_scenes():
    """测试2: 处理多个场景"""
    print("\n" + "="*80)
    print("测试2: 处理多个场景")
    print("="*80 + "\n")

    scenes = [
        SceneConfig(
            scene_name="12_17_alkylation",
            scene_dir=Path("/home/pjlab/fbh/LabUtopia/roomlayout/12_17"),
            room_assets_file="Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json",
            protocol_file="protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json",
            asset_lib_file="assets_annotated.json",
            offset_radius=0.6
        ),
        SceneConfig(
            scene_name="sodium_acetate_preparation",
            scene_dir=Path("/home/pjlab/fbh/LabUtopia/roomlayout"),
            room_assets_file="Preparation_of_Sodium_Acetate,_Crystallization,_and_Simple_Distillation_of_Residual_Liquor_room_isaacsim.json",
            protocol_file="protocol_Preparation_of_Sodium_Acetate,_20251111_165059.json",
            asset_lib_file="12_17/assets_annotated.json",
            offset_radius=0.6
        ),
    ]

    generator = MultiSceneNavGenerator(scenes, verbose=True)
    results = generator.process_all_scenes()
    generator.save_results(Path("outputs/batch_nav_test_multiple"))

    print("\n" + "="*80)
    print("测试2完成！")
    print("="*80)


def test_from_config_file():
    """测试3: 从配置文件加载"""
    print("\n" + "="*80)
    print("测试3: 从配置文件加载")
    print("="*80 + "\n")

    import json

    config_file = Path("config/batch_scenes_config_example.json")

    if not config_file.exists():
        print(f"❌ 配置文件不存在: {config_file}")
        return

    with open(config_file, "r") as f:
        config_data = json.load(f)

    scenes = []
    for scene_cfg in config_data.get("scenes", []):
        scene = SceneConfig(
            scene_name=scene_cfg["scene_name"],
            scene_dir=Path(scene_cfg["scene_dir"]),
            room_assets_file=scene_cfg["room_assets_file"],
            protocol_file=scene_cfg["protocol_file"],
            asset_lib_file=scene_cfg.get("asset_lib_file", "assets_annotated.json"),
            offset_radius=scene_cfg.get("offset_radius", 0.6)
        )
        scenes.append(scene)

    generator = MultiSceneNavGenerator(scenes, verbose=True)
    results = generator.process_all_scenes()

    output_dir = Path(config_data.get("output_dir", "outputs/batch_nav_from_config"))
    generator.save_results(output_dir)

    print("\n" + "="*80)
    print("测试3完成！")
    print("="*80)


def test_quick_demo():
    """测试4: 快速演示（简化输出）"""
    print("\n" + "="*80)
    print("测试4: 快速演示（简化输出）")
    print("="*80 + "\n")

    scene = SceneConfig(
        scene_name="12_17_demo",
        scene_dir=Path("/home/pjlab/fbh/LabUtopia/roomlayout/12_17"),
        room_assets_file="Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json",
        protocol_file="protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json",
        asset_lib_file="assets_annotated.json",
        offset_radius=0.6
    )

    # 使用 verbose=False 减少输出
    generator = MultiSceneNavGenerator([scene], verbose=False)
    results = generator.process_all_scenes()
    generator.save_results(Path("outputs/batch_nav_test_quick"))

    # 显示简短统计
    scene_name = scene.scene_name
    num_targets = results[scene_name]["num_targets"]
    print(f"\n✓ 场景 {scene_name}: 生成 {num_targets} 个导航点")


if __name__ == "__main__":
    print("\n" + "="*80)
    print("多场景导航点批量生成 - 测试套件")
    print("="*80)

    if len(sys.argv) > 1:
        test_num = int(sys.argv[1])
    else:
        print("\n请选择要运行的测试:")
        print("1. 单个场景（12_17）")
        print("2. 多个场景")
        print("3. 从配置文件加载")
        print("4. 快速演示（简化输出）")
        print("5. 运行所有测试")
        print("\n默认运行测试1...\n")
        test_num = 1

    if test_num == 1:
        test_single_scene()
    elif test_num == 2:
        test_multiple_scenes()
    elif test_num == 3:
        test_from_config_file()
    elif test_num == 4:
        test_quick_demo()
    elif test_num == 5:
        test_single_scene()
        test_multiple_scenes()
        test_from_config_file()
        test_quick_demo()
    else:
        print(f"未知的测试编号: {test_num}")

    print("\n" + "="*80)
    print("所有测试完成！")
    print("="*80 + "\n")
