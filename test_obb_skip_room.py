#!/usr/bin/env python3
"""
快速测试：验证OBB碰撞分析器跳过房间功能
"""

import sys
sys.path.insert(0, '/home/pjlab/fbh/LabUtopia')

from collision_analyzer_v4 import OBBCollisionAnalyzer

# 场景文件路径
scene_file = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101/Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_room_isaacsim.json"
assets_file = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview/assets_annotated.json"

print("="*70)
print("测试 OBB 碰撞分析器 - 跳过房间功能")
print("="*70)

# 初始化分析器
analyzer = OBBCollisionAnalyzer(scene_file, assets_file, default_inflation=0.3)

if analyzer.is_loaded:
    print(f"\n✓ 场景加载成功！")
    print(f"  加载物体数量: {len(analyzer.objects)}")

    # 列出前5个物体
    print(f"\n  前5个物体:")
    for i, obj in enumerate(analyzer.objects[:5]):
        print(f"    {i+1}. {obj['id']}")
        print(f"       中心: ({obj['center'][0]:.2f}, {obj['center'][1]:.2f})")
        print(f"       尺寸: {obj['long_edge']:.2f}m × {obj['short_edge']:.2f}m")
        print(f"       旋转: {obj['angle_deg']:.1f}°")

    # 检查是否包含LaboratoryRoom
    has_room = any(obj['id'] == 'LaboratoryRoom' for obj in analyzer.objects)
    if has_room:
        print(f"\n❌ 错误：仍然包含 LaboratoryRoom")
    else:
        print(f"\n✅ 正确：已跳过 LaboratoryRoom")

    # 测试碰撞检测
    print(f"\n" + "="*70)
    print("测试碰撞检测")
    print("="*70)

    test_points = [
        (2.5, 4.5),  # ExperimentalPlatform 附近
        (5.3, 7.7),  # GloveBox 附近
        (4.3, 4.5),  # 房间中心附近
    ]

    for i, (x, y) in enumerate(test_points):
        hits = analyzer.get_collisions_at_point(x, y, robot_radius=0.3)
        print(f"\n测试点 {i+1}: ({x:.2f}, {y:.2f})")
        print(f"  碰撞物体: {len(hits)} 个")
        for hit in hits[:3]:  # 只显示前3个
            print(f"    - {hit}")

    print(f"\n" + "="*70)
    print("测试完成！")
    print("="*70)

else:
    print("❌ 场景加载失败")
