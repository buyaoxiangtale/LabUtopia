#!/usr/bin/env python3
"""
测试配置生成器 - 仅扫描USD库，不生成文件
"""

from batch_config_generator import ConfigGenerator

# 创建生成器实例
generator = ConfigGenerator(
    usd_library_dir='gemini_3_flash_preview_1_16',
    navigation_template_path='config/navigation/navigation_assets_fbh2.yaml',
    main_config_template_path='config/level5_Navigation_parquet.yaml',
    output_dir='generated_configs'
)

# 扫描场景
scenes = generator.scan_usd_scenes()

print(f"\n找到 {len(scenes)} 个场景:\n")
for i, scene in enumerate(scenes[:5], 1):  # 只显示前5个
    print(f"{i}. {scene['name']}")
    print(f"   目录: {scene['scene_dir']}")
    print(f"   USD文件: {scene['usd_filename']}")
    print(f"   相对路径: {scene['usd_path']}")
    print(f"   Barrier图像: {scene.get('barrier_image_path', '未找到')}\n")

if len(scenes) > 5:
    print(f"... 还有 {len(scenes) - 5} 个场景\n")
