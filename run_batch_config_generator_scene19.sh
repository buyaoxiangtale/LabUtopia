#!/bin/bash
# 批量配置文件生成脚本 - 使用 gemini_scene_19 目录
# 此脚本会将 gemini_scene_19 中的所有 USD 场景生成对应的配置文件
# 相机配置：Camera_03

# 设置路径变量
USD_LIB_DIR="/home/pjlab/fbh/LabUtopia/gemini_scene_19"
NAV_TEMPLATE="config/navigation/navigation_assets_fbh2.yaml"
MAIN_TEMPLATE="config/level5_Navigation_parquet_1_20.yaml"
OUTPUT_DIR="generated_configs_scene19"

# 运行批量配置生成器
python3 batch_config_generator.py \
    --usd-lib "$USD_LIB_DIR" \
    --nav-template "$NAV_TEMPLATE" \
    --main-template "$MAIN_TEMPLATE" \
    --output-dir "$OUTPUT_DIR" \
    --prefix "batch_" \
    --max-episodes 100 \
    --max-steps 4000 \
    --force-single-zero-goal

echo ""
echo "================================================================"
echo "✅ 配置文件生成完成！"
echo "📷 相机配置: Camera_03"
echo "⚠️  所有goal_pairs已强制设置为 [0, 0, 0]"
echo ""
echo "📁 输出目录："
echo "  • navigation配置: $OUTPUT_DIR/navigation/"
echo "  • 主配置: $OUTPUT_DIR/main/"
echo "================================================================"
