#!/bin/bash
# 批量配置文件生成示例脚本
# 支持强制所有goal_pairs为[0,0,0]

# 设置路径变量
USD_LIB_DIR="gemini_3_flash_preview_1_16"
NAV_TEMPLATE="config/navigation/navigation_assets_fbh2.yaml"
MAIN_TEMPLATE="config/level5_Navigation_parquet.yaml"
OUTPUT_DIR="generated_configs"

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

echo "配置文件生成完成！"
echo "⚠️  所有goal_pairs已强制设置为 [0, 0, 0]"
echo "navigation配置位置: $OUTPUT_DIR/navigation/"
echo "主配置位置: $OUTPUT_DIR/main/"
