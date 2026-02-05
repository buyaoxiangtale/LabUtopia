#!/bin/bash

# ==============================================================================
# 配置测试脚本 - 在运行完整批量规划前使用
# ==============================================================================

BASE_DIR="/home/pjlab/fbh/LabUtopia"
SCENES_BASE_DIR="$BASE_DIR/gemini-3-flash-preview"
GOAL_PAIRS_BASE_DIR="$BASE_DIR/outputs/gemini_flash_nav_targets/goal_pairs"
DEFAULT_BARRIER_IMAGE="$BASE_DIR/roomlayout/scene_barrier.png"

echo "========================================"
echo "配置检查 - Gemini Flash 3 Preview"
echo "========================================"
echo ""

# 检查场景目录
echo "1. 检查场景目录..."
if [ -d "$SCENES_BASE_DIR" ]; then
    SCENE_COUNT=$(find "$SCENES_BASE_DIR" -maxdepth 1 -type d | wc -l)
    echo "   ✓ 场景目录存在: $SCENES_BASE_DIR"
    echo "   ✓ 场景数量: $((SCENE_COUNT - 1)) 个"
else
    echo "   ✗ 场景目录不存在: $SCENES_BASE_DIR"
    exit 1
fi
echo ""

# 检查 goal_pairs 目录
echo "2. 检查 Goal Pairs 目录..."
if [ -d "$GOAL_PAIRS_BASE_DIR" ]; then
    YAML_COUNT=$(find "$GOAL_PAIRS_BASE_DIR" -name "*_goal_pairs.yaml" | wc -l)
    echo "   ✓ Goal Pairs 目录存在: $GOAL_PAIRS_BASE_DIR"
    echo "   ✓ YAML 文件数量: $YAML_COUNT 个"
else
    echo "   ✗ Goal Pairs 目录不存在: $GOAL_PAIRS_BASE_DIR"
    exit 1
fi
echo ""

# 检查默认 barrier 图片
echo "3. 检查默认 Barrier 图片..."
if [ -f "$DEFAULT_BARRIER_IMAGE" ]; then
    IMG_SIZE=$(du -h "$DEFAULT_BARRIER_IMAGE" | cut -f1)
    echo "   ✓ 默认 Barrier 图片存在: $DEFAULT_BARRIER_IMAGE"
    echo "   ✓ 图片大小: $IMG_SIZE"
else
    echo "   ✗ 默认 Barrier 图片不存在: $DEFAULT_BARRIER_IMAGE"
    echo "   ⚠ 警告: 如果场景目录没有 barrier 图片，将无法进行路径规划"
fi
echo ""

# 统计场景和 goal_pairs 的匹配情况
echo "4. 检查场景和 Goal Pairs 的匹配情况..."
MATCHED=0
MISMATCHED=0

for scene_dir in "$SCENES_BASE_DIR"/*/; do
    scene_name=$(basename "$scene_dir")
    goal_pairs="$GOAL_PAIRS_BASE_DIR/${scene_name}_goal_pairs.yaml"

    if [ -f "$goal_pairs" ]; then
        MATCHED=$((MATCHED + 1))
    else
        MISMATCHED=$((MISMATCHED + 1))
        echo "   ⚠ 缺少 Goal Pairs: $scene_name"
    fi
done

echo "   ✓ 匹配成功: $MATCHED 个场景"
if [ $MISMATCHED -gt 0 ]; then
    echo "   ⚠ 缺少 Goal Pairs: $MISMATCHED 个场景"
fi
echo ""

# 显示前 3 个场景的详细信息
echo "5. 前 3 个场景的详细信息..."
COUNT=0
for scene_dir in "$SCENES_BASE_DIR"/*/; do
    if [ $COUNT -ge 3 ]; then
        break
    fi

    scene_name=$(basename "$scene_dir")
    goal_pairs="$GOAL_PAIRS_BASE_DIR/${scene_name}_goal_pairs.yaml"
    barrier_img=$(find "$scene_dir" -maxdepth 1 -name "*.png" -type f -size +1k | head -1)

    echo "   场景 $((COUNT + 1)): $scene_name"
    echo "     - Goal Pairs: $goal_pairs"
    if [ -f "$goal_pairs" ]; then
        echo "       ✓ 存在"
        # 显示起终点对数量
        PAIR_COUNT=$(grep -c "^- start:" "$goal_pairs" 2>/dev/null || echo "0")
        echo "       ✓ 起终点对数量: $PAIR_COUNT"
    else
        echo "       ✗ 不存在"
    fi
    echo "     - Barrier 图片:"
    if [ -n "$barrier_img" ]; then
        echo "       ✓ 场景内: $(basename "$barrier_img")"
    elif [ -f "$DEFAULT_BARRIER_IMAGE" ]; then
        echo "       ⚠ 将使用默认: $(basename "$DEFAULT_BARRIER_IMAGE")"
    else
        echo "       ✗ 无可用图片"
    fi
    echo ""

    COUNT=$((COUNT + 1))
done

echo "========================================"
echo "配置检查完成！"
echo ""

# 生成总结和建议
if [ $MISMATCHED -eq 0 ] && [ -f "$DEFAULT_BARRIER_IMAGE" ]; then
    echo "✅ 配置检查通过！可以运行批量路径规划："
    echo ""
    echo "   bash run_batch_path_planning_gemini_flash.sh"
    echo ""
    echo "或自定义参数："
    echo "   OFFSET_RADIUS=0.3 bash run_batch_path_planning_gemini_flash.sh"
else
    echo "⚠️  发现配置问题，请解决后再运行批量路径规划"
fi
echo "========================================"
