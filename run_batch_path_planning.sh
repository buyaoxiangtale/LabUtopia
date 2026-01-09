#!/bin/bash

BASE_DIR="/home/pjlab/fbh/LabUtopia"
BASE_OUTPUT_DIR="$BASE_DIR/outputs/path_planning_batch_results"

# 可配置参数：障碍物膨胀半径（米）
# 可以根据需要修改这个值
OFFSET_RADIUS=${OFFSET_RADIUS:-0.4}  # 默认 2.0 米，可通过环境变量覆盖

# 记录脚本开始时间
SCRIPT_START_TIME=$(date +%s)
# 创建带时间戳的输出目录（易读格式）
TIMESTAMP=$(date '+%Y-%m-%d_%H-%M-%S')
OUTPUT_DIR="$BASE_OUTPUT_DIR/run_$TIMESTAMP"

# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"

echo "========================================"
echo "开始批量路径规划"
echo "开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "输出目录: $OUTPUT_DIR"
echo "障碍物膨胀半径: ${OFFSET_RADIUS} 米"
echo "========================================"
echo ""

# 统计变量
TOTAL_SCENES=0
SUCCESS_SCENES=0
PARTIAL_FAILED_SCENES=0
FAILED_SCENES=0

# 路径规划任务统计（所有场景累计）
TOTAL_TASKS=0
SUCCESSFUL_TASKS=0

for scene_dir in "$BASE_DIR/roomlayout/layout_new"/*/; do
  scene_name=$(basename "$scene_dir")
  
  # 查找 PNG 文件（更灵活）
  barrier_img=$(find "$scene_dir" -maxdepth 1 -name "*.png" -type f | head -1)
  
  goal_pairs="$BASE_DIR/outputs/1_5_auto_batch/goal_pairs/${scene_name}_goal_pairs.yaml"
  
  # 检查文件存在性
  if [ -n "$barrier_img" ] && [ -f "$goal_pairs" ]; then
    TOTAL_SCENES=$((TOTAL_SCENES + 1))

    echo "========================================"
    echo "Processing: $scene_name"
    echo "Barrier image: $(basename "$barrier_img")"
    echo "========================================"

    # 记录场景开始时间
    SCENE_START_TIME=$(date +%s)

    PYTHONPATH=$BASE_DIR python3 "$BASE_DIR/utils/path_planner_with_goal_pairs.py" \
      --barrier_image "$barrier_img" \
      --goal_pairs_yaml "$goal_pairs" \
      --x_bounds 0 9 \
      --y_bounds 0 9 \
      --offset_radius "$OFFSET_RADIUS" \
      --output_dir "$OUTPUT_DIR/$scene_name"

    EXIT_CODE=$?

    # 计算场景耗时
    SCENE_END_TIME=$(date +%s)
    SCENE_DURATION=$((SCENE_END_TIME - SCENE_START_TIME))

    # 检查是否有失败任务详情文件
    FAILED_FILE="$OUTPUT_DIR/$scene_name/${scene_name}_failed_tasks.json"
    PARTIAL_FAILURE=""

    if [ -f "$FAILED_FILE" ]; then
      # 使用 Python 提取失败任务数量（避免 jq 依赖）
      FAILED_COUNT=$(PYTHONPATH=$BASE_DIR python3 -c "import json; data = json.load(open('$FAILED_FILE')); print(len(data))")
      PARTIAL_FAILURE=" [部分失败: $FAILED_COUNT 个任务]"
    fi

    # 统计该场景的路径规划任务
    ALL_RESULTS_FILE="$OUTPUT_DIR/$scene_name/${scene_name}_all_results.json"
    if [ -f "$ALL_RESULTS_FILE" ]; then
      # 使用 Python 提取任务统计信息
      SCENE_STATS=$(PYTHONPATH=$BASE_DIR python3 -c "
import json
try:
    with open('$ALL_RESULTS_FILE', 'r') as f:
        all_results = json.load(f)
    total = len(all_results)
    successful = sum(1 for r in all_results if r.get('is_success', False))
    print(f\"{total} {successful}\")
except:
    print(\"0 0\")
")
      SCENE_TOTAL=$(echo "$SCENE_STATS" | awk '{print $1}')
      SCENE_SUCCESS=$(echo "$SCENE_STATS" | awk '{print $2}')
      TOTAL_TASKS=$((TOTAL_TASKS + SCENE_TOTAL))
      SUCCESSFUL_TASKS=$((SUCCESSFUL_TASKS + SCENE_SUCCESS))
    fi

    if [ $EXIT_CODE -eq 0 ]; then
      if [ -z "$PARTIAL_FAILURE" ]; then
        SUCCESS_SCENES=$((SUCCESS_SCENES + 1))
        echo "✓ Success (耗时: ${SCENE_DURATION}s)"
      else
        # 部分失败的情况（退出码0但有failed_tasks文件）
        PARTIAL_FAILED_SCENES=$((PARTIAL_FAILED_SCENES + 1))
        echo "⚠ Partial Success${PARTIAL_FAILURE} (耗时: ${SCENE_DURATION}s)"
      fi
    else
      # 完全失败的情况（脚本崩溃或严重错误）
      FAILED_SCENES=$((FAILED_SCENES + 1))
      echo "✗ Failed (耗时: ${SCENE_DURATION}s)"
    fi
    echo ""
  else
    echo "⚠ Skipping $scene_name (missing files)"
    [ -z "$barrier_img" ] && echo "  - No PNG file found"
    [ ! -f "$goal_pairs" ] && echo "  - No goal_pairs file"
  fi
done

# 计算脚本总耗时
SCRIPT_END_TIME=$(date +%s)
TOTAL_DURATION=$((SCRIPT_END_TIME - SCRIPT_START_TIME))

# 格式化总耗时 (小时:分钟:秒)
HOURS=$((TOTAL_DURATION / 3600))
MINUTES=$(((TOTAL_DURATION % 3600) / 60))
SECONDS=$((TOTAL_DURATION % 60))

if [ $HOURS -gt 0 ]; then
  DURATION_FORMATTED="${HOURS}h ${MINUTES}m ${SECONDS}s"
elif [ $MINUTES -gt 0 ]; then
  DURATION_FORMATTED="${MINUTES}m ${SECONDS}s"
else
  DURATION_FORMATTED="${SECONDS}s"
fi

echo "========================================"
echo "批量处理完成！"
echo "结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "总耗时: $DURATION_FORMATTED"
echo ""
echo "统计信息:"
echo "  处理场景: $TOTAL_SCENES"
echo "  完全成功: $SUCCESS_SCENES"
echo "  部分失败: $PARTIAL_FAILED_SCENES"
echo "  完全失败: $FAILED_SCENES"
if [ $TOTAL_SCENES -gt 0 ]; then
  # 计算完全成功率（所有任务都成功）
  FULL_SUCCESS_RATE=$(awk "BEGIN {printf \"%.1f\", ($SUCCESS_SCENES * 100.0) / $TOTAL_SCENES}")
  echo "  完全成功率: ${FULL_SUCCESS_RATE}%"

  # 如果有部分失败的场景，额外显示
  if [ $PARTIAL_FAILED_SCENES -gt 0 ]; then
    PARTIAL_SUCCESS_RATE=$(awk "BEGIN {printf \"%.1f\", (($SUCCESS_SCENES + $PARTIAL_FAILED_SCENES) * 100.0) / $TOTAL_SCENES}")
    echo "  部分成功率 (含部分失败): ${PARTIAL_SUCCESS_RATE}%"
  fi
fi

echo ""
echo "路径规划任务统计:"
echo "  总任务数: $TOTAL_TASKS"
echo "  成功任务数: $SUCCESSFUL_TASKS"
if [ $TOTAL_TASKS -gt 0 ]; then
  # 计算全部路径生成的成功率
  PATH_SUCCESS_RATE=$(awk "BEGIN {printf \"%.1f\", ($SUCCESSFUL_TASKS * 100.0) / $TOTAL_TASKS}")
  echo "  全部路径生成成功率: ${PATH_SUCCESS_RATE}%"
fi
echo ""
echo "Results saved to: $OUTPUT_DIR"
echo "========================================"