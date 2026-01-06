#!/bin/bash

BASE_DIR="/home/pjlab/fbh/LabUtopia"
OUTPUT_DIR="$BASE_DIR/outputs/path_planning_batch_results"

for scene_dir in "$BASE_DIR/roomlayout/layout_new"/*/; do
  scene_name=$(basename "$scene_dir")
  
  # 查找 PNG 文件（更灵活）
  barrier_img=$(find "$scene_dir" -maxdepth 1 -name "*.png" -type f | head -1)
  
  goal_pairs="$BASE_DIR/outputs/auto_batch_nav_targets/goal_pairs/${scene_name}_goal_pairs.yaml"
  
  # 检查文件存在性
  if [ -n "$barrier_img" ] && [ -f "$goal_pairs" ]; then
    echo "========================================"
    echo "Processing: $scene_name"
    echo "Barrier image: $(basename "$barrier_img")"
    echo "========================================"
    
    PYTHONPATH=$BASE_DIR python3 "$BASE_DIR/utils/path_planner_with_goal_pairs.py" \
      --barrier_image "$barrier_img" \
      --goal_pairs_yaml "$goal_pairs" \
      --x_bounds 0 9 \
      --y_bounds 0 9 \
      --output_dir "$OUTPUT_DIR/$scene_name"
    
    if [ $? -eq 0 ]; then
      echo "✓ Success"
    else
      echo "✗ Failed"
    fi
    echo ""
  else
    echo "⚠ Skipping $scene_name (missing files)"
    [ -z "$barrier_img" ] && echo "  - No PNG file found"
    [ ! -f "$goal_pairs" ] && echo "  - No goal_pairs file"
  fi
done

echo "========================================"
echo "Batch processing complete!"
echo "Results saved to: $OUTPUT_DIR"
echo "========================================"