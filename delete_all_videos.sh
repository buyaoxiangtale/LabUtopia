#!/bin/bash
# 删除所有视频文件脚本
# 删除 observation.video.depth 和 observation.video.rgb 目录中的视频

# 基础目录
BASE_DIR="/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.20/18.32.23_level5_Navigation_Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop"

# 计数器
total_dirs=0
total_files=0
deleted_files=0
failed_deletes=0
empty_dirs=0

echo "🗑️  开始删除视频文件..."
echo "📁 目标目录: $BASE_DIR"
echo "🎯 删除目标:"
echo "   • observation.video.depth/*.mp4 (深度视频)"
echo "   • observation.video.rgb/*.mp4 (RGB视频)"
echo "================================================================"
echo ""

# 查找并删除所有视频目录中的 .mp4 文件
while IFS= read -r -d '' video_file; do
    total_files=$((total_files + 1))
    filename=$(basename "$video_file")
    dirpath=$(dirname "$video_file")
    dirname=$(basename "$dirpath")

    echo "📹 删除文件 [$total_files]:"
    echo "   类型: $dirname"
    echo "   文件: $filename"

    # 删除文件
    if rm "$video_file" 2>/dev/null; then
        echo "   ✅ 删除成功"
        deleted_files=$((deleted_files + 1))
    else
        echo "   ❌ 删除失败"
        failed_deletes=$((failed_deletes + 1))
    fi

    echo ""

done < <(find "$BASE_DIR" -type d \( -name "observation.video.depth" -o -name "observation.video.rgb" \) -print0 | \
       while IFS= read -r -d '' dir; do find "$dir" -maxdepth 1 -type f -name "*.mp4" -print0; done)

# 统计目录数
depth_dirs=$(find "$BASE_DIR" -type d -name "observation.video.depth" | wc -l)
rgb_dirs=$(find "$BASE_DIR" -type d -name "observation.video.rgb" | wc -l)
total_dirs=$((depth_dirs + rgb_dirs))

echo "================================================================"
echo "📊 删除完成！统计信息："
echo "  • 深度视频目录数: $depth_dirs"
echo "  • RGB视频目录数: $rgb_dirs"
echo "  • 总目录数: $total_dirs"
echo "  • 总共找到: $total_files 个视频文件"
echo "  • 成功删除: $deleted_files 个文件"
echo "  • 删除失败: $failed_deletes 个文件"
echo "================================================================"
