#!/bin/bash
# 删除所有 observation.video.depth 目录中的深度视频文件
# 目标目录：outputs/collect/2026.01.20/17.18.42_level5_Navigation_Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol

# 基础目录
BASE_DIR="/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.20/17.18.42_level5_Navigation_Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol"

# 计数器
total_dirs=0
total_files=0
deleted_files=0
failed_deletes=0

echo "🗑️  开始删除深度视频文件..."
echo "📁 目标目录: $BASE_DIR"
echo "🎯 删除目标: observation.video.depth/*.mp4"
echo "================================================================"
echo ""

# 查找所有 observation.video.depth 目录
while IFS= read -r -d '' depth_dir; do
    total_dirs=$((total_dirs + 1))

    echo "📂 处理目录 [$total_dirs]:"
    echo "    路径: $depth_dir"

    # 查找该目录下的所有文件
    file_count=0
    while IFS= read -r -d '' video_file; do
        file_count=$((file_count + 1))
        total_files=$((total_files + 1))

        filename=$(basename "$video_file")
        echo "    📹 删除文件: $filename"

        # 删除文件
        if rm "$video_file" 2>/dev/null; then
            echo "      ✅ 删除成功"
            deleted_files=$((deleted_files + 1))
        else
            echo "      ❌ 删除失败"
            failed_deletes=$((failed_deletes + 1))
        fi

    done < <(find "$depth_dir" -maxdepth 1 -type f -print0)

    if [ $file_count -eq 0 ]; then
        echo "    ℹ️  目录为空，无文件需要删除"
    fi

    echo ""

done < <(find "$BASE_DIR" -type d -name "observation.video.depth" -print0)

echo "================================================================"
echo "📊 删除完成！统计信息："
echo "  • 处理目录数: $total_dirs"
echo "  • 总共找到: $total_files 个文件"
echo "  • 成功删除: $deleted_files 个文件"
echo "  • 删除失败: $failed_deletes 个文件"
echo "================================================================"
