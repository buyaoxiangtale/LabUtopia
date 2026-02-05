#!/bin/bash
# 批量重命名点云文件脚本
# 将所有 episode_*_with_trajectory.ply 重命名为 path.ply

# 目标目录
BASE_DIR="/home/pjlab/fbh/LabUtopia/outputs/collect/2026.01.20/17.18.42_level5_Navigation_Chlorination_of_Triazolyl-Benzyl_Alcohol_Protocol"

# 计数器
total_files=0
renamed=0
failed=0
skipped=0

echo "🚀 开始批量重命名点云文件..."
echo "📁 目标目录: $BASE_DIR"
echo "================================================================"
echo ""

# 查找所有 .ply 文件并处理
while IFS= read -r -d '' ply_file; do
    total_files=$((total_files + 1))

    # 获取文件所在目录
    file_dir=$(dirname "$ply_file")

    # 目标文件路径
    target_file="$file_dir/path.ply"

    # 检查目标文件是否已存在
    if [ -f "$target_file" ]; then
        echo "⚠️  [$(printf "%03d" $total_files)] 跳过: 目标文件已存在"
        echo "    目录: $file_dir"
        skipped=$((skipped + 1))
        continue
    fi

    # 重命名文件
    if mv "$ply_file" "$target_file" 2>/dev/null; then
        echo "✅ [$(printf "%03d" $total_files)] 重命名成功"
        echo "    源文件: $(basename "$ply_file")"
        echo "    目标文件: path.ply"
        renamed=$((renamed + 1))
    else
        echo "❌ [$(printf "%03d" $total_files)] 重命名失败"
        echo "    文件: $ply_file"
        failed=$((failed + 1))
    fi

    echo ""

done < <(find "$BASE_DIR" -type f -name "*_with_trajectory.ply" -print0)

echo "================================================================"
echo "📊 重命名完成！统计信息："
echo "  • 总共找到: $total_files 个 .ply 文件"
echo "  • 成功重命名: $renamed 个文件"
echo "  • 跳过: $skipped 个文件（目标已存在）"
echo "  • 失败: $failed 个文件"
echo "================================================================"
