#!/bin/bash
# 将 gemini_3_flash_preview_1_16 中所有子文件夹的 .png 文件移动到 gemini_scene_19 对应的子文件夹

# 源目录和目标目录
SOURCE_DIR="/home/pjlab/fbh/LabUtopia/gemini_3_flash_preview_1_16"
TARGET_DIR="/home/pjlab/fbh/LabUtopia/gemini_scene_19"

# 检查源目录是否存在
if [ ! -d "$SOURCE_DIR" ]; then
    echo "❌ 错误：源目录不存在: $SOURCE_DIR"
    exit 1
fi

# 创建目标目录（如果不存在）
mkdir -p "$TARGET_DIR"

# 计数器
total_files=0
moved_files=0
failed_files=0

echo "🚀 开始移动 .png 文件..."
echo "📁 源目录: $SOURCE_DIR"
echo "📁 目标目录: $TARGET_DIR"
echo "================================================================"
echo ""

# 遍历源目录的所有子文件夹
for subdir in "$SOURCE_DIR"/*; do
    # 检查是否是目录
    if [ -d "$subdir" ]; then
        # 获取子文件夹名称
        dirname=$(basename "$subdir")

        # 在目标目录创建对应的子文件夹
        target_subdir="$TARGET_DIR/$dirname"
        mkdir -p "$target_subdir"

        echo "📂 处理文件夹: $dirname"

        # 查找并移动该子文件夹中的所有 .png 文件
        # 使用 find 命令递归查找（包括嵌套子文件夹）
        while IFS= read -r -d '' png_file; do
            total_files=$((total_files + 1))

            # 获取文件名（不包含路径）
            filename=$(basename "$png_file")

            # 如果文件在子子文件夹中，保留相对路径结构
            relative_path="${png_file#$subdir/}"
            target_file="$target_subdir/$filename"

            # 移动文件
            if mv "$png_file" "$target_file" 2>/dev/null; then
                moved_files=$((moved_files + 1))
                echo "  ✅ 移动: $filename"
            else
                failed_files=$((failed_files + 1))
                echo "  ❌ 失败: $filename"
            fi
        done < <(find "$subdir" -type f -name "*.png" -print0)

        echo ""
    fi
done

echo "================================================================"
echo "📊 移动完成！统计信息："
echo "  • 总共找到: $total_files 个 .png 文件"
echo "  • 成功移动: $moved_files 个文件"
echo "  • 移动失败: $failed_files 个文件"
echo ""
echo "📁 目标目录: $TARGET_DIR"
