# 批量路径规划脚本修改说明

## 📋 脚本对比

| 项目 | 原脚本 | 新脚本 |
|------|--------|--------|
| 文件名 | `run_batch_path_planning.sh` | `run_batch_path_planning_gemini_flash.sh` |
| 场景目录 | `roomlayout/layout_new` | `gemini-3-flash-preview` |
| Goal Pairs 目录 | `outputs/1_5_auto_batch/goal_pairs` | `outputs/gemini_flash_nav_targets/goal_pairs` |
| 输出目录 | `outputs/path_planning_batch_results_1_13` | `outputs/path_planning_batch_results_gemini_flash` |
| 默认 offset_radius | 0.4 米 | 0.3 米（与 YAML 配置一致） |

---

## 🔧 核心修改点详解

### 修改点 1: 场景目录路径 (第 28 行)

**原代码:**
```bash
for scene_dir in "$BASE_DIR/roomlayout/layout_new"/*/; do
```

**新代码:**
```bash
SCENES_BASE_DIR="$BASE_DIR/gemini-3-flash-preview"
# ...
for scene_dir in "$SCENES_BASE_DIR"/*/; do
```

**说明:**
- 将场景遍历目录从 `roomlayout/layout_new` 改为 `gemini-3-flash-preview`
- `gemini-3-flash-preview` 是一个符号链接，实际指向 `/media/pjlab/.../gemini-3-flash-preview`
- 该目录下包含 30 个场景子目录，如 `Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101/`

---

### 修改点 2: Goal Pairs 目录路径 (第 30-31 行)

**原代码:**
```bash
goal_pairs="$BASE_DIR/outputs/1_5_auto_batch/goal_pairs/${scene_name}_goal_pairs.yaml"
```

**新代码:**
```bash
GOAL_PAIRS_BASE_DIR="$BASE_DIR/outputs/gemini_flash_nav_targets/goal_pairs"
# ...
goal_pairs="$GOAL_PAIRS_BASE_DIR/${scene_name}_goal_pairs.yaml"
```

**说明:**
- 将 goal_pairs 查找目录从 `outputs/1_5_auto_batch/goal_pairs` 改为 `outputs/gemini_flash_nav_targets/goal_pairs`
- 新目录包含对应 gemini-3-flash-preview 场景的 goal_pairs yaml 文件
- 文件命名格式: `{场景名称}_goal_pairs.yaml`

**示例:**
```
场景名: Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101
Goal Pairs 文件: outputs/gemini_flash_nav_targets/goal_pairs/Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101_goal_pairs.yaml
```

---

### 修改点 3: 改进的 Barrier 图片查找逻辑 (第 46-58 行)

**原代码:**
```bash
barrier_img=$(find "$scene_dir" -maxdepth 1 -name "*.png" -type f | head -1)
```

**新代码:**
```bash
# 策略1: 优先使用场景目录中的 PNG 文件（大于 1KB）
barrier_img=$(find "$scene_dir" -maxdepth 1 -name "*.png" -type f -size +1k | head -1)

# 策略2: 如果场景目录没有合适的 barrier 图片，使用通用图片
if [ -z "$barrier_img" ]; then
    if [ -f "$DEFAULT_BARRIER_IMAGE" ]; then
        barrier_img="$DEFAULT_BARRIER_IMAGE"
        echo "  ⚠ 场景目录无 barrier 图片，使用通用图片: $(basename "$DEFAULT_BARRIER_IMAGE")"
    else
        echo "  ✗ 场景目录无 barrier 图片，且通用图片不存在"
    fi
fi
```

**说明:**
1. **添加文件大小过滤**: `-size +1k` 确保只选择大于 1KB 的 PNG 文件（避免选择太小的占位图）
2. **回退策略**: 如果场景目录没有 barrier 图片，自动使用通用的 `roomlayout/scene_barrier.png`
3. **友好提示**: 打印使用哪个 barrier 图片的信息

**原因:**
- `gemini-3-flash-preview` 场景目录中可能有小的占位 PNG（如 `Alky.png` 仅 654 字节）
- 这些小图片不是真正的 barrier 地图，需要过滤掉

---

### 修改点 4: 统一使用变量配置 (第 28-33 行)

**原代码:**
```bash
# 硬编码路径
for scene_dir in "$BASE_DIR/roomlayout/layout_new"/*/; do
  # ...
  goal_pairs="$BASE_DIR/outputs/1_5_auto_batch/goal_pairs/${scene_name}_goal_pairs.yaml"
```

**新代码:**
```bash
# 集中定义路径变量
SCENES_BASE_DIR="$BASE_DIR/gemini-3-flash-preview"
GOAL_PAIRS_BASE_DIR="$BASE_DIR/outputs/gemini_flash_nav_targets/goal_pairs"
DEFAULT_BARRIER_IMAGE="$BASE_DIR/roomlayout/scene_barrier.png"

# 使用变量
for scene_dir in "$SCENES_BASE_DIR"/*/; do
  # ...
  goal_pairs="$GOAL_PAIRS_BASE_DIR/${scene_name}_goal_pairs.yaml"
```

**说明:**
- 将所有路径配置集中定义在脚本开头，便于维护和修改
- 提高代码可读性和可维护性

---

## 📊 输出目录结构

运行脚本后，输出目录结构如下：

```
outputs/path_planning_batch_results_gemini_flash/
└── run_2026-01-13_HH-MM-SS/                    # 时间戳目录
    ├── Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101/
    │   ├── Alkylation..._all_results.json      # 所有结果（成功+失败）
    │   ├── Alkylation..._failed_tasks.json     # 失败任务详情
    │   ├── Alkylation..._nav_config.yaml       # 导航配置（仅成功）
    │   ├── Alkylation..._waypoints.json        # 路径点（仅成功）
    │   ├── Alkylation..._pair_000_path.png     # 路径可视化
    │   ├── Alkylation..._pair_001_path.png
    │   └── ...
    ├── Basic_Methanolysis_of_an_Acetate_Ester_20260112_202101/
    │   └── ...
    └── ... (其他 28 个场景)
```

---

## 🚀 使用方法

### 方法 1: 直接运行（使用默认参数）

```bash
cd /home/pjlab/fbh/LabUtopia
bash run_batch_path_planning_gemini_flash.sh
```

### 方法 2: 自定义 offset_radius

```bash
# 使用 0.5 米的障碍物膨胀半径
OFFSET_RADIUS=0.5 bash run_batch_path_planning_gemini_flash.sh
```

### 方法 3: 测试单个场景

```bash
# 手动运行单个场景测试
cd /home/pjlab/fbh/LabUtopia

PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 utils/path_planner_with_goal_pairs.py \
  --barrier_image roomlayout/scene_barrier.png \
  --goal_pairs_yaml outputs/gemini_flash_nav_targets/goal_pairs/Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101_goal_pairs.yaml \
  --x_bounds 0 10 \
  --y_bounds 0 10 \
  --offset_radius 0.3 \
  --output_dir outputs/test_single_scene
```

---

## 📈 预期输出示例

```
========================================
批量路径规划 - Gemini Flash 3 Preview
开始时间: 2026-01-13 15:30:00
输出目录: /home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-13_15-30-00
障碍物膨胀半径: 0.3 米
场景目录: /home/pjlab/fbh/LabUtopia/gemini-3-flash-preview
Goal Pairs 目录: /home/pjlab/fbh/LabUtopia/outputs/gemini_flash_nav_targets/goal_pairs
========================================

========================================
Processing: Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101
Barrier image: scene_barrier.png
Goal pairs: Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101_goal_pairs.yaml
========================================
✓ Alkylation..._pair_000: 成功 - 距离 3.45m, 路径点 45个
✓ Alkylation..._pair_001: 成功 - 距离 2.87m, 路径点 38个
✓ Alkylation..._pair_002: 成功 - 距离 4.12m, 路径点 52个
✓ Success (耗时: 12s)

========================================
Processing: Basic_Methanolysis_of_an_Acetate_Ester_20260112_202101
...
```

---

## 🔍 故障排查

### 问题 1: "No barrier image found"

**原因**: 场景目录没有 PNG 文件，且通用 barrier 图片不存在

**解决**:
```bash
# 检查通用 barrier 图片是否存在
ls -lh /home/pjlab/fbh/LabUtopia/roomlayout/scene_barrier.png

# 如果不存在，需要创建或从其他地方复制
```

### 问题 2: "No goal_pairs file"

**原因**: goal_pairs yaml 文件不存在或命名不匹配

**解决**:
```bash
# 列出所有 goal_pairs 文件
ls /home/pjlab/fbh/LabUtopia/outputs/gemini_flash_nav_targets/goal_pairs/

# 检查场景名是否匹配
# 场景名必须与 goal_pairs 文件名的前缀完全一致
```

### 问题 3: 路径规划失败率高

**原因**: offset_radius 设置过大，导致路径被阻塞

**解决**:
```bash
# 尝试减小 offset_radius
OFFSET_RADIUS=0.2 bash run_batch_path_planning_gemini_flash.sh
```

---

## 📝 修改建议

如果需要进一步修改脚本，主要关注以下配置变量（第 28-33 行）：

```bash
SCENES_BASE_DIR="$BASE_DIR/gemini-3-flash-preview"          # 场景目录
GOAL_PAIRS_BASE_DIR="$BASE_DIR/outputs/gemini_flash_nav_targets/goal_pairs"  # goal_pairs 目录
DEFAULT_BARRIER_IMAGE="$BASE_DIR/roomlayout/scene_barrier.png"  # 默认 barrier 图片
OFFSET_RADIUS=${OFFSET_RADIUS:-0.3}                          # 障碍物膨胀半径
```

---

## ✅ 修改总结

| 修改项 | 原值 | 新值 | 影响 |
|--------|------|------|------|
| 场景源 | `roomlayout/layout_new` | `gemini-3-flash-preview` | 处理新的 30 个 Gemini Flash 场景 |
| Goal Pairs | `outputs/1_5_auto_batch/...` | `outputs/gemini_flash_nav_targets/...` | 使用对应的 goal_pairs 配置 |
| 输出目录 | `path_planning_batch_results_1_13` | `path_planning_batch_results_gemini_flash` | 结果分开存储，避免混淆 |
| 默认半径 | 0.4 米 | 0.3 米 | 与 YAML 配置保持一致 |
| Barrier 查找 | 简单查找 | 智能回退策略 | 更健壮，自动使用通用图片 |

---

## 🎯 预期结果

运行成功后，将处理约 **30 个场景**，每个场景包含多个起终点对。最终输出包括：
- ✅ 所有场景的路径规划结果
- ✅ 成功率统计报告
- ✅ 路径可视化图片
- ✅ 可用于导航的 waypoints 文件
- ✅ 详细的失败分析

预计总耗时: **5-15 分钟**（取决于场景数量和复杂度）
