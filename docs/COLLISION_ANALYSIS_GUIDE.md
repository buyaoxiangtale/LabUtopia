# 路径规划失败分析和修复指南

## 📋 问题总结

根据对 `Boc_Deprotection_of_Piperazine_Derivative` 场景的分析，发现以下碰撞问题：

### 失败任务 1: pair_000
- **起点**: (2.8, 7.325)
- **终点**: (4.3, 2.333)
- **碰撞物体**:
  - `ReagentCabinet` (试剂柜) - 位置: (2.80, 8.52)
  - `ExperimentalPlatform` (实验平台) - 位置: (4.30, 4.00)

### 失败任务 2: pair_001
- **起点**: (4.3, 2.333)
- **终点**: (6.5, 6.44)
- **碰撞物体**:
  - `FumeHood` (通风橱) - 位置: (6.50, 8.24)
  - `ExperimentalPlatform` (实验平台) - 位置: (4.30, 4.00)

### 失败任务 3: pair_003
- **起点**: (6.485, 3.8)
- **终点**: (2.628, 3.5)
- **碰撞物体**:
  - `Chair` (椅子) - 位置: (3.90, 2.80)
  - `RotaryEvaporator` (旋转蒸发仪) - 位置: (7.93, 3.80)

---

## 🛠️ 工具使用

### 1. 碰撞分析工具 (`collision_analyzer.py`)

#### 基本用法

```bash
python utils/collision_analyzer.py \
  <场景JSON> \
  <资产库JSON> \
  <失败任务JSON> \
  [选项]
```

#### 选项说明

| 选项 | 说明 | 示例 |
|------|------|------|
| `--visualize <path>` | 生成可视化图像 | `--visualize output.png` |
| `--offset-radius <r>` | 设置障碍物膨胀半径 | `--offset-radius 0.3` |
| `--list` | 列出所有物体 | （通过 modify_scene_json.py） |

#### 示例

```bash
# 分析失败任务并生成可视化
python utils/collision_analyzer.py \
  roomlayout/layout_new/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/Boc_Deprotection_of_Piperazine_Derivative__Model___room_isaacsim.json \
  roomlayout/layout_new/assets_annotated.json \
  outputs/path_planning_batch_results/run_2026-01-06_18-19-51/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725_failed_tasks.json \
  --visualize outputs/collision_analysis.png \
  --offset-radius 0.3
```

---

### 2. 场景修改工具 (`modify_scene_json.py`)

#### 列出所有物体

```bash
python utils/modify_scene_json.py \
  roomlayout/layout_new/.../room.json \
  --list
```

#### 移除物体

```bash
python utils/modify_scene_json.py \
  roomlayout/layout_new/.../room.json \
  --remove <物体ID> \
  [--output 输出文件.json]
```

**示例** - 移除椅子（pair_003 的碰撞物体）：

```bash
python utils/modify_scene_json.py \
  roomlayout/layout_new/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/Boc_Deprotection_of_Piperazine_Derivative__Model___room_isaacsim.json \
  --remove Chair \
  --output roomlayout/layout_new/Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725/Boc_Deprotection_of_Piperazine_Derivative__Model___room_isaacsim_modified.json
```

#### 移动物体

```bash
python utils/modify_scene_json.py \
  roomlayout/layout_new/.../room.json \
  --move <物体ID> <x> <y> [z] \
  [--output 输出文件.json]
```

**示例** - 移动椅子避免碰撞：

```bash
# 将椅子从 (3.90, 2.80) 移到 (5.0, 1.5)
python utils/modify_scene_json.py \
  roomlayout/layout_new/.../room.json \
  --move Chair 5.0 1.5 \
  --output room_modified.json
```

---

## 💡 解决方案建议

### 方案 1: 调整障碍物膨胀半径

**问题**: 0.3米 的膨胀半径可能过大，导致过多碰撞

**解决**: 减小膨胀半径

```yaml
# 修改配置文件
offset_radius: 0.2  # 从 0.3 改为 0.2
```

**重新运行路径规划**:

```bash
python3 -m utils.path_planner_with_goal_pairs \
  --barrier_image "roomlayout/.../barrier.png" \
  --goal_pairs_yaml "outputs/.../goal_pairs.yaml" \
  --offset_radius 0.2 \  # 使用更小的半径
  --x_bounds 0 9 \
  --y_bounds 0 9
```

---

### 方案 2: 移除小障碍物

**问题**: 椅子等小物体阻碍路径

**解决**: 移除或移动物体

```bash
# 移除所有椅子
python utils/modify_scene_json.py room.json --remove Chair \
  --output room_no_chairs.json
```

---

### 方案 3: 调整起终点位置

**问题**: 起点或终点直接在障碍物上

**解决**: 微调位置

```bash
# 将起点从 (2.8, 7.325) 移到 (2.8, 7.0)
python utils/modify_scene_json.py room.json \
  --move <物体ID> 2.8 7.0 \
  --output room_adjusted.json
```

---

### 方案 4: 使用修改后的场景文件

**步骤**:

1. **分析碰撞**:
   ```bash
   python utils/collision_analyzer.py \
     scene.json assets.json failed_tasks.json \
     --visualize collision.png
   ```

2. **修改场景**（移除/移动物体）:
   ```bash
   python utils/modify_scene_json.py scene.json \
     --remove Chair \
     --output scene_fixed.json
   ```

3. **重新生成障碍物地图**（如果需要）

4. **重新规划路径**:
   ```bash
   python3 utils/path_planner_with_goal_pairs.py \
     --barrier_image "new_barrier.png" \
     --goal_pairs_yaml "goal_pairs.yaml" \
     --x_bounds 0 9 \
     --y_bounds 0 9
   ```

---

## 📊 输出说明

### 碰撞分析输出

```
任务: Boc_Deprotection_of_Piperazine_Derivative__Model___20251229_003725_pair_003
  起点: (6.485, 3.8)
  终点: (2.628, 3.5)
  失败原因: 路径被障碍物完全阻隔
  ✗ 碰撞物体:
    - Chair                    # ← 碰撞物体
      位置: (3.90, 2.80, 0.00)
      类型: furniture
    - RotaryEvaporator         # ← 碰撞物体
      位置: (7.93, 3.80, 0.00)
      类型: evaporation_equipment
```

### 可视化图像

生成的 `collision_analysis.png` 包含：
- **左图**: 场景布局
  - 蓝色虚线框: 原始物体边界
  - 红色实线框: 膨胀后的障碍物区域
  - 绿色圆圈: 起点
  - 红色圆圈: 终点
  - ✗ 标记: 碰撞物体

- **右图**: 文本分析结果
  - 每个任务的碰撞物体列表
  - 失败原因

---

## 🔧 高级用法

### 批量处理多个场景

```bash
# 创建批量分析脚本
for scene_dir in roomlayout/layout_new/*/; do
  scene_name=$(basename "$scene_dir")
  scene_json="${scene_dir}/${scene_name}_room_isaacsim.json"
  failed_tasks="outputs/path_planning_batch_results/run_*/${scene_name}/*_failed_tasks.json"

  if [ -f "$scene_json" ] && [ -n "$failed_tasks" ]; then
    echo "分析: $scene_name"
    python utils/collision_analyzer.py \
      "$scene_json" \
      roomlayout/layout_new/assets_annotated.json \
      $failed_tasks \
      --visualize "outputs/collision_${scene_name}.png"
  fi
done
```

### 调整膨胀半径重新分析

```bash
# 尝试不同的膨胀半径
for radius in 0.1 0.2 0.3 0.4; do
  echo "测试膨胀半径: $radius"
  python utils/collision_analyzer.py \
    scene.json assets.json failed_tasks.json \
    --offset-radius $radius
done
```

---

## 📝 注意事项

1. **备份原始文件**: 修改场景 JSON 前，先备份原始文件

   ```bash
   cp room.json room_backup.json
   ```

2. **验证修改**: 修改后重新运行路径规划验证效果

3. **USD 文件更新**: 如果修改了场景 JSON，可能需要重新生成 USD 文件

4. **膨胀半径权衡**:
   - 太小: 机器人可能与物体碰撞
   - 太大: 路径被过多阻挡
   - 建议: 根据机器人实际尺寸调整

---

## 🎯 快速参考

### 常用命令

```bash
# 1. 分析失败任务
python utils/collision_analyzer.py \
  <场景JSON> assets.json <失败任务JSON> \
  --visualize collision.png

# 2. 列出物体
python utils/modify_scene_json.py <场景JSON> --list

# 3. 移除物体
python utils/modify_scene_json.py <场景JSON> --remove <物体ID>

# 4. 移动物体
python utils/modify_scene_json.py <场景JSON> --move <物体ID> <x> <y>
```

### 物体 ID 参考

根据分析，常见碰撞物体：
- `Chair` - 椅子
- `ReagentCabinet` - 试剂柜
- `FumeHood` - 通风橱
- `ExperimentalPlatform` - 实验平台
- `RotaryEvaporator` - 旋转蒸发仪

---

需要帮助？查看完整文档或运行 `--help` 选项。
