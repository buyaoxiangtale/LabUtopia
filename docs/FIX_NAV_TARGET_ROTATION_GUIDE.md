# 导航点旋转角度修复工具使用指南

## 📋 问题说明

### 原始问题
在批量生成的导航目标点文件中，`rotation_z` 字段存储的是**物体的旋转角度**（如 180.0°, 90.0°），而不是**导航点的朝向角度**。

**示例**：
```json
{
  "step_number": 1,
  "location": "ReagentCabinet",
  "target_x": 2.2,
  "target_y": 6.81,
  "object_center_x": 2.2,
  "object_center_y": 8.305,
  "rotation_z": 180.0  // ❌ 这是物体的旋转角度，不是导航点朝向
}
```

### 修复后的结果
```json
{
  "step_number": 1,
  "location": "ReagentCabinet",
  "target_x": 2.2,
  "target_y": 6.81,
  "object_center_x": 2.2,
  "object_center_y": 8.305,
  "rotation_z": 180.0,  // 保留原始物体旋转角度
  "navigation_theta_deg": 0.0,  // ✅ 新增：导航点朝向（度）
  "navigation_theta_rad": 0.0   // ✅ 新增：导航点朝向（弧度）
}
```

---

## 🚀 快速开始

### 1. 修复单个 JSON 文件

```bash
# 将导航点朝向固定为 0 度
python utils/fix_nav_target_rotation.py \
    --input outputs/auto_batch_nav_targets_test/Alkylation*_nav_targets.json \
    --mode zero
```

### 2. 修复整个目录的所有文件

```bash
# 修复目录中所有 JSON 文件和报告
python utils/fix_nav_target_rotation.py \
    --input outputs/auto_batch_nav_targets_test/ \
    --mode zero
```

### 3. 使用"面向物体"模式

```bash
# 导航点朝向自动计算为面向物体中心的角度
python utils/fix_nav_target_rotation.py \
    --input outputs/auto_batch_nav_targets_test/ \
    --mode face_object
```

---

## 📖 修复模式说明

### 模式 1: `zero` （推荐）
导航点朝向固定为 **0 度**（朝向东方）。

**适用场景**：
- 机器人始终朝向固定方向
- 简化控制逻辑
- 与 `navigation_task_test_weizi.py` 配合使用

**示例输出**：
```
步骤 1: (2.200, 6.810) - ReagentCabinet
  → 修正: (2.200, 6.810, 0.0°)
```

### 模式 2: `face_object`
导航点朝向自动计算为**面向物体中心的角度**。

**适用场景**：
- 机器人需要始终面向目标物体
- 更自然的导航行为

**计算公式**：
```python
# 从导航点指向物体中心的角度
theta = atan2(object_center_y - nav_y, object_center_x - nav_x)
```

**示例输出**：
```
步骤 1: (2.200, 6.810) - ReagentCabinet
  → 修正: (2.200, 6.810, 90.0°)  # 朝向北方（物体在正上方）
```

---

## 📝 输出文件说明

### JSON 文件
- **原始文件**：`*_nav_targets.json`（不修改）
- **修复后文件**：`*_nav_targets_fixed.json`（新增）

**新增字段**：
```json
{
  "navigation_theta_deg": 0.0,    // 导航点朝向（度）
  "navigation_theta_rad": 0.0     // 导航点朝向（弧度）
}
```

### 报告文件
- **原始文件**：`summary_report.txt`（不修改）
- **修复后文件**：`summary_report_fixed.txt`（新增）

**格式变化**：
```
# 原始格式
步骤 1: (2.200, 6.810) - ReagentCabinet

# 修复后格式（保留原行 + 添加修正行）
步骤 1: (2.200, 6.810) - ReagentCabinet
  → 修正: (2.200, 6.810, 0.0°)
```

---

## 🔧 命令行参数

| 参数 | 说明 | 示例 |
|------|------|------|
| `--input` | 输入文件或目录路径（必需） | `--input outputs/test/` |
| `--mode` | 修复模式（可选） | `--mode zero` 或 `--mode face_object` |
| | 默认值：`face_object` | |

---

## 💡 使用建议

### 推荐工作流

1. **先测试单个文件**
   ```bash
   python utils/fix_nav_target_rotation.py \
       --input outputs/auto_batch_nav_targets_test/Alkylation*_nav_targets.json \
       --mode zero
   ```

2. **查看修复结果**
   ```bash
   # 查看生成的 JSON 文件
   cat outputs/auto_batch_nav_targets_test/Alkylation*_nav_targets_fixed.json | jq '.nav_targets[0]'

   # 查看修复后的报告
   head -30 outputs/auto_batch_nav_targets_test/summary_report_fixed.txt
   ```

3. **确认无误后批量处理**
   ```bash
   python utils/fix_nav_target_rotation.py \
       --input outputs/auto_batch_nav_targets_test/ \
       --mode zero
   ```

4. **使用修复后的文件**
   ```bash
   # 将 *_fixed.json 重命名为原始文件（备份原始文件）
   mv *_nav_targets.json *_nav_targets.json.bak
   mv *_nav_targets_fixed.json *_nav_targets.json
   ```

---

## 📊 修复效果对比

### JSON 文件对比

**修复前**：
```json
{
  "location": "ReagentCabinet",
  "target_x": 2.2,
  "target_y": 6.81,
  "object_center_x": 2.2,
  "object_center_y": 8.305,
  "rotation_z": 180.0  // 物体旋转角度
}
```

**修复后（zero 模式）**：
```json
{
  "location": "ReagentCabinet",
  "target_x": 2.2,
  "target_y": 6.81,
  "object_center_x": 2.2,
  "object_center_y": 8.305,
  "rotation_z": 180.0,  // 物体旋转角度（保留）
  "navigation_theta_deg": 0.0,  // ✅ 导航点朝向（新增）
  "navigation_theta_rad": 0.0
}
```

**修复后（face_object 模式）**：
```json
{
  "location": "ReagentCabinet",
  "target_x": 2.2,
  "target_y": 6.81,
  "object_center_x": 2.2,
  "object_center_y": 8.305,
  "rotation_z": 180.0,
  "navigation_theta_deg": 90.0,  // ✅ 导航点朝向物体（90°）
  "navigation_theta_rad": 1.5708
}
```

### 报告文件对比

**修复前**：
```
导航点列表:
  步骤 1: (2.200, 6.810) - ReagentCabinet
  步骤 2: (5.500, 5.960) - FumeHood
```

**修复后**：
```
导航点列表:
  步骤 1: (2.200, 6.810) - ReagentCabinet
    → 修正: (2.200, 6.810, 0.0°)
  步骤 2: (5.500, 5.960) - FumeHood
    → 修正: (5.500, 5.960, 0.0°)
```

---

## ⚠️ 注意事项

1. **备份原始文件**
   - 脚本不会覆盖原始文件
   - 原始文件保留为 `*_nav_targets.json`
   - 修复后文件保存为 `*_nav_targets_fixed.json`

2. **保留原始数据**
   - `rotation_z` 字段（物体旋转角度）始终保留
   - 新增字段：`navigation_theta_deg`, `navigation_theta_rad`

3. **模式选择**
   - `zero`：适用于固定朝向场景
   - `face_object`：适用于需要面向物体的场景

---

## 🔗 相关工具

| 工具 | 用途 |
|------|------|
| `auto_batch_scene_nav.py` | 批量生成导航点（原始工具） |
| `fix_nav_target_rotation.py` | 修复导航点旋转角度（本工具） |
| `navigation_task_test_weizi.py` | 使用修复后导航点的任务类 |

---

## ✅ 成功示例

```bash
$ python utils/fix_nav_target_rotation.py \
    --input outputs/auto_batch_nav_targets_test/ \
    --mode zero

================================================================================
导航点旋转角度修复工具
================================================================================
输入目录: outputs/auto_batch_nav_targets_test
修复模式: zero
  - face_object: 导航点朝向物体中心
  - zero: 固定为 0 度

找到 14 个 JSON 文件

处理文件: Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_nav_targets.json
  ✓ 修复完成: 9 个导航点
  ✓ 保存到: Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_nav_targets_fixed.json

...

处理报告: summary_report.txt
  ✓ 修复完成
  ✓ 保存到: summary_report_fixed.txt

================================================================================
✓ 所有文件处理完成
================================================================================
```

---

## 📞 常见问题

### Q1: 为什么不直接覆盖原始文件？

**A**: 为了安全起见，脚本不会修改原始文件。修复后的文件使用 `_fixed` 后缀。确认无误后，可以手动替换：

```bash
# 备份原始文件
mv *_nav_targets.json *_nav_targets.json.bak

# 使用修复后的文件
mv *_nav_targets_fixed.json *_nav_targets.json
```

### Q2: 如何在代码中使用修复后的数据？

**A**: 使用新增的字段：
```python
import json

with open('path/to/nav_targets_fixed.json') as f:
    data = json.load(f)

for target in data['nav_targets']:
    # 使用导航点朝向（度）
    theta_deg = target['navigation_theta_deg']

    # 使用导航点朝向（弧度）
    theta_rad = target['navigation_theta_rad']

    # 设置机器人初始位姿
    start_pose = [target['target_x'], target['target_y'], theta_rad]
```

### Q3: face_object 模式计算的朝向是什么？

**A**: 是从导航点指向物体中心的角度：
```python
# 计算公式
theta = atan2(object_y - nav_y, object_x - nav_x)
```

例如：
- 导航点：(2.2, 6.81)
- 物体中心：(2.2, 8.305)
- 朝向 = atan2(8.305 - 6.81, 2.2 - 2.2) = atan2(1.495, 0) = 90°（北方）

---

## 🎯 总结

这个修复工具可以：
- ✅ 修复导航点旋转角度错误
- ✅ 保留原始物体旋转角度
- ✅ 支持两种修复模式（zero / face_object）
- ✅ 批量处理 JSON 文件和报告
- ✅ 生成包含朝向信息的格式化输出

推荐使用 `--mode zero` 模式，使导航点朝向固定为 0 度，便于后续控制逻辑处理。
