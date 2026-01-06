# 路径规划工具 - 完整解决方案总结

## ✅ 问题已解决

针对你的需求：**从 `goal_pairs.yaml` 和障碍物图像进行批量路径规划**，我已经创建了一个专门的工具并成功运行。

---

## 📝 需求回顾

**你的输入文件：**
1. 障碍物图像：`/home/pjlab/fbh/LabUtopia/roomlayout/layout_new/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726/Alkylation.png`
2. 路径点配置：`/home/pjlab/fbh/LabUtopia/outputs/auto_batch_nav_targets/goal_pairs/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_goal_pairs.yaml`

**你需要的功能：**
- ✅ 使用 A* 算法在障碍物地图上规划路径
- ✅ 批量处理多个起终点对
- ✅ 生成包含角度信息的 waypoints
- ✅ 可视化路径结果

---

## 🎯 解决方案

### 新创建的工具

**文件位置**: `utils/path_planner_with_goal_pairs.py`

**核心功能**:
1. 从 `goal_pairs.yaml` 读取起终点对
2. 使用障碍物图像（omap）进行 A* 路径规划
3. 自动生成包含角度的 waypoints
4. 保存多种格式的结果（JSON, YAML, 可视化图像）

---

## 🚀 使用方法

### 快速开始

```bash
PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 utils/path_planner_with_goal_pairs.py \
  --barrier_image "/home/pjlab/fbh/LabUtopia/roomlayout/layout_new/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726/Alkylation.png" \
  --goal_pairs_yaml "/home/pjlab/fbh/LabUtopia/outputs/auto_batch_nav_targets/goal_pairs/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_goal_pairs.yaml" \
  --x_bounds 0 9 \
  --y_bounds 0 9 \
  --output_dir "outputs/path_planning_results/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro"
```

### 运行结果

```
✓ 加载障碍物地图: Alkylation.png
  尺寸: 179 x 179 像素
  X边界: [0.0, 9.0]
  Y边界: [0.0, 9.0]
  分辨率: 0.0503 米/像素
✓ 使用配置中的offset_radius: 0.6

======================================================================
开始批量路径规划
起终点对数量: 3
======================================================================

✓ Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_pair_000: 成功 - 距离 4.17m, 路径点 84个
✓ Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_pair_001: 成功 - 距离 2.16m, 路径点 44个
✗ Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_pair_002: 路径规划失败

======================================================================
规划完成
成功: 2/3 (66.7%)
失败: 1/3 (33.3%)
======================================================================

✓ 结果已保存
✓ Waypoints已保存
✓ 导航配置已保存
```

---

## 📦 生成的文件

### 1. 完整规划结果
**文件**: `Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_planning_results.json`

**内容**:
```json
[
  {
    "task_id": "场景名_pair_000",
    "start": [2.2, 6.81],
    "end": [5.5, 5.96],
    "waypoints": [
      [2.19, 6.81, 0.0],    // x, y, theta
      [2.24, 6.81, 0.0],
      ...
    ],
    "total_distance": 4.17,   // 米
    "num_waypoints": 84
  },
  ...
]
```

### 2. Waypoints数据
**文件**: `Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_waypoints.json`

**内容**: 简化的 waypoints 列表，便于后续使用

### 3. 导航任务配置
**文件**: `Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_nav_config.yaml`

**用途**: 可直接用于 `NavigationTaskNew`

### 4. 可视化图像
**文件**: `*_pair_000_path.png`, `*_pair_001_path.png`

**内容**:
- 黑色 = 障碍物
- 白色 = 可行走区域
- 红色线 = 规划的路径
- 绿色点 = 起点
- 蓝色点 = 终点

---

## 📊 成功率分析

### 成功的路径 (2/3)

1. **路径1**: 起点 [2.2, 6.81] → 终点 [5.5, 5.96]
   - ✅ 成功
   - 距离: 4.17米
   - 路径点: 84个

2. **路径2**: 起点 [5.5, 5.96] → 终点 [6.185, 4.5]
   - ✅ 成功
   - 距离: 2.16米
   - 路径点: 44个

### 失败的路径 (1/3)

3. **路径3**: 起点 [6.185, 4.5] → 终点 [5.207, 1.801]
   - ❌ 失败 - 可能原因：
     - 起点或终点在障碍物上
     - offset_radius (0.6米) 太大，堵塞了通道
     - 路径被障碍物完全阻挡

**解决方案**: 尝试减小 offset_radius
```bash
--offset_radius 0.3  # 从 0.6 减小到 0.3
```

---

## 🔧 完整工作流程

### 步骤1: 准备输入文件

确保你有以下文件：
```
roomlayout/layout_new/[场景名]/
  └── [场景名].png          ← 障碍物图像

outputs/auto_batch_nav_targets/goal_pairs/
  └── [场景名]_goal_pairs.yaml  ← 起终点配置
```

### 步骤2: 确定场景边界

**方法1**: 从配置文件读取
```bash
cat config/navigation/navigation_assets_fbh.yaml
# 输出: x_bounds: [0, 9], y_bounds: [0, 9]
```

**方法2**: 从goal_pairs推断
```bash
# 如果坐标大约在 0-9 范围内，使用 [0, 9]
```

**方法3**: 自动估算（不推荐）
```bash
# 不指定 --x_bounds 和 --y_bounds，脚本自动估算
```

### 步骤3: 运行路径规划

```bash
PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 utils/path_planner_with_goal_pairs.py \
  --barrier_image "障碍物图像路径" \
  --goal_pairs_yaml "goal_pairs.yaml路径" \
  --x_bounds 0 9 \
  --y_bounds 0 9 \
  --output_dir "输出目录"
```

### 步骤4: 验证结果

1. **查看可视化图像**
   ```bash
   ls -la outputs/path_planning_results/*_path.png
   # 用图像查看器打开，检查路径是否合理
   ```

2. **检查waypoints**
   ```bash
   cat outputs/path_planning_results/*_waypoints.json | python3 -m json.tool | less
   ```

3. **查看统计报告**
   - 脚本会自动打印成功率、平均距离等统计信息

### 步骤5: 使用结果

#### 用于导航任务

生成的 `nav_config.yaml` 可以直接使用：

```python
# 在你的任务配置中
task:
  navigation_config_path: "outputs/path_planning_results/.../nav_config.yaml"
```

或在代码中加载：

```python
import yaml

with open("nav_config.yaml") as f:
    nav_config = yaml.safe_load(f)

task.set_goal_pairs(nav_config['goal_pairs'])
```

---

## 💡 高级用法

### Python代码调用

```python
from utils.path_planner_with_goal_pairs import ScenePathPlanner

# 创建规划器
planner = ScenePathPlanner(
    barrier_image_path="path/to/Alkylation.png",
    x_bounds=[0, 9],
    y_bounds=[0, 9],
    offset_radius=0.6
)

# 执行规划
results = planner.plan_from_goal_pairs_yaml(
    yaml_path="path/to/goal_pairs.yaml",
    visualize=True,
    output_dir="outputs/my_results"
)

# 处理结果
for result in results:
    waypoints = result['waypoints']
    # 使用waypoints进行导航...
```

### 批量处理多个场景

创建批处理脚本 `batch_plan_all_scenes.sh`:

```bash
#!/bin/bash

BASE_DIR="/home/pjlab/fbh/LabUtopia"
OUTPUT_DIR="$BASE_DIR/outputs/path_planning_batch_results"

for scene_dir in "$BASE_DIR/roomlayout/layout_new"/*/; do
  scene_name=$(basename "$scene_dir")
  barrier_img="${scene_dir}${scene_name}.png"
  goal_pairs="$BASE_DIR/outputs/auto_batch_nav_targets/goal_pairs/${scene_name}_goal_pairs.yaml"

  if [ -f "$barrier_img" ] && [ -f "$goal_pairs" ]; then
    echo "========================================"
    echo "Processing: $scene_name"
    echo "========================================"

    PYTHONPATH=$BASE_DIR python3 "$BASE_DIR/utils/path_planner_with_goal_pairs.py" \
      --barrier_image "$barrier_img" \
      --goal_pairs_yaml "$goal_pairs" \
      --x_bounds 0 9 \
      --y_bounds 0 9 \
      --output_dir "$OUTPUT_DIR/$scene_name"

    echo ""
  fi
done

echo "========================================"
echo "Batch processing complete!"
echo "Results saved to: $OUTPUT_DIR"
echo "========================================"
```

运行：
```bash
chmod +x batch_plan_all_scenes.sh
./batch_plan_all_scenes.sh
```

---

## 📚 相关文档

1. **[PATH_PLANNING_USAGE_GUIDE.md](PATH_PLANNING_USAGE_GUIDE.md)** - 详细使用说明
2. **[PROJECT_STRUCTURE.md](../PROJECT_STRUCTURE.md)** - 项目结构
3. **[API_REFERENCE.md](../API_REFERENCE.md)** - API参考
4. **[utils/a_star.py](../utils/a_star.py)** - A*算法实现

---

## 🎓 核心概念回顾

### A*算法与Omap

你的路径规划基于以下核心组件：

1. **Omap (Occupancy Map)**: `Alkylation.png`
   - 白色像素 = 可行走 (0)
   - 黑色像素 = 障碍物 (1)

2. **障碍物膨胀**: `inflate_obstacles()`
   - 机器人有体积，不能贴着障碍物走
   - 膨胀半径 = 机器人安全半径 (0.6米)

3. **A*搜索**: `astar()`
   - 使用曼哈顿距离启发式
   - 4方向移动（上下左右）
   - 找到从起点到终点的最短路径

4. **路径生成**: `plan_navigation_path()`
   - 网格坐标 → 真实坐标转换
   - 添加角度信息（计算朝向）
   - 计算路径总距离

### 成功判断

任务成功通过以下三个层次判断：

1. **路径规划阶段** (运行前)
   - ✅ 起点和终点不在障碍物上
   - ✅ A*能找到有效路径

2. **路径跟踪阶段** (运行中)
   - ✅ 到达当前路径点 (距离 < 0.08米)
   - ✅ 切换到下一个路径点

3. **任务完成阶段** (最终判断)
   - ✅ 到达最后路径点
   - ✅ 线速度接近0
   - ✅ 角速度 < 0.1 rad/s
   - ✅ 角度误差 < 0.1 rad

---

## ✨ 总结

你现在有了完整的路径规划解决方案：

✅ **新工具**: `path_planner_with_goal_pairs.py` - 专门处理你的场景
✅ **成功运行**: 已在你的 Alkylation 场景上测试
✅ **详细文档**: 完整的使用说明和示例
✅ **多种输出**: JSON、YAML、可视化图像

**下一步**:
1. 使用生成的 `nav_config.yaml` 运行导航任务
2. 如果有路径规划失败，调整 `offset_radius` 参数
3. 对其他场景使用相同的流程

祝你使用顺利！🚀
