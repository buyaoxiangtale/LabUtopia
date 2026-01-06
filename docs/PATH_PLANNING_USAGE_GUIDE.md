# 路径规划工具使用说明

## 📋 概述

`path_planner_with_goal_pairs.py` 是一个专门为你的场景设计的批量路径规划工具，它可以从 `goal_pairs.yaml` 文件和障碍物图像生成完整的导航路径。

---

## 🎯 主要功能

1. ✅ 从 `goal_pairs.yaml` 读取起终点对
2. ✅ 使用 A* 算法在障碍物地图上规划路径
3. ✅ 生成包含角度信息的 waypoints
4. ✅ 自动生成可视化图像
5. ✅ 输出可直接用于导航任务的配置文件

---

## 📂 输入文件

### 1. 障碍物图像
- **文件路径**: `roomlayout/layout_new/[场景名]/[场景名].png`
- **格式**: PNG 图像
- **内容**:
  - 白色像素 = 可行走区域
  - 黑色像素 = 障碍物

### 2. 起终点配置
- **文件路径**: `outputs/auto_batch_nav_targets/goal_pairs/[场景名]_goal_pairs.yaml`
- **格式**:
  ```yaml
  goal_pairs:
  - start: [x, y, z]  # 起点（3D坐标）
    end: [x, y, z]    # 终点（3D坐标）
  - start: [...]
    end: [...]
  metadata:
    scene_name: "场景名称"
    num_goal_pairs: 3
    config:
      offset_radius: 0.6  # 障碍物膨胀半径（米）
  ```

---

## 🚀 使用方法

### 方法1: 命令行使用

```bash
PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 utils/path_planner_with_goal_pairs.py \
  --barrier_image "障碍物图像路径" \
  --goal_pairs_yaml "goal_pairs.yaml路径" \
  --x_bounds 0 9 \
  --y_bounds 0 9 \
  --offset_radius 0.6 \
  --output_dir "输出目录"
```

### 参数说明

| 参数 | 说明 | 必需 | 示例 |
|------|------|------|------|
| `--barrier_image` | 障碍物图像路径 | ✅ | `roomlayout/layout_new/.../Alkylation.png` |
| `--goal_pairs_yaml` | goal_pairs.yaml文件路径 | ✅ | `outputs/.../goal_pairs.yaml` |
| `--x_bounds` | X轴边界（米） | ❌ | `0 9` (默认自动估算) |
| `--y_bounds` | Y轴边界（米） | ❌ | `0 9` (默认自动估算) |
| `--offset_radius` | 障碍物膨胀半径（米） | ❌ | `0.6` (默认从配置读取) |
| `--output_dir` | 输出目录 | ❌ | `outputs/path_planning_results` |
| `--no_visualize` | 不生成可视化图像 | ❌ | (默认生成可视化) |

### 实际运行示例

```bash
# 基础使用
PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 utils/path_planner_with_goal_pairs.py \
  --barrier_image "/home/pjlab/fbh/LabUtopia/roomlayout/layout_new/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726/Alkylation.png" \
  --goal_pairs_yaml "/home/pjlab/fbh/LabUtopia/outputs/auto_batch_nav_targets/goal_pairs/Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726_goal_pairs.yaml" \
  --x_bounds 0 9 \
  --y_bounds 0 9
```

---

## 📦 输出文件

脚本会在输出目录生成以下文件：

### 1. `*_planning_results.json`
完整的规划结果，包含每条路径的详细信息。

```json
[
  {
    "task_id": "场景名_pair_000",
    "start": [2.2, 6.81],
    "end": [5.5, 5.96],
    "waypoints": [
      [x, y, theta],  // 每个路径点包含 x, y 坐标和朝向角度
      [x, y, theta],
      ...
    ],
    "total_distance": 4.17,  // 总路径长度（米）
    "num_waypoints": 84       // 路径点数量
  },
  ...
]
```

### 2. `*_waypoints.json`
仅包含 waypoints 数据的简化版本，用于后续导航任务。

```json
[
  [[x, y, theta], [x, y, theta], ...],  // 路径1的waypoints
  [[x, y, theta], [x, y, theta], ...],  // 路径2的waypoints
  ...
]
```

### 3. `*_nav_config.yaml`
可直接用于导航任务的配置文件。

```yaml
goal_pairs:
  - start: [x, y, z]
    end: [x, y, z]
  - start: [x, y, z]
    end: [x, y, z]
metadata:
  scene_name: "场景名称"
  num_goal_pairs: 2
  barrier_image_path: "..."
  x_bounds: [0, 9]
  y_bounds: [0, 9]
  offset_radius: 0.6
```

### 4. `*_pair_XXX_path.png`
路径可视化图像，显示：
- 障碍物地图（黑色=障碍，白色=可行走）
- 规划的路径（红色线）
- 起点（绿色点）
- 终点（蓝色点）

---

## 📊 运行结果示例

```
✓ 加载障碍物地图: Alkylation.png
  尺寸: 179 x 179 像素
  X边界: [0.0, 9.0]
  Y边界: [0.0, 9.0]
  分辨率: 0.0503 米/像素
✓ 使用配置中的offset_radius: 0.6

======================================================================
开始批量路径规划
场景: Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726
起终点对数量: 3
======================================================================

✓ 场景名_pair_000: 成功 - 距离 4.17m, 路径点 84个
✓ 场景名_pair_001: 成功 - 距离 2.16m, 路径点 44个
✗ 场景名_pair_002: 路径规划失败 - 起点 [6.185, 4.5] → 终点 [5.207, 1.801]

======================================================================
规划完成
成功: 2/3 (66.7%)
失败: 1/3 (33.3%)

路径长度统计:
  平均: 3.17 米
  最短: 2.16 米
  最长: 4.17 米
======================================================================
```

---

## 🔧 方法2: Python代码使用

```python
from utils.path_planner_with_goal_pairs import ScenePathPlanner

# 创建规划器
planner = ScenePathPlanner(
    barrier_image_path="path/to/barrier.png",
    x_bounds=[0, 9],      # 可选，默认自动估算
    y_bounds=[0, 9],      # 可选，默认自动估算
    offset_radius=0.6     # 可选，默认从yaml读取
)

# 执行规划
results = planner.plan_from_goal_pairs_yaml(
    yaml_path="path/to/goal_pairs.yaml",
    visualize=True,                    # 生成可视化
    output_dir="outputs/my_results"    # 输出目录
)

# 访问结果
for result in results:
    print(f"任务ID: {result['task_id']}")
    print(f"起点: {result['start']}")
    print(f"终点: {result['end']}")
    print(f"路径长度: {result['total_distance']:.2f}m")
    print(f"路径点数: {result['num_waypoints']}")
    print(f"Waypoints: {result['waypoints'][:5]}...")  # 前5个点
```

---

## ⚠️ 常见问题

### 1. 路径规划失败

**原因**:
- 起点或终点在障碍物上
- 障碍物膨胀半径（`offset_radius`）太大，堵塞了通道
- 场景边界设置不正确

**解决方案**:
```bash
# 尝试减小 offset_radius
--offset_radius 0.3  # 从 0.6 减小到 0.3

# 检查并调整边界
--x_bounds 0 10
--y_bounds 0 10
```

### 2. 自动估算的边界不准确

如果未指定 `--x_bounds` 和 `--y_bounds`，脚本会自动估算为10米x10米的场景。

**如何确定正确的边界**:
1. 查看场景配置文件中的 `x_bounds` 和 `y_bounds`
2. 从 goal_pairs 中的坐标值推断
3. 查看障碍物图像的宽高比

### 3. 所有路径都规划失败

**检查清单**:
- [ ] 障碍物图像路径是否正确？
- [ ] 图像是否是正确的格式（PNG）？
- [ ] 白色像素是否代表可行走区域？
- [ ] goal_pairs 中的坐标是否在边界范围内？
- [ ] offset_radius 是否合理？

---

## 📐 如何确定场景边界

### 方法1: 从现有配置文件读取

```bash
# 查看导航资产配置
cat config/navigation/navigation_assets_fbh.yaml
```

输出示例：
```yaml
assets:
- x_bounds: [0, 9]
  y_bounds: [0, 9]
```

### 方法2: 从 goal_pairs 推断

查看 goal_pairs.yaml 中的坐标范围：
```bash
# 提取所有x, y坐标
grep -A 2 "start:\|end:" goal_pairs.yaml | grep -E "^\s+- [0-9]"
```

如果坐标范围大约是 0-9，则边界设置为 `[0, 9]`。

### 方法3: 从图像尺寸计算

```python
from PIL import Image

img = Image.open("Alkylation.png")
width, height = img.size

# 假设场景为9米x9米
aspect_ratio = width / height
if width > height:
    x_bounds = [0, 9]
    y_bounds = [0, 9 / aspect_ratio]
else:
    x_bounds = [0, 9 * aspect_ratio]
    y_bounds = [0, 9]
```

---

## 🔍 结果验证

### 1. 检查可视化图像

查看生成的 `*_pair_XXX_path.png` 图像：
- ✅ 红色路径应该连接起点和终点
- ✅ 路径应该避开黑色障碍物
- ✅ 路径应该尽量短且平滑

### 2. 检查 waypoints

```bash
# 查看生成的 waypoints
cat outputs/path_planning_results/*_waypoints.json | python3 -m json.tool | less
```

验证要点：
- [ ] 起点和终点是否正确
- [ ] 路径点是否连续
- [ ] 角度（theta）是否合理

### 3. 在仿真中测试

使用生成的 `nav_config.yaml` 运行导航任务：

```bash
python main.py \
  --config-name level5_Navigation \
  --headless
```

---

## 📈 性能优化

### 批量处理多个场景

```bash
# 创建批处理脚本
for scene_dir in roomlayout/layout_new/*/; do
  scene_name=$(basename "$scene_dir")
  barrier_img="${scene_dir}/${scene_name}.png"
  goal_pairs="outputs/auto_batch_nav_targets/goal_pairs/${scene_name}_goal_pairs.yaml"

  if [ -f "$barrier_img" ] && [ -f "$goal_pairs" ]; then
    echo "Processing: $scene_name"
    PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 utils/path_planner_with_goal_pairs.py \
      --barrier_image "$barrier_img" \
      --goal_pairs_yaml "$goal_pairs" \
      --x_bounds 0 9 \
      --y_bounds 0 9 \
      --output_dir "outputs/path_planning_results/$scene_name"
  fi
done
```

---

## 🔗 与其他工具的集成

### 1. 用于导航任务

生成的 `nav_config.yaml` 可以直接用于 `NavigationTaskNew`：

```python
# 在配置文件中指定
task:
  navigation_config_path: "outputs/path_planning_results/.../nav_config.yaml"
```

### 2. 用于数据收集

规划好的 waypoints 可以用于收集导航数据：

```bash
python main.py \
  --config-name level5_Navigation \
  --headless \
  --no-video
```

### 3. 用于可视化

使用生成的路径图像进行报告和演示：

```python
from PIL import Image
import matplotlib.pyplot as plt

# 显示路径
img = Image.open("outputs/*_pair_000_path.png")
plt.imshow(img)
plt.title("Planned Path")
plt.show()
```

---

## 📚 相关文档

- [PROJECT_STRUCTURE.md](../PROJECT_STRUCTURE.md) - 项目结构说明
- [API_REFERENCE.md](../API_REFERENCE.md) - API 参考
- [utils/a_star.py](../utils/a_star.py) - A* 算法实现

---

**最后更新**: 2025-01-05
**作者**: LabUtopia Team
