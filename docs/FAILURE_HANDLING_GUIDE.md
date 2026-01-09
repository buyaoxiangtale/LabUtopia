# 路径规划失败处理指南

## 概述

批量路径规划工具现在具备完整的失败处理和诊断机制，能够：
- ✅ 优雅地处理规划失败，不会中断批量处理
- ✅ 生成详细的失败原因分析
- ✅ 为失败样例生成可视化图像
- ✅ 保存完整的失败记录到JSON文件
- ✅ 提供失败原因分类统计

---

## 失败判别机制

### 1. **核心判别逻辑**

```python
# 检查A*规划结果
path_result = plan_navigation_path(task_info)

# 失败判别：real_path is None
if path_result is None or path_result[0] is None:
    # 进入失败处理流程
```

### 2. **失败原因分类**

工具会自动识别以下4种失败原因：

| 失败类别 | 说明 | 网格值检测 |
|---------|------|-----------|
| `start_on_obstacle` | 起点在障碍物上 | `grid[start] != 0` |
| `end_on_obstacle` | 终点在障碍物上 | `grid[end] != 0` |
| `both_on_obstacle` | 起点和终点都在障碍物上 | `grid[start] != 0` 且 `grid[end] != 0` |
| `blocked` | 路径被障碍物完全阻隔 | 起终点正常但无法到达 |

---

## 输出文件说明

运行批量路径规划后，会在输出目录生成以下文件：

### 成功的样例
```
output_dir/
├── {scene_name}_pair_000_path.png       # 成功路径可视化
├── {scene_name}_pair_001_path.png
└── ...
```

### 失败的样例
```
output_dir/
├── {scene_name}_pair_002_FAILED.png     # 失败可视化（标注起终点）
├── {scene_name}_pair_005_FAILED.png
└── ...
```

### 结果文件
```
output_dir/
├── {scene_name}_all_results.json        # 所有结果（成功+失败）
├── {scene_name}_failed_tasks.json       # 仅失败任务的详细记录
├── {scene_name}_waypoints.json          # 仅成功任务的waypoints
└── {scene_name}_nav_config.yaml         # 仅成功任务的导航配置
```

---

## 失败记录格式

### 1. **JSON文件结构**

`{scene_name}_failed_tasks.json`:
```json
[
  {
    "task_id": "scene_pair_002",
    "start": [1.2, 3.4],
    "end": [5.6, 7.8],
    "is_success": false,
    "failure_reason": "终点在障碍物上",
    "failure_category": "end_on_obstacle",
    "start_grid": [10, 20],
    "end_grid": [30, 40],
    "start_on_obstacle": false,
    "end_on_obstacle": true
  }
]
```

### 2. **控制台输出示例**

```
✗ scene_pair_002: 路径规划失败
   原因: 终点在障碍物上
   起点 [1.2, 3.4] (grid: [10, 20], on_obstacle: False)
   终点 [5.6, 7.8] (grid: [30, 40], on_obstacle: True)
失败可视化已保存到: outputs/scene_pair_002_FAILED.png
```

---

## 统计报告示例

批量处理完成后会显示：

```
======================================================================
规划完成
成功: 45/50 (90.0%)
失败: 5/50 (10.0%)

失败原因分类:
  - end_on_obstacle: 3 (60.0%)
  - blocked: 2 (40.0%)

成功路径长度统计:
  平均: 5.23 米
  最短: 2.15 米
  最长: 8.76 米
======================================================================
```

---

## 失败可视化说明

### 可视化图像元素

- **绿色圆点**: 起点（位置正常）
- **黄色圆点**: 起点（在障碍物上）
- **橙色圆点**: 终点（位置正常）
- **红色圆点**: 终点（在障碍物上）
- **白色区域**: 可通行区域
- **黑色区域**: 障碍物
- **红色文字标注**: 失败原因说明

### 示例图说明

```
失败情况1: 起点在障碍物上
┌─────────────────────┐
│  ⬜⬜⬜⬛⬜⬜⬜⬜⬜⬜  │
│  ⬜⬜⬛⬛⬛⬜⬜⬜⬜⬜  │
│  ⬜⬛💚⬛⬜⬜⬜⬜⬜⬜  │  ← 绿色点在障碍物上
│  ⬜⬜⬛⬛⬛⬜⬜⬜⬜⬜  │
│  ⬜⬜⬜⬛⬜⬜⬜⬛🧡⬜  │  ← 橙色点（终点）
│  ⬜⬜⬜⬜⬜⬜⬜⬛⬛⬜  │
└─────────────────────┘
```

---

## 使用建议

### 1. **处理失败任务**

查看失败记录：
```bash
# 查看失败任务详情
cat outputs/{scene}_failed_tasks.json | jq .
```

### 2. **调整配置重试**

根据失败原因调整：
- **起点/终点在障碍物上**: 检查起终点坐标设置，或减小 `offset_radius`
- **路径被阻隔**: 增大场景边界，或检查障碍物图像是否正确

### 3. **批量重试失败任务**

提取失败任务并单独处理：
```python
import json

# 加载失败任务
with open('outputs/{scene}_failed_tasks.json') as f:
    failed_tasks = json.load(f)

# 对特定类别的失败任务进行重试
retry_tasks = [t for t in failed_tasks if t['failure_category'] == 'blocked']
```

---

## 代码示例

### 失败处理流程

```python
from utils.path_planner_with_goal_pairs import ScenePathPlanner

# 创建规划器
planner = ScenePathPlanner(
    barrier_image_path='barrier.png',
    x_bounds=[0, 9],
    y_bounds=[0, 9]
)

# 执行批量规划（会自动处理失败）
result_summary = planner.plan_from_goal_pairs_yaml(
    yaml_path='goal_pairs.yaml',
    visualize=True,
    output_dir='outputs/results'
)

# 访问失败任务
failed_tasks = result_summary['failed_results']
print(f"失败任务数: {len(failed_tasks)}")

# 按失败原因分组
from collections import defaultdict
failure_by_category = defaultdict(list)
for task in failed_tasks:
    failure_by_category[task['failure_category']].append(task)

# 打印统计
for category, tasks in failure_by_category.items():
    print(f"{category}: {len(tasks)} 个")
```

---

## 常见问题解决

### Q1: 大量任务失败（起点/终点在障碍物上）
**原因**: 起终点生成算法未考虑障碍物膨胀
**解决**:
1. 减小 `offset_radius` 参数（默认0.6米）
2. 重新生成起终点，使用更保守的安全距离

### Q2: 路径被阻隔失败
**原因**: 场景中存在完全封闭的区域
**解决**:
1. 检查障碍物图像是否正确
2. 增大场景边界
3. 使用更小的膨胀半径

### Q3: 想重新运行失败的任务
**方法**:
```bash
# 从失败记录中提取任务
python utils/extract_failed_tasks.py \
  --failed_json outputs/scene_failed_tasks.json \
  --output retry_goal_pairs.yaml

# 重新运行
python utils/path_planner_with_goal_pairs.py \
  --barrier_image barrier.png \
  --goal_pairs_yaml retry_goal_pairs.yaml \
  --x_bounds 0 9 \
  --y_bounds 0 9
```

---

## 技术细节

### 诊断函数

失败检测使用 `real_to_grid()` 将实际坐标转换为网格坐标：

```python
from utils.a_star import real_to_grid

start_grid = real_to_grid(
    start[0], start[1],  # x, y 坐标
    x_bounds, y_bounds,  # 场景边界
    (W, H)               # 图像尺寸
)

# 检查网格值
if grid[start_grid[0]][start_grid[1]] != 0:
    # 起点在障碍物上
```

### 可视化函数

失败可视化使用 `visualize_failure()` 函数：
- 自动标注起终点位置
- 用颜色区分是否在障碍物上
- 添加文字说明失败原因

---

## 总结

新的失败处理机制确保：
1. **稳定性**: 批量处理不会因个别失败而中断
2. **可追溯**: 所有失败任务都有详细记录
3. **可调试**: 可视化图像帮助理解失败原因
4. **可改进**: 分类统计指导参数优化

现在可以放心运行大规模批量路径规划！
