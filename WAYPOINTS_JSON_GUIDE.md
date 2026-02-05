# Waypoints JSON保存功能使用指南

## 📋 功能概述

这个功能会在导航任务执行时，自动将A*路径规划生成的waypoints保存为JSON格式的文件，与批量路径规划结果保持完全一致的格式。

## 🎯 保存格式

每个episode的waypoints会保存为独立的JSON文件：

```json
{
  "task_id": "episode_0000",
  "start": [2.2, 7.11],
  "end": [5.5, 6.26],
  "is_success": true,
  "waypoints": [
    [2.2, 7.11, 0.0],
    [2.25, 7.11, 0.0],
    [2.3, 7.11, 0.0],
    ...
  ],
  "total_distance": 4.173,
  "num_waypoints": 84
}
```

## 📁 文件结构

```
outputs/collect/2026.01.15/14.01.35_level5_Navigation_smooth_1_11/
├── dataset/
│   ├── episode_0000.h5
│   ├── episode_0001.h5
│   └── ...
├── config.yaml
├── waypoints/                          ← 新增：waypoints JSON目录
│   ├── episode_0000_waypoints.json     ← 单个episode的waypoints
│   ├── episode_0001_waypoints.json
│   └── ...
└── all_waypoints.json                  ← 新增：所有waypoints的合并文件
```

## 🚀 使用方法

### 1. 正常运行导航任务

```bash
python main.py --config config/level5_Navigation_smooth_1_11.yaml
```

**自动保存**：
- ✅ 每次路径规划成功后，自动保存waypoints到JSON
- ✅ 文件保存在`outputs/collect/<run_dir>/waypoints/`目录
- ✅ 文件名格式：`episode_XXXX_waypoints.json`

### 2. 查看保存的waypoints

```bash
# 查看waypoints目录
ls outputs/collect/2026.01.15/14.01.35_level5_Navigation_smooth_1_11/waypoints/

# 查看单个文件
cat outputs/collect/2026.01.15/14.01.35_level5_Navigation_smooth_1_11/waypoints/episode_0000_waypoints.json
```

### 3. 测试验证

```bash
# 运行测试脚本
python test_waypoints_json_save.py
```

测试脚本会验证：
- ✅ JSON文件格式是否正确
- ✅ 必需字段是否完整
- ✅ waypoints数据是否有效
- ✅ 格式是否与批量路径规划结果一致

## 📊 数据读取示例

### Python读取waypoints

```python
import json
from pathlib import Path

# 读取单个episode的waypoints
waypoints_file = "outputs/collect/.../waypoints/episode_0000_waypoints.json"
with open(waypoints_file, 'r') as f:
    data = json.load(f)

print(f"任务ID: {data['task_id']}")
print(f"起点: {data['start']}")
print(f"终点: {data['end']}")
print(f"路径点数量: {data['num_waypoints']}")
print(f"路径总长度: {data['total_distance']}米")

# 遍历waypoints
for i, wp in enumerate(data['waypoints']):
    x, y, theta = wp
    print(f"  点{i}: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
```

### 读取所有waypoints

```python
import json
from pathlib import Path

# 读取合并的文件
all_waypoints_file = "outputs/collect/.../all_waypoints.json"
with open(all_waypoints_file, 'r') as f:
    all_episodes = json.load(f)

# 遍历所有episodes
for episode in all_episodes:
    print(f"Episode {episode['task_id']}")
    print(f"  起点: {episode['start']}")
    print(f"  终点: {episode['end']}")
    print(f"  长度: {episode['total_distance']:.2f}米")
```

## 🔧 与HDF5保存的区别

### 之前的HDF5方式（已废弃）
```python
# 在episode_0000.h5文件中
dataset['waypoints']  # Shape: (N, 3)
dataset['base_pose']  # Shape: (T, 3)
```

**问题**：
- ❌ 需要专门的HDF5库才能读取
- ❌ 不便于与其他数据集合并
- ❌ 不易于可视化和分析

### 现在的JSON方式（推荐）
```json
// 在episode_0000_waypoints.json文件中
{
  "task_id": "episode_0000",
  "waypoints": [[x, y, theta], ...],
  "total_distance": 4.173,
  ...
}
```

**优势**：
- ✅ 标准JSON格式，任何语言都能读取
- ✅ 与批量路径规划结果格式完全一致
- ✅ 易于合并、分析和可视化
- ✅ 人类可读，便于调试

## 📈 应用场景

### 1. 路径可视化

```python
import matplotlib.pyplot as plt
import json

with open('episode_0000_waypoints.json', 'r') as f:
    data = json.load(f)

waypoints = data['waypoints']
x_coords = [wp[0] for wp in waypoints]
y_coords = [wp[1] for wp in waypoints]

plt.figure(figsize=(10, 8))
plt.plot(x_coords, y_coords, 'b-', label='规划路径')
plt.scatter(x_coords[0], y_coords[0], c='g', s=100, label='起点')
plt.scatter(x_coords[-1], y_coords[-1], c='r', s=100, label='终点')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.title(f"路径可视化 - {data['task_id']}")
plt.savefig('path_visualization.png')
```

### 2. 数据集合并

```python
import json
from pathlib import Path

# 合并多个运行的结果
all_waypoints = []

for run_dir in Path("outputs/collect").glob("*"):
    waypoints_dir = run_dir / "waypoints"
    if waypoints_dir.exists():
        for json_file in waypoints_dir.glob("episode_*.json"):
            with open(json_file, 'r') as f:
                all_waypoints.append(json.load(f))

# 保存合并结果
with open('merged_all_waypoints.json', 'w') as f:
    json.dump(all_waypoints, f, indent=2)

print(f"合并了 {len(all_waypoints)} 个episodes")
```

### 3. 统计分析

```python
import json
import numpy as np

with open('all_waypoints.json', 'r') as f:
    episodes = json.load(f)

# 统计路径长度
distances = [ep['total_distance'] for ep in episodes]
print(f"平均路径长度: {np.mean(distances):.2f}米")
print(f"最短路径: {np.min(distances):.2f}米")
print(f"最长路径: {np.max(distances):.2f}米")

# 统计waypoints数量
wp_counts = [ep['num_waypoints'] for ep in episodes]
print(f"平均waypoints数: {np.mean(wp_counts):.0f}")
```

## 🔄 从批量路径规划结果导入

如果您已经有批量路径规划结果的JSON文件，可以直接使用：

```python
import json

# 读取批量路径规划结果
with open('path_planning_results.json', 'r') as f:
    batch_results = json.load(f)

# 直接使用（格式完全一致）
for result in batch_results:
    print(f"任务: {result['task_id']}")
    print(f"起点: {result['start']}")
    print(f"终点: {result['end']}")
    print(f"waypoints: {len(result['waypoints'])}个")
```

## ⚙️ 配置选项

### 关闭JSON保存（如果不需要）

在`tasks/navigation_task_test_weizi.py`的`_plan_and_set_path`方法中，注释掉保存代码：

```python
# if self.waypoints_saver:
#     try:
#         self.waypoints_saver.save_waypoints(...)
#     except Exception as e:
#         print(f"⚠️ 保存waypoints到JSON失败: {e}")
```

### 修改保存位置

在`setup_objects`方法中修改输出目录：

```python
# 默认：保存到运行目录
output_dir = self.cfg.multi_run.run_dir

# 自定义：保存到指定目录
output_dir = "outputs/my_waypoints"
```

## 🐛 故障排除

### 问题1：没有生成waypoints目录

**原因**：
- 路径规划失败
- 导航任务未正确执行

**解决**：
1. 检查任务日志，确认路径规划是否成功
2. 检查起点和终点是否在障碍物内

### 问题2：JSON文件为空或格式错误

**原因**：
- 保存过程中出现异常

**解决**：
1. 查看控制台输出的错误信息
2. 检查文件写入权限

### 问题3：读取JSON时编码错误

**解决**：
```python
# 使用utf-8编码
with open(file_path, 'r', encoding='utf-8') as f:
    data = json.load(f)
```

## 📚 相关文件

- `utils/waypoints_json_saver.py` - Waypoints JSON保存工具
- `tasks/navigation_task_test_weizi.py` - 导航任务（已集成JSON保存）
- `test_waypoints_json_save.py` - 测试验证脚本

## ✅ 总结

**核心优势**：
- ✅ 自动保存：无需手动操作
- ✅ 标准格式：与批量路径规划结果一致
- ✅ 易于使用：标准JSON，任何语言都能读取
- ✅ 完整信息：包含起点、终点、路径点、距离等

**下一步**：
1. 运行导航任务，自动生成waypoints JSON
2. 使用`test_waypoints_json_save.py`验证格式
3. 根据需要进行数据分析或可视化
