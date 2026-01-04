# 批量路径规划工具使用指南

## 📋 概述

`utils/batch_path_planning.py` 提供了批量处理多组起点终点对和占用地图的功能，支持：

1. ✅ **灵活的配置方式**：支持 JSON/YAML 配置文件或代码手动添加
2. ✅ **多占用地图支持**：每组起终点可配备独立的占用地图
3. ✅ **并行处理**：支持多进程并行提升效率
4. ✅ **详细报告**：生成统计报告、可视化图像和 CSV 导出
5. ✅ **失败处理**：自动记录失败原因并提供详细日志

---

## 🚀 快速开始

### 方式1：从 YAML 配置批量添加（共享占用地图）

最简单的方式，适用于所有任务使用同一个场景和占用地图的情况：

```python
from utils.batch_path_planning import BatchPathPlanner

# 创建规划器
planner = BatchPathPlanner()

# 定义多组起终点
goal_pairs = [
    ([1.0, 1.0], [8.0, 8.0]),
    ([2.0, 2.0], [7.0, 7.0]),
    ([1.5, 1.5], [6.0, 8.0]),
]

# 从 YAML 场景配置批量添加
planner.add_tasks_from_yaml_assets(
    yaml_path="config/navigation/navigation_assets_fbh.yaml",
    goal_pairs=goal_pairs
)

# 执行批量规划
results = planner.run_batch(verbose=True)

# 保存结果
planner.save_results(output_dir="outputs/batch_planning")
planner.generate_summary_report(output_dir="outputs/batch_planning")
```

### 方式2：手动添加任务（独立占用地图）

适用于每个任务使用不同占用地图的场景：

```python
planner = BatchPathPlanner()

# 手动添加第一个任务（场景1）
planner.add_task(
    task_id="scene1_task_001",
    start=[1.0, 1.0],
    end=[8.0, 8.0],
    barrier_image_path="data/navigation_scenes/barrier_scene1.png",
    x_bounds=[0.0, 10.0],
    y_bounds=[0.0, 10.0],
    offset_radius=0.3
)

# 手动添加第二个任务（场景2，不同的占用地图）
planner.add_task(
    task_id="scene2_task_001",
    start=[2.0, 2.0],
    end=[7.0, 7.0],
    barrier_image_path="data/navigation_scenes/barrier_scene2.png",
    x_bounds=[0.0, 10.0],
    y_bounds=[0.0, 10.0],
    offset_radius=0.3
)

# 执行规划
results = planner.run_batch(verbose=True)
```

### 方式3：从 JSON 配置文件加载

适用于预定义好的批量任务配置：

**配置文件格式** (`config/batch_tasks_example.json`):

```json
{
  "tasks": [
    {
      "task_id": "scene1_task_001",
      "start": [1.0, 1.0],
      "end": [8.0, 8.0],
      "barrier_image_path": "data/navigation_scenes/barrier_scene1.png",
      "x_bounds": [0.0, 10.0],
      "y_bounds": [0.0, 10.0],
      "offset_radius": 0.3
    },
    {
      "task_id": "scene2_task_001",
      "start": [2.0, 2.0],
      "end": [7.0, 7.0],
      "barrier_image_path": "data/navigation_scenes/barrier_scene2.png",
      "x_bounds": [0.0, 10.0],
      "y_bounds": [0.0, 10.0],
      "offset_radius": 0.3
    }
  ]
}
```

**使用代码**:

```python
# 从配置文件加载任务
planner = BatchPathPlanner("config/batch_tasks_example.json")

# 执行规划
results = planner.run_batch(verbose=True)
```

---

## 📊 输出结果说明

### 1. 目录结构

批量规划会在指定的输出目录生成以下文件：

```
outputs/batch_planning/
├── batch_results.json          # 完整的规划结果（JSON格式）
├── successful_tasks.txt        # 成功任务的 task_id 列表
├── failed_tasks.txt           # 失败任务的 task_id 列表
├── summary_report.txt         # 文本格式的统计报告
├── statistics_plots.png       # 统计可视化图表
├── results.csv                # CSV 格式的结果表格
└── *_path.png                 # 各任务的路径可视化图像
```

### 2. JSON 结果格式

```json
[
  {
    "task_id": "task_001",
    "success": true,
    "start": [1.0, 1.0],
    "end": [8.0, 8.0],
    "barrier_image_path": "data/...png",
    "waypoints": [
      [1.0, 1.0, 0.78],
      [1.5, 1.5, 0.78],
      ...
    ],
    "total_distance": 9.899,
    "num_waypoints": 45,
    "segments": [
      {
        "segment_idx": 0,
        "from": [1.0, 1.0],
        "to": [1.5, 1.5],
        "distance": 0.707,
        "direction": 0.785,
        "direction_deg": 45.0,
        "velocity_magnitude": 0.02,
        "estimated_time": 35.35
      },
      ...
    ]
  }
]
```

### 3. CSV 表格格式

```csv
task_id,success,start_x,start_y,end_x,end_y,total_distance,num_waypoints,barrier_image_path
task_001,Yes,1.0,1.0,8.0,8.0,9.899,45,data/...png
task_002,Yes,2.0,2.0,7.0,7.0,7.071,32,data/...png
```

---

## 🔧 高级功能

### 1. 并行处理

对于大量任务，可以启用并行处理加速：

```python
# 启用并行，使用 4 个工作进程
results = planner.run_batch(parallel=True, max_workers=4, verbose=True)
```

**注意事项**：
- 并行处理适用于计算密集型任务（大量路径规划）
- 对于小批量任务（< 10），串行模式可能更快（避免进程启动开销）
- 确保有足够的 CPU 核心数

### 2. 路径可视化

```python
# 可视化前 10 条路径
planner.visualize_paths(output_dir="outputs/batch_planning", max_paths=10)
```

会为每条路径生成占用地图 + 路径轨迹的可视化图像。

### 3. 统计报告

```python
# 生成统计报告和可视化图表
planner.generate_summary_report(output_dir="outputs/batch_planning")
```

**报告内容包括**：
- 成功/失败任务统计
- 路径长度统计（平均、最短、最长、标准差）
- 路径点数量统计
- 路径长度和路径点数量的分布直方图

---

## 📖 OMAP（占用地图）详细说明

### OMAP 的数据结构

OMAP（Occupancy Map，占用地图）是一个二维数组，表示场景中的障碍物分布：

```python
grid, W, H = load_grid(barrier_image_path)

# grid[i][j] 的含义：
# - i: 行索引（0 ~ H-1），对应图像的 y 轴（从上到下）
# - j: 列索引（0 ~ W-1），对应图像的 x 轴（从左到右）
# - grid[i][j] == 0: 自由空间（白色像素）
# - grid[i][j] == 1: 障碍物（非白色像素）
```

### OMAP 的加载

```python
from utils.a_star import load_grid

# 从 PNG 图像加载占用地图
grid, W, H = load_grid("path/to/barrier_image.png")

# PNG 图像要求：
# - 白色像素 (255, 255, 255) → 自由空间 (0)
# - 其他颜色 → 障碍物 (1)
```

### OMAP 在路径规划中的使用

```python
from utils.a_star import plan_navigation_path

# 1. 原始占用地图
grid, W, H = load_grid(barrier_image_path)

# 2. 障碍物膨胀（为机器人留出安全距离）
inflated_grid = inflate_obstacles(grid, radius_pixels)

# 3. 检查点是否在障碍物上
if inflated_grid[start_i][start_j] == 1:
    # 起点在障碍物上，无法规划
    return None

# 4. A* 算法使用膨胀后的 grid 搜索路径
path_grid = astar(inflated_grid, start_grid, end_grid)
```

### 坐标转换

真实坐标（米）↔ 网格坐标（像素）：

```python
from utils.a_star import real_to_grid, grid_to_real

# 真实坐标 → 网格坐标
i, j = real_to_grid(
    x=5.0, y=5.0,
    x_bounds=[0.0, 10.0],
    y_bounds=[0.0, 10.0],
    grid_size=(W, H)
)

# 网格坐标 → 真实坐标
x, y = grid_to_real(
    i=50, j=50,
    x_bounds=[0.0, 10.0],
    y_bounds=[0.0, 10.0],
    grid_size=(W, H)
)
```

**坐标系对应关系**：
- `grid[0][0]` → 图像左上角 → 真实坐标 `(x_min, y_max)`
- `grid[H-1][W-1]` → 图像右下角 → 真实坐标 `(x_max, y_min)`

---

## 🧪 测试脚本

提供了完整的测试脚本 `test_batch_path_planning.py`：

```bash
# 运行所有测试
python test_batch_path_planning.py 5

# 运行单个测试
python test_batch_path_planning.py 1  # YAML 配置批量添加
python test_batch_path_planning.py 2  # 手动添加任务
python test_batch_path_planning.py 3  # JSON 配置文件加载
python test_batch_path_planning.py 4  # 并行处理模式
```

---

## 💡 使用建议

1. **选择合适的配置方式**：
   - 单一场景 + 多组起终点 → 使用方式1（YAML 配置）
   - 多场景 + 各自起终点 → 使用方式2（手动添加）或方式3（JSON 配置）

2. **并行处理的适用场景**：
   - 任务数 > 20 时，建议启用并行
   - 任务数 < 10 时，串行模式可能更快

3. **结果验证**：
   - 检查 `failed_tasks.txt` 查看失败任务
   - 查看路径可视化图像验证路径合理性
   - 查看统计报告了解整体情况

4. **占用地图准备**：
   - 确保PNG图像分辨率足够高（建议至少 500x500）
   - 白色区域表示可通行，其他颜色表示障碍物
   - 可通过调整 `offset_radius` 控制安全距离

---

## 📚 相关文件

- `utils/batch_path_planning.py` - 批量路径规划主程序
- `utils/path_planning_precompute.py` - 单路径预计算工具
- `utils/a_star.py` - A* 路径规划算法和 OMAP 处理
- `test_batch_path_planning.py` - 测试脚本
- `config/batch_tasks_example.json` - JSON 配置示例

---

## 🐛 常见问题

### Q1: 路径规划失败怎么办？

**A**: 检查以下几点：
1. 起终点是否在障碍物上
2. 是否存在可行路径（查看可视化图像）
3. 尝试增大 `offset_radius` 或调整起终点位置

### Q2: 如何批量生成随机起终点？

**A**: 使用 `PathPrecomputer` 的随机生成功能：

```python
from utils.path_planning_precompute import PathPrecomputer

precomputer = PathPrecomputer("config/navigation/navigation_assets_fbh.yaml")
random_paths = precomputer.generate_random_paths(num_paths=10, min_distance=3.0)
```

### Q3: 并行处理报错怎么办？

**A**:
- 确保 Python 支持多进程（Linux/Mac 通常没问题，Windows 可能需要 `if __name__ == "__main__"`）
- 减少 `max_workers` 数量
- 检查是否有文件读写冲突

---

## 📝 更新日志

- **2024-01-03**: 初始版本
  - 支持批量路径规划
  - 支持多占用地图
  - 支持并行处理
  - 生成详细报告和可视化
