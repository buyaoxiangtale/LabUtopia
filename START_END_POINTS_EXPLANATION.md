# 起点和终点的来源说明

## 📍 起点和终点的数据流

### 1. **数据来源**

起点和终点来自**批量路径规划结果**的JSON文件：

```
/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/
└── run_2026-01-14_14-00-11/
    └── [场景名称]/
        └── [场景名称]_all_results.json  ← 这里存储了所有任务
```

### 2. **JSON数据格式**

每个任务包含以下字段：

```json
{
  "task_id": "Alkylation_of_Ethyl_Acetoacetate_with_Bis4-fluorop_20260112_202101_pair_000",
  "start": [1.29, 5.5],        ← 起点坐标 [x, y]
  "end": [1.71, 6.955],        ← 终点坐标 [x, y]
  "is_success": true,           ← 路径规划是否成功
  "waypoints": [
    [x, y, theta],
    ...
  ],
  "total_distance": 1.85,
  "num_waypoints": 35
}
```

### 3. **代码中的使用位置**

在 `collision_analyzer_v4.py` 的 `main()` 函数中：

```python
# 第410-416行：读取所有任务结果
for f in Path(base_dir).rglob("*_all_results.json"):
    try:
        with open(f) as fp:
            results.extend(json.load(fp))  # 加载所有任务
    except:
        pass

# 第416行：筛选失败的任务
failed_tasks = [t for t in results if not t.get('is_success', False)]

# 第419-424行：按场景分组
tasks_by_scene = defaultdict(list)
for t in failed_tasks:
    tid = t.get('task_id', 'unknown')
    s_name = '_'.join(tid.split('_')[:-1])  # 从task_id提取场景名
    tasks_by_scene[s_name].append(t)
```

### 4. **提取起点和终点**

```python
# 第447-450行：遍历每个失败的任务
for task in tasks:
    start = task.get('start')   # ← 从JSON获取起点 [x, y]
    end = task.get('end')       # ← 从JSON获取终点 [x, y]
    tid = task.get('task_id')[-3:]  # 任务ID的后3位

    # 第453-454行：检测起点和终点是否碰撞
    s_hits = analyzer.get_collisions_at_point(start[0], start[1], robot_radius)
    e_hits = analyzer.get_collisions_at_point(end[0], end[1], robot_radius)
```

## 🔍 完整数据流

```
批量路径规划脚本 (run_batch_path_planning_gemini_flash.sh)
    ↓
为每个场景生成 goal_pairs (起终点对)
    ↓
运行 A* 路径规划 (utils/path_planner_with_goal_pairs.py)
    ↓
保存结果到 JSON 文件:
    ├── [场景名]_all_results.json      ← 所有任务（成功+失败）
    ├── [场景名]_failed_tasks.json    ← 只包含失败的任务
    └── [场景名]_waypoints.json       ← 只包含成功的waypoints
    ↓
collision_analyzer_v4.py 读取这些文件
    ↓
提取每个任务的 start 和 end 字段
    ↓
使用 OBB 碰撞检测分析起点/终点是否在障碍物内
```

## 📋 起终点如何生成的？

### 方法1：从 goal_pairs.yaml 文件生成

在批量路径规划中，起点和终点来自配置文件：

```yaml
# goal_pairs.yaml
goal_pairs:
  - start: [1.29, 5.5, 0.0]     # 第1个任务的起点
    end: [1.71, 6.955, 0.0]     # 第1个任务的终点

  - start: [1.71, 6.955, 0.0]   # 第2个任务的起点
    end: [6.825, 7.1, 0.0]      # 第2个任务的终点

  - start: [6.825, 7.1, 0.0]    # 第3个任务的起点
    end: [4.589, 4.5, 0.0]      # 第3个任务的终点
```

**生成位置**：
```bash
/home/pjlab/fbh/LabUtopia/outputs/gemini_flash_nav_targets_1_13_18_47/goal_pairs/
└── [场景名]_goal_pairs.yaml
```

### 方法2：随机生成（如果未提供goal_pairs）

在导航任务中（`tasks/navigation_task.py`），如果没有提供goal_pairs，会随机生成：

```python
def _generate_random_navigation_task(self):
    # 随机生成起点和终点
    start_point, end_point = self._generate_random_points(
        x_bounds,
        y_bounds,
        self.grid
    )
```

## 🎯 关键代码片段

### 1. 读取JSON并提取起点终点

```python
# 读取所有任务结果
results = []
for f in Path(base_dir).rglob("*_all_results.json"):
    with open(f) as fp:
        results.extend(json.load(fp))

# 筛选失败的任务
failed_tasks = [t for t in results if not t.get('is_success', False)]

# 遍历每个失败任务
for task in failed_tasks:
    start = task.get('start')  # [x, y]
    end = task.get('end')      # [x, y]

    # 碰撞检测
    s_hits = analyzer.get_collisions_at_point(start[0], start[1], robot_radius)
    e_hits = analyzer.get_collisions_at_point(end[0], end[1], robot_radius)
```

### 2. 场景名称解析

```python
# task_id 格式：SceneName_YYYYMMDD_HHMMSS_pair_XXX
# 例如：Alkylation_20260112_202101_pair_000

tid = task.get('task_id', 'unknown')
# 提取场景名称：去掉 _pair_XXX 后缀
s_name = '_'.join(tid.split('_')[:-1])
# 结果：Alkylation_20260112_202101
```

### 3. 查找对应的场景文件

```python
def find_scene_file(scene_name: str, scene_base_dir: str):
    import re

    # 去除日期后缀，获取核心场景名
    core_name = re.sub(r'_20\d{6}_\d{6}$', '', scene_name)

    # 在场景目录中查找匹配的文件夹
    base = Path(scene_base_dir)
    candidate_dirs = [d for d in base.iterdir()
                     if d.is_dir() and core_name in d.name]

    # 在文件夹中查找场景JSON文件
    for d in candidate_dirs:
        json_files = list(d.glob("*_room_isaacsim.json"))
        if json_files:
            return str(json_files[0])

    return None
```

## 📊 数据格式示例

### 输入：JSON文件

```json
[
  {
    "task_id": "SceneName_pair_000",
    "start": [1.29, 5.5],
    "end": [1.71, 6.955],
    "is_success": false,  ← 失败的任务会被分析
    "waypoints": [],
    "total_distance": 0.0,
    "num_waypoints": 0
  },
  {
    "task_id": "SceneName_pair_001",
    "start": [1.71, 6.955],
    "end": [6.825, 7.1],
    "is_success": true,   ← 成功的任务不会被分析
    ...
  }
]
```

### 输出：碰撞分析结果

```
场景: SceneName
  ✓ 成功加载场景 (OBB模式)
    总物体数: 26, 跳过房间: 1, 实际加载: 25

  任务 000:
    ❌ 起点碰撞: ['Table1']
    ❌ 终点碰撞: ['Cabinet2']

  任务 001:
    ❓ 路径通畅（可能原因：算法参数或局部极小值）
```

## 💡 总结

1. **起点和终点来自哪里**：
   - 从 `*_all_results.json` 文件中读取
   - 这些文件由批量路径规划脚本生成
   - 每个任务包含 `start` 和 `end` 字段

2. **只分析失败的任务**：
   - `is_success: false` 的任务会被分析
   - `is_success: true` 的任务会被跳过

3. **分析过程**：
   - 读取所有任务的 start 和 end
   - 使用 OBB 碰撞检测判断是否在障碍物内
   - 报告哪些物体导致了碰撞

4. **数据流向**：
   ```
goal_pairs.yaml → A*路径规划 → JSON结果文件
   → collision_analyzer_v4.py → 碰撞分析报告
   ```
