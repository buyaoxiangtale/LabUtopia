# path_planner_with_goal_pairs.py 代码详解

> 本文档详细讲解路径规划工具的实现原理和PNG图像处理格式

## 📋 目录

1. [代码整体架构](#代码整体架构)
2. [PNG图像处理详解](#png图像处理详解)
3. [核心函数解析](#核心函数解析)
4. [完整工作流程](#完整工作流程)
5. [关键数据结构](#关键数据结构)

---

## 代码整体架构

### 类结构

```python
ScenePathPlanner
├── __init__()              # 初始化规划器
├── _estimate_bounds_from_image()  # 估算场景边界
├── plan_single_pair()      # 规划单条路径
├── plan_from_goal_pairs_yaml()  # 批量规划
├── _visualize_single_path() # 可视化路径
├── _save_results()         # 保存结果
└── _generate_nav_config()  # 生成导航配置
```

---

## PNG图像处理详解

### 1️⃣ 图像加载与二值化

**函数位置**: `utils/a_star.py:64-76`

```python
def load_grid(image_path):
    """将图像转换为二进制网格表示"""
    with Image.open(image_path) as img:
        W, H = img.size  # 获取图像尺寸
        return (
            [
                [0 if is_white(img.getpixel((j, i))) else 1
                 for j in range(W)]  # 列遍历（x方向）
                for i in range(H)]   # 行遍历（y方向）
            ),
            W, H
        )
```

**关键点**:
- 使用 `PIL.Image.open()` 打开PNG图像
- `img.size` 返回 (宽度, 高度) = (W, H)
- `img.getpixel((j, i))` 获取第 i 行、第 j 列的像素值

**网格结构**:
```python
grid = [
    [pixel(0,0), pixel(1,0), ..., pixel(W-1,0)],   # 第0行
    [pixel(0,1), pixel(1,1), ..., pixel(W-1,1)],   # 第1行
    ...
    [pixel(0,H-1), ..., pixel(W-1,H-1)]            # 第H-1行
]
```

**注意**: `grid[i][j]` 表示第 i 行、第 j 列

---

### 2️⃣ 像素值判断

**函数位置**: `utils/a_star.py:78-84`

```python
def is_white(pixel):
    """判断像素是否代表白色（可行走区域）"""
    # 情况1: 灰度图（单通道）
    if isinstance(pixel, int):
        return pixel == 255  # 白色 = 255

    # 情况2: RGB图像（三通道）
    channels = pixel[:3]  # 取RGB三个通道
    return channels == (255, 255, 255)  # 纯白
```

**像素值含义**:
| 像素值 | 含义 | 在grid中的值 | 可通行性 |
|--------|------|--------------|----------|
| (255, 255, 255) | 白色 | 0 | ✅ 可行走 |
| (0, 0, 0) | 黑色 | 1 | ❌ 障碍物 |
| 其他值 | 灰度 | 1 | ❌ 障碍物 |

**示例**:
```python
# 白色像素
pixel = (255, 255, 255)
is_white(pixel)  # 返回 True → grid[i][j] = 0

# 黑色像素
pixel = (0, 0, 0)
is_white(pixel)  # 返回 False → grid[i][j] = 1

# 灰度像素
pixel = 128
is_white(pixel)  # 返回 False → grid[i][j] = 1
```

---

### 3️⃣ 障碍物膨胀

**函数位置**: `utils/a_star.py:86-89`

```python
def inflate_obstacles(grid: np.ndarray, radius: int) -> np.ndarray:
    """
    膨胀障碍物，考虑机器人半径

    Args:
        grid: 原始网格
        radius: 膨胀半径（像素）

    Returns:
        膨胀后的网格
    """
    from scipy.ndimage import binary_dilation

    # 创建结构元素（正方形）
    struct = np.ones((2 * radius + 1, 2 * radius + 1))

    # 执行二值膨胀
    return binary_dilation(grid, structure=struct).astype(np.int32)
```

**膨胀原理**:

```
原始网格 (radius=1):
0 0 0 0 0        0 = 可行走
0 0 1 0 0        1 = 障碍物
0 0 0 0 0

膨胀后网格:
0 1 1 1 0        障碍物向外扩张1像素
0 1 1 1 0
0 1 1 1 0
```

**为什么需要膨胀？**
- 机器人不是点，有体积
- 机器人中心离障碍物至少要 offset_radius 米
- 膨胀确保机器人不会太靠近障碍物

**计算示例**:
```python
# 场景边界: 9米 x 9米
# 图像尺寸: 179 x 179 像素
# 分辨率: 9/179 ≈ 0.0503 米/像素

# 机器人半径: 0.6米
radius_pixels = int(0.6 / 0.0503)  # ≈ 12 像素

# 膨胀后的障碍物会向外扩张12个像素
# 相当于实际场景中的0.6米安全距离
```

---

### 4️⃣ 坐标转换系统

这是代码中最重要的部分！

#### 4.1 真实坐标 → 网格坐标

**函数位置**: `utils/a_star.py:92-100`

```python
def real_to_grid(x, y, x_bounds, y_bounds, grid_size):
    """
    将真实坐标转换为网格索引

    Args:
        x, y: 真实坐标（米）
        x_bounds: [x_min, x_max]（米）
        y_bounds: [y_min, y_max]（米）
        grid_size: (W, H) 像素

    Returns:
        (i, j): 网格索引
    """
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    W, H = grid_size

    # Y坐标反转（图像坐标系 vs 笛卡尔坐标系）
    i = min(max(int((y_max - y) / (y_max - y_min) * H), 0), H - 1)

    # X坐标正常
    j = min(max(int((x - x_min) / (x_max - x_min) * W), 0), W - 1)

    return (i, j)
```

**坐标系统对比**:

```
真实坐标系统（笛卡尔坐标系）:
      ↑ Y
      |
      |
      +----→ X

图像坐标系统:
      +----→ j (列)
      |
      ↓
      i (行)
```

**转换示例**:
```python
# 场景设置
x_bounds = [0, 9]    # 米
y_bounds = [0, 9]    # 米
grid_size = (179, 179)  # 像素

# 真实坐标
x, y = 4.5, 4.5  # 米（场景中心）

# 转换
i = int((9 - 4.5) / 9 * 179)  # = 89
j = int((4.5 - 0) / 9 * 179)  # = 89

grid[89][89]  # 对应真实坐标 (4.5, 4.5)
```

#### 4.2 网格坐标 → 真实坐标

**函数位置**: `utils/a_star.py:103-111`

```python
def grid_to_real(i, j, x_bounds, y_bounds, grid_size):
    """
    将网格索引转换为真实坐标

    Args:
        i, j: 网格索引
        x_bounds, y_bounds: 边界
        grid_size: (W, H)

    Returns:
        (x, y): 真实坐标（米）
    """
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    W, H = grid_size

    # X坐标
    x = x_min + (j + 0.5) * (x_max - x_min) / W

    # Y坐标（反转）
    y = y_max - (i + 0.5) * (y_max - y_min) / H

    return (x, y)
```

**关键点**: `+ 0.5` 表示取网格中心点

**示例**:
```python
# 网格索引
i, j = 89, 89

# 转换
x = 0 + (89 + 0.5) * 9 / 179  # = 4.5028 米
y = 9 - (89 + 0.5) * 9 / 179  # = 4.4972 米

# 真实坐标
real_pos = (4.50, 4.50)  # ≈ 场景中心
```

---

## 核心函数解析

### 1️⃣ 初始化函数

```python
def __init__(
    self,
    barrier_image_path: str,
    x_bounds: Optional[List[float]] = None,
    y_bounds: Optional[List[float]] = None,
    offset_radius: float = 0.6
):
    # 保存参数
    self.barrier_image_path = barrier_image_path
    self.offset_radius = offset_radius

    # 🔑 关键步骤1: 加载障碍物地图
    self.grid, self.W, self.H = load_grid(barrier_image_path)
    # self.grid: 二维数组，0=可行走，1=障碍物
    # self.W, self.H: 图像尺寸（像素）

    print(f"✓ 加载障碍物地图: {barrier_image_path}")
    print(f"  尺寸: {self.W} x {self.H} 像素")

    # 🔑 关键步骤2: 设置场景边界
    if x_bounds is None or y_bounds is None:
        x_bounds, y_bounds = self._estimate_bounds_from_image()

    self.x_bounds = x_bounds  # [x_min, x_max]
    self.y_bounds = y_bounds  # [y_min, y_max]

    # 🔑 关键步骤3: 计算分辨率
    resolution = (x_bounds[1] - x_bounds[0]) / self.W
    print(f"  分辨率: {resolution:.4f} 米/像素")
```

**分辨率计算**:
```python
# 示例
W = 179 像素
x_bounds = [0, 9] 米
resolution = 9 / 179 ≈ 0.0503 米/像素

# 含义: 每个像素代表约5厘米的实际距离
```

---

### 2️⃣ 单路径规划

```python
def plan_single_pair(self, start, end, task_id):
    """规划单组起终点对的路径"""

    # 步骤1: 提取XY坐标
    start_xy = [start[0], start[1]]  # 忽略Z坐标
    end_xy = [end[0], end[1]]

    # 步骤2: 构建任务信息
    task_info = {
        'asset': {
            'barrier_image_path': self.barrier_image_path,
            'x_bounds': self.x_bounds,
            'y_bounds': self.y_bounds,
            'offset_radius': self.offset_radius
        },
        'start': start_xy,
        'end': end_xy
    }

    # 步骤3: 🔑 调用A*规划
    path_result = plan_navigation_path(task_info)

    if path_result is None:
        print(f"✗ 规划失败")
        return None

    # 步骤4: 解析结果
    real_path, path_grid, total_distance = path_result
    # real_path: [[x, y, 0], [x, y, 0], ...]  真实坐标
    # path_grid: [(i1, j1), (i2, j2), ...]    网格索引
    # total_distance: 总距离（米）

    # 步骤5: 🔑 生成waypoints（添加角度）
    waypoints = []
    for i, (x, y, _) in enumerate(real_path):
        if i < len(real_path) - 1:
            # 计算朝向：指向下一个点
            nx, ny, _ = real_path[i + 1]
            theta = np.arctan2(ny - y, nx - x)
        else:
            # 最后一个点使用前一个角度
            theta = waypoints[-1][2] if waypoints else 0.0

        waypoints.append([x, y, theta])

    return {
        'task_id': task_id,
        'start': start_xy,
        'end': end_xy,
        'waypoints': waypoints,  # [[x, y, theta], ...]
        'total_distance': total_distance,
        'num_waypoints': len(waypoints)
    }
```

**角度计算详解**:
```python
# 示例: 路径点序列
points = [
    [2.0, 2.0],  # 点A
    [3.0, 2.0],  # 点B
    [3.0, 3.0],  # 点C
]

# 计算点A的角度
dx = 3.0 - 2.0  # = 1.0
dy = 2.0 - 2.0  # = 0.0
theta_A = atan2(0.0, 1.0)  # = 0.0 弧度 (指向东)

# 计算点B的角度
dx = 3.0 - 3.0  # = 0.0
dy = 3.0 - 2.0  # = 1.0
theta_B = atan2(1.0, 0.0)  # = π/2 弧度 (指向北)

# 结果
waypoints = [
    [2.0, 2.0, 0.0],      # 朝东
    [3.0, 2.0, 1.5708],   # 朝北
    [3.0, 3.0, 1.5708],   # 朝北
]
```

---

### 3️⃣ A*路径规划

**函数位置**: `utils/a_star.py:9-45`

```python
def astar(grid, start, end):
    """A*寻路算法"""

    # 4个方向：上、下、左、右
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # 优先队列（开放列表）
    open_heap = []
    heapq.heappush(open_heap, (0, *start))

    # 记录路径
    came_from = {}

    # G值：从起点到当前点的实际代价
    g_scores = {start: 0}

    # F值：G值 + 启发式估计
    f_scores = {start: heuristic(start, end)}

    while open_heap:
        # 取出F值最小的节点
        current_f, cx, cy = heapq.heappop(open_heap)

        # 到达终点？
        if (cx, cy) == end:
            return reconstruct_path(came_from, end)

        # 检查4个邻居
        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy

            # 越界或障碍物？
            if (not (0 <= nx < len(grid) and 0 <= ny < len(grid[0]))
                or grid[nx][ny] != 0):
                continue

            # 计算代价
            move_cost = 1.0  # 直线移动
            tentative_g = g_scores[(cx, cy)] + move_cost

            # 找到更优路径？
            if tentative_g < g_scores.get((nx, ny), float("inf")):
                came_from[(nx, ny)] = (cx, cy)
                g_scores[(nx, ny)] = tentative_g
                f = tentative_g + heuristic((nx, ny), end)
                heapq.heappush(open_heap, (f, nx, ny))

    # 找不到路径
    return None
```

**A*算法核心**:
- `G值`: 从起点到当前点的实际距离
- `H值`: 启发式估计（到终点的直线距离）
- `F值 = G + H`: 综合评估
- 每次选择F值最小的节点扩展

---

### 4️⃣ 启发式函数

**函数位置**: `utils/a_star.py:48-54`

```python
def heuristic(a, b):
    """
    启发式函数：估算从a到b的代价

    使用切比雪夫距离（适用于4方向移动）
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])

    # max(dx, dy): 切比雪夫距离
    return max(dx, dy)
```

**距离公式对比**:
```python
# 曼哈顿距离（4方向）
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# 欧几里得距离（8方向）
def euclidean(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# 切比雪夫距离（4或8方向）
def chebyshev(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))
```

本代码使用切比雪夫距离，因为只允许4方向移动。

---

## 完整工作流程

### 流程图

```
┌─────────────────────────────────────────────────────────────┐
│ 1. 初始化阶段                              │
└─────────────────────────────────────────────────────────────┘
                          ↓
        加载 PNG 图像 → 转换为二值网格 → 存储 (grid, W, H)
                          ↓
        设置场景边界 → 计算分辨率
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. 路径规划阶段                        │
└─────────────────────────────────────────────────────────────┘
                          ↓
        真实坐标 → 网格坐标
                          ↓
        障碍物膨胀 (考虑机器人半径)
                          ↓
        A* 算法搜索 → 网格路径
                          ↓
        网格坐标 → 真实坐标
                          ↓
        计算角度 → 生成 waypoints
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. 结果输出阶段                                             │
└─────────────────────────────────────────────────────────────┘
                          ↓
        保存 JSON (planning_results.json)
        保存 JSON (waypoints.json)
        保存 YAML (nav_config.yaml)
        保存 PNG (可视化图像)
```

---

## 关键数据结构

### 1. 网格 (Grid)

```python
# 类型: List[List[int]]
# 维度: H x W (行 x 列)

grid = [
    [0, 0, 1, 0, 0],  # 第0行
    [0, 0, 1, 0, 0],  # 第1行
    [0, 0, 0, 0, 0],  # 第2行
]

# 访问方式
pixel = grid[i][j]  # 第i行，第j列

# 含义
# 0 = 可行走区域（白色像素）
# 1 = 障碍物（黑色像素）
```

### 2. Waypoints

```python
# 类型: List[List[float]]
# 结构: [[x, y, theta], ...]

waypoints = [
    [2.19, 6.81, 0.0],         # 第1个路径点
    [2.24, 6.81, 0.0],         # 第2个路径点
    [2.29, 6.81, 0.0],         # 第3个路径点
    ...
]

# 每个点包含:
# x: X坐标（米）
# y: Y坐标（米）
# theta: 朝向角度（弧度，范围 -π 到 π）
```

### 3. 路径规划结果

```python
result = {
    'task_id': '场景_pair_000',
    'start': [2.2, 6.81],           # 起点
    'end': [5.5, 5.96],             # 终点
    'waypoints': [[x, y, theta], ...],  # 路径点
    'total_distance': 4.17,         # 总距离（米）
    'num_waypoints': 84             # 路径点数量
}
```

---

## PNG图像格式要求

### 支持的格式

| 格式 | 颜色模式 | 说明 |
|------|----------|------|
| PNG | 灰度 (L) | 单通道，0-255 |
| PNG | RGB | 三通道，(R,G,B) |
| PNG | RGBA | 四通道，(R,G,B,A) |

### 颜色要求

```python
# ✅ 可行走区域
白色 = (255, 255, 255) → grid[i][j] = 0

# ❌ 障碍物
黑色 = (0, 0, 0) → grid[i][j] = 1
灰色 = (128, 128, 128) → grid[i][j] = 1
任何非白色 → grid[i][j] = 1
```

### 推荐的图像生成

```python
from PIL import Image, ImageDraw
import numpy as np

# 方法1: 从零开始创建
W, H = 179, 179
img = Image.new('RGB', (W, H), 'white')  # 白色背景
draw = ImageDraw.Draw(img)

# 绘制障碍物（黑色矩形）
draw.rectangle([50, 50, 80, 80], fill='black')

img.save('barrier.png')

# 方法2: 从NumPy数组创建
grid = np.zeros((H, W), dtype=np.uint8)
grid[50:80, 50:80] = 255  # 障碍物区域
img = Image.fromarray(grid, mode='L')
img.save('barrier.png')
```

### 图像尺寸建议

```python
# 计算最佳尺寸
scene_size = 9.0  # 米（场景边长）
resolution = 0.05  # 米/像素（每像素代表的距离）

optimal_pixels = int(scene_size / resolution)  # = 180 像素

# 推荐: 179x179, 200x200, 256x256
# 避免过大: >1000x1000（计算慢）
# 避免过小: <100x100（精度低）
```

---

## 常见问题解析

### Q1: 为什么路径规划失败？

```python
# 原因1: 起点或终点在障碍物上
if grid[start_i][start_j] == 1:
    return None  # 起点不可达

# 原因2: 膨胀半径太大
# 解决: 减小 offset_radius
--offset_radius 0.3  # 从0.6减到0.3

# 原因3: 起点被障碍物完全包围
# 解决: 检查图像，确保有连通的可行走区域
```

### Q2: 路径点为什么这么多？

```python
# A*算法访问的每个网格点都会生成一个路径点
# 示例: 179x179 图像
# 路径长度: 4.17米
# 路径点数: 84个
# 平均点间距: 4.17 / 84 ≈ 0.05米 = 5厘米

# 如何减少路径点？
# 方法1: 降低图像分辨率（增加每像素代表的米数）
# 方法2: 路径简化（删除共线的中间点）
# 方法3: 增加障碍物膨胀半径（避开狭窄通道）
```

### Q3: 角度是如何计算的？

```python
# 方法: 指向下一个点
for i in range(len(waypoints) - 1):
    curr = waypoints[i]
    next_ = waypoints[i + 1]

    dx = next_[0] - curr[0]  # Δx
    dy = next_[1] - curr[1]  # Δy

    # 使用 atan2 计算（考虑象限）
    theta = np.arctan2(dy, dx)  # 弧度

    waypoints[i][2] = theta

# 最后一个点使用前一个点的角度
waypoints[-1][2] = waypoints[-2][2]
```

---

## 总结

这个工具的核心流程是：

1. **PNG图像** → **二值网格** (0=可行走, 1=障碍物)
2. **真实坐标** → **网格坐标** (考虑Y轴反转)
3. **A*算法** → **网格路径** (在网格上搜索)
4. **网格坐标** → **真实坐标** (转换回去)
5. **路径点** → **Waypoints** (添加角度信息)

关键点：
- ✅ 图像必须是白底黑障
- ✅ 分辨率影响路径精度
- ✅ offset_radius 确保安全距离
- ✅ 角度用于机器人朝向控制

---

**最后更新**: 2025-01-05
