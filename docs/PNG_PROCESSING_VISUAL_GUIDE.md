# PNG 图像处理可视化指南

> 用图示和示例说明 PNG 图像如何被处理为路径规划网格

## 🖼️ PNG 图像处理完整流程

### 流程概览

```
PNG 文件
  ↓
PIL.Image.open()
  ↓
像素读取 (getpixel)
  ↓
颜色判断 (is_white?)
  ↓
二值网格 (grid[i][j] = 0/1)
  ↓
障碍物膨胀 (binary_dilation)
  ↓
A* 搜索网格
  ↓
路径输出
```

---

## 第1步：PNG 图像加载

### 原始 PNG 图像

```python
from PIL import Image

img = Image.open("Alkylation.png")
print(f"模式: {img.mode}")   # RGB, L, RGBA等
print(f"尺寸: {img.size}")   # (宽, 高) = (179, 179)
```

### 图像坐标系

```
PNG 图像坐标系统:
  j (列/ X轴)
  0   1   2   3   4
  +---+---+---+---+---+
0 |   |   | ■ |   |   |
  +---+---+---+---+---+
1 |   | ■ | ■ |   |   |
  +---+---+---+---+---+
2 |   |   |   |   |   |
  +---+---+---+---+---+
i |
  |
↓
(行/ Y轴)

像素访问:
pixel = img.getpixel((j, i))
# j = 列索引 (0 到 W-1)
# i = 行索引 (0 到 H-1)
```

---

## 第2步：颜色判断与二值化

### 像素值判断逻辑

```python
def is_white(pixel):
    """判断像素是否为白色"""
    # 情况1: 灰度图
    if isinstance(pixel, int):
        return pixel == 255

    # 情况2: RGB图
    if len(pixel) >= 3:
        return pixel[:3] == (255, 255, 255)

    return False
```

### 二值化示例

```
原始图像像素值:
(255,255,255) (255,255,255) (0,0,0)     (255,255,255)
(255,255,255) (128,128,128) (0,0,0)     (255,255,255)
(255,255,255) (255,255,255) (255,255,255) (255,255,255)

转换为二值网格:
0              0              1              0
0              1              1              0
0              0              0              0

网格存储 (grid[i][j]):
grid = [
    [0, 0, 1, 0],  # 第0行
    [0, 1, 1, 0],  # 第1行
    [0, 0, 0, 0],  # 第2行
]
```

---

## 第3步：障碍物膨胀

### 膨胀原理

```
原始网格 (radius=1):
0 0 0 0 0
0 0 1 0 0
0 0 0 0 0

结构元素 (3x3):
1 1 1
1 1 1
1 1 1

膨胀后网格:
0 1 1 1 0    ← 障碍物向外扩张1层
0 1 1 1 0
0 1 1 1 0
```

### 代码实现

```python
from scipy.ndimage import binary_dilation
import numpy as np

# 原始网格
grid = np.array([
    [0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0]
])

# 膨胀半径（像素）
radius_pixels = 1

# 创建结构元素
struct = np.ones((2*radius_pixels + 1, 2*radius_pixels + 1))
# struct = [[1, 1, 1],
#           [1, 1, 1],
#           [1, 1, 1]]

# 执行膨胀
inflated_grid = binary_dilation(grid, structure=struct)
```

### 膨胀计算

```python
# 场景参数
x_bounds = [0, 9]      # 米
y_bounds = [0, 9]      # 米
W, H = 179, 179       # 像素

# 机器人半径
offset_radius = 0.6   # 米

# 计算膨胀半径（像素）
meters_per_pixel = (x_bounds[1] - x_bounds[0]) / W
# = 9 / 179 ≈ 0.0503 米/像素

radius_pixels = int(offset_radius / meters_per_pixel)
# = int(0.6 / 0.0503)
# ≈ 12 像素

# 膨胀后的障碍物会向外扩张 12 个像素
```

---

## 第4步：坐标系统转换

### 真实坐标系 vs 图像坐标系

```
真实坐标系 (笛卡尔):
      ↑ Y (米)
      |  (4.5, 4.5)
      |     ●
      |
      +----→ X (米)
    (0, 0)

图像坐标系 (像素):
  j=0  j=89 j=179
  +----+-----+---->
  |    |  ●  |     i=89
  +----+-----+----+
  |              |
  +--------------+ i=179
  ↓

关键差异:
1. Y轴方向相反（真实坐标向上，图像坐标向下）
2. 单位不同（米 vs 像素）
3. 原点位置（真实坐标在左下角，图像坐标在左上角）
```

### 坐标转换公式

```python
# 真实坐标 → 图像坐标
def real_to_grid(x, y, x_bounds, y_bounds, grid_size):
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    W, H = grid_size

    # Y轴反转 + 归一化
    i = int((y_max - y) / (y_max - y_min) * H)

    # X轴直接 + 归一化
    j = int((x - x_min) / (x_max - x_min) * W)

    return (i, j)

# 图像坐标 → 真实坐标
def grid_to_real(i, j, x_bounds, y_bounds, grid_size):
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    W, H = grid_size

    # X轴
    x = x_min + (j + 0.5) * (x_max - x_min) / W

    # Y轴反转
    y = y_max - (i + 0.5) * (y_max - y_min) / H

    return (x, y)
```

### 转换示例

```python
# 参数
x_bounds = [0, 9]
y_bounds = [0, 9]
W, H = 179, 179

# 示例1: 场景中心
real_pos = (4.5, 4.5)
grid_pos = real_to_grid(4.5, 4.5, x_bounds, y_bounds, (W, H))
# grid_pos = (89, 89)

# 示例2: 左下角
real_pos = (0.0, 0.0)
grid_pos = real_to_grid(0.0, 0.0, x_bounds, y_bounds, (W, H))
# grid_pos = (179, 0)

# 示例3: 右上角
real_pos = (9.0, 9.0)
grid_pos = real_to_grid(9.0, 9.0, x_bounds, y_bounds, (W, H))
# grid_pos = (0, 179)
```

---

## 第5步：A* 搜索

### 搜索示例

```
网格 (5x5):
  0   1   2   3   4
0 +---+---+---+---+---+
  |   |   | ■ |   |   |  grid[0][2] = 1
1 +---+---+---+---+---+
  |   |   | ■ |   |   |  grid[1][2] = 1
2 +---+---+---+---+---+
  |   |   |   |   |   |  ← 起点 (0, 0)
3 +---+---+---+---+---+
  |   |   |   |   |   |
4 +---+---+---+---+---+
                  ↑
              终点 (4, 3)

A* 搜索过程:
1. 从 (0,0) 开始
2. 检查邻居: (0,1), (1,0)
3. 选择F值最小的节点
4. 避开障碍物 (0,2) 和 (1,2)
5. 最终到达 (4,3)

生成的路径 (网格坐标):
[(0,0), (0,1), (1,1), (2,1), (3,1), (3,2), (3,3), (4,3)]
```

### 方向移动

```python
# 代码中定义的方向
directions = [(-1, 0),  # 上: (i-1, j)
              (1, 0),   # 下: (i+1, j)
              (0, -1),  # 左: (i, j-1)
              (0, 1)]   # 右: (i, j+1)

# 示例: 当前位置 (2, 2)
# 上 → (1, 2)
# 下 → (3, 2)
# 左 → (2, 1)
# 右 → (2, 3)
```

---

## 第6步：Waypoints 生成

### 角度计算

```python
# 真实路径
real_path = [
    [2.19, 6.81, 0.0],
    [2.24, 6.81, 0.0],
    [2.29, 6.81, 0.0],
    [2.34, 6.81, 0.0],
    [2.39, 6.81, 0.0],
    [2.44, 6.81, 0.0],
    [2.44, 6.76, 0.0],  # ← 转弯点
    [2.44, 6.71, 0.0],
]

# 计算角度
for i in range(len(real_path) - 1):
    curr = real_path[i]    # [x, y, _]
    next_ = real_path[i+1]  # [x, y, _]

    dx = next_[0] - curr[0]  # Δx
    dy = next_[1] - curr[1]  # Δy

    # atan2: 返回 (-π, π] 范围的角度
    theta = np.arctan2(dy, dx)

    # 前6个点: dx=0.05, dy=0 → theta=0 (向东)
    # 转弯后: dx=0, dy=-0.05 → theta=-π/2 (向南)
```

### 角度方向

```
角度 (弧度)    角度 (度)    方向
0            0°          → 东 (正X)
π/2          90°         ↑ 北 (正Y)
π            180°        ← 西 (负X)
-π/2         -90°        ↓ 南 (负Y)
```

---

## 完整示例

### 输入图像

```
Alkylation.png (179x179):
┌─────────────────────────┐
│ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ │  第0行 (y最大)
│ ■       ■       ■     │
│ ■   ●   ■   ▲   ■     │  ● = 起点
│ ■       ■ /│\■       │  ▲ = 终点
│ ■       ■ / ■ ■       │  /│\ = 障碍
│ ■       ■             │
│ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ │  第178行 (y最小)
└─────────────────────────┘
  0                     178 (j)
```

### 处理过程

```python
# 步骤1: 加载图像
img = Image.open("Alkylation.png")
W, H = img.size  # (179, 179)

# 步骤2: 转换为网格
grid = []
for i in range(H):
    row = []
    for j in range(W):
        pixel = img.getpixel((j, i))
        row.append(0 if is_white(pixel) else 1)
    grid.append(row)

# 步骤3: 膨胀障碍物
radius_pixels = 12
inflated_grid = inflate_obstacles(grid, radius_pixels)

# 步骤4: 坐标转换
start_real = (2.2, 6.81)
end_real = (5.5, 5.96)

start_grid = real_to_grid(start_real[0], start_real[1],
                          x_bounds, y_bounds, (W, H))
# start_grid ≈ (24, 134)

end_grid = real_to_grid(end_real[0], end_real[1],
                        x_bounds, y_bounds, (W, H))
# end_grid ≈ (71, 112)

# 步骤5: A* 搜索
path_grid = astar(inflated_grid, start_grid, end_grid)
# path_grid = [(24,134), (25,134), (26,134), ..., (71,112)]

# 步骤6: 转换回真实坐标
waypoints = []
for (i, j) in path_grid:
    x, y = grid_to_real(i, j, x_bounds, y_bounds, (W, H))
    waypoints.append([x, y, 0.0])

# 步骤7: 添加角度
for k in range(len(waypoints) - 1):
    dx = waypoints[k+1][0] - waypoints[k][0]
    dy = waypoints[k+1][1] - waypoints[k][1]
    waypoints[k][2] = np.arctan2(dy, dx)
```

### 输出结果

```python
waypoints = [
    [2.19, 6.81, 0.0],
    [2.24, 6.81, 0.0],
    [2.29, 6.81, 0.0],
    ...
    [4.70, 6.76, -1.57],  # 转向南
    [4.70, 6.71, -1.57],
    [4.70, 6.66, -1.57],
    ...
    [5.51, 5.96, -1.57]   # 终点
]
```

---

## 可视化图像

### 生成路径可视化

```python
import matplotlib.pyplot as plt
import numpy as np

# 加载网格
grid, W, H = load_grid("Alkylation.png")

# 路径
path = [(24,134), (25,134), (30,140), ...]

# 绘制
plt.figure(figsize=(10, 10))
plt.imshow(grid, cmap='binary')  # 黑白显示

# 绘制路径
path_i, path_j = zip(*path)
plt.plot(path_j, path_i, 'r-', linewidth=2)  # 红色路径
plt.plot(path_j[0], path_i[0], 'go', markersize=10)  # 绿色起点
plt.plot(path_j[-1], path_i[-1], 'bo', markersize=10)  # 蓝色终点

plt.grid(True)
plt.savefig("path_visualization.png")
```

### 图像说明

```
可视化图像:
┌─────────────────────────┐
│ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ │
│ ■       ╱─────●       │  ● = 终点 (蓝色)
│ ■   ○─────────►       │  ○ = 起点 (绿色)
│ ■       ■ ╲ │ ╱ ■     │  ╱ = 路径 (红色)
│ ■       ■   ● ■       │  ■ = 障碍物 (黑色)
│ ■       ■             │
│ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ │
└─────────────────────────┘
```

---

## 关键要点总结

### ✅ PNG 图像要求

1. **颜色格式**: RGB 或 灰度
2. **颜色含义**: 白色=可行走，黑色=障碍物
3. **推荐尺寸**: 179x179, 200x200, 256x256
4. **文件格式**: PNG (无损压缩)

### 🔄 坐标转换

1. **真实坐标 → 网格坐标**: Y轴反转
2. **网格坐标 → 真实坐标**: Y轴再反转回来
3. **分辨率**: 每像素代表的米数
4. **边界**: 场景的 X 和 Y 范围

### 🎯 A* 搜索

1. **方向**: 4个方向（上下左右）
2. **启发式**: 切比雪夫距离
3. **代价**: 每步代价 = 1
4. **输出**: 网格索引路径

### 📍 Waypoints

1. **位置**: 真实坐标 (x, y)
2. **角度**: 指向下一个点的方向 (theta)
3. **用途**: 机器人导航指令

---

**最后更新**: 2025-01-05
