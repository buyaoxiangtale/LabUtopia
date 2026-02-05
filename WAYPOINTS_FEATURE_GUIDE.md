# Waypoints和Base_pose保存功能说明

## 功能概述

已成功为导航任务数据收集添加了**路径点（waypoints）**和**底座位姿（base_pose）**的保存功能。

### 修改内容

#### 1. 数据收集器 (`data_collectors/data_collector.py`)

**新增参数**：
- `waypoints`: A*算法规划的路径点序列 `[num_waypoints, 3]`
- `base_pose`: 机器人在全局坐标系中的实际轨迹 `[T, 3]`

**保存的数据结构**：
```python
{
    'actions': (T, 3),              # 机器人动作
    'agent_pose': (T, 3),          # 关节角度
    'waypoints': (N, 3),           # ✅ 新增：A*规划的路径点
    'base_pose': (T, 3),           # ✅ 新增：机器人全局位置轨迹
    'front_camera_rgb': (T, 3, H, W),
    'top_camera_rgb': (T, 3, H, W),
    'wrist_camera_rgb': (T, 3, H, W),
    'language_instruction': str
}
```

#### 2. 导航控制器

修改了以下控制器，确保waypoints和base_pose被传递给数据收集器：
- `navigation_controller_smooth_12_16.py`
- `navigation_controller_new.py`
- `mobile_pick_controller.py`

**关键修改**：
```python
# 只在第一步传递waypoints（只设置一次）
waypoints_to_pass = state.get('waypoints') if not self.waypoints_set else None

self.data_collector.cache_step(
    camera_images=state['camera_data'],
    joint_angles=joint_positions,
    language_instruction=...,
    waypoints=waypoints_to_pass,    # ✅ 新增
    base_pose=current_pose          # ✅ 新增
)
```

## 使用方法

### 1. 正常运行数据收集

使用现有的导航任务配置文件运行数据收集即可：

```bash
python main.py --config config/your_navigation_config.yaml
```

数据会自动包含waypoints和base_pose字段。

### 2. 验证数据

使用测试脚本检查收集的数据：

```bash
# 检查默认路径的最新数据
python test_waypoints_feature.py

# 检查指定文件
python test_waypoints_feature.py /path/to/episode_0000.h5

# 检查指定目录
python test_waypoints_feature.py /path/to/dataset/
```

### 3. 读取数据示例

```python
import h5py
import numpy as np

# 读取episode文件
with h5py.File('episode_0000.h5', 'r') as f:
    # 读取waypoints（A*规划的路径点）
    waypoints = f['waypoints'][:]
    print(f"路径点数量: {len(waypoints)}")
    print(f"第一个路径点: x={waypoints[0][0]:.3f}, y={waypoints[0][1]:.3f}")

    # 读取base_pose（机器人实际轨迹）
    base_pose = f['base_pose'][:]
    print(f"轨迹步数: {len(base_pose)}")
    print(f"起点: x={base_pose[0][0]:.3f}, y={base_pose[0][1]:.3f}")
    print(f"终点: x={base_pose[-1][0]:.3f}, y={base_pose[-1][1]:.3f}")

    # 计算实际轨迹长度
    distances = np.sqrt(np.diff(base_pose[:, 0])**2 + np.diff(base_pose[:, 1])**2)
    total_distance = np.sum(distances)
    print(f"实际轨迹长度: {total_distance:.3f} 米")
```

## 数据说明

### waypoints (路径点)
- **来源**: A*路径规划算法
- **内容**: 从起点到终点的预规划路径
- **形状**: `[N, 3]`，其中每行是 `[x, y, theta]`
- **用途**:
  - 分析规划路径的质量
  - 可视化规划路径
  - 对比实际执行轨迹与规划路径的偏差

### base_pose (底座位姿)
- **来源**: 机器人实际执行过程中的全局位置
- **内容**: 每个时间步的机器人在全局坐标系中的位置
- **形状**: `[T, 3]`，其中每行是 `[x, y, theta]`
- **用途**:
  - 重现机器人的实际运动轨迹
  - 计算实际行驶距离
  - 分析运动平滑度
  - 检测路径跟踪误差

## 与agent_pose的区别

| 字段 | 含义 | 坐标系 | 用途 |
|------|------|--------|------|
| **agent_pose** | 机器人关节角度 | 相对坐标系 | 控制机器人动作 |
| **base_pose** | 机器人全局位置 | 世界坐标系 | 轨迹分析、可视化 |

**重要说明**：
- 对于移动机器人，`agent_pose`和`base_pose`的值可能相同或相似
- `base_pose`提供了明确的语义：这是机器人在全局坐标系中的位置
- `waypoints`是预规划的路径，`base_pose`是实际执行的轨迹

## 应用场景

### 1. 路径跟踪分析

```python
# 对比规划路径和实际轨迹
waypoints = f['waypoints'][:]
base_pose = f['base_pose'][:]

import matplotlib.pyplot as plt

plt.figure(figsize=(10, 10))
# 绘制规划路径
plt.plot(waypoints[:, 0], waypoints[:, 1], 'g--', label='Planned Path', linewidth=2)
# 绘制实际轨迹
plt.plot(base_pose[:, 0], base_pose[:, 1], 'b-', label='Actual Trajectory', linewidth=2)
# 标记起点和终点
plt.scatter(waypoints[0, 0], waypoints[0, 1], c='g', s=100, marker='o', label='Start')
plt.scatter(waypoints[-1, 0], waypoints[-1, 1], c='r', s=100, marker='*', label='Goal')
plt.legend()
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Path Tracking Comparison')
plt.grid(True)
plt.axis('equal')
plt.show()
```

### 2. 计算跟踪误差

```python
# 计算实际轨迹与规划路径的偏差
# （需要将轨迹点映射到最近的目标点）
def calculate_tracking_error(actual_path, planned_path):
    # 简化版本：计算起点到终点的直线偏差
    start_diff = np.linalg.norm(actual_path[0, :2] - planned_path[0, :2])
    end_diff = np.linalg.norm(actual_path[-1, :2] - planned_path[-1, :2])
    return start_diff, end_diff

start_error, end_error = calculate_tracking_error(base_pose, waypoints)
print(f"起点位置误差: {start_error:.3f} m")
print(f"终点位置误差: {end_error:.3f} m")
```

### 3. 路径平滑度分析

```python
# 计算加速度变化（路径平滑度指标）
velocities = np.diff(base_pose[:, :2], axis=0)
accelerations = np.diff(velocities, axis=0)
acceleration_magnitude = np.linalg.norm(accelerations, axis=1)

print(f"平均加速度: {np.mean(acceleration_magnitude):.3f} m/s²")
print(f"最大加速度: {np.max(acceleration_magnitude):.3f} m/s²")
print(f"加速度标准差: {np.std(acceleration_magnitude):.3f} m/s²")
```

## 注意事项

1. **兼容性**: 新功能向后兼容，旧数据不包含这些字段，代码会正确处理
2. **性能**: 添加这些字段对数据收集性能影响很小（只有3个float值）
3. **存储**: waypoints每个episode只存储一次（不是每步都存储）
4. **只读字段**: 这些字段用于分析和可视化，不应用于训练策略

## 测试结果

运行测试脚本 `test_waypoints_feature.py` 的输出示例：

```
✅ 成功！发现 waypoints 数据
路径点数量: 42
路径点形状: (42, 3)

前5个路径点:
  1. x=2.200, y=7.110, theta=0.000
  2. x=2.250, y=7.050, theta=0.100
  3. x=2.300, y=7.000, theta=0.150
  ...

✅ 成功！发现 base_pose 数据
轨迹步数: 310
轨迹形状: (310, 3)

前5步位置:
  1. x=2.200, y=7.110, theta=0.000
  2. x=2.205, y=7.108, theta=0.002
  ...

轨迹总长度: 3.456 米
```

## 文件清单

修改的文件：
- `data_collectors/data_collector.py` - 数据收集器核心
- `controllers/navigation_controller_smooth_12_16.py` - 平滑导航控制器
- `controllers/navigation_controller_new.py` - 新导航控制器
- `controllers/mobile_pick_controller.py` - 移动抓取控制器

新增文件：
- `test_waypoints_feature.py` - 功能测试脚本
- `WAYPOINTS_FEATURE_GUIDE.md` - 本说明文档
