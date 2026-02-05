# 固定相机外参修复

## 📋 问题描述

**之前的错误**：
- `observation.camera_extrinsic` 存储的是相机在世界坐标系中的第一帧位姿（来自 `trajectory[0]`）
- 这导致外参不再是"相机相对于机器人的固定位姿"，而是"相机在世界坐标系中的初始位置"

**正确的外参定义**：
- 相机外参应该是**相机相对于机器人的固定位姿**（固定不变）
- 不应该随机器人运动而变化

---

## ✅ 修复方案

### 1. 添加固定外参参数

在 `ParquetFormatCollector.__init__()` 中添加 `fixed_camera_extrinsic` 参数：

```python
def __init__(
    self,
    save_dir: str,
    camera_configs: List[dict],
    ...
    fixed_camera_extrinsic: Optional[np.ndarray] = None  # 新增参数
):
    # 固定相机外参（相机相对于机器人的位姿）
    if fixed_camera_extrinsic is not None:
        self.fixed_camera_extrinsic = np.array(fixed_camera_extrinsic)
    else:
        # 默认值：Ridgeback机器人的相机外参
        self.fixed_camera_extrinsic = np.array([
            [ 0,  0, -1,  1.1],  # 相机在机器人前方 1.1m
            [-1,  0,  0,  0.0],
            [ 0,  1,  0,  1.0],  # 相机高度 1.0m
            [ 0,  0,  0,  1. ]
        ])
```

### 2. 修改保存逻辑

在 `_save_parquet()` 方法中，使用固定外参而不是从轨迹提取：

```python
# ❌ 旧代码（错误）
first_pose = self.temp_trajectory[0][cam_name]
extrinsic_data = [first_pose.tolist()] + [None] * (num_steps - 1)

# ✅ 新代码（正确）
extrinsic_data = [self.fixed_camera_extrinsic.tolist()] + [None] * (num_steps - 1)
```

---

## 📊 固定外参矩阵

### Ridgeback 机器人默认外参

```python
[[ 0,  0, -1,  1.1],  # 第1行
 [-1,  0,  0,  0.0],  # 第2行
 [ 0,  1,  0,  1.0],  # 第3行
 [ 0,  0,  0,  1.]]   # 第4行
```

### 物理含义

**平移部分** (最后一列):
- `tx = 1.1m`: 相机在机器人前方 1.1m（沿相机 -Z 方向，即机器人 -X 方向）
- `ty = 0.0m`: 相机在机器人侧方 0m（无偏移）
- `tz = 1.0m`: 相机高度 1.0m

**旋转部分** (前3列):
- 相机 X 轴 `[0, -1, 0]` 指向机器人 **-Z 方向**
- 相机 Y 轴 `[0, 0, 1]` 指向机器人 **+Y 方向**
- 相机 Z 轴（光轴）`[-1, 0, 0]` 指向机器人 **-X 方向**

### 坐标系关系

```
机器人坐标系                   相机坐标系
    ↑ Y                          ↑ Y
    |                            |
    |                            |
    +----→ -X (相机朝向)          +----→ -Z (光轴)
   /                           /
  Z                           X
```

---

## 🔧 使用方法

### 方法 1: 使用默认外参（推荐）

```python
from data_collectors.parquet_format_collector import ParquetFormatCollector

collector = ParquetFormatCollector(
    save_dir='output',
    camera_configs=[{
        'name': 'observation',
        'image_type': 'rgb+depth',
        'resolution': [256, 256]
    }]
)

# 默认使用 Ridgeback 机器人的外参
```

### 方法 2: 自定义外参

```python
import numpy as np

# 定义你的相机外参（相机相对于机器人）
custom_extrinsic = np.array([
    [ 0,  0, -1,  1.1],  # 根据你的实际安装调整
    [-1,  0,  0,  0.0],
    [ 0,  1,  0,  1.0],
    [ 0,  0,  0,  1. ]
])

collector = ParquetFormatCollector(
    save_dir='output',
    camera_configs=[...],
    fixed_camera_extrinsic=custom_extrinsic  # 传入自定义外参
)
```

---

## 📂 数据存储结构

### Parquet 文件中的列

| 列名 | 存储方式 | 帧数 | 说明 |
|------|---------|------|------|
| `observation.camera_intrinsic` | 稀疏 | 1/556 | 相机内参（固定不变） |
| `observation.camera_extrinsic` | **稀疏** | **1/556** | **相机外参（固定不变）** ✅ |
| `action` | 密集 | 556/556 | 相机在世界坐标系中的轨迹 |

### 关键区别

| 数据类型 | 定义 | 是否变化 | 用途 |
|---------|------|---------|------|
| **camera_extrinsic** | 相机相对于机器人的位姿 | ❌ 不变（固定） | 坐标系转换 |
| **action** | 相机在世界坐标系中的位姿 | ✅ 变化（每帧） | 训练标签 |

---

## 🧪 验证结果

测试显示外参已正确设置为固定值：

```
✅ 外参完全匹配！

📊 相机外参矩阵:
  [   0.0,    0.0,   -1.0,    1.1]
  [  -1.0,    0.0,    0.0,    0.0]
  [   0.0,    1.0,    0.0,    1.0]
  [   0.0,    0.0,    0.0,    1.0]

物理含义:
  - 相机安装在机器人前方 1.1m 处
  - 相机高度 1.0m
  - 相机朝向: 机器人 -X 方向
  - 相机坐标系: X→机器人-Z, Y→机器人+Y, Z→机器人-X
```

---

## 📝 坐标变换关系

### 使用外参进行坐标转换

```python
# 给定一个点在机器人坐标系中的位置 P_robot
# 计算它在相机坐标系中的位置 P_cam

P_cam = T_extrinsic @ P_robot

# 其中 T_extrinsic 是相机外参矩阵
# T_extrinsic = [[R, t], [0, 1]]
#   R: 旋转矩阵（3x3）
#   t: 平移向量（3x1）
```

### 实际应用示例

```python
import numpy as np

# 相机外参
T_ex = np.array([
    [ 0,  0, -1,  1.1],
    [-1,  0,  0,  0.0],
    [ 0,  1,  0,  1.0],
    [ 0,  0,  0,  1. ]
])

# 机器人前方 2m 处的一个障碍物点（机器人坐标系）
P_robot = np.array([2.0, 0.0, 0.0, 1.0])  # 齐次坐标

# 转换到相机坐标系
P_cam = T_ex @ P_robot
# 结果: [-0.0, 2.0, -0.9, 1.0]
# 解释: 在相机前方 0.9m 处（Z轴负方向），左侧 2m
```

---

## ⚠️ 常见错误

### 错误 1: 混淆外参和轨迹

```python
# ❌ 错误：把轨迹的第一帧当作外参
extrinsic = trajectory[0]  # 这是相机在世界坐标系中的位姿！

# ✅ 正确：使用固定的相机外参
extrinsic = fixed_camera_extrinsic  # 这是相机相对于机器人的位姿
```

### 错误 2: 外参随时间变化

```python
# ❌ 错误：每帧都更新外参
for i in range(num_frames):
    data['camera_extrinsic'][i] = compute_extrinsic(pose[i])

# ✅ 正确：外参固定不变
data['camera_extrinsic'][0] = fixed_extrinsic
data['camera_extrinsic'][1:] = [None] * (num_frames - 1)  # 稀疏存储
```

### 错误 3: 混淆坐标系

```python
# ❌ 错误：直接使用世界坐标位姿作为外参
extrinsic = camera_world_pose  # 这是 action，不是 extrinsic！

# ✅ 正确：计算相机相对于机器人的位姿
extrinsic = robot_pose_inv @ camera_world_pose  # 或者直接使用固定值
```

---

## 🎯 总结

1. ✅ **相机外参** = 相机相对于机器人的**固定位姿**
2. ✅ **相机轨迹 (action)** = 相机在世界坐标系中的**运动序列**
3. ✅ 外参使用**稀疏存储**（只存第1帧）
4. ✅ Action 使用**密集存储**（每帧都存）
5. ✅ 代码已修改为使用你提供的固定外参矩阵

---

## 📚 相关文件

- **主要修改**: `data_collectors/parquet_format_collector.py`
  - `__init__()`: 添加 `fixed_camera_extrinsic` 参数
  - `_save_parquet()`: 使用固定外参而不是轨迹第一帧

- **测试验证**: `verify_parquet_fix.py`

- **相关文档**:
  - `PARQUET_FIX_SUMMARY.md`: Parquet 数据修复总结
  - `/home/pjlab/fbh/InternNav/dataset_compatibility_analysis.md`: 数据集格式说明
