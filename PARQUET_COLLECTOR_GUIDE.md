# Parquet格式数据收集器使用指南

## 概述

`ParquetFormatCollector` 是一个专门用于收集包含相机内参、外参和轨迹数据的数据收集器。数据保存为 Parquet 格式，便于后续的机器学习数据处理。

## 数据结构（LeRobot 标准格式）

每个 episode 保存为一个 `.parquet` 文件，**符合 LeRobot 数据集标准**：

```
episode_000000.parquet
├── episode_index: int              # episode 索引
├── timestamp: str                  # ISO 格式时间戳
├── observation.state: List[List]   # 机器人状态 [x, y, theta]
├── observation.observation.camera_intrinsic: List[List[List]]  # 3x3 内参（仅第一帧）
├── observation.observation.camera_extrinsic: List[List[List]]  # 4x4 外参（仅第一帧）
├── action: List[List[List]]        # 相机轨迹（每帧 4x4 位姿矩阵）
└── task: str                       # 任务描述
```

**关键特点：**
- ✅ **LeRobot 兼容**：使用 `observation.*` 命名空间
- ✅ **嵌套数组**：Parquet 支持存储多维数组
- ✅ **稀疏存储**：内参和外参仅在第一帧存储（其余帧为 None）
- ✅ **轨迹作为 action**：相机轨迹存储在 `action` 字段中

## 文件结构

```
outputs/collect/2026.01.15/15.40.54_level5_Navigation_parquet/
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet
│       ├── episode_000001.parquet
│       └── ...
├── videos/                   # 可选（如果 save_videos=true）
│   └── chunk-000/
│       ├── observation.video.trajectory/
│       └── observation.video.depth/
├── episode_000000.json       # 元数据
├── episode_000001.json
└── ...
```

## 快速开始

### 1. 安装依赖

```bash
pip install pandas pyarrow scipy
```

### 2. 配置文件

创建或修改配置文件（例如 `config/level5_Navigation_parquet.yaml`）：

```yaml
dataset:
  type: "parquet_format"
  chunk_size: 1000
  save_videos: true  # 可选：同时保存视频

collector:
  type: "parquet_format"
```

### 3. 在任务中使用

#### 方法一：直接使用收集器

```python
from data_collectors.parquet_format_collector import create_parquet_format_collector
from utils.camera_utils import (
    get_camera_intrinsic,
    get_camera_extrinsic,
    get_camera_trajectory_matrix
)

# 创建收集器
collector = create_parquet_format_collector(
    camera_configs=cfg.cameras,
    save_dir='outputs/parquet_data',
    max_episodes=100,
    save_videos=True
)

# 在 episode 开始时
def reset(self):
    # 获取并设置相机内参（每个episode一次）
    intrinsics = {}
    for camera in self.cameras:
        intrinsics[camera.name] = get_camera_intrinsic(camera)
    collector.set_camera_intrinsics(intrinsics)

# 在每个时间步
def step(self):
    # 获取相机轨迹
    camera_poses = {}
    for camera in self.cameras:
        camera_poses[camera.name] = get_camera_trajectory_matrix(camera)

    # 获取机器人状态
    joint_angles = self.robot.get_joint_positions()

    # 缓存数据
    collector.cache_step(
        camera_poses=camera_poses,
        joint_angles=joint_angles
    )

# 在 episode 结束时
def on_task_complete(self, success):
    final_positions = self.robot.get_joint_positions()
    collector.write_cached_data(final_positions)
```

#### 方法二：使用工厂函数

```python
from data_collectors.collector_factory import create_collector_from_cfg

collector = create_collector_from_cfg(self.cfg, save_dir='outputs/data')
```

## API 参考

### ParquetFormatCollector

#### 初始化

```python
ParquetFormatCollector(
    save_dir: str,              # 保存目录
    camera_configs: List[dict], # 相机配置列表
    chunk_size: int = 1000,     # 每个 chunk 的 episode 数量
    max_episodes: int = 100,    # 最大 episode 数量
    save_videos: bool = True,   # 是否同时保存视频
    video_config: dict = None   # 视频配置
)
```

#### 主要方法

##### set_camera_intrinsics(intrinsics)

设置相机内参（每个 episode 开始时调用一次）

**参数:**
- `intrinsics` (Dict[str, np.ndarray]): {camera_name: intrinsic_matrix (3,3)}

**示例:**
```python
intrinsic = get_camera_intrinsic(camera)
collector.set_camera_intrinsics({'observation': intrinsic})
```

##### cache_step(camera_poses, joint_angles, camera_images=None, language_instruction=None)

缓存每步数据

**参数:**
- `camera_poses` (Dict[str, np.ndarray]): {camera_name: pose_matrix (4,4)}
- `joint_angles` (np.ndarray): 机器人关节角度
- `camera_images` (Dict[str, np.ndarray], optional): 相机图像
- `language_instruction` (str, optional): 语言指令

**示例:**
```python
collector.cache_step(
    camera_poses={'observation': pose_matrix},
    joint_angles=joint_angles
)
```

##### write_cached_data(final_joint_positions)

写入 episode 数据

**参数:**
- `final_joint_positions` (np.ndarray): 最终关节位置

### 相机工具函数

位于 `utils/camera_utils.py`

#### get_camera_intrinsic(camera)

获取相机内参矩阵 (3x3)

**返回:**
```python
[[fx, 0,  cx],
 [0,  fy, cy],
 [0,  0,  1 ]]
```

#### get_camera_extrinsic(camera, robot)

获取相机外参矩阵 (4x4)

**返回:**
- 相机相对于机器人基座的变换矩阵

#### get_camera_trajectory_matrix(camera)

获取相机世界坐标位姿矩阵 (4x4)

**返回:**
- 相机在世界坐标系中的齐次变换矩阵

#### pose_to_matrix(position, quaternion)

将位置和四元数转换为4x4齐次变换矩阵

**参数:**
- `position`: [x, y, z]
- `quaternion`: [w, x, y, z] (Isaac Sim格式)

## 读取数据

### 基本读取（LeRobot 标准格式）

```python
import pandas as pd
import numpy as np

# 读取 Parquet 文件
df = pd.read_parquet('outputs/parquet_data/data/chunk-000/episode_000000.parquet')

# 查看基本信息
print(f"Shape: {df.shape}")
print(f"Columns: {df.columns.tolist()}")
print(df.head())
```

### 提取相机内参（仅在第一帧）

```python
# 提取内参矩阵（第一帧）
intrinsic = df['observation.observation.camera_intrinsic'].iloc[0]

print("Camera Intrinsic (3x3):")
print(np.array(intrinsic))
```

### 提取相机外参（仅在第一帧）

```python
# 提取外参矩阵（第一帧）
extrinsic = df['observation.observation.camera_extrinsic'].iloc[0]

print("Camera Extrinsic (4x4):")
print(np.array(extrinsic))
```

### 提取相机轨迹（action字段）

```python
# 提取相机轨迹（存储在 action 字段中）
trajectory = df['action'].tolist()  # List of 4x4 matrices

print(f"Trajectory length: {len(trajectory)}")  # N 个时间步
print("First pose (4x4):")
print(np.array(trajectory[0]))
```

### 提取机器人状态

```python
# 提取机器人状态
states = df['observation.state'].tolist()  # List of [x, y, theta]

print(f"Number of states: {len(states)}")
print("First state:", states[0])
```

### 完整示例（LeRobot 格式）

```python
import pandas as pd
import numpy as np

def process_lerobot_parquet(parquet_path):
    """处理 LeRobot 标准格式的 Parquet 数据"""

    # 读取数据
    df = pd.read_parquet(parquet_path)

    # 提取元数据
    episode_index = df['episode_index'].iloc[0]
    num_steps = len(df)

    # 提取相机内参（第一帧）
    intrinsic = np.array(df['observation.observation.camera_intrinsic'].iloc[0])

    # 提取相机外参（第一帧）
    extrinsic = np.array(df['observation.observation.camera_extrinsic'].iloc[0])

    # 提取相机轨迹（action 字段）
    trajectory = [np.array(pose) for pose in df['action'].tolist()]

    # 提取机器人状态
    states = df['observation.state'].tolist()

    return {
        'episode_index': episode_index,
        'num_steps': num_steps,
        'camera_intrinsic': intrinsic,
        'camera_extrinsic': extrinsic,
        'camera_trajectory': trajectory,
        'robot_states': states
    }

# 使用示例
data = process_lerobot_parquet('episode_000000.parquet')
print(f"Episode: {data['episode_index']}")
print(f"Steps: {data['num_steps']}")
print(f"Intrinsic shape: {data['camera_intrinsic'].shape}")
print(f"Trajectory length: {len(data['camera_trajectory'])}")
```

### 转换为其他格式

```python
def convert_to_frame_format(df):
    """将 LeRobot 格式转换为逐帧格式"""

    num_steps = len(df)

    # 提取数据
    intrinsic = np.array(df['observation.observation.camera_intrinsic'].iloc[0])
    extrinsic = np.array(df['observation.observation.camera_extrinsic'].iloc[0])
    trajectory = [np.array(pose) for pose in df['action'].tolist()]

    # 转换为逐帧字典
    frames = []
    for i in range(num_steps):
        frame = {
            'step': i,
            'camera_intrinsic': intrinsic,  # 每帧相同
            'camera_extrinsic': extrinsic,  # 每帧相同
            'camera_pose': trajectory[i],   # 每帧不同
            'robot_state': df['observation.state'].iloc[i],
        }
        frames.append(frame)

    return frames
```

## 与其他收集器对比

| 特性 | HDF5 (DataCollector) | Video (VideoFormatCollector) | Parquet (ParquetFormatCollector) |
|------|---------------------|----------------------------|--------------------------------|
| 图像存储 | HDF5 数据集 | JPG/PNG 文件 | 可选（视频或图像路径） |
| 视频存储 | ❌ | MP4 文件 | 可选 |
| 相机内参 | ❌ | ❌ | ✅ |
| 相机外参 | ❌ | ❌ | ✅ |
| 相机轨迹 | ❌ | ❌ | ✅ |
| 元数据格式 | HDF5 属性 | JSON 文件 | Parquet + JSON |
| 数据访问 | 需要特殊工具 | 通用图像查看器 | Pandas/PyArrow |
| 适用场景 | 快速原型 | 视频分析 | 机器学习训练 |

## 性能优化建议

1. **压缩**: Parquet 使用 Snappy 压缩（默认），平衡压缩率和速度
2. **分块**: 使用 `chunk_size` 参数控制每个 chunk 的 episode 数量
3. **视频**: 如果不需要视频，设置 `save_videos=False` 可以减少存储和计算开销
4. **批处理**: 读取时使用 `pd.read_parquet()` 的 `filters` 参数进行过滤

## 常见问题

### Q: 如何处理多个相机？

A: 相机数据会自动按名称分别存储：

```python
# 两个相机
camera_configs = [
    {'name': 'observation', ...},
    {'name': 'wrist', ...}
]

# 数据列名会包含相机名称
# - observation_camera_intrinsic_0_0
# - observation_camera_trajectory_0_0
# - wrist_camera_intrinsic_0_0
# - wrist_camera_trajectory_0_0
```

### Q: Parquet 文件太大怎么办？

A: 可以：
1. 减小 `chunk_size`
2. 设置 `save_videos=False`
3. 使用更低的图像分辨率

### Q: 如何转换为其他格式？

A: 使用 Pandas 可以轻松转换：

```python
# 转换为 HDF5
df = pd.read_parquet('episode_000000.parquet')
df.to_hdf('episode_000000.h5', 'data')

# 转换为 CSV
df.to_csv('episode_000000.csv')
```

## 示例代码

完整示例请参考：
- `examples/example_parquet_collector_usage.py` - 基本使用示例
- `config/level5_Navigation_parquet.yaml` - 配置文件示例

## 参考资料

- [Pandas Parquet 文档](https://pandas.pydata.org/docs/reference/api/pandas.read_parquet.html)
- [PyArrow 文档](https://arrow.apache.org/docs/python/)
- [Isaac Sim Camera API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html)
