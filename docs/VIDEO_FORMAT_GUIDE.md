# 视频格式数据收集器使用指南

本文档说明如何使用视频格式数据收集器，无需修改 BaseTask。

## 📋 目录

- [快速开始](#快速开始)
- [文件说明](#文件说明)
- [配置说明](#配置说明)
- [使用示例](#使用示例)
- [数据集格式](#数据集格式)
- [工具和实用程序](#工具和实用程序)

---

## 🚀 快速开始

### 1. 使用示例配置运行数据收集

```bash
# 使用提供的配置文件
python main.py \
  --config-name=level5_Navigation_video_format \
  mode=collect \
  max_episodes=100
```

### 2. 查看输出结果

```bash
# 输出目录结构
tree outputs/collect/2026.01.06/15.30.00_level5_Navigation_video_format/

# 预期输出:
# ├── videos/
# │   └── chunk-000/
# │       ├── observation.images.observation/
# │       │   ├── 0.jpg, 1.jpg, ...
# │       ├── observation.images.observation_depth/
# │       │   ├── 0.png, 1.png, ...
# │       ├── observation.video.trajectory/
# │       │   ├── episode_000000.mp4, ...
# │       └── observation.video.depth/
# │           ├── episode_000000.mp4, ...
# ├── episode_000000.json
# ├── episode_000001.json
# └── ...
```

### 3. 查看视频

```bash
# 使用 VLC 或其他播放器
vlc outputs/.../videos/chunk-000/observation.video.trajectory/episode_000000.mp4

# 或使用 ffplay
ffplay episode_000000.mp4
```

---

## 📁 文件说明

### 核心文件

| 文件 | 说明 |
|------|------|
| `data_collectors/video_format_collector.py` | 视频格式数据收集器核心实现 |
| `data_collectors/collector_factory.py` | 数据收集器工厂和适配器 |
| `utils/video_dataset_loader.py` | 数据集加载和分析工具 |
| `config/level5_Navigation_video_format.yaml` | 示例配置文件 |
| `examples/video_collection_example.py` | 使用示例代码 |

---

## ⚙️ 配置说明

### YAML 配置文件

```yaml
# config/your_video_format_config.yaml

# 基本配置
name: your_experiment_name
mode: "collect"
max_episodes: 100

# 相机配置
cameras:
  - prim_path: "/World/Ridgebase/base_link/Camera"
    name: "observation"          # 相机名称
    resolution: [256, 256]       # 分辨率
    image_type: "rgb+depth"      # 数据类型：rgb, depth, pointcloud, 或组合

# 数据集配置
dataset:
  type: "video_format"
  chunk_size: 1000              # 每个 chunk 的 episode 数量
  save_images: true             # 保存单帧图像
  save_videos: true             # 保存视频

  # 视频编码配置
  video:
    rgb_codec: "mp4v"           # 编码器：mp4v (H.264), avc1, etc.
    rgb_fps: 30                 # 视频帧率
    rgb_quality: 18             # JPEG 质量 (1-31)
    depth_codec: "mp4v"
    depth_fps: 30
    depth_colormap: "jet"       # jet/viridis/gray

  # 图像保存配置
  image:
    rgb_format: "jpg"           # jpg 或 png
    rgb_quality: 95             # JPEG 质量 (1-100)
    depth_format: "png"
    depth_normalize: true       # 归一化深度图到 0-255

# 数据收集器类型（关键配置）
collector:
  type: "video_format"          # 指定使用视频格式收集器
```

### 相机类型配置

#### RGB 单一类型
```yaml
cameras:
  - name: "front_camera"
    image_type: "rgb"
```

#### RGB + 深度图
```yaml
cameras:
  - name: "front_camera"
    image_type: "rgb+depth"
```

#### 多个相机
```yaml
cameras:
  - name: "observation"
    image_type: "rgb+depth"

  - name: "wrist"
    image_type: "rgb"

  - name: "top"
    image_type: "depth"
```

---

## 💡 使用示例

### 示例 1: 在现有代码中集成

#### 修改前（HDF5 格式）
```python
from data_collectors.data_collector import DataCollector

collector = DataCollector(
    camera_configs=cfg.cameras,
    save_dir=save_dir,
    max_episodes=cfg.max_episodes,
    compression=cfg.collector.compression
)
```

#### 修改后（自动选择格式）
```python
from data_collectors.collector_factory import create_collector_from_cfg

collector = create_collector_from_cfg(cfg, save_dir)
# 根据 cfg.collector.type 自动选择格式！
```

### 示例 2: 加载和使用数据集

```python
from utils.video_dataset_loader import VideoFormatDataset

# 加载数据集
dataset = VideoFormatDataset("outputs/collect/...")

# 查看统计信息
dataset.print_summary()

# 访问单个 episode
episode = dataset[0]
frames = episode['frames']        # [T, H, W, 3]
actions = episode['actions']      # [T, num_joints]

# 获取视频路径
video_path = dataset.get_video_path(0, 'trajectory')
print(f"Video: {video_path}")

# 可视化 episode
dataset.visualize_episode(0, save_path='episode_0.png')
```

### 示例 3: 命令行工具

```bash
# 查看数据集统计
python utils/video_dataset_loader.py /path/to/dataset --action stats

# 可视化 episode
python utils/video_dataset_loader.py /path/to/dataset \
  --action visualize \
  --episode_idx 0 \
  --output episode_0.png

# 从视频提取帧
python utils/video_dataset_loader.py video.mp4 \
  --action extract \
  --output extracted_frames/
```

---

## 📊 数据集格式

### 目录结构

```
dataset/
├── videos/
│   └── chunk-000/                    # 第 1 个 chunk (episodes 0-999)
│       ├── observation.images.{camera_name}/
│       │   ├── 0.jpg                 # RGB 帧
│       │   ├── 1.jpg
│       │   └── ...
│       ├── observation.images.{camera_name}_depth/
│       │   ├── 0.png                 # 深度帧
│       │   ├── 1.png
│       │   └── ...
│       ├── observation.video.trajectory/
│       │   ├── episode_000000.mp4    # RGB 轨迹视频
│       │   ├── episode_000001.mp4
│       │   └── ...
│       └── observation.video.depth/
│           ├── episode_000000.mp4    # 深度视频
│           ├── episode_000001.mp4
│           └── ...
├── chunk-001/                        # 第 2 个 chunk (episodes 1000-1999)
│   └── ...
├── episode_000000.json               # Episode 元数据
├── episode_000001.json
└── ...
```

### Episode 元数据格式

```json
{
  "episode_index": 0,
  "length": 100,
  "language_instruction": "Navigate to target",
  "observations": {
    "image": [
      {"camera_name": "observation"},
      {"camera_name": "observation"},
      ...
    ],
    "state": [
      [0.1, 0.2, ...],  # step 0
      [0.15, 0.25, ...], # step 1
      ...
    ]
  },
  "actions": [
    [0.15, 0.25, ...],  # action for step 0
    [0.2, 0.3, ...],    # action for step 1
    ...
  ],
  "metadata": {
    "timestamp": "2026-01-06T15:30:00",
    "num_cameras": 1,
    "camera_types": ["rgb+depth"]
  }
}
```

---

## 🛠️ 工具和实用程序

### VideoFormatDataset 类

```python
from utils.video_dataset_loader import VideoFormatDataset

dataset = VideoFormatDataset("path/to/dataset")

# 方法
len(dataset)                    # Episode 数量
dataset[0]                      # 获取 episode 0
dataset.get_stats()             # 获取统计信息
dataset.print_summary()         # 打印摘要
dataset.get_video_path(0, 'trajectory')  # 获取视频路径
dataset.visualize_episode(0)    # 可视化 episode
```

### 提取视频帧

```python
from utils.video_dataset_loader import extract_frames_from_video

extract_frames_from_video(
    video_path='episode_000000.mp4',
    output_dir='frames/',
    fps=10  # 提取 10 fps
)
```

---

## 🔧 高级配置

### 调整视频质量

#### 高质量（文件较大）
```yaml
video:
  rgb_codec: "avc1"      # 更好的编码器
  rgb_fps: 60
  rgb_quality: 10        # 更高质量

image:
  rgb_format: "png"      # 无损格式
```

#### 低质量（文件较小）
```yaml
video:
  rgb_codec: "mp4v"
  rgb_fps: 15
  rgb_quality: 25        # 更低质量

image:
  rgb_format: "jpg"
  rgb_quality: 70
```

### 仅保存图像或仅保存视频

```yaml
dataset:
  save_images: true      # 仅保存图像
  save_videos: false

  # 或
  save_images: false     # 仅保存视频
  save_videos: true
```

### Chunk 大小调整

```yaml
dataset:
  chunk_size: 500        # 每个 chunk 500 个 episodes
```

---

## 📈 性能和存储

### 存储空间估算

| 配置 | 单个 Episode | 100 Episodes | 1000 Episodes |
|------|--------------|--------------|---------------|
| 仅 RGB 视频 (30fps) | ~5 MB | ~500 MB | ~5 GB |
| RGB + 深度视频 | ~8 MB | ~800 MB | ~8 GB |
| 视频 + 图像 | ~12 MB | ~1.2 GB | ~12 GB |

### 性能建议

1. **大收集任务** (>1000 episodes):
   - 使用 `chunk_size: 1000` 分块存储
   - 考虑仅保存视频：`save_images: false`

2. **调试和快速迭代**:
   - 减少分辨率：`resolution: [128, 128]`
   - 降低帧率：`rgb_fps: 15`
   - 减少质量：`rgb_quality: 25`

3. **生产数据集**:
   - 使用高质量设置
   - 同时保存图像和视频
   - 考虑使用 PNG 保存 RGB（无损）

---

## ❓ 常见问题

### Q: 如何在现有代码中使用而不修改 BaseTask？

A: 使用 `collector_factory.py` 中的工厂函数：

```python
# 在你的任务初始化代码中
from data_collectors.collector_factory import create_collector_from_cfg

self.data_collector = create_collector_from_cfg(self.cfg, save_dir)
```

### Q: 如何从 HDF5 格式转换为视频格式？

A: 创建转换脚本（见 `scripts/convert_hdf5_to_video.py`）

### Q: 视频播放失败怎么办？

A: 检查编码器支持：
```bash
# 查看支持的编码器
ffmpeg -codecs | grep "Video"

# 如果 mp4v 不可用，尝试其他编码器
rgb_codec: "avc1"  # H.264
rgb_codec: "x264"  # 开源 H.264
```

### Q: 如何加快数据收集速度？

A:
1. 降低视频质量
2. 减少分辨率
3. 仅保存视频（不保存图像）
4. 使用更快的磁盘（SSD）

---

## 📚 更多资源

- 视频格式数据集标准: [Open X-Embodiment](https://robotics-transformer-x.github.io/)
- HDF5 格式收集器: `data_collectors/data_collector.py`
- 相机工具: `utils/camera_utils.py`
- 示例代码: `examples/video_collection_example.py`

---

## 🤝 贡献

如果发现问题或有改进建议，请提交 issue 或 PR。
