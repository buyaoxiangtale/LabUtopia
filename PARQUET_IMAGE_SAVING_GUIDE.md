# ParquetFormatCollector 图片保存功能详解

## 📸 概述

`ParquetFormatCollector` 支持保存单帧图像，采用**三级目录结构**（环境 → 场景 → 轨迹），每个轨迹完全独立，完全符合 **Open X-Embodiment / BridgeData** 标准格式。

## 📂 目录结构详解

### 三级目录结构（每个轨迹完全独立）

```
outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/
│
├── trajectory_000000/          # 轨迹 0（Episode 0）- 完全独立
│   ├── data/
│   │   └── episode_000000.parquet      # Parquet 元数据（相机内参、外参、轨迹）
│   │
│   ├── videos/
│   │   ├── observation.images.rgb/     # RGB 图像
│   │   │   ├── episode_000000_000.jpg  # 第 0 帧
│   │   │   ├── episode_000000_001.jpg  # 第 1 帧
│   │   │   ├── episode_000000_002.jpg  # 第 2 帧
│   │   │   └── ...
│   │   │
│   │   ├── observation.images.depth/    # 深度图像
│   │   │   ├── episode_000000_000.png  # 第 0 帧
│   │   │   ├── episode_000000_001.png
│   │   │   └── ...
│   │   │
│   │   ├── observation.video.trajectory/  # RGB 视频（可选）
│   │   │   └── episode_000000.mp4
│   │   │
│   │   └── observation.video.depth/       # 深度视频（可选）
│   │       └── episode_000000.mp4
│   │
│   └── meta/
│       └── episode_000000.json           # Episode 0 元数据
│
├── trajectory_000001/          # 轨迹 1（Episode 1）- 完全独立
│   ├── data/
│   │   └── episode_000001.parquet
│   ├── videos/
│   │   ├── observation.images.rgb/
│   │   ├── observation.images.depth/
│   │   ├── observation.video.trajectory/
│   │   └── observation.video.depth/
│   └── meta/
│       └── episode_000001.json           # Episode 1 元数据
│
├── trajectory_000002/          # 轨迹 2（Episode 2）- 完全独立
│   ├── data/
│   ├── videos/
│   └── meta/
│
└── ...                         # 更多轨迹
```

### 目录结构说明

#### **第一级：环境**
```
outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/
```
- 这是场景目录，对应一个特定的环境配置
- 时间戳标识：`2026.01.17/00.06.38`
- 任务名称：`level5_Navigation_parquet`

#### **第二级：场景**
整个目录本身就是一个场景（例如：特定的房间布局）
- 所有轨迹都在同一个场景下
- 每个轨迹代表该场景内的一条不同导航路径

#### **第三级：轨迹（Trajectory）**
```
trajectory_000000/
trajectory_000001/
trajectory_000002/
...
```
- 每个轨迹对应一个 episode
- 轨迹编号从 `000000` 开始
- 每个轨迹包含该 episode 的所有数据（Parquet、图像、视频）

### 关键特点

✅ **三级结构**：环境 → 场景 → 轨迹，清晰组织
✅ **轨迹完全独立**：每个轨迹自包含 data/、videos/、meta/，便于管理和分发
✅ **从 0 开始编号**：每个轨迹的帧索引从 000 开始
✅ **类型分离**：RGB 和深度图分开存储
✅ **标准格式**：RGB 用 JPG，深度用 PNG
✅ **兼容 LeRobot**：目录结构符合机器人学习数据集标准

## 🔧 实现细节

### 1. 图像保存流程（三级目录结构）

```python
# 在 ParquetFormatCollector.write_cached_data() 中：

# 获取轨迹目录（三级结构：环境 → 场景 → 轨迹）
trajectory_dir = self._get_trajectory_dir(self.episode_count)
data_dir = trajectory_dir / "data"
videos_dir = trajectory_dir / "videos"
meta_dir = trajectory_dir / "meta"

# 步骤 1: 保存单帧图像到 trajectory_XXXXXX/videos/
if self.save_images and len(self.temp_images) > 0:
    self._save_images(videos_dir, episode_idx)

# 步骤 2: 保存视频到 trajectory_XXXXXX/videos/（可选）
if self.save_videos and len(self.temp_images) > 0:
    self._save_videos(videos_dir, episode_name)

# 步骤 3: 保存 Parquet 元数据到 trajectory_XXXXXX/data/
self._save_parquet(data_dir, episode_name)

# 步骤 4: 保存 episode 元数据到 trajectory_XXXXXX/meta/
self._save_metadata(meta_dir, episode_name)
```

### 2. _save_images 方法详解

```python
def _save_images(self, videos_dir: Path, episode_idx: int):
    """
    保存单帧图像（三级目录结构）

    目录结构:
        trajectory_XXXXXX/
            videos/
                observation.images.rgb/
                    episode_000000_000.jpg, episode_000000_001.jpg, ...
                observation.images.depth/
                    episode_000000_000.png, episode_000000_001.png, ...
    """
    import cv2

    episode_name = f"episode_{episode_idx:06d}"

    for cam_name_type, frames in self.temp_images.items():
        # 步骤 1: 解析相机名称和图像类型
        # "observation_rgb" -> cam_name="observation", img_type="rgb"
        # "observation_depth" -> cam_name="observation", img_type="depth"
        if '_rgb' in cam_name_type:
            cam_name = cam_name_type.replace('_rgb', '')
            img_type = 'rgb'
        elif '_depth' in cam_name_type:
            cam_name = cam_name_type.replace('_depth', '')
            img_type = 'depth'
        else:
            continue

        # 步骤 2: 创建目录结构（在 trajectory_XXXXXX/videos/ 下）
        image_dir_name = f"observation.images.{img_type}"  # 例如: observation.images.rgb
        image_dir = videos_dir / image_dir_name
        image_dir.mkdir(parents=True, exist_ok=True)

        # 步骤 3: 保存每一帧，命名格式: episode_XXXXXX_YYY.jpg
        for i, frame in enumerate(frames):
            frame_name = f"{episode_name}_{i:03d}"  # 例如: episode_000000_000

            if img_type == 'rgb':
                # RGB 图像处理
                # 转换 [C, H, W] -> [H, W, C]
                frame_hwc = np.transpose(frame, (1, 2, 0))

                # 保存为 JPG（默认）或 PNG
                if self.image_config['rgb_format'] == 'jpg':
                    save_path = image_dir / f"{frame_name}.jpg"
                    cv2.imwrite(
                        str(save_path),
                        cv2.cvtColor(frame_hwc, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, self.image_config['rgb_quality']]
                    )
                else:  # png
                    save_path = image_dir / f"{frame_name}.png"
                    cv2.imwrite(str(save_path), cv2.cvtColor(frame_hwc, cv2.COLOR_RGB2BGR))

            elif img_type == 'depth':
                # 深度图处理
                # 转换 [1, H, W] -> [H, W]
                frame_hw = frame.squeeze()

                if self.image_config['depth_normalize']:
                    # 归一化到 0-255 并应用 colormap
                    frame_vis = cv2.normalize(frame_hw, None, 0, 255, cv2.NORM_MINMAX)
                    frame_vis = frame_vis.astype(np.uint8)

                    # 应用 colormap（jet/viridis/gray）
                    if self.image_config['depth_colormap'] == 'jet':
                        frame_vis = cv2.applyColorMap(frame_vis, cv2.COLORMAP_JET)
                    elif self.image_config['depth_colormap'] == 'viridis':
                        frame_vis = cv2.applyColorMap(frame_vis, cv2.COLORMAP_VIRIDIS)
                    else:
                        frame_vis = cv2.cvtColor(frame_vis, cv2.COLOR_GRAY2BGR)
                else:
                    # 保存原始深度值（转换为毫米，uint16）
                    frame_vis = (frame_hw * 1000).astype(np.uint16)
                    frame_vis = cv2.cvtColor(frame_vis, cv2.COLOR_GRAY2BGR)

                # 保存为 PNG
                save_path = image_dir / f"{frame_idx}.png"
                cv2.imwrite(str(save_path), frame_vis)
```

### 3. 图像缓存机制

```python
# 在 cache_step() 中缓存图像：

def cache_step(
    self,
    camera_images: Dict[str, np.ndarray],  # 例如: {"observation_rgb": [3,256,256], "observation_depth": [1,256,256]}
    joint_angles: np.ndarray,
    ...
):
    # 缓存图像（如果提供且需要保存图片或视频）
    if (self.save_images or self.save_videos) and camera_images is not None:
        for cam_name, image in camera_images.items():
            if cam_name not in self.temp_images:
                self.temp_images[cam_name] = []
            self.temp_images[cam_name].append(image)  # 每一帧追加到列表

    # 同时也缓存其他数据...
    self.temp_poses.append(joint_angles)
    self.temp_trajectory.append(camera_poses)
```

## ⚙️ 配置说明

### 完整配置示例

```yaml
dataset:
  type: "parquet_format"
  chunk_size: 1000

  # 是否保存图像和视频
  save_images: true   # 保存单帧图像
  save_videos: true   # 保存视频

  # 视频配置（如果 save_videos=true）
  video:
    rgb_codec: "mp4v"
    rgb_fps: 30
    depth_codec: "mp4v"
    depth_fps: 30
    depth_colormap: "jet"

  # 图像配置（如果 save_images=true）
  image:
    # RGB 图像配置
    rgb_format: "jpg"      # jpg 或 png
    rgb_quality: 95        # JPEG 质量 (1-100，越高越好)

    # 深度图配置
    depth_format: "png"    # png（推荐，无损）
    depth_normalize: true  # 归一化深度图到 0-255
    depth_colormap: "jet"  # 彩色映射: jet/viridis/gray
```

### 配置参数说明

#### RGB 图像配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `rgb_format` | str | `"jpg"` | 图像格式：`"jpg"` 或 `"png"` |
| `rgb_quality` | int | `95` | JPEG 质量（1-100），仅当 `rgb_format="jpg"` 时有效 |

**建议**：
- 使用 `"jpg"` 格式，质量设置为 `90-95`：文件小，质量好
- 使用 `"png"` 格式：无损，但文件较大

#### 深度图配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `depth_format` | str | `"png"` | 固定为 `"png"`（深度图使用PNG格式） |
| `depth_normalize` | bool | `true` | 是否归一化到 0-255 |
| `depth_colormap` | str | `"jet"` | 彩色映射：`"jet"` / `"viridis"` / `"gray"` |

**说明**：
- `depth_normalize=true`：应用彩色映射，便于可视化
- `depth_normalize=false`：保存原始深度值（转换为毫米，uint16）

## 🎯 使用示例

### 基本使用

```bash
# 运行数据收集（同时保存图像和视频）
python main.py --config-name level5_Navigation_parquet
```

### 只保存图像，不保存视频

修改配置文件：
```yaml
dataset:
  save_images: true
  save_videos: false   # 不保存视频
```

### 调整图像质量

```yaml
dataset:
  image:
    rgb_format: "jpg"
    rgb_quality: 90        # 降低质量减小文件
    depth_colormap: "viridis"  # 使用不同的颜色映射
```

### 查看保存的图像

```python
from pathlib import Path
import cv2
import matplotlib.pyplot as plt

# 指定图像路径（三级目录结构）
rgb_path = "outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/videos/observation.images.rgb/episode_000000_000.jpg"
depth_path = "outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/videos/observation.images.depth/episode_000000_000.png"

# 读取图像
rgb_img = cv2.imread(rgb_path)
rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)

depth_img = cv2.imread(depth_path)
depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2RGB)

# 显示
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].imshow(rgb_img)
axes[0].set_title("RGB Image")
axes[0].axis('off')

axes[1].imshow(depth_img)
axes[1].set_title("Depth Image (Jet Colormap)")
axes[1].axis('off')

plt.tight_layout()
plt.show()
```

### 批量处理图像

```python
from pathlib import Path
import cv2
import numpy as np

# 读取轨迹的所有帧（三级目录结构）
def load_trajectory_images(base_dir, trajectory_idx=0):
    """
    读取一个轨迹的所有图像

    Args:
        base_dir: 基础目录（场景目录）
        trajectory_idx: 轨迹索引

    Returns:
        rgb_images: List[np.ndarray] - RGB 图像列表
        depth_images: List[np.ndarray] - 深度图列表
    """
    trajectory_name = f"trajectory_{trajectory_idx:06d}"
    episode_name = f"episode_{trajectory_idx:06d}"

    # RGB 图像（在 trajectory_XXXXXX/videos/observation.images.rgb/ 下）
    rgb_dir = Path(base_dir) / trajectory_name / "videos" / "observation.images.rgb"
    rgb_files = sorted(rgb_dir.glob(f"{episode_name}_*.jpg")) if rgb_dir.exists() else []

    # 深度图（在 trajectory_XXXXXX/videos/observation.images.depth/ 下）
    depth_dir = Path(base_dir) / trajectory_name / "videos" / "observation.images.depth"
    depth_files = sorted(depth_dir.glob(f"{episode_name}_*.png")) if depth_dir.exists() else []

    # 读取图像
    rgb_images = [cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2RGB) for f in rgb_files]
    depth_images = [cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2RGB) for f in depth_files]

    return rgb_images, depth_images

# 使用示例
base_dir = "outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet"
rgb_imgs, depth_imgs = load_trajectory_images(base_dir, trajectory_idx=0)

print(f"Trajectory 0: {len(rgb_imgs)} RGB images, {len(depth_imgs)} depth images")
```

## 📊 存储空间估算

### 单帧图像大小

| 类型 | 格式 | 分辨率 | 大小（约） |
|------|------|--------|-----------|
| RGB | JPG (质量95) | 256x256 | ~15 KB |
| RGB | PNG | 256x256 | ~50 KB |
| Depth | PNG | 256x256 | ~10 KB |

### 轨迹存储空间

假设一个轨迹（Trajectory）有 **1000 帧**：

| 配置 | RGB 大小 | Depth 大小 | 总计 |
|------|----------|-----------|------|
| JPG + PNG | 15 MB | 10 MB | **25 MB/trajectory** |
| PNG + PNG | 50 MB | 10 MB | **60 MB/trajectory** |

### 100 轨迹存储

| 配置 | 总大小 |
|------|--------|
| JPG + PNG | **2.5 GB** |
| PNG + PNG | **6 GB** |

## 🆚 与 VideoFormatCollector 的对比

| 特性 | VideoFormatCollector | ParquetFormatCollector |
|------|---------------------|----------------------|
| 图像目录结构 | ✅ 标准 | ✅ 标准（相同） |
| 视频保存 | ✅ 支持 | ✅ 支持 |
| Parquet 元数据 | ❌ | ✅ **支持（相机内参/外参/轨迹）** |
| LeRobot 兼容 | 部分 | **完全兼容** |

## ✅ 验证安装

运行数据收集后，检查目录结构：

```bash
# 检查三级目录结构（只有 trajectory 目录，没有全局 meta/）
ls outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/

# 预期输出：
# trajectory_000000/  trajectory_000001/  trajectory_000002/  ...

# 检查轨迹 0 的完整结构
ls outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/

# 预期输出：
# data/  meta/  videos/

# 检查图像目录
ls outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/videos/

# 预期输出：
# observation.images.rgb/  observation.images.depth/

# 检查元数据目录
ls outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/meta/

# 预期输出：
# episode_000000.json

# 检查 trajectory 0 的 RGB 图像
ls outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/videos/observation.images.rgb/

# 预期输出：
# episode_000000_000.jpg  episode_000000_001.jpg  episode_000000_002.jpg  ...

# 统计文件数量
find outputs/collect/2026.01.17/00.06.38_level5_Navigation_parquet/trajectory_000000/videos/observation.images.rgb/ | wc -l
```

## 🎉 总结

现在 `ParquetFormatCollector` 完全支持：
- ✅ **三级目录结构**：环境 → 场景 → 轨迹，清晰组织
- ✅ **轨迹完全独立**：每个轨迹自包含 data/、videos/、meta/，便于管理和分发
- ✅ **标准图像保存**：符合 Open X-Embodiment / BridgeData 格式
- ✅ **类型分离**：RGB 和深度图分开存储
- ✅ **灵活配置**：可调整格式、质量、colormap
- ✅ **完整元数据**：同时保存 Parquet 文件（含相机内参/外参/轨迹）
- ✅ **LeRobot 兼容**：完全兼容 LeRobot 标准格式
