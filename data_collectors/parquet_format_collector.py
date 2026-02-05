#!/usr/bin/env python3
"""
Parquet格式数据收集器
支持保存相机内参、外参、轨迹等元数据到Parquet格式

数据格式（三级目录结构：环境 → 场景 → 轨迹，兼容 LeRobot 格式）:
    outputs/collect/YYYY.MM.DD/HH.MM.SS_task_name/
    ├── trajectory_000000/          # 轨迹 0（每个轨迹完全独立）
    │   ├── data/
    │   │   └── chunk-000/          # chunk 子目录
    │   │       ├── episode_000000.parquet
    │   │       └── path.ply        # 点云文件（用户自行生成）
    │   └── videos/
    │       └── chunk-000/          # chunk 子目录
    │           ├── observation.images.rgb/
    │           │   ├── 0.jpg       # 简化命名（从 0 开始）
    │           │   ├── 1.jpg
    │           │   └── ...
    │           └── observation.images.depth/
    │               ├── 0.png
    │               ├── 1.png
    │               └── ...
    ├── trajectory_000001/          # 轨迹 1
    │   ├── data/
    │   │   └── chunk-000/
    │   └── videos/
    │       └── chunk-000/
    └── ...
"""

import os
import json
import numpy as np
import pandas as pd
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime


class ParquetFormatCollector:
    """
    Parquet格式数据收集器（兼容 LeRobot / InternNav 格式）

    目录结构：
        outputs/collect/YYYY.MM.DD/HH.MM.SS_task_name/
        ├── trajectory_000000/
        │   ├── data/
        │   │   └── chunk-000/          # chunk 子目录
        │   │       ├── episode_000000.parquet
        │   │       └── path.ply        # 点云文件（用户自行生成）
        │   └── videos/
        │       └── chunk-000/          # chunk 子目录
        │           ├── observation.images.rgb/
        │           │   ├── 0.jpg       # 简化命名
        │           │   ├── 1.jpg
        │           │   └── ...
        │           └── observation.images.depth/
        │               ├── 0.png
        │               ├── 1.png
        │               └── ...

    特点：
    - 兼容 LeRobot / InternNav 数据集格式
    - 使用 chunk-000 子目录结构
    - 简化图像命名（从 0 开始：0.jpg, 1.jpg, ...）
    - 保存相机内参矩阵 (3x3)
    - 保存相机外参矩阵 (4x4)
    - 保存相机轨迹 (N, 4, 4)
    - LeRobot 标准格式兼容
    """

    def __init__(
        self,
        save_dir: str,
        camera_configs: List[dict],
        chunk_size: int = 1000,
        max_episodes: int = 100,
        save_videos: bool = True,
        save_images: bool = True,
        video_config: Optional[dict] = None,
        image_config: Optional[dict] = None,
        fixed_camera_extrinsic: Optional[np.ndarray] = None
    ):
        """
        初始化Parquet格式数据收集器

        Args:
            save_dir: 根目录（场景目录）
            camera_configs: 相机配置列表
                [{"name": "observation", "image_type": "rgb+depth"}, ...]
            chunk_size: 每个 chunk 的 episode 数量（保留参数，暂不使用）
            max_episodes: 最大 episode 数量
            save_videos: 是否同时保存视频（可选）
            save_images: 是否保存单帧图像
            video_config: 视频配置（如果save_videos=True）
            image_config: 图像配置（如果save_images=True）
            fixed_camera_extrinsic: 固定的相机外参矩阵（4x4），相机相对于机器人的位姿
                如果提供，将使用此固定值作为所有相机的外参
                默认值（Ridgeback机器人）:
                    [[ 0,  0, -1,  1.1],     # 相机在机器人前方 1.1m
                     [-1,  0,  0,  0.0],     # 相机朝向机器人 -X 方向
                     [ 0,  1,  0,  1.0],     # 相机高度 1.0m
                     [ 0,  0,  0,  1.]]
        """
        self.save_dir = Path(save_dir)
        self.camera_configs = camera_configs
        self.chunk_size = chunk_size
        self.max_episodes = max_episodes
        self.save_videos = save_videos
        self.save_images = save_images

        # 固定相机外参（相机相对于机器人的位姿）
        if fixed_camera_extrinsic is not None:
            self.fixed_camera_extrinsic = np.array(fixed_camera_extrinsic)
            print(f"✓ 使用固定相机外参: \n{self.fixed_camera_extrinsic}")
        else:
            # 默认值：Ridgeback机器人的相机外参
            self.fixed_camera_extrinsic = np.array([
                [ 0,  0, -1,  1.1],
                [-1,  0,  0,  0.0],
                [ 0,  1,  0,  1.0],
                [ 0,  0,  0,  1. ]
            ])
            print(f"✓ 使用默认相机外参（Ridgeback）: \n{self.fixed_camera_extrinsic}")

        # 视频配置
        self.video_config = video_config or {
            'rgb_codec': 'mp4v',
            'rgb_fps': 30,
            'depth_codec': 'mp4v',
            'depth_fps': 30,
            # 深度视频可视化方式：
            # - depth_colormap: jet/viridis/gray
            # - depth_grayscale: True 时强制灰度（覆盖 depth_colormap）
        }

        # 图像配置（新增）
        self.image_config = image_config or {
            'rgb_format': 'jpg',       # jpg 或 png
            'rgb_quality': 95,         # JPEG 质量 (1-100)
            'depth_format': 'png',     # 深度图使用 PNG
            'depth_normalize': True,   # 是否归一化深度图到 0-255
            # 深度图可视化方式：
            # - depth_colormap: jet/viridis/gray
            # - depth_grayscale: True 时强制灰度（覆盖 depth_colormap）
        }

        # 创建目录结构
        self.current_chunk_idx = 0
        self.episode_count = 0
        self._setup_directories()

        # 缓存
        self.temp_intrinsics = {}  # {camera_name: intrinsic_matrix (3,3)}
        self.temp_trajectory = []  # [{camera_name: pose_matrix (4,4)}, ...]
        self.temp_poses = []  # [joint_angle_array, ...]
        self.temp_actions = []
        self.temp_images = {}  # {camera_name_type: [frames]} (用于保存图片和视频)
        self.temp_language_instruction = None

        print(f"✓ ParquetFormatCollector initialized")
        print(f"  Save dir: {self.save_dir}")
        print(f"  Chunk size: {self.chunk_size}")
        print(f"  Save videos: {self.save_videos}")
        print(f"  Save images: {self.save_images}")  # 新增

    def _setup_directories(self):
        """创建目录结构（三级：环境→场景→轨迹）"""
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # 注意：不再创建全局的 data/、videos/、meta/ 目录
        # 改为在每个轨迹目录内创建 data/、videos/、meta/ 子目录

    def _get_trajectory_dir(self, episode_idx: int) -> Path:
        """
        获取轨迹目录（兼容 LeRobot 格式，使用 chunk-000 子目录）

        Args:
            episode_idx: episode 索引

        Returns:
            tuple: (trajectory_dir, data_chunk_dir, videos_chunk_dir)
                - trajectory_dir: 轨迹根目录，例如: trajectory_000000/
                - data_chunk_dir: data/chunk-000/ 目录
                - videos_chunk_dir: videos/chunk-000/ 目录（如果需要）
        """
        trajectory_name = f"trajectory_{episode_idx:06d}"
        trajectory_dir = self.save_dir / trajectory_name
        trajectory_dir.mkdir(parents=True, exist_ok=True)

        # 创建 data/chunk-000/ 子目录（LeRobot 标准格式）
        data_chunk_dir = trajectory_dir / "data" / "chunk-000"
        data_chunk_dir.mkdir(parents=True, exist_ok=True)

        # 创建 videos/chunk-000/ 子目录（LeRobot 标准格式）
        videos_chunk_dir = None
        if self.save_videos or self.save_images:
            videos_chunk_dir = trajectory_dir / "videos" / "chunk-000"
            videos_chunk_dir.mkdir(parents=True, exist_ok=True)

        return trajectory_dir, data_chunk_dir, videos_chunk_dir

    def set_camera_intrinsics(self, intrinsics: Dict[str, np.ndarray]):
        """
        设置相机内参（每个episode开始时调用一次）

        Args:
            intrinsics: {camera_name: intrinsic_matrix (3,3)}
        """
        self.temp_intrinsics = intrinsics

    def cache_step(
        self,
        camera_images: Dict[str, np.ndarray],
        joint_angles: np.ndarray,
        language_instruction: Optional[str] = None,
        waypoints: Optional[np.ndarray] = None,
        base_pose: Optional[np.ndarray] = None,
        camera_poses: Optional[Dict[str, np.ndarray]] = None
    ):
        """
        缓存每步数据

        Args:
            camera_images: {camera_name_type: image_array} 图像数据
            joint_angles: 机器人关节角度
            language_instruction: 语言指令（可选）
            waypoints: A*路径点（可选，不保存到Parquet）
            base_pose: 机器人基座姿态（可选）
            camera_poses: {camera_name: pose_matrix (4,4)} 相机世界坐标位姿（可选）
                      如果不提供，将从base_pose推算
        """
        # 保存语言指令
        if language_instruction is not None:
            self.temp_language_instruction = language_instruction

        # 如果提供了 camera_poses，直接使用
        # 否则，如果提供了 base_pose，可以用于推算相机位姿
        if camera_poses is not None:
            # 使用提供的相机位姿
            self.temp_trajectory.append(camera_poses)
        elif base_pose is not None:
            # 这里暂时不处理自动推算，因为需要相机实例
            # 用户应该在外部获取相机位姿后传入
            pass

        # 缓存图像（如果提供且需要保存图片或视频）
        if (self.save_images or self.save_videos) and camera_images is not None:
            for cam_name, image in camera_images.items():
                if cam_name not in self.temp_images:
                    self.temp_images[cam_name] = []
                self.temp_images[cam_name].append(image)

        # 缓存关节角度
        self.temp_poses.append(joint_angles)

        # waypoints 和 base_pose 暂时不保存到 Parquet
        # 如果需要，可以添加额外的存储逻辑

    def write_cached_data(self, final_joint_positions: np.ndarray):
        """
        写入一个 episode 的数据（三级目录结构）

        Args:
            final_joint_positions: 最终关节位置
        """
        if self.episode_count >= self.max_episodes:
            self.close()
            return

        if len(self.temp_poses) == 0:
            print("⚠️  No data to write")
            return

        # 构造 actions
        actions = self.temp_poses[1:] + [final_joint_positions]

        # 获取轨迹目录（兼容 LeRobot 格式，使用 chunk-000 子目录）
        trajectory_dir, data_chunk_dir, videos_chunk_dir = self._get_trajectory_dir(self.episode_count)

        episode_idx = self.episode_count
        episode_name = f"episode_{episode_idx:06d}"

        print(f"\n{'='*70}")
        print(f"Writing episode {episode_idx}")
        print(f"  Trajectory: {trajectory_dir.name}")
        print(f"  Data chunk: {data_chunk_dir.relative_to(trajectory_dir)}")
        if videos_chunk_dir:
            print(f"  Videos chunk: {videos_chunk_dir.relative_to(trajectory_dir)}")
        print(f"  Frames: {len(self.temp_poses)}")
        print(f"  Cameras: {list(self.temp_intrinsics.keys())}")

        # 1. 保存单帧图像（如果启用）- 保存到 trajectory_XXXXXX/videos/chunk-000/
        if self.save_images and len(self.temp_images) > 0 and videos_chunk_dir:
            self._save_images(videos_chunk_dir, episode_idx)

        # 2. 保存视频（如果启用）- 保存到 trajectory_XXXXXX/videos/chunk-000/
        if self.save_videos and len(self.temp_images) > 0 and videos_chunk_dir:
            self._save_videos(videos_chunk_dir, episode_name)

        # 3. 保存 Parquet 文件 - 保存到 trajectory_XXXXXX/data/chunk-000/
        self._save_parquet(data_chunk_dir, episode_name)

        # 4. path.ply 点云文件由用户自行生成
        # 提示用户在这里生成 path.ply 文件
        # 示例代码：
        # import open3d as o3d
        # path_pcd = create_path_point_cloud(trajectory_data)
        # o3d.io.write_point_cloud(data_chunk_dir / "path.ply", path_pcd)
        print(f"  ℹ️  Note: path.ply should be generated by user at {data_chunk_dir}/path.ply")

        print(f"✓ Episode {episode_idx} saved to {trajectory_dir.name}/")

        # 5. 清空缓存
        self._clear_cache()

        self.episode_count += 1

    def _save_images(self, videos_chunk_dir: Path, episode_idx: int):
        """
        保存单帧图像（LeRobot 标准格式，简化命名）

        目录结构:
            trajectory_XXXXXX/
                videos/
                    chunk-000/
                        observation.images.rgb/
                            0.jpg, 1.jpg, 2.jpg, ...
                        observation.images.depth/
                            0.png, 1.png, 2.png, ...

        Args:
            videos_chunk_dir: trajectory_XXXXXX/videos/chunk-000/ 目录路径
            episode_idx: episode 索引
        """
        import cv2

        # 仅在一个 episode 内打印一次“uint8 depth 无法保存为 u16”的提示
        warned_u8_depth = False

        for cam_name_type, frames in self.temp_images.items():
            # 解析相机名称和类型
            # 例如: "observation_rgb" -> name="observation", type="rgb"
            if '_rgb' in cam_name_type:
                cam_name = cam_name_type.replace('_rgb', '')
                img_type = 'rgb'
            elif '_depth' in cam_name_type:
                cam_name = cam_name_type.replace('_depth', '')
                img_type = 'depth'
            else:
                continue

            # 图像保存目录 - 在 trajectory_XXXXXX/videos/chunk-000/ 下
            image_dir_name = f"observation.images.{img_type}"
            image_dir = videos_chunk_dir / image_dir_name
            image_dir.mkdir(parents=True, exist_ok=True)

            # 保存每一帧，命名格式: 0.jpg, 1.jpg, 2.jpg, ... （简化命名，兼容 LeRobot）
            for i, frame in enumerate(frames):
                frame_idx = i  # 从 0 开始：0, 1, 2, ...

                if img_type == 'rgb':
                    # 转换 [C, H, W] -> [H, W, C]
                    frame_hwc = np.transpose(frame, (1, 2, 0))

                    if self.image_config['rgb_format'] == 'jpg':
                        save_path = image_dir / f"{frame_idx}.jpg"
                        cv2.imwrite(
                            str(save_path),
                            cv2.cvtColor(frame_hwc, cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY, self.image_config['rgb_quality']]
                        )
                    else:  # png
                        save_path = image_dir / f"{frame_idx}.png"
                        cv2.imwrite(
                            str(save_path),
                            cv2.cvtColor(frame_hwc, cv2.COLOR_RGB2BGR)
                        )

                elif img_type == 'depth':
                    """
                    深度图保存策略（修复“全蓝/全黑”）：
                    - 如果输入是 float（通常单位：米），用固定范围 [vis_min_m, vis_max_m] 做可视化映射
                      （避免 per-frame min/max 造成 max==min -> 全0）
                    - 如果输入已经是 uint8（0..255），则认为上游已做过可视化映射，直接上色/保存
                    - depth_normalize=True：保存“可视化深度图”(默认 jet)
                    - depth_normalize=False：保存“原始深度值”16-bit PNG（毫米）
                    """
                    frame_hw = frame.squeeze()

                    # 仅打印每个 episode 第一帧的统计信息（便于排查）
                    if frame_idx == 0:
                        try:
                            finite = np.isfinite(frame_hw)
                            msg = (
                                f"  [Depth] dtype={frame_hw.dtype}, "
                                f"min={float(np.nanmin(frame_hw[finite])) if finite.any() else 'NA'}, "
                                f"max={float(np.nanmax(frame_hw[finite])) if finite.any() else 'NA'}, "
                                f"mean={float(np.nanmean(frame_hw[finite])) if finite.any() else 'NA'}"
                            )
                            print(msg)
                        except Exception:
                            pass

                    save_path = image_dir / f"{frame_idx}.png"

                    # 仅保存“单通道 uint16 深度 PNG”：
                    # - 约定：float 输入为“米”
                    # - 若输入已经是 uint8（通常是伪彩/灰度可视化），无法无损还原物理深度，直接跳过
                    if frame_hw.dtype == np.uint8:
                        if not warned_u8_depth:
                            print("  ⚠️  [Depth] Input depth is uint8 (already visualized). "
                                  "Skip saving u16 depth PNG. Please ensure camera.get_depth() "
                                  "returns float meters for raw depth saving.")
                            warned_u8_depth = True
                        continue

                    # ✅ 将米制 depth 转为 uint16（单通道）
                    # - depth_u16 = round(depth_m * depth_u16_scale)
                    # - 默认 depth_u16_scale=10000 -> 可表示 0~6.5535m（65535/10000）
                    # - < min_m 或无效值（inf/nan/<=0）置 0（黑）
                    # - > max_m 饱和到 65535（白）
                    depth_m = frame_hw.astype(np.float32)
                    depth_u16 = np.zeros(depth_m.shape, dtype=np.uint16)

                    scale = float(self.image_config.get("depth_u16_scale", 10000.0))
                    min_m = float(self.image_config.get("depth_u16_min_m", 0.1))
                    max_m = float(self.image_config.get("depth_u16_max_m", 65535.0 / scale))

                    valid = np.isfinite(depth_m) & (depth_m > 0)
                    if valid.any():
                        depth_clip = np.clip(depth_m[valid], 0.0, max_m)
                        depth_scaled = np.rint(depth_clip * scale).astype(np.uint16)
                        depth_u16[valid] = depth_scaled

                        too_near = valid & (depth_m < min_m)
                        depth_u16[too_near] = np.uint16(0)

                        too_far = valid & (depth_m > max_m)
                        depth_u16[too_far] = np.uint16(65535)

                    from PIL import Image
                    Image.fromarray(depth_u16, mode="I;16").save(str(save_path))

        print(f"  ✓ Images saved to {videos_chunk_dir.name}/")

    def _save_parquet(self, chunk_dir: Path, episode_name: str):
        """
        保存 Parquet 文件（LeRobot 标准格式，修复列名和 action 数据）

        修复内容：
        1. 列名从 observation.observation.camera_intrinsic 改为 observation.camera_intrinsic
        2. Action 从单位矩阵改为真实的相机轨迹
        """
        num_steps = len(self.temp_poses)

        # 准备基础数据 - 使用 LeRobot 标准列名
        data = {
            'episode_index': [self.episode_count] * num_steps,
            'timestamp': [datetime.now().isoformat()] * num_steps,
            # observation.state: 机器人状态（可选）
            'observation.state': self.temp_poses,  # list of arrays
            # action: 相机轨迹 (N, 4, 4) - 每个时间步的相机位姿矩阵
            'action': [],
            # 任务信息
            'task': [self.temp_language_instruction] * num_steps if self.temp_language_instruction else [''] * num_steps,
        }

        # 添加相机数据（LeRobot 格式）
        for cam_cfg in self.camera_configs:
            cam_name = cam_cfg['name']

            # 检查是否有内参数据
            if cam_name not in self.temp_intrinsics:
                print(f"  ⚠️  Warning: No intrinsic data for camera '{cam_name}', skipping...")
                continue

            # 内参 (3x3) - 仅在第一帧存储
            intrinsic = self.temp_intrinsics[cam_name]
            # 第一帧存储内参，其他帧为None
            intrinsic_data = [intrinsic.tolist()] + [None] * (num_steps - 1)

            # ✅ 修复：去掉嵌套的 observation
            # 错误：observation.observation.camera_intrinsic
            # 正确：observation.camera_intrinsic
            if cam_name == 'observation':
                data['observation.camera_intrinsic'] = intrinsic_data
            else:
                data[f'observation.{cam_name}.camera_intrinsic'] = intrinsic_data

            # 外参和轨迹
            if len(self.temp_trajectory) > 0 and cam_name in self.temp_trajectory[0]:
                # ✅ 使用固定的相机外参（相机相对于机器人的位姿）
                # 不再使用轨迹的第一帧，而是使用固定的外参矩阵
                extrinsic_data = [self.fixed_camera_extrinsic.tolist()] + [None] * (num_steps - 1)

                # ✅ 修复：去掉嵌套的 observation
                if cam_name == 'observation':
                    data['observation.camera_extrinsic'] = extrinsic_data
                else:
                    data[f'observation.{cam_name}.camera_extrinsic'] = extrinsic_data

                # ✅ 修复：使用真实的相机轨迹，而不是单位矩阵
                # action 是每个时间步的相机位姿矩阵 (N, 4, 4)
                trajectory = [step[cam_name].tolist() for step in self.temp_trajectory]

                # 调整轨迹长度以匹配 num_steps
                if len(trajectory) < num_steps:
                    # 如果轨迹太短，重复最后一个位姿
                    last_pose = trajectory[-1]
                    trajectory = trajectory + [last_pose] * (num_steps - len(trajectory))
                elif len(trajectory) > num_steps:
                    # 如果轨迹太长，截断
                    trajectory = trajectory[:num_steps]

                # 确保 action 长度正确
                if len(data['action']) == 0:
                    # 第一个相机，直接使用轨迹
                    data['action'] = trajectory
                else:
                    # 如果有多个相机，这里只保存第一个相机的轨迹
                    # 或者可以将多个相机的轨迹拼接
                    pass

                print(f"  ✓ Camera '{cam_name}': intrinsic {intrinsic.shape}, trajectory {len(trajectory)} poses")
            else:
                print(f"  ⚠️  Warning: No trajectory data for camera '{cam_name}'")
                # 添加占位符
                intrinsic_placeholder = [np.eye(3).tolist()] + [None] * (num_steps - 1)
                # ✅ 使用固定外参而不是单位矩阵
                extrinsic_placeholder = [self.fixed_camera_extrinsic.tolist()] + [None] * (num_steps - 1)

                if cam_name == 'observation':
                    data['observation.camera_intrinsic'] = intrinsic_placeholder
                    data['observation.camera_extrinsic'] = extrinsic_placeholder
                else:
                    data[f'observation.{cam_name}.camera_intrinsic'] = intrinsic_placeholder
                    data[f'observation.{cam_name}.camera_extrinsic'] = extrinsic_placeholder

                if len(data['action']) == 0:
                    # ⚠️ 仍然使用单位矩阵作为占位符（但应该从轨迹数据获取！）
                    print(f"  ❌ ERROR: No trajectory data, using identity matrix as placeholder!")
                    data['action'] = [np.eye(4).tolist()] * num_steps

        # 创建 DataFrame
        df = pd.DataFrame(data)

        # 保存为 Parquet
        parquet_path = chunk_dir / f"{episode_name}.parquet"
        df.to_parquet(parquet_path, index=False, compression='snappy')

        print(f"  ✓ Parquet saved: {parquet_path}")
        print(f"    Shape: {df.shape}, Size: {parquet_path.stat().st_size / 1024:.2f} KB")
        print(f"    Columns: {list(df.columns)}")

    def _save_videos(self, videos_chunk_dir: Path, episode_name: str):
        """
        保存视频（LeRobot 标准格式）

        目录结构:
            trajectory_XXXXXX/
                videos/
                    chunk-000/
                        observation.video.trajectory/
                            episode_000000.mp4
                        observation.video.depth/
                            episode_000000.mp4

        Args:
            videos_chunk_dir: trajectory_XXXXXX/videos/chunk-000/ 目录路径
            episode_name: episode 名称
        """
        import cv2

        def _ensure_uint8_bgr(img: np.ndarray) -> np.ndarray:
            """
            VideoWriter 仅接受 CV_8U / CV_16U。这里统一转成 uint8 的 BGR 三通道。
            - 若输入是 float 且范围在 [0, 1]，映射到 [0, 255]
            - 若输入是 float 且范围在 [0, 255]，直接 clip
            - 若输入是 uint16，压到 8bit（除以 256）
            """
            if img is None:
                return img

            # 确保是 HWC
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            if img.dtype == np.uint8:
                return img

            if img.dtype == np.uint16:
                img8 = (img / 256).astype(np.uint8)
                if img8.ndim == 2:
                    img8 = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
                return img8

            # float / int 统一处理到 uint8
            img_f = img.astype(np.float32)
            maxv = float(np.nanmax(img_f)) if img_f.size > 0 else 0.0
            if maxv <= 1.0 + 1e-6:
                img_f = img_f * 255.0
            img_f = np.clip(img_f, 0.0, 255.0)
            return img_f.astype(np.uint8)

        for cam_name_type, frames in self.temp_images.items():
            # 解析相机名称和类型
            if '_rgb' in cam_name_type:
                video_type = 'trajectory'
                codec = self.video_config['rgb_codec']
                fps = self.video_config['rgb_fps']
            elif '_depth' in cam_name_type:
                video_type = 'depth'
                codec = self.video_config['depth_codec']
                fps = self.video_config['depth_fps']
            else:
                continue

            # 视频保存目录 - 在 trajectory_XXXXXX/videos/chunk-000/ 下
            video_dir = videos_chunk_dir / f"observation.video.{video_type}"
            video_dir.mkdir(parents=True, exist_ok=True)

            video_path = video_dir / f"{episode_name}.mp4"

            # 获取视频尺寸
            if len(frames) == 0:
                continue

            first_frame = frames[0]
            if len(first_frame.shape) == 3:
                height, width = first_frame.shape[1], first_frame.shape[2]
            else:
                height, width = first_frame.shape[0], first_frame.shape[1]

            # 创建视频写入器
            fourcc = cv2.VideoWriter_fourcc(*codec)
            video_writer = cv2.VideoWriter(str(video_path), fourcc, fps, (width, height))

            # 写入帧
            for i, frame in enumerate(frames):
                if len(frame.shape) == 3:
                    # [C, H, W] -> [H, W, C]
                    frame_bgr = cv2.cvtColor(
                        np.transpose(frame, (1, 2, 0)),
                        cv2.COLOR_RGB2BGR
                    )
                    frame_bgr = _ensure_uint8_bgr(frame_bgr)
                else:
                    # [1, H, W] -> [H, W]
                    frame_hw = frame.squeeze()
                    # 视频可视化范围（米）
                    vis_min_m = float(self.image_config.get('depth_vis_min_m', 0.1))
                    vis_max_m = float(self.image_config.get('depth_vis_max_m', 10.0))

                    if i == 0:
                        try:
                            finite = np.isfinite(frame_hw)
                            msg = (
                                f"  [VideoDepth] dtype={frame_hw.dtype}, "
                                f"min={float(np.nanmin(frame_hw[finite])) if finite.any() else 'NA'}, "
                                f"max={float(np.nanmax(frame_hw[finite])) if finite.any() else 'NA'}"
                            )
                            print(msg)
                        except Exception:
                            pass

                    if frame_hw.dtype == np.uint8:
                        depth_u8 = frame_hw
                    else:
                        valid = np.isfinite(frame_hw) & (frame_hw > 0)
                        depth_vis = frame_hw.copy()
                        depth_vis[~valid] = vis_max_m
                        depth_u8 = ((np.clip(depth_vis, vis_min_m, vis_max_m) - vis_min_m) /
                                    max(vis_max_m - vis_min_m, 1e-6) * 255.0).astype(np.uint8)

                    # 是否强制保存为灰度（不使用伪彩色）
                    if self.video_config.get('depth_grayscale', False) or self.image_config.get('depth_grayscale', False):
                        cmap = 'gray'
                    else:
                        cmap = self.video_config.get('depth_colormap', self.image_config.get('depth_colormap', 'jet'))
                    if cmap == 'viridis':
                        frame_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_VIRIDIS)
                    elif cmap == 'gray':
                        frame_bgr = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2BGR)
                    else:
                        frame_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
                    frame_bgr = _ensure_uint8_bgr(frame_bgr)

                video_writer.write(frame_bgr)

            video_writer.release()

        print(f"  ✓ Videos saved to {videos_chunk_dir.name}/")

    def _clear_cache(self):
        """清空缓存"""
        self.temp_intrinsics = {}
        self.temp_trajectory = []
        self.temp_poses = []
        self.temp_actions = []
        self.temp_images = {}
        self.temp_language_instruction = None

    def clear_cache(self):
        """公开的清空缓存方法（用于放弃当前 episode）"""
        self._clear_cache()

    def close(self):
        """关闭收集器"""
        print(f"\n{'='*70}")
        print(f"ParquetFormatCollector closed")
        print(f"  Total episodes: {self.episode_count}")
        print(f"  Output directory: {self.save_dir}")
        print(f"{'='*70}\n")


def create_parquet_format_collector(
    camera_configs: List[dict],
    save_dir: str,
    max_episodes: int = 100,
    chunk_size: int = 1000,
    save_videos: bool = True,
    video_config: Optional[dict] = None
) -> ParquetFormatCollector:
    """
    创建Parquet格式数据收集器的工厂函数

    Args:
        camera_configs: 相机配置列表
        save_dir: 保存目录
        max_episodes: 最大 episode 数量
        chunk_size: 每个 chunk 的 episode 数量
        save_videos: 是否保存视频
        video_config: 视频配置

    Returns:
        ParquetFormatCollector 实例
    """
    return ParquetFormatCollector(
        save_dir=save_dir,
        camera_configs=camera_configs,
        chunk_size=chunk_size,
        max_episodes=max_episodes,
        save_videos=save_videos,
        video_config=video_config
    )
