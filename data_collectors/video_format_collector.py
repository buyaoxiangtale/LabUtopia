#!/usr/bin/env python3
"""
视频格式数据收集器
支持标准机器人学习数据集格式（Open X-Embodiment / BridgeData）

数据格式:
    dataset/
    ├── videos/
    │   └── chunk-000/
    │       ├── observation.images.rgb/
    │       │   ├── episode_000000/
    │       │   │   ├── 0.jpg, 1.jpg, ...
    │       │   ├── episode_000001/
    │       │   │   ├── 0.jpg, 1.jpg, ...
    │       ├── observation.images.depth/
    │       │   ├── episode_000000/
    │       │   │   ├── 0.png, 1.png, ...
    │       │   ├── episode_000001/
    │       │   │   ├── 0.png, 1.png, ...
    │       ├── observation.video.trajectory/
    │       │   ├── episode_000000.mp4, ...
    │       └── observation.video.depth/
    │           ├── episode_000000.mp4, ...
    ├── episode_000000.json
    └── episode_000001.json
"""

import os
import json
import cv2
import numpy as np
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime


class VideoFormatCollector:
    """
    视频格式数据收集器

    特点：
    - 保存单帧图像（RGB: JPG, Depth: PNG）
    - 生成 MP4 视频（RGB 和深度）
    - 生成 JSON 元数据
    - 自动分块（chunk）管理
    """

    def __init__(
        self,
        save_dir: str,
        camera_configs: List[dict],
        chunk_size: int = 1000,
        max_episodes: int = 100,
        save_images: bool = True,
        save_videos: bool = True,
        video_config: Optional[dict] = None,
        image_config: Optional[dict] = None,
        save_waypoints: bool = False,
        save_base_pose: bool = False,
        waypoints_format: str = "json",
        waypoints_dir: str = "waypoints",
        log_waypoints_stats: bool = True,
        waypoints_stats_interval: int = 10,
        save_metadata: bool = True
    ):
        """
        初始化视频格式数据收集器

        Args:
            save_dir: 根目录
            camera_configs: 相机配置列表
                [{"name": "observation", "image_type": "rgb+depth"}, ...]
            chunk_size: 每个 chunk 的 episode 数量
            max_episodes: 最大 episode 数量
            save_images: 是否保存单帧图像
            save_videos: 是否保存视频
            video_config: 视频配置
            image_config: 图像配置
            save_waypoints: 是否保存 A* 规划的路径点（默认 False，遵循视频格式标准）
            save_base_pose: 是否保存机器人基座全局姿态轨迹
            waypoints_format: 路径点保存格式 ("json"/"h5"/"npy")
            waypoints_dir: 路径点文件保存子目录
            log_waypoints_stats: 是否在控制台输出路径点统计信息
            waypoints_stats_interval: 每多少步输出一次路径点统计
        """
        self.save_dir = Path(save_dir)
        self.camera_configs = camera_configs
        self.chunk_size = chunk_size
        self.max_episodes = max_episodes
        self.save_images = save_images
        self.save_videos = save_videos

        # 路径点相关配置
        self.save_waypoints = save_waypoints
        self.save_base_pose = save_base_pose
        self.waypoints_format = waypoints_format
        self.waypoints_dir = waypoints_dir
        self.log_waypoints_stats = log_waypoints_stats
        self.waypoints_stats_interval = waypoints_stats_interval
        self.save_metadata = save_metadata

        # 默认视频配置
        self.video_config = video_config or {
            'rgb_codec': 'mp4v',       # H.264
            'rgb_fps': 30,
            'rgb_quality': 18,         # JPEG 质量 (1-31, 越小越好)
            'depth_codec': 'mp4v',
            'depth_fps': 30,
            'depth_colormap': 'jet'    # jet/viridis/gray
        }

        # 默认图像配置
        self.image_config = image_config or {
            'rgb_format': 'jpg',       # jpg 或 png
            'rgb_quality': 95,         # JPEG 质量 (1-100)
            'depth_format': 'png',     # 深度图使用 PNG（无损）
            'depth_normalize': True    # 是否归一化到 0-255
        }

        # 创建目录结构
        self.current_chunk_idx = 0
        self.episode_count = 0
        self._setup_directories()

        # 缓存
        self.temp_frames = {}  # {camera_name_type: [frames]}
        self.temp_poses = []
        self.temp_actions = []
        self.temp_language_instruction = None

        print(f"✓ VideoFormatCollector initialized")
        print(f"  Save dir: {self.save_dir}")
        print(f"  Chunk size: {self.chunk_size}")
        print(f"  Save images: {self.save_images}")
        print(f"  Save videos: {self.save_videos}")

    def _setup_directories(self):
        """创建目录结构"""
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.videos_dir = self.save_dir / "videos"
        self.videos_dir.mkdir(parents=True, exist_ok=True)

        # 创建第一个 chunk
        self.current_chunk_dir = self.videos_dir / f"chunk-{self.current_chunk_idx:03d}"
        self.current_chunk_dir.mkdir(parents=True, exist_ok=True)

    def _get_chunk_dir(self, episode_idx: int) -> Path:
        """获取 episode 所属的 chunk 目录"""
        chunk_idx = episode_idx // self.chunk_size
        chunk_dir = self.videos_dir / f"chunk-{chunk_idx:03d}"

        if chunk_dir != self.current_chunk_dir:
            self.current_chunk_dir = chunk_dir
            self.current_chunk_dir.mkdir(parents=True, exist_ok=True)

        return chunk_dir

    def cache_step(
        self,
        camera_images: Dict[str, np.ndarray],
        joint_angles: np.ndarray,
        language_instruction: Optional[str] = None,
        waypoints: Optional[List] = None,
        base_pose: Optional[List] = None
    ):
        """
        缓存每步数据

        Args:
            camera_images: {camera_name_type: image_array}
                例如: {"observation_rgb": [3, 256, 256], "observation_depth": [1, 256, 256]}
            joint_angles: [num_joints]
            language_instruction: 语言指令
            waypoints: 路径点列表（可选，用于兼容性）
            base_pose: 机器人基座姿态（可选，用于兼容性）
        """
        # 保存语言指令
        if language_instruction is not None:
            self.temp_language_instruction = language_instruction

        # 缓存图像帧
        for cam_name, image in camera_images.items():
            if cam_name not in self.temp_frames:
                self.temp_frames[cam_name] = []
            self.temp_frames[cam_name].append(image)

        # 缓存关节角度
        self.temp_poses.append(joint_angles)

        # 处理路径点统计（如果启用）
        if self.log_waypoints_stats and waypoints is not None:
            step_count = len(self.temp_poses)
            if step_count % self.waypoints_stats_interval == 0:
                num_waypoints = len(waypoints) if hasattr(waypoints, '__len__') else 0
                print(f"  [路径点统计] 步 {step_count}: {num_waypoints} 个路径点")

        # 注意：VideoFormatCollector 不实际保存路径点数据（遵循视频格式标准）
        # 如果需要保存路径点，请使用其他收集器或修改此实现

    def write_cached_data(self, final_joint_positions: np.ndarray):
        """
        写入一个 episode 的数据

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

        # 获取 chunk 目录
        chunk_dir = self._get_chunk_dir(self.episode_count)

        episode_idx = self.episode_count
        episode_name = f"episode_{episode_idx:06d}"

        print(f"\n{'='*70}")
        print(f"Writing episode {episode_idx}")
        print(f"  Chunk: chunk-{episode_idx // self.chunk_size:03d}")
        print(f"  Frames: {len(self.temp_poses)}")
        print(f"  Cameras: {list(self.temp_frames.keys())}")

        # 1. 保存单帧图像
        if self.save_images:
            self._save_images(chunk_dir, episode_idx)

        # 2. 保存视频
        if self.save_videos:
            self._save_videos(chunk_dir, episode_name)

        # 3. 保存元数据 JSON（如果启用）
        if self.save_metadata:
            self._save_metadata(episode_name, self.temp_poses, actions)

        print(f"✓ Episode {episode_idx} saved successfully")

        # 4. 清空缓存
        self._clear_cache()

        self.episode_count += 1

    def _save_images(self, chunk_dir: Path, episode_idx: int):
        """保存单帧图像"""
        for cam_name_type, frames in self.temp_frames.items():
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

            # 图像保存目录 - 使用标准格式目录名
            # 标准: observation.images.rgb/episode_000000/ 和 observation.images.depth/episode_000000/
            image_dir_name = f"observation.images.{img_type}"
            episode_name = f"episode_{episode_idx:06d}"

            image_dir = chunk_dir / image_dir_name / episode_name
            image_dir.mkdir(parents=True, exist_ok=True)

            # 每个episode的帧从0开始编号
            for i, frame in enumerate(frames):
                frame_idx = i

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
                    # 转换 [1, H, W] -> [H, W]
                    frame_hw = frame.squeeze()

                    if self.image_config['depth_normalize']:
                        # 归一化到 0-255 用于可视化
                        frame_vis = cv2.normalize(frame_hw, None, 0, 255, cv2.NORM_MINMAX)
                        frame_vis = frame_vis.astype(np.uint8)
                        # 应用 colormap
                        if self.video_config['depth_colormap'] == 'jet':
                            frame_vis = cv2.applyColorMap(frame_vis, cv2.COLORMAP_JET)
                        elif self.video_config['depth_colormap'] == 'viridis':
                            frame_vis = cv2.applyColorMap(frame_vis, cv2.COLORMAP_VIRIDIS)
                        else:
                            frame_vis = cv2.cvtColor(frame_vis, cv2.COLOR_GRAY2BGR)
                    else:
                        # 保存原始深度值（转换为毫米，uint16）
                        frame_vis = (frame_hw * 1000).astype(np.uint16)
                        frame_vis = cv2.cvtColor(frame_vis, cv2.COLOR_GRAY2BGR)

                    save_path = image_dir / f"{frame_idx}.png"
                    cv2.imwrite(str(save_path), frame_vis)

        print(f"  ✓ Images saved to {chunk_dir}")

    def _save_videos(self, chunk_dir: Path, episode_name: str):
        """保存视频"""
        for cam_name_type, frames in self.temp_frames.items():
            # 解析相机名称和类型
            if '_rgb' in cam_name_type:
                cam_name = cam_name_type.replace('_rgb', '')
                video_type = 'trajectory'  # RGB 轨迹视频
                codec = self.video_config['rgb_codec']
                fps = self.video_config['rgb_fps']
            elif '_depth' in cam_name_type:
                cam_name = cam_name_type.replace('_depth', '')
                video_type = 'depth'  # 深度视频
                codec = self.video_config['depth_codec']
                fps = self.video_config['depth_fps']
            else:
                continue

            # 视频保存目录
            video_dir = chunk_dir / f"observation.video.{video_type}"
            video_dir.mkdir(parents=True, exist_ok=True)

            video_path = video_dir / f"{episode_name}.mp4"

            # 获取视频尺寸
            if len(frames) == 0:
                continue

            first_frame = frames[0]
            if len(first_frame.shape) == 3:
                # [C, H, W]
                height, width = first_frame.shape[1], first_frame.shape[2]
            else:
                # [1, H, W] or [H, W]
                if len(first_frame.shape) == 3:
                    height, width = first_frame.shape[0], first_frame.shape[2]
                else:
                    height, width = first_frame.shape[0], first_frame.shape[1]

            # 创建视频写入器
            fourcc = cv2.VideoWriter_fourcc(*codec)
            video_writer = cv2.VideoWriter(
                str(video_path),
                fourcc,
                fps,
                (width, height)
            )

            # 写入帧
            for frame in frames:
                if len(frame.shape) == 3:
                    # [C, H, W] -> [H, W, C]
                    frame_bgr = cv2.cvtColor(
                        np.transpose(frame, (1, 2, 0)),
                        cv2.COLOR_RGB2BGR
                    )
                else:
                    # [1, H, W] -> [H, W]
                    frame_hw = frame.squeeze()

                    # 归一化和 colormap
                    frame_vis = cv2.normalize(frame_hw, None, 0, 255, cv2.NORM_MINMAX)
                    frame_vis = frame_vis.astype(np.uint8)

                    if self.video_config['depth_colormap'] == 'jet':
                        frame_bgr = cv2.applyColorMap(frame_vis, cv2.COLORMAP_JET)
                    elif self.video_config['depth_colormap'] == 'viridis':
                        frame_bgr = cv2.applyColorMap(frame_vis, cv2.COLORMAP_VIRIDIS)
                    else:
                        frame_bgr = cv2.cvtColor(frame_vis, cv2.COLOR_GRAY2BGR)

                video_writer.write(frame_bgr)

            video_writer.release()

        print(f"  ✓ Videos saved to {chunk_dir}")

    def _save_metadata(self, episode_name: str, poses: List[np.ndarray], actions: List[np.ndarray]):
        """保存 episode 元数据"""
        # 转换 numpy 为 list
        poses_list = [p.tolist() if isinstance(p, np.ndarray) else p for p in poses]
        actions_list = [a.tolist() if isinstance(a, np.ndarray) else a for a in actions]

        metadata = {
            'episode_index': self.episode_count,
            'length': len(poses),
            'language_instruction': self.temp_language_instruction,
            'observations': {
                'image': [{'camera_name': 'observation'} for _ in poses],
                'state': poses_list
            },
            'actions': actions_list,
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'num_cameras': len(self.camera_configs),
                'camera_types': [c.get('image_type', 'rgb') for c in self.camera_configs]
            }
        }

        metadata_path = self.save_dir / f"{episode_name}.json"
        with open(metadata_path, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)

        print(f"  ✓ Metadata saved to {metadata_path}")

    def _clear_cache(self):
        """清空缓存"""
        self.temp_frames = {}
        self.temp_poses = []
        self.temp_actions = []
        self.temp_language_instruction = None

    def clear_cache(self):
        """公开的清空缓存方法（用于放弃当前 episode）"""
        self._clear_cache()

    def close(self):
        """关闭收集器"""
        print(f"\n{'='*70}")
        print(f"VideoFormatCollector closed")
        print(f"  Total episodes: {self.episode_count}")
        print(f"  Output directory: {self.save_dir}")
        print(f"{'='*70}\n")


# 为了保持与 DataCollector 相同的接口
def create_video_format_collector(
    camera_configs: List[dict],
    save_dir: str,
    max_episodes: int = 100,
    chunk_size: int = 1000,
    save_images: bool = True,
    save_videos: bool = True,
    video_config: Optional[dict] = None,
    image_config: Optional[dict] = None,
    save_waypoints: bool = False,
    save_base_pose: bool = False,
    waypoints_format: str = "json",
    waypoints_dir: str = "waypoints",
    log_waypoints_stats: bool = True,
    waypoints_stats_interval: int = 10,
    save_metadata: bool = True
) -> VideoFormatCollector:
    """
    创建视频格式数据收集器的工厂函数

    Args:
        camera_configs: 相机配置列表
        save_dir: 保存目录
        max_episodes: 最大 episode 数量
        chunk_size: 每个 chunk 的 episode 数量
        save_images: 是否保存图像
        save_videos: 是否保存视频
        video_config: 视频配置
        image_config: 图像配置
        save_waypoints: 是否保存路径点（默认 False）
        save_base_pose: 是否保存基座姿态（默认 False）
        waypoints_format: 路径点保存格式
        waypoints_dir: 路径点保存目录
        log_waypoints_stats: 是否输出路径点统计
        waypoints_stats_interval: 统计输出间隔
        save_metadata: 是否保存 JSON 元数据文件（默认 True）
            save_metadata: 是否保存 JSON 元数据文件（默认 True）

    Returns:
        VideoFormatCollector 实例
    """
    return VideoFormatCollector(
        save_dir=save_dir,
        camera_configs=camera_configs,
        chunk_size=chunk_size,
        max_episodes=max_episodes,
        save_images=save_images,
        save_videos=save_videos,
        video_config=video_config,
        image_config=image_config,
        save_waypoints=save_waypoints,
        save_base_pose=save_base_pose,
        waypoints_format=waypoints_format,
        waypoints_dir=waypoints_dir,
        log_waypoints_stats=log_waypoints_stats,
        waypoints_stats_interval=waypoints_stats_interval,
        save_metadata=save_metadata
    )
