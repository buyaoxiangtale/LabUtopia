#!/usr/bin/env python3
"""
视频格式数据集加载器和分析工具

支持：
- 加载视频格式数据集
- 查看 episode 信息
- 提取帧和视频
- 数据集统计
"""

import json
import cv2
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import matplotlib.pyplot as plt
from tqdm import tqdm


class VideoFormatDataset:
    """
    视频格式数据集加载器

    用法:
        dataset = VideoFormatDataset('path/to/dataset')

        # 遍历所有 episodes
        for episode_data in dataset:
            frames = episode_data['frames']  # [T, H, W, 3]
            actions = episode_data['actions']  # [T, num_joints]
            ...

        # 或者访问单个 episode
        episode_0 = dataset[0]
    """

    def __init__(self, dataset_dir: str, load_images: bool = True):
        """
        初始化数据集

        Args:
            dataset_dir: 数据集根目录
            load_images: 是否加载图像（False 则只加载元数据）
        """
        self.dataset_dir = Path(dataset_dir)
        self.load_images = load_images
        self.episodes = self._load_episodes()

        print(f"✓ Dataset loaded: {self.dataset_dir}")
        print(f"  Total episodes: {len(self.episodes)}")

    def _load_episodes(self) -> List[Dict]:
        """加载所有 episode 元数据"""
        episodes = []
        json_files = sorted(self.dataset_dir.glob("episode_*.json"))

        if len(json_files) == 0:
            raise FileNotFoundError(f"No episode metadata found in {self.dataset_dir}")

        for json_file in tqdm(json_files, desc="Loading episodes"):
            with open(json_file, 'r') as f:
                episode_data = json.load(f)
                episode_data['json_path'] = str(json_file)
                episodes.append(episode_data)

        return episodes

    def __len__(self):
        return len(self.episodes)

    def __getitem__(self, idx: int) -> Dict:
        """
        获取单个 episode

        Returns:
            字典包含:
            - episode_index: int
            - frames: np.ndarray [T, H, W, 3] RGB 帧
            - depths: np.ndarray [T, H, W] 深度帧 (如果有)
            - actions: np.ndarray [T, num_joints]
            - language_instruction: str or None
        """
        if not self.load_images:
            raise ValueError("Set load_images=True to access episode data")

        episode = self.episodes[idx]
        episode_idx = episode['episode_index']

        # 计算 chunk
        chunk_idx = episode_idx // 1000
        chunk_dir = self.dataset_dir / "videos" / f"chunk-{chunk_idx:03d}"

        if not chunk_dir.exists():
            raise FileNotFoundError(f"Chunk directory not found: {chunk_dir}")

        result = {
            'episode_index': episode_idx,
            'language_instruction': episode.get('language_instruction'),
            'metadata': episode.get('metadata', {})
        }

        # 加载 RGB 图像
        rgb_dir = chunk_dir / "observation.images.observation"
        if rgb_dir.exists():
            rgb_frames = self._load_images_from_dir(
                rgb_dir,
                episode_idx,
                episode['length']
            )
            result['frames'] = np.array(rgb_frames)
        else:
            result['frames'] = None

        # 加载深度图
        depth_dir = chunk_dir / "observation.images.observation_depth"
        if depth_dir.exists():
            depth_frames = self._load_images_from_dir(
                depth_dir,
                episode_idx,
                episode['length'],
                is_depth=True
            )
            result['depths'] = np.array(depth_frames)
        else:
            result['depths'] = None

        # 加载 actions
        if 'actions' in episode:
            result['actions'] = np.array(episode['actions'])

        return result

    def _load_images_from_dir(
        self,
        image_dir: Path,
        episode_idx: int,
        length: int,
        is_depth: bool = False
    ) -> List[np.ndarray]:
        """从目录加载图像"""
        frame_start = episode_idx * length
        frames = []

        for i in range(length):
            frame_idx = frame_start + i

            # 尝试 jpg 和 png
            jpg_path = image_dir / f"{frame_idx}.jpg"
            png_path = image_dir / f"{frame_idx}.png"
            frame_path = jpg_path if jpg_path.exists() else png_path

            if frame_path.exists():
                if is_depth:
                    frame = cv2.imread(str(frame_path), cv2.IMREAD_UNCHANGED)
                else:
                    frame = cv2.imread(str(frame_path))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                if frame is not None:
                    frames.append(frame)
                else:
                    print(f"⚠️  Failed to load: {frame_path}")

        return frames

    def get_video_path(self, episode_idx: int, video_type: str = 'trajectory') -> Path:
        """
        获取视频路径

        Args:
            episode_idx: episode 索引
            video_type: 'trajectory' (RGB) 或 'depth'

        Returns:
            视频文件路径
        """
        chunk_idx = episode_idx // 1000
        episode_name = f"episode_{episode_idx:06d}"

        if video_type == 'trajectory':
            video_dir = self.dataset_dir / "videos" / f"chunk-{chunk_idx:03d}" / "observation.video.trajectory"
        elif video_type == 'depth':
            video_dir = self.dataset_dir / "videos" / f"chunk-{chunk_idx:03d}" / "observation.video.depth"
        else:
            raise ValueError(f"Unknown video type: {video_type}")

        return video_dir / f"{episode_name}.mp4"

    def get_stats(self) -> Dict:
        """获取数据集统计信息"""
        lengths = [ep['length'] for ep in self.episodes]

        stats = {
            'total_episodes': len(self.episodes),
            'total_frames': sum(lengths),
            'avg_length': np.mean(lengths),
            'min_length': np.min(lengths),
            'max_length': np.max(lengths),
            'std_length': np.std(lengths)
        }

        return stats

    def print_summary(self):
        """打印数据集摘要"""
        stats = self.get_stats()

        print(f"\n{'='*70}")
        print(f"Dataset Summary: {self.dataset_dir.name}")
        print(f"{'='*70}")
        print(f"Total episodes:  {stats['total_episodes']}")
        print(f"Total frames:    {stats['total_frames']}")
        print(f"Avg length:      {stats['avg_length']:.1f} frames")
        print(f"Min length:      {stats['min_length']} frames")
        print(f"Max length:      {stats['max_length']} frames")
        print(f"Std length:      {stats['std_length']:.1f}")
        print(f"{'='*70}\n")

    def visualize_episode(self, idx: int, save_path: Optional[str] = None):
        """可视化 episode 的帧网格"""
        episode = self[idx]
        frames = episode['frames']

        if frames is None or len(frames) == 0:
            print(f"No frames to visualize for episode {idx}")
            return

        # 选择 16 帧均匀分布
        num_display = min(16, len(frames))
        indices = np.linspace(0, len(frames) - 1, num_display, dtype=int)

        fig, axes = plt.subplots(4, 4, figsize=(16, 16))
        axes = axes.flatten()

        for i, idx in enumerate(indices):
            axes[i].imshow(frames[idx])
            axes[i].set_title(f"Frame {idx}")
            axes[i].axis('off')

        plt.suptitle(f"Episode {idx} - {len(frames)} frames")
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=100, bbox_inches='tight')
            print(f"✓ Visualization saved to {save_path}")
        else:
            plt.show()

        plt.close()


def extract_frames_from_video(video_path: str, output_dir: str, fps: int = 10):
    """
    从视频提取帧

    Args:
        video_path: 视频文件路径
        output_dir: 输出目录
        fps: 提取帧率
    """
    import os

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(video_path)

    video_fps = cap.get(cv2.CAP_PROP_FPS)
    frame_interval = int(video_fps / fps)

    frame_count = 0
    saved_count = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        if frame_count % frame_interval == 0:
            output_path = output_dir / f"frame_{saved_count:06d}.jpg"
            cv2.imwrite(str(output_path), frame)
            saved_count += 1

        frame_count += 1

    cap.release()
    print(f"✓ Extracted {saved_count} frames from {video_path}")


def compare_datasets(dir1: str, dir2: str):
    """
    比较两个数据集

    Args:
        dir1: 数据集 1 路径
        dir2: 数据集 2 路径
    """
    dataset1 = VideoFormatDataset(dir1, load_images=False)
    dataset2 = VideoFormatDataset(dir2, load_images=False)

    stats1 = dataset1.get_stats()
    stats2 = dataset2.get_stats()

    print(f"\n{'='*70}")
    print(f"Dataset Comparison")
    print(f"{'='*70}")
    print(f"{'Metric':<20} {Path(dir1).name:<20} {Path(dir2).name:<20}")
    print(f"{'-'*70}")
    print(f"{'Episodes':<20} {stats1['total_episodes']:<20} {stats2['total_episodes']:<20}")
    print(f"{'Total frames':<20} {stats1['total_frames']:<20} {stats2['total_frames']:<20}")
    print(f"{'Avg length':<20} {stats1['avg_length']:<20.1f} {stats2['avg_length']:<20.1f}")
    print(f"{'='*70}\n")


# 命令行使用
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='视频格式数据集工具')
    parser.add_argument('dataset_dir', type=str, help='数据集目录')
    parser.add_argument('--action', type=str, default='stats',
                       choices=['stats', 'visualize', 'extract', 'compare'],
                       help='操作类型')
    parser.add_argument('--episode_idx', type=int, default=0,
                       help='Episode 索引')
    parser.add_argument('--output', type=str, help='输出路径')

    args = parser.parse_args()

    if args.action == 'stats':
        dataset = VideoFormatDataset(args.dataset_dir, load_images=False)
        dataset.print_summary()

    elif args.action == 'visualize':
        dataset = VideoFormatDataset(args.dataset_dir)
        save_path = args.output if args.output else None
        dataset.visualize_episode(args.episode_idx, save_path)

    elif args.action == 'extract':
        # 从视频提取帧
        video_path = args.dataset_dir
        output_dir = args.output or 'extracted_frames'
        extract_frames_from_video(video_path, output_dir)
