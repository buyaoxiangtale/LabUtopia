#!/usr/bin/env python3
"""
数据收集器工厂
根据配置自动选择合适的数据收集器，无需修改 BaseTask

支持的数据收集器类型：
- default: 原始 HDF5 格式 (DataCollector)
- video_format: 视频格式 (VideoFormatCollector)
- parquet_format: Parquet格式 (ParquetFormatCollector)
"""

from typing import List, Dict, Any, Optional, Union
import warnings


def create_collector(
    collector_type: str,
    camera_configs: List[dict],
    save_dir: str,
    max_episodes: int = 100,
    **kwargs
) -> Union['DataCollector', 'VideoFormatCollector', 'ParquetFormatCollector']:
    """
    创建数据收集器的工厂函数

    Args:
        collector_type: 收集器类型 ('default', 'video_format', 'parquet_format')
        camera_configs: 相机配置列表
        save_dir: 保存目录
        max_episodes: 最大 episode 数量
        **kwargs: 其他传递给收集器的参数

    Returns:
        数据收集器实例

    Examples:
        # 创建默认 HDF5 收集器
        collector = create_collector(
            collector_type='default',
            camera_configs=cfg.cameras,
            save_dir='outputs/data',
            max_episodes=100,
            compression='gzip'
        )

        # 创建视频格式收集器
        collector = create_collector(
            collector_type='video_format',
            camera_configs=cfg.cameras,
            save_dir='outputs/data',
            max_episodes=100,
            chunk_size=1000,
            save_images=True,
            save_videos=True
        )

        # 创建 Parquet 格式收集器
        collector = create_collector(
            collector_type='parquet_format',
            camera_configs=cfg.cameras,
            save_dir='outputs/data',
            max_episodes=100,
            chunk_size=1000,
            save_videos=True
        )
    """
    if collector_type == 'parquet_format':
        # 导入 Parquet 格式收集器
        from data_collectors.parquet_format_collector import ParquetFormatCollector

        # 提取 Parquet 格式相关参数
        chunk_size = kwargs.get('chunk_size', 1000)
        save_videos = kwargs.get('save_videos', True)
        video_config = kwargs.get('video_config')

        return ParquetFormatCollector(
            save_dir=save_dir,
            camera_configs=camera_configs,
            chunk_size=chunk_size,
            max_episodes=max_episodes,
            save_videos=save_videos,
            video_config=video_config
        )

    elif collector_type == 'video_format':
        # 导入视频格式收集器
        from data_collectors.video_format_collector import VideoFormatCollector

        # 提取视频格式相关参数
        video_config = kwargs.get('video_config')
        image_config = kwargs.get('image_config')
        chunk_size = kwargs.get('chunk_size', 1000)
        save_images = kwargs.get('save_images', True)
        save_videos = kwargs.get('save_videos', True)

        return VideoFormatCollector(
            save_dir=save_dir,
            camera_configs=camera_configs,
            chunk_size=chunk_size,
            max_episodes=max_episodes,
            save_images=save_images,
            save_videos=save_videos,
            video_config=video_config,
            image_config=image_config
        )

    elif collector_type == 'default' or collector_type is None:
        # 导入默认 HDF5 收集器
        from data_collectors.data_collector import DataCollector

        compression = kwargs.get('compression', None)
        max_workers = kwargs.get('max_workers', 4)

        return DataCollector(
            camera_configs=camera_configs,
            save_dir=save_dir,
            max_episodes=max_episodes,
            max_workers=max_workers,
            compression=compression
        )

    else:
        raise ValueError(f"Unknown collector type: {collector_type}. "
                        f"Supported types: 'default', 'video_format', 'parquet_format'")


def create_collector_from_cfg(cfg: Any, save_dir: str) -> Union['DataCollector', 'VideoFormatCollector']:
    """
    从 Hydra 配置对象创建数据收集器

    Args:
        cfg: Hydra 配置对象
        save_dir: 保存目录

    Returns:
        数据收集器实例

    Examples:
        # 在 BaseTask 中使用
        collector = create_collector_from_cfg(self.cfg, save_dir)
    """
    # 获取收集器类型
    if hasattr(cfg, 'collector') and hasattr(cfg.collector, 'type'):
        collector_type = cfg.collector.type
    else:
        collector_type = 'default'
        warnings.warn("No collector type specified, using 'default' (HDF5 format)")

    # 准备相机配置
    camera_configs = []
    if hasattr(cfg, 'cameras'):
        for cam_cfg in cfg.cameras:
            camera_configs.append({
                'name': cam_cfg.name,
                'image_type': cam_cfg.image_type,
                'resolution': cam_cfg.resolution,
                'prim_path': cam_cfg.prim_path
            })

    # 基本参数
    max_episodes = getattr(cfg, 'max_episodes', 100)

    # 收集器特定参数
    kwargs = {}

    # HDF5 收集器参数
    if hasattr(cfg.collector, 'compression'):
        kwargs['compression'] = cfg.collector.compression
    if hasattr(cfg.collector, 'max_workers'):
        kwargs['max_workers'] = cfg.collector.max_workers

    # 视频格式收集器参数
    if collector_type == 'video_format' and hasattr(cfg, 'dataset'):
        dataset_cfg = cfg.dataset

        if hasattr(dataset_cfg, 'chunk_size'):
            kwargs['chunk_size'] = dataset_cfg.chunk_size
        if hasattr(dataset_cfg, 'save_images'):
            kwargs['save_images'] = dataset_cfg.save_images
        if hasattr(dataset_cfg, 'save_videos'):
            kwargs['save_videos'] = dataset_cfg.save_videos
        if hasattr(dataset_cfg, 'video'):
            kwargs['video_config'] = dataset_cfg.video
        if hasattr(dataset_cfg, 'image'):
            kwargs['image_config'] = dataset_cfg.image

    return create_collector(
        collector_type=collector_type,
        camera_configs=camera_configs,
        save_dir=save_dir,
        max_episodes=max_episodes,
        **kwargs
    )


class CollectorAdapter:
    """
    数据收集器适配器

    提供统一的接口，自动切换底层实现
    无需修改现有代码即可使用不同的数据收集器
    """

    def __init__(self, collector: Union['DataCollector', 'VideoFormatCollector']):
        """
        初始化适配器

        Args:
            collector: 数据收集器实例
        """
        self.collector = collector

    def cache_step(self, camera_images: dict, joint_angles, language_instruction: Optional[str] = None):
        """缓存每步数据"""
        self.collector.cache_step(camera_images, joint_angles, language_instruction)

    def write_cached_data(self, final_joint_positions):
        """写入缓存的数据"""
        self.collector.write_cached_data(final_joint_positions)

    def clear_cache(self):
        """清空缓存"""
        if hasattr(self.collector, 'clear_cache'):
            self.collector.clear_cache()

    def close(self, merge: bool = False):
        """关闭收集器"""
        self.collector.close(merge=merge if hasattr(self.collector, 'close') else False)

    def __getattr__(self, name):
        """转发其他属性和方法到底层收集器"""
        return getattr(self.collector, name)


# 便捷函数
def create_hdf5_collector(camera_configs, save_dir, max_episodes=100, compression='gzip', max_workers=4):
    """创建 HDF5 格式收集器（便捷函数）"""
    return create_collector(
        collector_type='default',
        camera_configs=camera_configs,
        save_dir=save_dir,
        max_episodes=max_episodes,
        compression=compression,
        max_workers=max_workers
    )


def create_video_collector(
    camera_configs,
    save_dir,
    max_episodes=100,
    chunk_size=1000,
    save_images=True,
    save_videos=True,
    video_config=None,
    image_config=None
):
    """创建视频格式收集器（便捷函数）"""
    return create_collector(
        collector_type='video_format',
        camera_configs=camera_configs,
        save_dir=save_dir,
        max_episodes=max_episodes,
        chunk_size=chunk_size,
        save_images=save_images,
        save_videos=save_videos,
        video_config=video_config,
        image_config=image_config
    )


def create_parquet_collector(
    camera_configs,
    save_dir,
    max_episodes=100,
    chunk_size=1000,
    save_videos=True,
    video_config=None
):
    """创建 Parquet 格式收集器（便捷函数）"""
    return create_collector(
        collector_type='parquet_format',
        camera_configs=camera_configs,
        save_dir=save_dir,
        max_episodes=max_episodes,
        chunk_size=chunk_size,
        save_videos=save_videos,
        video_config=video_config
    )
