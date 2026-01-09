#!/usr/bin/env python3
"""
视频格式数据收集使用示例

展示如何在现有代码中使用视频格式数据收集器，
无需修改 BaseTask
"""

from data_collectors.collector_factory import create_collector_from_cfg
from utils.camera_utils import process_camera_image
import numpy as np


def example_usage():
    """
    示例：在现有的仿真循环中使用视频格式收集器
    """
    print("="*70)
    print("视频格式数据收集示例")
    print("="*70)

    # 假设这是你的 Hydra 配置
    class MockConfig:
        class Collector:
            type = "video_format"  # 关键：指定使用视频格式收集器
            compression = None

        class Dataset:
            chunk_size = 1000
            save_images = True
            save_videos = True

            class Video:
                rgb_codec = "mp4v"
                rgb_fps = 30
                depth_codec = "mp4v"
                depth_fps = 30
                depth_colormap = "jet"

            class Image:
                rgb_format = "jpg"
                rgb_quality = 95
                depth_format = "png"
                depth_normalize = True

        cameras = [
            {
                'name': 'observation',
                'image_type': 'rgb+depth',
                'resolution': [256, 256],
                'prim_path': '/World/Ridgebase/base_link/Camera'
            }
        ]

        max_episodes = 10

    cfg = MockConfig()

    # 创建视频格式收集器（与创建普通收集器完全相同的方式）
    save_dir = "outputs/example_video_collection"
    collector = create_collector_from_cfg(cfg, save_dir)

    print(f"\n✓ 数据收集器创建成功")
    print(f"  类型: {type(collector).__name__}")
    print(f"  保存目录: {save_dir}")

    # 仿真循环（示例）
    print("\n开始仿真循环...")

    for episode_idx in range(cfg.max_episodes):
        print(f"\nEpisode {episode_idx}")

        # 重置仿真...
        # world.reset()

        for step in range(100):  # 假设每个 episode 100 步
            # 1. 获取相机数据
            # camera_data, display_data = task.get_camera_data()

            # 模拟相机数据
            camera_data = {
                'observation_rgb': np.random.randint(0, 255, (3, 256, 256), dtype=np.uint8),
                'observation_depth': np.random.rand(1, 256, 256).astype(np.float32)
            }

            # 2. 获取机器人姿态
            # joint_angles = robot.get_joint_angles()
            joint_angles = np.random.rand(12).astype(np.float32)

            # 3. 缓存数据
            language_instruction = f"Navigate to target {episode_idx}"
            collector.cache_step(camera_data, joint_angles, language_instruction)

            # 4. 执行动作
            # world.step()

        # Episode 结束，写入数据
        final_joint_positions = np.random.rand(12).astype(np.float32)
        collector.write_cached_data(final_joint_positions)

        # 检查是否需要重置
        # if task.need_reset():
        #     task.reset()

    # 关闭收集器
    collector.close()

    print("\n" + "="*70)
    print("数据收集完成！")
    print(f"输出目录: {save_dir}")
    print("="*70)


def example_switching_between_formats():
    """
    示例：在不同格式之间切换
    """
    print("\n" + "="*70)
    print("示例：切换数据收集格式")
    print("="*70)

    # 方式 1: HDF5 格式（默认）
    print("\n[方式 1] HDF5 格式（默认）")
    collector_hdf5 = create_collector_from_cfg(
        type='default',
        camera_configs=[],
        save_dir="outputs/hdf5_format",
        max_episodes=10,
        compression='gzip'
    )
    print(f"  收集器类型: {type(collector_hdf5).__name__}")

    # 方式 2: 视频格式
    print("\n[方式 2] 视频格式")
    collector_video = create_collector_from_cfg(
        type='video_format',
        camera_configs=[],
        save_dir="outputs/video_format",
        max_episodes=10,
        chunk_size=1000,
        save_images=True,
        save_videos=True
    )
    print(f"  收集器类型: {type(collector_video).__name__}")

    print("\n✓ 两种格式使用相同的接口，可以无缝切换！")


def example_adapting_existing_code():
    """
    示例：修改现有代码以支持视频格式

    展示最小改动的集成方式
    """
    print("\n" + "="*70)
    print("示例：最小改动集成到现有代码")
    print("="*70)

    code_before = """
    # 原代码（使用 HDF5 格式）
    from data_collectors.data_collector import DataCollector

    collector = DataCollector(
        camera_configs=cfg.cameras,
        save_dir=save_dir,
        max_episodes=cfg.max_episodes,
        compression=cfg.collector.compression
    )
    """

    code_after = """
    # 修改后（自动选择格式）
    from data_collectors.collector_factory import create_collector_from_cfg

    collector = create_collector_from_cfg(cfg, save_dir)
    # 根据 cfg.collector.type 自动选择：
    # - 'default' -> HDF5 格式
    # - 'video_format' -> 视频格式
    """

    print("\n【修改前】")
    print(code_before)

    print("\n【修改后】")
    print(code_after)

    print("\n✓ 只需修改导入和创建方式，其他代码完全不变！")
    print("✓ collector.cache_step() 和 write_cached_data() 接口完全相同")


def example_loading_video_dataset():
    """
    示例：加载和使用视频格式数据集
    """
    print("\n" + "="*70)
    print("示例：加载视频格式数据集")
    print("="*70)

    from utils.video_dataset_loader import VideoFormatDataset

    # 假设已经收集了数据
    dataset_dir = "outputs/example_video_collection"

    print(f"\n加载数据集: {dataset_dir}")
    # dataset = VideoFormatDataset(dataset_dir, load_images=True)

    print("\n可执行的操作：")
    print("  1. 查看统计信息:")
    print("     dataset.print_summary()")
    print()
    print("  2. 访问单个 episode:")
    print("     episode = dataset[0]")
    print("     frames = episode['frames']  # [T, H, W, 3]")
    print("     actions = episode['actions']  # [T, num_joints]")
    print()
    print("  3. 获取视频路径:")
    print("     video_path = dataset.get_video_path(0, 'trajectory')")
    print()
    print("  4. 可视化 episode:")
    print("     dataset.visualize_episode(0, save_path='episode_0.png')")


if __name__ == "__main__":
    # 运行示例
    example_usage()
    example_switching_between_formats()
    example_adapting_existing_code()
    example_loading_video_dataset()

    print("\n" + "="*70)
    print("所有示例运行完成！")
    print("="*70)
    print("\n下一步：")
    print("  1. 使用 config/level5_Navigation_video_format.yaml 运行数据收集")
    print("  2. 检查输出目录中的视频和图像文件")
    print("  3. 使用 VideoFormatDataset 加载和分析数据")
    print()
