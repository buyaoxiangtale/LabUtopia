#!/usr/bin/env python3
"""
Parquet格式数据收集示例脚本

演示如何使用 ParquetFormatCollector 收集包含相机内参、外参和轨迹的数据。

数据结构：
    episode_000000.parquet:
        - camera_intrinsic (3x3): 相机内参矩阵
        - camera_extrinsic (4x4): 相机外参矩阵
        - camera_trajectory (N, 4, 4): 相机轨迹
"""

import numpy as np
from pathlib import Path
from data_collectors.parquet_format_collector import create_parquet_format_collector
from utils.camera_utils import (
    get_camera_intrinsic,
    get_camera_extrinsic,
    get_camera_trajectory_matrix
)


def example_basic_usage():
    """基本使用示例"""
    print("=" * 70)
    print("示例 1: ParquetFormatCollector 基本使用")
    print("=" * 70)

    # 相机配置
    camera_configs = [
        {
            'name': 'observation',
            'image_type': 'rgb+depth',
            'resolution': [256, 256],
            'prim_path': '/World/Ridgebase/base_link/Camera_02'
        }
    ]

    # 创建收集器
    collector = create_parquet_format_collector(
        camera_configs=camera_configs,
        save_dir='outputs/parquet_example',
        max_episodes=2,
        chunk_size=1000,
        save_videos=False  # 只保存 Parquet，不保存视频
    )

    # 模拟收集数据（实际使用时从 Isaac Sim 获取）
    for episode_idx in range(2):
        print(f"\n--- Episode {episode_idx} ---")

        # 模拟相机内参（每个 episode 获取一次）
        # 实际使用: intrinsic = get_camera_intrinsic(camera)
        intrinsic = np.array([
            [256.0, 0.0, 128.0],
            [0.0, 256.0, 128.0],
            [0.0, 0.0, 1.0]
        ])
        collector.set_camera_intrinsics({'observation': intrinsic})

        # 模拟轨迹收集（每个 time step）
        num_steps = 10
        for step in range(num_steps):
            # 模拟相机世界坐标位姿
            # 实际使用: pose = get_camera_trajectory_matrix(camera)
            camera_pose = np.eye(4)
            camera_pose[0, 3] = step * 0.1  # x 方向移动
            camera_pose[1, 3] = step * 0.05  # y 方向移动

            # 模拟机器人关节角度
            joint_angles = np.random.rand(7)  # 7个关节

            # 缓存数据
            collector.cache_step(
                camera_poses={'observation': camera_pose},
                joint_angles=joint_angles
            )

        # 写入 episode 数据
        final_joint_positions = np.random.rand(7)
        collector.write_cached_data(final_joint_positions)

    # 关闭收集器
    collector.close()

    print("\n✓ 示例完成！数据已保存到: outputs/parquet_example/")


def example_read_parquet():
    """读取 Parquet 文件示例（LeRobot 标准格式）"""
    print("\n" + "=" * 70)
    print("示例 2: 读取 Parquet 文件（LeRobot 标准格式）")
    print("=" * 70)

    import pandas as pd

    parquet_path = 'outputs/parquet_example/data/chunk-000/episode_000000.parquet'

    if not Path(parquet_path).exists():
        print(f"⚠️  文件不存在: {parquet_path}")
        print("请先运行示例 1 生成数据")
        return

    # 读取 Parquet 文件
    df = pd.read_parquet(parquet_path)

    print(f"\n数据形状: {df.shape}")
    print(f"列名: {list(df.columns)}")

    # 提取相机内参（仅在第一帧）
    if 'observation.observation.camera_intrinsic' in df.columns:
        intrinsic = df['observation.observation.camera_intrinsic'].iloc[0]
        if intrinsic is not None:
            intrinsic_arr = np.array(intrinsic)
            print(f"\n相机内参 (3x3):\n{intrinsic_arr}")
        else:
            print(f"\n⚠️  第一帧的内参为 None")
    else:
        print(f"\n⚠️  没有找到 observation.observation.camera_intrinsic 列")

    # 提取相机外参（仅在第一帧）
    if 'observation.observation.camera_extrinsic' in df.columns:
        extrinsic = df['observation.observation.camera_extrinsic'].iloc[0]
        if extrinsic is not None:
            extrinsic_arr = np.array(extrinsic)
            print(f"\n相机外参 (4x4):\n{extrinsic_arr}")
        else:
            print(f"\n⚠️  第一帧的外参为 None")
    else:
        print(f"\n⚠️  没有找到 observation.observation.camera_extrinsic 列")

    # 提取相机轨迹（action 字段）
    trajectory = [np.array(pose) for pose in df['action'].tolist()]
    print(f"\n相机轨迹长度: {len(trajectory)}")
    print(f"第1步位姿 (4x4):\n{trajectory[0]}")
    if len(trajectory) > 1:
        print(f"第2步位姿 (4x4):\n{trajectory[1]}")

    # 提取机器人状态
    states = df['observation.state'].tolist()
    print(f"\n机器人状态数量: {len(states)}")
    print(f"第1步状态: {states[0]}")

    # 验证格式
    print(f"\n格式验证:")
    print(f"  ✅ episode_index: {df['episode_index'].iloc[0]}")
    print(f"  ✅ observation.state: {len(states)} 个状态")
    print(f"  ✅ action (轨迹): {len(trajectory)} 个位姿")
    if 'observation.observation.camera_intrinsic' in df.columns:
        print(f"  ✅ observation.observation.camera_intrinsic: 存在")


def example_with_isaac_sim():
    """与 Isaac Sim 集成的示例（伪代码）"""
    print("\n" + "=" * 70)
    print("示例 3: 与 Isaac Sim 集成（伪代码）")
    print("=" * 70)

    example_code = '''
# 在任务类中使用 ParquetFormatCollector

from utils.camera_utils import (
    get_camera_intrinsic,
    get_camera_extrinsic,
    get_camera_trajectory_matrix
)

class MyTask(BaseTask):
    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)

        # 创建 Parquet 收集器
        from data_collectors.parquet_format_collector import create_parquet_format_collector
        self.collector = create_parquet_format_collector(
            camera_configs=cfg.cameras,
            save_dir='outputs/parquet_data',
            max_episodes=cfg.max_episodes,
            save_videos=True  # 同时保存视频
        )

    def step(self):
        # ... 现有的任务逻辑 ...

        # 第一步：获取并设置相机内参
        if self.frame_idx == 0:
            intrinsics = {}
            for camera in self.cameras:
                intrinsics[camera.name] = get_camera_intrinsic(camera)
            self.collector.set_camera_intrinsics(intrinsics)

        # 每步：获取相机轨迹和图像
        camera_poses = {}
        camera_images = {}

        for camera, cam_cfg in zip(self.cameras, self.cfg.cameras):
            cam_name = cam_cfg['name']

            # 获取相机轨迹（世界坐标位姿）
            camera_poses[cam_name] = get_camera_trajectory_matrix(camera)

            # 获取图像（如果需要保存视频）
            if self.collector.save_videos:
                from utils.camera_utils import process_camera_image
                image_data, _ = process_camera_image(camera, cam_cfg['image_type'])
                if isinstance(image_data, dict):
                    for img_type, img in image_data.items():
                        camera_images[f'{cam_name}_{img_type}'] = img

        # 获取机器人状态
        joint_angles = self.robot.get_joint_positions()

        # 缓存数据
        self.collector.cache_step(
            camera_poses=camera_poses,
            joint_angles=joint_angles,
            camera_images=camera_images if self.collector.save_videos else None
        )

        # ... 其他逻辑 ...

    def on_task_complete(self, success):
        # 任务完成时写入数据
        final_positions = self.robot.get_joint_positions()
        self.collector.write_cached_data(final_positions)
    '''

    print(example_code)


def example_data_structure():
    """展示数据结构"""
    print("\n" + "=" * 70)
    print("示例 4: Parquet 数据结构说明")
    print("=" * 70)

    structure = '''
Parquet 文件包含以下列（以单个相机 'observation' 为例）：

1. 元数据列:
   - episode_index: int (每行相同)
   - step: int (0, 1, 2, ..., N-1)
   - timestamp: str (ISO格式时间戳)

2. 相机内参列 (每行相同):
   - observation_camera_intrinsic_0_0: float (fx)
   - observation_camera_intrinsic_0_1: float (0)
   - observation_camera_intrinsic_0_2: float (cx)
   - observation_camera_intrinsic_1_0: float (0)
   - observation_camera_intrinsic_1_1: float (fy)
   - observation_camera_intrinsic_1_2: float (cy)
   - observation_camera_intrinsic_2_0: float (0)
   - observation_camera_intrinsic_2_1: float (0)
   - observation_camera_intrinsic_2_2: float (1)

3. 相机外参列 (每行相同，使用第一步的外参):
   - observation_camera_extrinsic_0_0 到 observation_camera_extrinsic_3_3
   - 共16列，表示 4x4 矩阵

4. 相机轨迹列 (每行不同):
   - observation_camera_trajectory_0_0 到 observation_camera_trajectory_3_3
   - 共16列，每行表示该时刻的 4x4 位姿矩阵

5. 机器人状态列（可选）:
   - state_0, state_1, ..., state_N: 关节角度

6. 动作列（可选）:
   - action_0, action_1, ..., action_N: 控制动作

读取和使用示例：
    import pandas as pd
    import numpy as np

    # 读取数据
    df = pd.read_parquet('episode_000000.parquet')

    # 提取内参
    intrinsic = df[[f'observation_camera_intrinsic_{i}_{j}'
                    for i in range(3) for j in range(3)]].iloc[0].values.reshape(3, 3)

    # 提取外参
    extrinsic = df[[f'observation_camera_extrinsic_{i}_{j}'
                    for i in range(4) for j in range(4)]].iloc[0].values.reshape(4, 4)

    # 提取轨迹
    trajectory = df[[f'observation_camera_trajectory_{i}_{j}'
                     for i in range(4) for j in range(4)]].values.reshape(-1, 4, 4)
    '''

    print(structure)


if __name__ == '__main__':
    # 运行所有示例
    example_basic_usage()
    example_read_parquet()
    example_with_isaac_sim()
    example_data_structure()

    print("\n" + "=" * 70)
    print("所有示例运行完成！")
    print("=" * 70)
