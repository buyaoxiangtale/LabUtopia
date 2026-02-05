import numpy as np
from typing import Dict, Any, Tuple, Optional
from .base_controller import BaseController
from .robot_controllers.ridgebase.ridgebase_controller_smooth import RidgebaseControllerSmooth


class NavigationControllerParquet(BaseController):
    """
    导航控制器 - 支持Parquet格式数据收集（包含相机内参、外参、轨迹）

    与普通导航控制器的区别：
    - 自动获取相机内参矩阵
    - 自动记录相机轨迹（世界坐标位姿）
    - 适用于ParquetFormatCollector

    Attributes:
        ridgebase_controller: Ridgebase低级运动控制器
        cameras: 相机实例列表（从任务传入）
        waypoints_set: 是否已设置路径点
    """

    def __init__(self, cfg, robot, cameras=None):
        """
        初始化导航控制器

        Args:
            cfg: 配置对象
            robot: 机器人实例
            cameras: 相机实例列表（可选，用于收集相机数据）
        """
        try:
            super().__init__(cfg, robot, use_default_config=False)
        except Exception as e:
            self.cfg = cfg
            self.robot = robot
            self.object_utils = None
            self.reset_needed = False
            self._last_success = False
            self._episode_num = 0
            self.success_count = 0
            self._language_instruction = ""
            self.REQUIRED_SUCCESS_STEPS = 60
            self.check_success_counter = 0
            self.rmp_controller = None
            self.gripper_control = None

            if hasattr(cfg, "mode"):
                self.mode = cfg.mode
                if self.mode == "collect":
                    self._init_collect_mode(cfg, robot)
                elif self.mode == "infer":
                    self._init_infer_mode(cfg, robot)

        # 初始化低级控制器
        self.ridgebase_controller = RidgebaseControllerSmooth(
            robot_articulation=robot,
            max_linear_speed=cfg.task.max_linear_speed if hasattr(cfg.task, 'max_linear_speed') else 0.02,
            max_angular_speed=cfg.task.max_angular_speed if hasattr(cfg.task, 'max_angular_speed') else 1.5,
            position_threshold=cfg.task.position_threshold if hasattr(cfg.task, 'position_threshold') else 0.08,
            angle_threshold=cfg.task.angle_threshold if hasattr(cfg.task, 'angle_threshold') else 0.1
        )

        self.waypoints_set = False

        # 保存相机引用
        self.cameras = cameras if cameras is not None else []

        # 相机数据收集标志
        self._camera_intrinsics_set = False

    def set_cameras(self, cameras):
        """
        设置相机实例（用于收集相机数据）

        Args:
            cameras: 相机实例列表
        """
        self.cameras = cameras
        print(f"✓ Set {len(cameras)} cameras for data collection")

    def reset(self) -> None:
        """重置控制器状态"""
        super().reset()
        self.waypoints_set = False
        self._waypoints_collected = False
        self._camera_intrinsics_set = False  # 重置相机内参标志

    def step(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        执行一步控制

        Args:
            state: 当前状态字典，包含current_pose, waypoints等

        Returns:
            tuple: (action, done, is_success)
        """
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        收集模式下的控制步骤

        Args:
            state: 状态字典

        Returns:
            tuple: (action, done, is_success)
        """
        # 检查是否需要设置路径点
        if not self.waypoints_set and state.get('waypoints') is not None:
            self.ridgebase_controller.set_waypoints(state['waypoints'])
            self.waypoints_set = True

        current_pose = state['current_pose']
        action, done = self.ridgebase_controller.get_action(current_pose)

        # 数据收集
        if 'camera_data' in state and not done:
            joint_positions = np.array([
                current_pose[0],
                current_pose[1],
                current_pose[2]
            ])

            # 第一步：设置相机内参
            if not self._camera_intrinsics_set and len(self.cameras) > 0:
                self._set_camera_intrinsics()
                self._camera_intrinsics_set = True

            # 每步：获取相机位姿
            camera_poses = None
            if len(self.cameras) > 0:
                camera_poses = self._get_camera_poses()

            # 获取waypoints（只在第一步）
            waypoints_to_pass = None
            if not hasattr(self, '_waypoints_collected'):
                self._waypoints_collected = False

            if not self._waypoints_collected and state.get('waypoints') is not None:
                waypoints_to_pass = state['waypoints']
                self._waypoints_collected = True

            # 缓存数据（包含相机位姿）
            self.data_collector.cache_step(
                camera_images=state['camera_data'],
                joint_angles=joint_positions,
                language_instruction=self.get_language_instruction(),
                waypoints=waypoints_to_pass,
                base_pose=current_pose,
                camera_poses=camera_poses  # 传递相机位姿
            )

        # 导航完成
        if done or self.ridgebase_controller.is_path_complete():
            self._last_success = True
            self.reset_needed = True

            if hasattr(self, 'data_collector'):
                final_joint_positions = np.array([
                    current_pose[0],
                    current_pose[1],
                    current_pose[2]
                ])
                self.data_collector.write_cached_data(final_joint_positions)

            return action, True, True

        return action, False, False

    def _set_camera_intrinsics(self):
        """设置相机内参（在episode开始时调用一次）"""
        from utils.camera_utils import get_camera_intrinsic

        intrinsics = {}
        for camera in self.cameras:
            try:
                intrinsic = get_camera_intrinsic(camera)
                intrinsics[camera.name] = intrinsic
                print(f"  ✓ Got intrinsic for camera '{camera.name}': {intrinsic.shape}")
            except Exception as e:
                print(f"  ⚠️  Failed to get intrinsic for camera '{camera.name}': {e}")

        if intrinsics and hasattr(self.data_collector, 'set_camera_intrinsics'):
            self.data_collector.set_camera_intrinsics(intrinsics)
            print(f"✓ Set camera intrinsics for {len(intrinsics)} camera(s)")
        elif not intrinsics:
            print("⚠️  Warning: No camera intrinsics were collected")

    def _get_camera_poses(self) -> Dict[str, np.ndarray]:
        """
        获取所有相机的当前位姿

        Returns:
            Dict[str, np.ndarray]: {camera_name: pose_matrix (4,4)}
        """
        from utils.camera_utils import get_camera_trajectory_matrix

        camera_poses = {}
        for camera in self.cameras:
            try:
                pose = get_camera_trajectory_matrix(camera)
                camera_poses[camera.name] = pose
            except Exception as e:
                print(f"  ⚠️  Failed to get pose for camera '{camera.name}': {e}")

        return camera_poses

    def _step_infer(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        推断模式下的控制步骤（预留接口）

        Args:
            state: 状态字典

        Returns:
            tuple: (action, done, is_success)
        """
        if not self.waypoints_set and state.get('waypoints') is not None:
            self.ridgebase_controller.set_waypoints(state['waypoints'])
            self.waypoints_set = True

        current_pose = state['current_pose']
        action, done = self.ridgebase_controller.get_action(current_pose)

        if done or self.ridgebase_controller.is_path_complete():
            self._last_success = True
            self.reset_needed = True
            return action, True, True

        return action, False, False

    def _init_collect_mode(self, cfg, robot=None):
        """初始化收集模式"""
        from factories.collector_factory import create_collector

        # 构建收集器参数
        collector_kwargs = {
            'camera_configs': cfg.cameras,
            'save_dir': cfg.multi_run.run_dir,
            'max_episodes': cfg.max_episodes,
        }

        # 根据收集器类型添加特定参数
        if cfg.collector.type == 'parquet_format':
            # ParquetFormatCollector 参数
            if hasattr(cfg.collector, 'chunk_size'):
                collector_kwargs['chunk_size'] = cfg.collector.chunk_size

            # 图像保存配置
            if hasattr(cfg.dataset, 'save_images'):
                collector_kwargs['save_images'] = cfg.dataset.save_images
            else:
                collector_kwargs['save_images'] = True  # 默认保存图片

            # 视频保存配置
            if hasattr(cfg.dataset, 'save_videos'):
                collector_kwargs['save_videos'] = cfg.dataset.save_videos
            else:
                collector_kwargs['save_videos'] = True  # 默认保存视频

            if hasattr(cfg.dataset, 'video'):
                collector_kwargs['video_config'] = cfg.dataset.video

            if hasattr(cfg.dataset, 'image'):
                collector_kwargs['image_config'] = cfg.dataset.image

        elif cfg.collector.type == 'video_format':
            # VideoFormatCollector 参数
            if hasattr(cfg.collector, 'chunk_size'):
                collector_kwargs['chunk_size'] = cfg.collector.chunk_size
            if hasattr(cfg.collector, 'save_images'):
                collector_kwargs['save_images'] = cfg.collector.save_images
            if hasattr(cfg.collector, 'save_videos'):
                collector_kwargs['save_videos'] = cfg.collector.save_videos
            if hasattr(cfg.collector, 'video'):
                collector_kwargs['video_config'] = cfg.collector.video
            if hasattr(cfg.collector, 'image'):
                collector_kwargs['image_config'] = cfg.collector.image

            # 路径点相关配置
            if hasattr(cfg.collector, 'save_waypoints'):
                collector_kwargs['save_waypoints'] = cfg.collector.save_waypoints
            if hasattr(cfg.collector, 'save_base_pose'):
                collector_kwargs['save_base_pose'] = cfg.collector.save_base_pose
            if hasattr(cfg.collector, 'waypoints_format'):
                collector_kwargs['waypoints_format'] = cfg.collector.waypoints_format
            if hasattr(cfg.collector, 'waypoints_dir'):
                collector_kwargs['waypoints_dir'] = cfg.collector.waypoints_dir
            if hasattr(cfg.collector, 'log_waypoints_stats'):
                collector_kwargs['log_waypoints_stats'] = cfg.collector.log_waypoints_stats
            if hasattr(cfg.collector, 'waypoints_stats_interval'):
                collector_kwargs['waypoints_stats_interval'] = cfg.collector.waypoints_stats_interval
            if hasattr(cfg.collector, 'save_metadata'):
                collector_kwargs['save_metadata'] = cfg.collector.save_metadata
        else:
            # 其他收集器参数 (default/mock)
            if hasattr(cfg.collector, 'compression'):
                collector_kwargs['compression'] = cfg.collector.compression

        self.data_collector = create_collector(
            cfg.collector.type,
            **collector_kwargs
        )

        print(f"✓ Created {cfg.collector.type} collector")

    def _init_infer_mode(self, cfg, robot=None):
        """初始化推断模式（预留接口）"""
        pass

    def get_language_instruction(self) -> Optional[str]:
        """
        获取任务的语言指令

        Returns:
            str: 语言指令
        """
        self._language_instruction = "Navigate to the target position"
        return self._language_instruction
