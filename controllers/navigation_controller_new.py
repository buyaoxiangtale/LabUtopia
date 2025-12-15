import numpy as np
from typing import Dict, Any, Tuple, Optional
from .base_controller import BaseController
from .robot_controllers.ridgebase.ridgebase_controller_new import RidgebaseControllerNew


class NavigationControllerNew(BaseController):
    """
    改进的导航控制器，解决抖动问题。

    主要改进：
    - 使用PID控制替代简单的P控制
    - 添加速度平滑过渡
    - 实现死区控制减少微小误差抖动
    - 添加低通滤波平滑控制输出
    - 改进角度控制稳定性

    支持两种模式：
    - collect mode：收集导航轨迹数据
    - infer mode：使用学习策略导航（预留接口）

    Attributes:
        ridgebase_controller: Ridgebase低级运动控制器
        waypoints_set: 是否已设置路径点
    """

    def __init__(self, cfg, robot):
        """
        初始化导航控制器。

        Args:
            cfg: 配置对象
            robot: 机器人实例
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

        # 初始化改进的Ridgebase控制器
        self.ridgebase_controller = RidgebaseControllerNew(
            robot_articulation=robot,
            max_linear_speed=cfg.task.max_linear_speed if hasattr(cfg.task, 'max_linear_speed') else 0.02,
            max_angular_speed=cfg.task.max_angular_speed if hasattr(cfg.task, 'max_angular_speed') else 1.5,
            position_threshold=cfg.task.position_threshold if hasattr(cfg.task, 'position_threshold') else 0.08,
            angle_threshold=cfg.task.angle_threshold if hasattr(cfg.task, 'angle_threshold') else 0.1
        )

        self.waypoints_set = False

    def reset(self) -> None:
        """重置控制器状态"""
        super().reset()
        self.waypoints_set = False
        if hasattr(self, 'ridgebase_controller'):
            self.ridgebase_controller.reset()

    def step(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        执行一步控制。

        Args:
            state: 当前状态字典，包含current_pose, waypoints等

        Returns:
            tuple: (action, done, is_success)
                - action: 控制动作
                - done: 任务是否完成
                - is_success: 任务是否成功
        """
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        收集模式下的控制步骤。

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

        if 'camera_data' in state and not done:
            joint_positions = np.array([
                current_pose[0],
                current_pose[1],
                current_pose[2]
            ])

            self.data_collector.cache_step(
                camera_images=state['camera_data'],
                joint_angles=joint_positions,
                language_instruction=self.get_language_instruction()
            )

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

    def _step_infer(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        推理模式下的控制步骤（预留接口）。

        Args:
            state: 状态字典

        Returns:
            tuple: (action, done, is_success)
        """
        if not self.waypoints_set and state.get('waypoints') is not None:
            self.ridgebase_controller.set_waypoints(state['waypoints'])
            self.waypoints_set = True

        # 获取当前姿态
        current_pose = state['current_pose']

        # 计算控制动作
        action, done = self.ridgebase_controller.get_action(current_pose)

        # 如果导航完成
        if done or self.ridgebase_controller.is_path_complete():
            self._last_success = True
            self.reset_needed = True
            return action, True, True

        return action, False, False

    def _init_collect_mode(self, cfg, robot=None):
        """初始化收集模式"""
        from factories.collector_factory import create_collector
        self.data_collector = create_collector(
            cfg.collector.type,
            camera_configs=cfg.cameras,
            save_dir=cfg.multi_run.run_dir,
            max_episodes=cfg.max_episodes,
            compression=cfg.collector.compression
        )

    def _init_infer_mode(self, cfg, robot=None):
        """初始化推理模式（预留接口）"""
        pass

    def get_language_instruction(self) -> Optional[str]:
        """
        获取任务的语言指令。

        Returns:
            str: 语言指令
        """
        self._language_instruction = "Navigate to the target position"
        return self._language_instruction