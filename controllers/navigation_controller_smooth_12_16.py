import numpy as np
from typing import Dict, Any, Tuple, Optional
from .base_controller import BaseController
from .robot_controllers.ridgebase.ridgebase_controller_smooth import RidgebaseControllerSmooth


class NavigationControllerSmooth1216(BaseController):
    """
    Navigation controller for controlling the Ridgebase robot to navigate along path points.
    
    Supports two modes:
    - collect mode：Collect navigation trajectory data
    - infer mode：Use learned policies to navigate (reserved interface)
    
    Attributes:
        ridgebase_controller: Ridgebase low-level motion controller
        waypoints_set: Whether the path points have been set
    """
    
    def __init__(self, cfg, robot):
        """
        Initialize the navigation controller.
        
        Args:
            cfg: Configuration object
            robot: Robot instance
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
        
        self.ridgebase_controller = RidgebaseControllerSmooth(
            robot_articulation=robot,
            max_linear_speed=cfg.task.max_linear_speed if hasattr(cfg.task, 'max_linear_speed') else 0.02,
            max_angular_speed=cfg.task.max_angular_speed if hasattr(cfg.task, 'max_angular_speed') else 1.5,
            position_threshold=cfg.task.position_threshold if hasattr(cfg.task, 'position_threshold') else 0.08,
            angle_threshold=cfg.task.angle_threshold if hasattr(cfg.task, 'angle_threshold') else 0.1
        )
        
        self.waypoints_set = False
        
    def reset(self) -> None:
        """Reset the controller state"""
        super().reset()
        self.waypoints_set = False
    
    def step(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        Perform one control step.
        
        Args:
            state: The current state dictionary, containing current_pose, waypoints etc.
            
        Returns:
            tuple: (action, done, is_success)
                - action: The control action
                - done: Whether the task is complete
                - is_success: Whether the task is successful
        """
        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)
    
    def _step_collect(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        Control step in collect mode.
        
        Args:
            state: The state dictionary
            
        Returns:
            tuple: (action, done, is_success)
        """
        # 检查是否需要设置新的路径点
        if not self.waypoints_set and state.get('waypoints') is not None:
            waypoints = state['waypoints']
            print(f"🎯 Controller: 接收新路径，共 {len(waypoints)} 个路径点")
            print(f"   第一个点: [{waypoints[0][0]:.2f}, {waypoints[0][1]:.2f}, {waypoints[0][2]*180/np.pi:.1f}°]")
            print(f"   最后一个点: [{waypoints[-1][0]:.2f}, {waypoints[-1][1]:.2f}, {waypoints[-1][2]*180/np.pi:.1f}°]")
            print(f"   当前机器人位置: [{state['current_pose'][0]:.2f}, {state['current_pose'][1]:.2f}, {state['current_pose'][2]*180/np.pi:.1f}°]")
            
            self.ridgebase_controller.set_waypoints(waypoints)
            self.waypoints_set = True
        
        current_pose = state['current_pose']

        action, done = self.ridgebase_controller.get_action(current_pose)

        # if 'camera_data' in state and not done:
        #     joint_positions = np.array([
        #         current_pose[0],
        #         current_pose[1],
        #         current_pose[2]
        #     ])
            
        #     self.data_collector.cache_step(
        #         camera_images=state['camera_data'],
        #         joint_angles=joint_positions,
        #         language_instruction=self.get_language_instruction()
        #     )
        
        # 路径完成：不返回成功，而是准备接收新目标点
        if done or self.ridgebase_controller.is_path_complete():
            print(f"✅ Controller: 当前路径已完成，当前位置: [{current_pose[0]:.2f}, {current_pose[1]:.2f}, {current_pose[2]*180/np.pi:.1f}°]")
            
            # 重置标志，允许接收新的路径点
            self.waypoints_set = False
            print(f"   waypoints_set 已重置为 False，等待新目标点...")
            
            # 如果任务层明确表示所有任务完成（通过 state['all_goals_done']）
            if state.get('all_goals_done', False):
                self._last_success = True
                self.reset_needed = True
                
                # if hasattr(self, 'data_collector'):
                #     final_joint_positions = np.array([
                #         current_pose[0],
                #         current_pose[1],
                #         current_pose[2]
                #     ])
                #     self.data_collector.write_cached_data(final_joint_positions)
                
                print(f"🏁 Controller: 所有任务完成，返回成功")
                return action, True, True
            
            # 否则继续等待下一个目标点（不返回成功，不触发重置）
            return action, False, False
        
        return action, False, False
    
    def _step_infer(self, state: Dict[str, Any]) -> Tuple[Any, bool, bool]:
        """
        Control step in inference mode (reserved interface).
        
        Args:
            state: The state dictionary
            
        Returns:
            tuple: (action, done, is_success)
        """
        if not self.waypoints_set and state.get('waypoints') is not None:
            self.ridgebase_controller.set_waypoints(state['waypoints'])
            self.waypoints_set = True
        
        # Get the current pose
        current_pose = state['current_pose']
        
        # Calculate the control action
        action, done = self.ridgebase_controller.get_action(current_pose)
        
        # If the navigation is complete
        if done or self.ridgebase_controller.is_path_complete():
            self._last_success = True
            self.reset_needed = True
            return action, True, True
        
        return action, False, False
    
    def _init_collect_mode(self, cfg, robot=None):
        """Initialize the collect mode"""
        from factories.collector_factory import create_collector
        self.data_collector = create_collector(
            cfg.collector.type,
            camera_configs=cfg.cameras,
            save_dir=cfg.multi_run.run_dir,
            max_episodes=cfg.max_episodes,
            compression=cfg.collector.compression
        )
    
    def _init_infer_mode(self, cfg, robot=None):
        """Initialize the inference mode (reserved interface)"""
        pass
    
    def get_language_instruction(self) -> Optional[str]:
        """
        Get the language instruction for the task.

        Returns:
            str: The language instruction
        """
        self._language_instruction = "Navigate to the target position"
        return self._language_instruction

