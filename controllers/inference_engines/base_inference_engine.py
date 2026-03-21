from abc import ABC, abstractmethod
import torch
import numpy as np
from collections import deque
from typing import Dict, Any, Optional, Tuple


class GripperStateLogger:
    """
    夹爪状态日志记录器

    用于在推理过程中记录和输出夹爪的状态信息，帮助调试夹爪张开/闭合逻辑
    """

    # 夹爪状态常量
    GRIPPER_OPEN = 0.04      # 完全打开 (米)
    GRIPPER_CLOSED = 0.0     # 完全闭合 (米)

    # 状态阈值
    OPEN_THRESHOLD = 0.03    # 大于此值认为夹爪打开
    CLOSED_THRESHOLD = 0.01  # 小于此值认为夹爪闭合

    def __init__(self, cfg):
        """
        初始化夹爪状态日志记录器

        Args:
            cfg: 配置对象，包含 gripper_debug 配置
        """
        self.enabled = False
        self.log_interval = 10
        self.log_action_values = True
        self._log_joint_positions = True  # 改为私有属性，避免与方法冲突
        self.log_phase_changes = True

        # 从配置中读取参数
        if hasattr(cfg, 'infer') and hasattr(cfg.infer, 'gripper_debug'):
            debug_cfg = cfg.infer.gripper_debug
            self.enabled = getattr(debug_cfg, 'enabled', False)
            self.log_interval = getattr(debug_cfg, 'log_interval', 10)
            self.log_action_values = getattr(debug_cfg, 'log_action_values', True)
            self._log_joint_positions = getattr(debug_cfg, 'log_joint_positions', True)
            self.log_phase_changes = getattr(debug_cfg, 'log_phase_changes', True)

        self.frame_count = 0
        self.last_gripper_state = None  # 'open', 'closed', 'partial'
        self.gripper_history = deque(maxlen=100)  # 保存最近的夹爪状态

        if self.enabled:
            print("\n" + "="*60)
            print("🔧 夹爪调试日志已启用")
            print("="*60)
            print(f"  日志间隔: 每 {self.log_interval} 帧")
            print(f"  夹爪打开阈值: > {self.OPEN_THRESHOLD*1000:.1f}mm")
            print(f"  夹爪闭合阈值: < {self.CLOSED_THRESHOLD*1000:.1f}mm")
            print(f"  完全打开位置: {self.GRIPPER_OPEN*1000:.1f}mm")
            print(f"  完全闭合位置: {self.GRIPPER_CLOSED*1000:.1f}mm")
            print("="*60 + "\n")

    def get_gripper_state_name(self, gripper_value: float) -> str:
        """
        根据夹爪值判断夹爪状态

        Args:
            gripper_value: 夹爪位置值（米）

        Returns:
            状态名称字符串
        """
        if gripper_value >= self.OPEN_THRESHOLD:
            return "🔓 打开 (OPEN)"
        elif gripper_value <= self.CLOSED_THRESHOLD:
            return "🔒 闭合 (CLOSED)"
        else:
            return "⚙️  半开 (PARTIAL)"

    def log_gripper_action(self, action: np.ndarray, phase: str = "inference"):
        """
        记录推理得到的夹爪动作

        Args:
            action: 推理得到的动作数组，最后一维是夹爪
            phase: 当前任务阶段
        """
        if not self.enabled or not self.log_action_values:
            return

        self.frame_count += 1

        # 从动作中提取夹爪值
        if action is not None:
            if len(action.shape) == 2:
                # action shape: [horizon, action_dim]
                # 夹爪通常是最后一个维度
                gripper_values = action[:, -1] if action.shape[1] >= 8 else None
                if gripper_values is not None:
                    current_gripper = gripper_values[0]  # 取第一个时间步的值
                else:
                    current_gripper = None
            elif len(action.shape) == 1:
                current_gripper = action[-1] if len(action) >= 8 else None
            else:
                current_gripper = None

            if current_gripper is not None:
                state_name = self.get_gripper_state_name(current_gripper)

                # 检测状态变化
                state_changed = False
                current_state = "open" if current_gripper >= self.OPEN_THRESHOLD else \
                               "closed" if current_gripper <= self.CLOSED_THRESHOLD else "partial"

                if self.last_gripper_state != current_state:
                    state_changed = True
                    self.last_gripper_state = current_state

                # 根据条件输出日志
                should_log = (self.frame_count % self.log_interval == 0) or state_changed

                if should_log or (self.log_phase_changes and state_changed):
                    print(f"\n{'='*50}")
                    print(f"📍 [Frame {self.frame_count}] 阶段: {phase}")
                    print(f"{'='*50}")
                    print(f"  🎯 推理夹爪值: {current_gripper*1000:.2f} mm")
                    print(f"  📊 夹爪状态: {state_name}")

                    if len(action.shape) == 2 and action.shape[1] >= 8:
                        # 显示完整的动作信息
                        print(f"  📈 动作序列 (前3步):")
                        for i, step_action in enumerate(action[:3]):
                            gripper_val = step_action[-1] * 1000
                            arm_joints = step_action[:7]
                            print(f"     步骤 {i+1}: 夹爪={gripper_val:.2f}mm, 关节=[{', '.join([f'{j:.3f}' for j in arm_joints[:3]])}...]")

                    if state_changed:
                        print(f"  ⚡ 状态变化: {self.last_gripper_state} -> {current_state}")

                    self.gripper_history.append({
                        'frame': self.frame_count,
                        'value': current_gripper,
                        'state': current_state,
                        'phase': phase
                    })

    def log_joint_positions(self, joint_positions: np.ndarray):
        """
        记录实际的关节位置（包括夹爪）

        Args:
            joint_positions: 完整的关节位置数组
        """
        if not self.enabled or not self._log_joint_positions:
            return

        if joint_positions is not None and len(joint_positions) >= 9:
            # Franka 有9个DOF: 7个机械臂关节 + 2个夹爪关节
            arm_joints = joint_positions[:7]
            gripper_left = joint_positions[7]
            gripper_right = joint_positions[8]

            print(f"\n  🔧 实际关节状态:")
            print(f"     左手指: {gripper_left*1000:.2f}mm | 右手指: {gripper_right*1000:.2f}mm")
            print(f"     机械臂关节: [{', '.join([f'{j:.3f}' for j in arm_joints])}]")

    def log_summary(self):
        """输出夹爪状态历史摘要"""
        if not self.enabled or len(self.gripper_history) == 0:
            return

        print("\n" + "="*60)
        print("📋 夹爪状态历史摘要")
        print("="*60)

        # 统计各状态出现次数
        state_counts = {'open': 0, 'closed': 0, 'partial': 0}
        for record in self.gripper_history:
            state_counts[record['state']] += 1

        print(f"  打开次数: {state_counts['open']}")
        print(f"  闭合次数: {state_counts['closed']}")
        print(f"  半开次数: {state_counts['partial']}")

        # 显示最近的状态变化
        print(f"\n  最近的状态记录:")
        for record in list(self.gripper_history)[-10:]:
            print(f"    Frame {record['frame']}: {record['value']*1000:.2f}mm ({record['state']}) - {record['phase']}")

        print("="*60 + "\n")


class BaseInferenceEngine(ABC):
    """
    Abstract inference engine base class, defining common inference processes and interfaces.

    All specific inference engines must inherit this class and implement the necessary abstract methods.
    """

    def __init__(self, cfg, trajectory_controller):
        """
        Initialize the inference engine base class

        Args:
            cfg: Configuration object
            trajectory_controller: Trajectory controller
        """
        self.cfg = cfg
        self.trajectory_controller = trajectory_controller

        # Allow device override from config, default to auto-detect
        device_override = getattr(cfg.infer, 'device', None)
        if device_override:
            self.device = torch.device(device_override)
        else:
            # Auto-detect: try CUDA, fallback to CPU if incompatible
            try:
                if torch.cuda.is_available():
                    # Test CUDA compatibility with a small tensor operation
                    test_tensor = torch.zeros(1, device="cuda")
                    _ = test_tensor + 1
                    self.device = torch.device("cuda")
                    del test_tensor
                else:
                    self.device = torch.device("cpu")
            except RuntimeError as e:
                print(f"[InferenceEngine] CUDA not compatible ({e}), falling back to CPU")
                self.device = torch.device("cpu")

        print(f"[InferenceEngine] Using device: {self.device}")

        # 初始化夹爪状态日志记录器
        self.gripper_logger = GripperStateLogger(cfg)

        # Observation related configuration
        self.obs_names = cfg.infer.obs_names
        self.camera_to_obs = {k: v for k, v in self.obs_names.items()}
        self.n_obs_steps = self._get_n_obs_steps()

        # Initialize observation history
        self.obs_history_dict = {
            obs_key: deque(maxlen=self.n_obs_steps)
            for obs_key in self.obs_names.values()
        }
        self.obs_history_pose = deque(maxlen=self.n_obs_steps)

        # Initialize language instruction history
        self.language_instruction = ""

        # Initialize inference engine
        self._init_inference_engine()

    @abstractmethod
    def _get_n_obs_steps(self) -> int:
        """Get the number of observation steps"""
        pass

    @abstractmethod
    def _init_inference_engine(self):
        """Initialize the specific inference engine"""
        pass

    @abstractmethod
    def _predict_action(self, obs_dict: Dict[str, torch.Tensor], language_instruction: str = "") -> np.ndarray:
        """
        Use specific inference methods to predict actions

        Args:
            obs_dict: Observation data dictionary
            language_instruction: Language instruction string, if not empty

        Returns:
            Predicted action array
        """
        pass

    def reset(self):
        """Reset the inference engine state"""
        self.obs_history_dict = {
            obs_key: deque(maxlen=self.n_obs_steps)
            for obs_key in self.obs_names.values()
        }
        self.obs_history_pose = deque(maxlen=self.n_obs_steps)
        self.language_instruction = ""
        self.trajectory_controller.reset()

    def update_observations(self, state: Dict[str, Any]):
        """
        Update observation history

        Args:
            state: Current state dictionary
        """
        for cam_name, image in state['camera_data'].items():
            if cam_name in self.camera_to_obs:
                obs_key = self.camera_to_obs[cam_name]
                self.obs_history_dict[obs_key].append(image)

        self.obs_history_pose.append(state['joint_positions'][:-1])

        if 'language_instruction' in state:
            self.language_instruction = state['language_instruction']
        else:
            self.language_instruction = ""

    def _check_histories_complete(self) -> bool:
        """Check if the observation history is complete"""
        return (
            len(self.obs_history_pose) == self.n_obs_steps and
            all(len(hist) == self.n_obs_steps for hist in self.obs_history_dict.values())
        )

    def _prepare_observation_dict(self) -> Dict[str, torch.Tensor]:
        """
        Prepare observation data dictionary

        Returns:
            Processed observation data dictionary
        """
        obs_dict = {
            obs_key: torch.from_numpy(np.stack(list(hist))).float().to(self.device) / 255.0
            for obs_key, hist in self.obs_history_dict.items()
        }
        obs_dict['agent_pose'] = torch.from_numpy(
            np.stack(list(self.obs_history_pose))
        ).float().to(self.device)

        for key in obs_dict.keys():
            if obs_dict[key].shape[0] != 1:
                obs_dict[key] = obs_dict[key].unsqueeze(0)

        return obs_dict

    def step_inference(self, state: Dict[str, Any], phase: str = "inference") -> Optional[np.ndarray]:
        """
        Execute one step of inference

        Args:
            state: Current state dictionary
            phase: Current task phase (e.g., "picking", "placing")

        Returns:
            Action array, if waiting return None
        """
        self.update_observations(state)

        if self.trajectory_controller.is_trajectory_complete() and self._check_histories_complete():
            obs_dict = self._prepare_observation_dict()

            joint_positions = self._predict_action(obs_dict, self.language_instruction)

            # 记录夹爪动作
            self.gripper_logger.log_gripper_action(joint_positions, phase=phase)

            self.trajectory_controller.generate_trajectory(joint_positions[:40, :])

        action = self.trajectory_controller.get_next_action()

        # 记录实际关节位置（如果状态中有）
        if action is not None and 'joint_positions' in state:
            self.gripper_logger.log_joint_positions(state['joint_positions'])

        return action

    def close(self):
        """Close the inference engine, release resources"""
        # 输出夹爪状态摘要
        self.gripper_logger.log_summary()
        pass