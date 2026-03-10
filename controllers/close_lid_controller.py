"""
Task controller for closing lid operations.
Manages the complete workflow of closing a hinged lid (e.g., centrifuge tube cap)
in both data collection and inference modes.
"""

import re
from typing import Optional
from robots.franka.rmpflow_controller import RMPFlowController
import numpy as np
from scipy.spatial.transform import Rotation as R

from controllers.atomic_actions.close_lid_controller import CloseLidController
from .base_controller import BaseController
from .robot_controllers.trajectory_controller import FrankaTrajectoryController
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from .inference_engines.inference_engine_factory import InferenceEngineFactory


class CloseLidTaskController(BaseController):
    """
    Controller for managing the task of closing a centrifuge lid.

    Motion sequence:
    1. Grasp the open lid edge
    2. Pull down in an arc to ~15 degrees
    3. Release and press to fully close

    Args:
        cfg: Configuration object containing mode and other parameters.
        robot: Robot articulation instance.
    """

    def __init__(self, cfg, robot):
        self.lid_type = cfg.task.get("lid_type", "centrifuge")
        self.arc_close_angle = cfg.task.get("arc_close_angle", 75.0)  # Close to 15° with arc
        self.final_close_angle = cfg.task.get("final_close_angle", 90.0)  # Full close
        self.lid_length = cfg.task.get("lid_length", 0.17)
        super().__init__(cfg, robot)
        self.initial_lid_position = None

    def _init_collect_mode(self, cfg, robot):
        """
        Initialize the controller for data collection mode.

        Creates the CloseLidController atomic action controller with RMPFlow
        for motion planning.

        Args:
            cfg: Configuration object for collect mode.
            robot: Robot articulation instance.
        """
        super()._init_collect_mode(cfg, robot)

        self.close_lid_controller = CloseLidController(
            name="close_lid_controller",
            cspace_controller=RMPFlowController(
                name="target_follower_controller",
                robot_articulation=robot
            ),
            gripper=robot.gripper,
            lid_type=self.lid_type,
            lid_length=self.lid_length,
            arc_close_angle=self.arc_close_angle,
            final_close_angle=self.final_close_angle,
        )

    def _init_infer_mode(self, cfg, robot):
        """
        Initialize components for inference mode.

        Creates inference engine and trajectory controller for running
        trained policies.

        Args:
            cfg: Configuration object containing model paths and settings
            robot: Robot instance to control
        """
        self.trajectory_controller = FrankaTrajectoryController(
            name="trajectory_controller",
            robot_articulation=robot
        )

        self.inference_engine = InferenceEngineFactory.create_inference_engine(
            cfg, self.trajectory_controller
        )

    def reset(self):
        """Reset the controller to its initial state."""
        super().reset()
        self.initial_lid_position = None
        if self.mode == "collect":
            self.close_lid_controller.reset()
        else:
            self.inference_engine.reset()

    def step(self, state):
        """
        Execute one step of the task based on the current state.

        Args:
            state: Current state of the environment containing:
                - joint_positions: Robot joint positions
                - object_position: Lid position (edge to push)
                - hinge_position: Position of the hinge (optional)
                - gripper_position: End effector position
                - camera_data: Camera images (for data collection)

        Returns:
            Tuple containing:
                - action: ArticulationAction or None
                - done: Whether the episode is complete
                - success: Whether the task succeeded
        """
        self.state = state

        if self.initial_lid_position is None:
            self.initial_lid_position = np.array(state['object_position']).copy()

        if self.mode == "collect":
            return self._step_collect(state)
        else:
            return self._step_infer(state)

    def _step_collect(self, state):
        """
        Execute a step in collect mode using the close lid controller.

        Args:
            state: Current state of the environment.

        Returns:
            Tuple containing the action, done flag, and success flag.
        """
        if not self.close_lid_controller.is_done():
            # Get hinge position if available
            hinge_position = state.get('hinge_position', None)

            # End effector orientation: angled to grasp the vertical lid edge
            # The gripper approaches from the front (negative Y direction)
            end_effector_orientation = euler_angles_to_quats(
                [0, 135, 0], degrees=True, extrinsic=False
            )

            action = self.close_lid_controller.forward(
                lid_position=np.array(state['object_position']),
                current_joint_positions=state['joint_positions'],
                gripper_position=state['gripper_position'],
                hinge_position=hinge_position,
                end_effector_orientation=end_effector_orientation,
            )

            # Cache data for training
            if 'camera_data' in state:
                self.data_collector.cache_step(
                    camera_images=state['camera_data'],
                    joint_angles=state['joint_positions'][:-1],
                    language_instruction=self.get_language_instruction()
                )

            # Check success
            if self._check_success(state):
                self.check_success_counter += 1
            else:
                self.check_success_counter = 0

            return action, False, False

        # Controller is done
        success = self.check_success_counter >= self.REQUIRED_SUCCESS_STEPS

        if success:
            print("Task success!")
            self.data_collector.write_cached_data(state['joint_positions'][:-1])
            self._last_success = True
        else:
            print("Task failed!")
            self.data_collector.clear_cache()
            self._last_success = False

        self.reset_needed = True
        return None, True, success

    def _step_infer(self, state):
        """
        Execute a step in infer mode using the trained policy.

        Args:
            state: Current state of the environment.

        Returns:
            Tuple containing the action, done flag, and success flag.
        """
        language_instruction = self.get_language_instruction()
        if language_instruction is not None:
            state['language_instruction'] = language_instruction
        else:
            state['language_instruction'] = "Close the lid of the container"

        action = self.inference_engine.step_inference(state)

        if self._check_success(state):
            self.check_success_counter += 1
        else:
            self.check_success_counter = 0

        success = self.check_success_counter >= self.REQUIRED_SUCCESS_STEPS

        if success:
            print("Task success!")
            self._last_success = True
            self.reset_needed = True
            return None, True, True

        return action, False, False

    def _check_success(self, state):
        """
        Check if the lid closing task has been successfully completed.

        Success criteria:
        1. The lid has moved significantly from its initial position
        2. The lid's Z position has decreased (moved downward)
        3. The gripper has retreated from the lid

        Args:
            state: Current state of the environment.

        Returns:
            bool: True if the task is successful, False otherwise.
        """
        current_pos = np.array(state['object_position'])
        gripper_position = np.array(state['gripper_position'])

        # Check if lid has moved down (Z decreased)
        z_movement = self.initial_lid_position[2] - current_pos[2]

        # Check total displacement
        total_displacement = np.linalg.norm(current_pos - self.initial_lid_position)

        # Check gripper has retreated
        gripper_distance = np.linalg.norm(gripper_position - current_pos)

        return (
            z_movement > 0.01 and  # Lid moved down at least 1cm
            total_displacement > 0.015 and  # Total movement > 1.5cm
            gripper_distance > 0.05  # Gripper retreated > 5cm
        )

    def get_language_instruction(self) -> Optional[str]:
        """
        Get the language instruction for the current task.

        Returns:
            Optional[str]: The language instruction describing the task
        """
        if not hasattr(self, 'state') or self.state is None:
            return "Close the lid of the centrifuge tube"

        object_name = self.state.get('object_name', 'container')
        # Clean up the object name
        object_name = re.sub(r'\d+', '', object_name).replace('_', ' ').strip().lower()

        self._language_instruction = f"Close the lid of the {object_name}"
        return self._language_instruction
