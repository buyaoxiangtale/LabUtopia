"""
Task class for closing lid operations in simulation.
Manages the environment setup and state collection for lid closing tasks.
"""

import os
from .single_object_task import SingleObjectTask
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage


class CloseLidTask(SingleObjectTask):
    """
    A task class for robotic lid closing operations.

    This task handles closing hinged lids that rotate around the X-axis,
    such as centrifuge tube caps, vial lids, and sample container covers.

    The task manages:
    - Object placement and visibility
    - State information collection (lid position, hinge position)
    - Camera data acquisition
    - Success condition tracking
    """

    def __init__(self, cfg, world, stage, robot):
        """
        Initialize the CloseLidTask.

        Args:
            cfg: Task configuration containing object paths, camera settings, etc.
            world: Isaac Sim world instance
            stage: USD stage instance
            robot: Robot articulation instance
        """
        super().__init__(cfg, world, stage, robot)
        self.lid_sub_path = cfg.task.get("lid_path", None)
        self.hinge_sub_path = cfg.task.get("hinge_path", None)
        self.body_sub_path = cfg.task.get("body_path", None)

    def setup_objects(self) -> None:
        """
        Set up objects in the simulation world.
        Supports both existing scene objects and dynamic USD loading.
        """
        self.obj_configs = []

        if hasattr(self.cfg, 'task') and hasattr(self.cfg.task, 'obj_paths'):
            for obj in self.cfg.task.obj_paths:
                if isinstance(obj, str):
                    obj_config = {
                        'path': obj,
                        'position_range': {
                            'x': [0.30, 0.35],
                            'y': [-0.05, 0.05],
                            'z': [0.85, 0.85]
                        }
                    }
                else:
                    obj_config = dict(obj)

                # Check if we need to load USD file dynamically
                if 'usd_path' in obj_config:
                    usd_file = obj_config['usd_path']
                    prim_path = obj_config['path']

                    # Add the USD reference to the stage
                    if os.path.exists(usd_file):
                        add_reference_to_stage(
                            usd_path=os.path.abspath(usd_file),
                            prim_path=prim_path
                        )
                        print(f"Added USD reference: {usd_file} -> {prim_path}")
                    else:
                        print(f"Warning: USD file not found: {usd_file}")

                self.obj_configs.append(obj_config)

    def reset(self):
        """
        Reset the task state.

        Initializes robot position, updates materials, and places objects.
        Sets up the paths to the lid and hinge for state tracking.
        """
        super().reset()

        # Build full paths for sub-objects
        # Handle both absolute paths and relative paths
        def build_full_path(sub_path, default_name):
            if sub_path:
                if sub_path.startswith("/"):
                    return sub_path
                else:
                    return self.current_obj_path + "/" + sub_path
            else:
                return self.current_obj_path + "/" + default_name

        self.current_lid_path = build_full_path(self.lid_sub_path, "Group")
        self.current_body_path = build_full_path(self.body_sub_path, "Group_01")

        if self.hinge_sub_path:
            self.current_hinge_path = build_full_path(self.hinge_sub_path, None)
        else:
            # Will estimate hinge position from body/lid position
            self.current_hinge_path = None

        print(f"[CloseLidTask] Reset - Object: {self.current_obj_path}")
        print(f"[CloseLidTask] Reset - Lid path: {self.current_lid_path}")
        print(f"[CloseLidTask] Reset - Body path: {self.current_body_path}")

    def step(self):
        """
        Execute one simulation step and return current state.

        Returns:
            dict: Current state dictionary containing:
                - joint_positions: Robot joint positions
                - object_position: Lid edge position (where to push)
                - object_size: Lid dimensions
                - hinge_position: Position of the hinge (rotation center)
                - body_position: Position of the container body
                - camera_data: Camera image data
                - done: Whether episode is complete
                - object_name: Name of current target object
                - gripper_position: End effector position
        """
        self.frame_idx += 1

        if not self.check_frame_limits():
            return None

        # Debug: print paths on first frame
        if self.frame_idx == 6:
            print(f"[CloseLidTask] current_obj_path: {self.current_obj_path}")
            print(f"[CloseLidTask] current_lid_path: {self.current_lid_path}")
            print(f"[CloseLidTask] current_body_path: {self.current_body_path}")

        # Get lid center position and size
        lid_center = self.object_utils.get_geometry_center(
            object_path=self.current_lid_path
        )
        lid_size = self.object_utils.get_object_size(
            object_path=self.current_lid_path
        )

        # Check if lid position is valid
        if lid_center is None:
            print(f"[CloseLidTask] Warning: Could not get lid position from path: {self.current_lid_path}")
            # Try to get position from parent object
            lid_center = self.object_utils.get_geometry_center(
                object_path=self.current_obj_path
            )
            if lid_center is None:
                print(f"[CloseLidTask] Error: Could not get any valid position")
                return None

        if lid_size is None:
            lid_size = np.array([0.1, 0.17, 0.02])  # Default size estimate [X, Y, Z]

        # Calculate lid edge position from center
        # Geometry: lid is vertical (open 90°), hinge at back (+Y), edge at front (-Y)
        # Edge position = center - (lid_length/2) in Y direction
        lid_length = lid_size[1]  # Y dimension is the lid length
        lid_edge_position = lid_center.copy()
        lid_edge_position[1] -= lid_length / 2  # Edge is at front of lid center

        print(f"[CloseLidTask] Lid center: {lid_center}, Edge: {lid_edge_position}, Length: {lid_length}")

        # Get body position
        body_position = None
        try:
            body_position = self.object_utils.get_geometry_center(
                object_path=self.current_body_path
            )
        except Exception as e:
            print(f"[CloseLidTask] Warning: Could not get body position: {e}")

        # Get hinge position if path is specified
        hinge_position = None
        if self.current_hinge_path:
            try:
                hinge_position = self.object_utils.get_geometry_center(
                    object_path=self.current_hinge_path
                )
            except Exception:
                hinge_position = None

        # If no hinge position, estimate it from lid geometry
        # Geometry: hinge is at back of lid (+Y), at SAME HEIGHT as edge (not below!)
        if hinge_position is None:
            hinge_position = lid_center.copy()
            hinge_position[1] += lid_length / 2  # Hinge is at back of lid center
            # hinge_position[2] stays the same as lid_center (same height as edge)
            print(f"[CloseLidTask] Estimated hinge position: {hinge_position}")

        additional_info = {
            'object_position': lid_edge_position,  # Use edge position (where to grasp)
            'lid_center': lid_center,              # Also provide center for reference
            'object_size': lid_size,
            'lid_length': lid_length,              # Explicitly provide lid length
            'hinge_position': hinge_position,
            'body_position': body_position,
            'lid_path': self.current_lid_path,
        }

        # Add revolute joint info if available (for physics-based simulation)
        joint_path = None
        if hasattr(self.cfg.task, 'joint_path') and self.cfg.task.joint_path:
            joint_path = self.cfg.task.joint_path
            if not joint_path.startswith("/"):
                joint_path = self.current_obj_path + "/" + joint_path

        if joint_path:
            try:
                joint_position = self.object_utils.get_revolute_joint_positions(
                    joint_path=joint_path
                )
                additional_info['revolute_joint_position'] = joint_position
            except Exception as e:
                print(f"[CloseLidTask] Warning: Could not get joint position from {joint_path}: {e}")

        return self.get_basic_state_info(
            object_path=self.current_obj_path,
            additional_info=additional_info
        )
