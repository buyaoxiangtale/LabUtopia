"""
Controller for closing a centrifuge lid with grip + arc motion + press.

Motion sequence:
1. Approach lid edge from above
2. Open gripper
3. Align with lid edge
4. Close gripper (grasp lid)
5. Arc motion - pull lid down to ~10-15 degrees
6. Open gripper (release lid)
7. Move above the lid
8. Press down to fully close
9. Retreat
"""

from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction
import numpy as np
import typing
from isaacsim.core.utils.rotations import euler_angles_to_quat
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R


class CloseLidController(BaseController):
    """
    Controller for closing a centrifuge lid using grip + arc motion + press.

    The robot:
    1. Grasps the open lid edge
    2. Pulls it down in an arc (rotating around X-axis)
    3. Releases and presses to fully close

    Coordinate system (based on centrifuge_eppendorf_5430.usd):
    - X-axis: Rotation axis of the lid hinge
    - Y-axis: Front-back direction (hinge at back, +Y)
    - Z-axis: Vertical (up is positive)
    """

    def __init__(
        self,
        name: str,
        cspace_controller: BaseController,
        gripper=None,
        events_dt: typing.Optional[typing.List[float]] = None,
        lid_type: str = "centrifuge",
        lid_length: float = 0.17,
        arc_close_angle: float = 75.0,  # Close to this angle with arc motion
        final_close_angle: float = 90.0,  # Total angle (arc + press)
    ) -> None:
        """
        Initialize the CloseLidController.

        Args:
            name: Controller name
            cspace_controller: Configuration space controller
            gripper: Robot gripper
            events_dt: Time step for each event phase
            lid_type: Type of lid
            lid_length: Distance from hinge to lid edge (meters)
            arc_close_angle: Angle to close with arc motion (degrees)
            final_close_angle: Total closing angle (degrees)
        """
        BaseController.__init__(self, name=name)
        self._event = 0
        self._t = 0
        self._cspace_controller = cspace_controller
        self._gripper = gripper
        self.lid_type = lid_type
        self.lid_length = lid_length
        self.arc_close_angle = arc_close_angle
        self.final_close_angle = final_close_angle
        self.position_rotation_interp_iter = None
        self.init_lid_position = None
        self.hinge_position = None
        self._start = True

        # Event phases:
        # 0: Approach (move above lid)
        # 1: Open gripper
        # 2: Align (position at lid edge)
        # 3: Close gripper (grasp)
        # 4: Arc motion (pull down)
        # 5: Open gripper (release)
        # 6: Move above lid
        # 7: Press down
        # 8: Retreat
        if events_dt is None:
            self._events_dt = [0.002, 0.05, 0.003, 0.05, 0.004, 0.05, 0.003, 0.003, 0.008]
        else:
            self._events_dt = events_dt
            if len(self._events_dt) != 9:
                raise Exception(f"events_dt length must be 9, got {len(self._events_dt)}")

        self._position_threshold = 0.015 / get_stage_units()

    def forward(
        self,
        lid_position: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
        hinge_position: np.ndarray = None,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """
        Execute one step of lid closing control.

        Args:
            lid_position: Current position of the lid edge
            current_joint_positions: Current robot joint positions
            gripper_position: Current gripper position
            hinge_position: Position of the hinge
            end_effector_orientation: Target end effector orientation

        Returns:
            ArticulationAction: Control action
        """
        # Handle start state - open gripper first
        if self._start:
            self._start = False
            return self._open_gripper(current_joint_positions)

        # Default orientation: gripper approaching from front, angled to grasp lid edge
        if end_effector_orientation is None:
            # Gripper tilted forward to grasp the vertical lid edge
            end_effector_orientation = euler_angles_to_quat(
                [0, 135, 0], degrees=True, extrinsic=False
            )

        # Store initial lid edge position (lid_position is now the edge, not center)
        if self.init_lid_position is None:
            self.init_lid_position = lid_position.copy()
            print(f"[CloseLidController] Initial lid edge position: {self.init_lid_position}")

        # Get or estimate hinge position
        # NOTE: lid_position is the EDGE position, hinge is at back (+Y), same height
        if hinge_position is not None and self.hinge_position is None:
            self.hinge_position = np.array(hinge_position).copy()
            print(f"[CloseLidController] Using provided hinge position: {self.hinge_position}")
        elif self.hinge_position is None:
            # Fallback: estimate hinge from edge position
            # Geometry: hinge is behind the edge (+Y direction), at SAME HEIGHT
            self.hinge_position = self.init_lid_position.copy()
            self.hinge_position[1] += self.lid_length  # Hinge is behind edge
            # hinge_z stays same as edge_z (same height!)
            print(f"[CloseLidController] Estimated hinge position: {self.hinge_position}")

        self._t += self._events_dt[self._event]

        target_joint_positions = self._execute_phase(
            lid_position,
            end_effector_orientation,
            current_joint_positions,
            gripper_position,
        )

        if self._t >= 1.0:
            self._event += 1
            self._t = 0

        return target_joint_positions

    def _calculate_lid_edge_position(self) -> np.ndarray:
        """
        Calculate the actual front edge position of the open lid.

        When the lid is open at 90 degrees (vertical):
        - The lid rotates around X-axis (hinge at back)
        - Edge rotates from above hinge to in front of hinge
        - edge_y = hinge_y - lid_length (in front of hinge)
        - edge_z = hinge_z (at hinge height, NOT above it)

        Geometry:
            关闭时 (0°):  边缘在铰链正上方
            打开时 (90°): 边缘在铰链正前方，与铰链同高

        Returns:
            np.ndarray: Position of the lid's front edge
        """
        edge_position = self.hinge_position.copy()
        # When lid is vertical (open 90°):
        # - Edge is in front of hinge (negative Y direction) by lid_length
        # - Edge is at the SAME height as hinge (not above)
        edge_position[1] -= self.lid_length  # Y: 在铰链前方 lid_length 距离
        # edge_position[2] 保持不变 = hinge_z (与铰链同高)
        return edge_position

    def _calculate_lid_center_position(self) -> np.ndarray:
        """
        Calculate the center position of the lid.

        When lid is nearly horizontal (after arc motion at ~15°):
        - Center is halfway between hinge and edge
        - Center Y = hinge_y (midpoint between hinge and edge)
        - Center Z ≈ hinge_z (nearly horizontal, close to hinge height)

        Geometry:
            盖子长度 = lid_length
            铰链位置 Y = hinge_y
            边缘位置 Y = hinge_y - lid_length
            中央位置 Y = hinge_y - lid_length/2

        Returns:
            np.ndarray: Position of the lid's center
        """
        center_position = self.hinge_position.copy()
        # Center is halfway between hinge and edge
        # Center Y = hinge_y - lid_length/2
        center_position[1] -= self.lid_length / 2
        # Z stays at hinge level (lid is nearly horizontal after arc motion)
        return center_position

    def _execute_phase(
        self,
        lid_position: np.ndarray,
        end_effector_orientation: np.ndarray,
        current_joint_positions: np.ndarray,
        gripper_position: np.ndarray,
    ) -> ArticulationAction:
        """Execute the current phase of the lid closing action."""

        # lid_position is now directly the edge position (calculated in Task)
        lid_edge_position = lid_position.copy()

        if self._event == 0:
            # Phase 0: Approach - move in front of and above the lid edge
            target_position = lid_edge_position.copy()
            target_position[1] -= 0.02  # 2cm in front of edge
            target_position[2] += 0.40  # 40cm above edge

            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation
            )

            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold * 2:
                self._event += 1
                self._t = 0
                print(f"[CloseLidController] Phase 0 complete: Approached lid edge at {lid_edge_position}")

        elif self._event == 1:
            # Phase 1: Open gripper
            target_joint_positions = self._open_gripper(current_joint_positions)

        elif self._event == 2:
            # Phase 2: Align - position gripper at lid edge for grasping
            # Use edge position, with small offset for gripper approach
            target_position = lid_edge_position.copy()
            # target_position[0] -= 0.05
            target_position[1] += 0.28  # 1cm in front of edge for gripper clearance
            target_position[2] += 0.3  # 5cm above edge
            # target_position[0] -= 0.2
            # target_position[1] += 0.312  # 1cm in front of edge for gripper clearance
            # target_position[2] += 0.15  # 5cm above edge


            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=end_effector_orientation
            )

            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold:
                self._event += 1
                self._t = 0
                print(f"[CloseLidController] Phase 2 complete: Aligned with lid edge at {target_position}")

        elif self._event == 3:
            # Phase 3: Close gripper - grasp the lid edge
            target_joint_positions = self._close_gripper(current_joint_positions)

        elif self._event == 4:
            # Phase 4: Arc motion - pull lid down
            if self.position_rotation_interp_iter is None:
                self.arc_start_position = gripper_position.copy()

                # Calculate target position after arc rotation
                self.arc_target_position = self.rotate_around_x_axis(
                    self.arc_start_position,
                    self.hinge_position,
                    -self.arc_close_angle
                )

                # End effector orientation at end of arc
                self.arc_end_orientation = self.rotate_quaternion_around_x(
                    end_effector_orientation,
                    -self.arc_close_angle
                )

                # Generate arc trajectory
                arc_length = np.linalg.norm(self.arc_start_position - self.arc_target_position)
                num_interpolation = max(int(400 * arc_length), 80)
                alphas = np.linspace(start=0, stop=1, num=num_interpolation)[1:]

                position_rotation_interp_list = self.arc_interpolation_x_axis(
                    self.arc_start_position,
                    end_effector_orientation,
                    self.arc_target_position,
                    self.arc_end_orientation,
                    alphas,
                    hinge_pos=self.hinge_position
                )
                self.position_rotation_interp_iter = iter(position_rotation_interp_list)
                print(f"[CloseLidController] Phase 4: Starting arc motion with {len(position_rotation_interp_list)} points")

            try:
                self.trans_interp, self.rotation_interp = next(self.position_rotation_interp_iter)
                target_joint_positions = self._cspace_controller.forward(
                    target_end_effector_position=self.trans_interp,
                    target_end_effector_orientation=self.rotation_interp
                )
            except StopIteration:
                self._event += 1
                self._t = 0
                print(f"[CloseLidController] Phase 4 complete: Arc motion done, lid at ~{90 - self.arc_close_angle}°")
                target_joint_positions = self._cspace_controller.forward(
                    target_end_effector_position=self.trans_interp,
                    target_end_effector_orientation=self.rotation_interp
                )

        elif self._event == 5:
            # Phase 5: Open gripper - release the lid
            target_joint_positions = self._open_gripper(current_joint_positions)

        elif self._event == 6:
            # Phase 6: Move above the lid CENTER for pressing
            # The lid is now nearly horizontal, move above its center
            press_orientation = euler_angles_to_quat(
                [0, 180, 0], degrees=True, extrinsic=False  # Straight down
            )

            # Calculate lid center position for pressing
            # Center is between hinge and edge, at lid_length/2 from hinge
            lid_center_position = self._calculate_lid_center_position()

            target_position = lid_center_position.copy()
            target_position[2] += 0.08  # Above the lid center

            print(f"[CloseLidController] Phase 6: Moving to lid center {target_position}")

            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=press_orientation
            )

            distance = np.linalg.norm(gripper_position - target_position)
            if distance < self._position_threshold * 2:
                self._event += 1
                self._t = 0
                self.press_start_position = gripper_position.copy()
                self.lid_center_for_press = target_position.copy()  # Store for Phase 7
                print(f"[CloseLidController] Phase 6 complete: Positioned above lid CENTER for press")

        elif self._event == 7:
            # Phase 7: Press down at lid CENTER to fully close the lid
            press_orientation = euler_angles_to_quat(
                [0, 180, 0], degrees=True, extrinsic=False
            )

            # Press target: move down to close the remaining angle
            remaining_angle = self.final_close_angle - self.arc_close_angle
            press_distance = self.lid_length * np.sin(np.radians(remaining_angle))

            # Use lid center position for pressing (not edge position)
            if hasattr(self, 'lid_center_for_press') and self.lid_center_for_press is not None:
                target_position = self.lid_center_for_press.copy()
            elif hasattr(self, 'press_start_position'):
                target_position = self.press_start_position.copy()
            else:
                target_position = gripper_position.copy()

            target_position[2] -= (0.08 + press_distance + 0.02)  # Press down

            # Get movement action
            move_action = self._cspace_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=press_orientation
            )

            # Create joint positions array with correct size
            num_joints = current_joint_positions.shape[0]
            joint_positions = [None] * num_joints

            # Copy movement joint positions if available
            if move_action.joint_positions is not None:
                for i, pos in enumerate(move_action.joint_positions):
                    if i < num_joints:
                        joint_positions[i] = pos

            # Set gripper to closed position (indices 7 and 8 for Franka)
            gripper_close = 0.01 / get_stage_units()
            if num_joints > 8:
                joint_positions[7] = gripper_close
                joint_positions[8] = gripper_close

            target_joint_positions = ArticulationAction(joint_positions=joint_positions)

            z_distance = abs(gripper_position[2] - target_position[2])
            if z_distance < 0.03:
                self._event += 1
                self._t = 0
                print(f"[CloseLidController] Phase 7 complete: Lid pressed closed")

        elif self._event == 8:
            # Phase 8: Retreat
            retreat_position = gripper_position.copy()
            retreat_position[2] += 0.15  # Move up

            press_orientation = euler_angles_to_quat(
                [0, 180, 0], degrees=True, extrinsic=False
            )

            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=retreat_position,
                target_end_effector_orientation=press_orientation
            )

            distance = np.linalg.norm(gripper_position - retreat_position)
            if distance < self._position_threshold * 3:
                self._event += 1
                self._t = 0
                print(f"[CloseLidController] Phase 8 complete: Retreated")

        else:
            # Done
            target_joint_positions = ArticulationAction(
                joint_positions=[None] * current_joint_positions.shape[0]
            )

        return target_joint_positions

    def _open_gripper(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        """Open the gripper."""
        num_joints = current_joint_positions.shape[0]
        target_joint_positions = [None] * num_joints
        gripper_open = 0.04 / get_stage_units()
        # Set gripper positions with bounds check (indices 7 and 8 for Franka)
        if num_joints > 8:
            target_joint_positions[7] = gripper_open
            target_joint_positions[8] = gripper_open
        elif num_joints > 7:
            target_joint_positions[7] = gripper_open
        return ArticulationAction(joint_positions=target_joint_positions)

    def _close_gripper(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        """Close the gripper."""
        num_joints = current_joint_positions.shape[0]
        target_joint_positions = [None] * num_joints
        gripper_close = 0.01 / get_stage_units()
        # Set gripper positions with bounds check (indices 7 and 8 for Franka)
        if num_joints > 8:
            target_joint_positions[7] = gripper_close
            target_joint_positions[8] = gripper_close
        elif num_joints > 7:
            target_joint_positions[7] = gripper_close
        return ArticulationAction(joint_positions=target_joint_positions)

    def reset(self) -> None:
        """Reset controller state."""
        BaseController.reset(self)
        self._event = 0
        self._t = 0
        self.position_rotation_interp_iter = None
        self.init_lid_position = None
        self.hinge_position = None
        self._start = True
        self.lid_center_for_press = None  # Reset lid center position for pressing

    def is_done(self) -> bool:
        """Check if controller has completed all phases."""
        return self._event >= len(self._events_dt)

    def rotate_around_x_axis(
        self,
        point: np.ndarray,
        center: np.ndarray,
        angle_deg: float
    ) -> np.ndarray:
        """
        Rotate a point around the X-axis passing through a center point.
        """
        angle_rad = np.deg2rad(angle_deg)
        point_relative = point - center

        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle_rad), -np.sin(angle_rad)],
            [0, np.sin(angle_rad), np.cos(angle_rad)]
        ])

        point_rotated_relative = rotation_matrix @ point_relative
        point_rotated = point_rotated_relative + center

        return point_rotated

    def rotate_quaternion_around_x(
        self,
        q: np.ndarray,
        angle_deg: float
    ) -> np.ndarray:
        """Rotate a quaternion around the X-axis."""
        angle_rad = np.deg2rad(angle_deg)

        # Quaternion for rotation around X-axis
        q_rot = np.array([
            np.cos(angle_rad / 2),
            np.sin(angle_rad / 2),
            0,
            0
        ])

        # Convert to scipy format and multiply
        r = R.from_quat([q[1], q[2], q[3], q[0]])
        r_rot = R.from_quat([q_rot[1], q_rot[2], q_rot[3], q_rot[0]])
        r_new = r * r_rot

        result = r_new.as_quat()
        return np.array([result[3], result[0], result[1], result[2]])

    def arc_interpolation_x_axis(
        self,
        trans_start: np.ndarray,
        rotation_start: np.ndarray,
        trans_end: np.ndarray,
        rotation_end: np.ndarray,
        alphas: np.ndarray,
        hinge_pos: np.ndarray
    ) -> typing.List[typing.Tuple[np.ndarray, np.ndarray]]:
        """
        Interpolate positions and orientations along an arc around the X-axis.
        """
        action_list = []

        # Quaternion SLERP
        rotation_start_xyzw = rotation_start[[1, 2, 3, 0]]
        rotation_end_xyzw = rotation_end[[1, 2, 3, 0]]
        key_rots = R.from_quat(np.stack([rotation_start_xyzw, rotation_end_xyzw]))
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)
        interp_rots = slerp(alphas).as_quat()
        interp_rots = interp_rots[:, [3, 0, 1, 2]]

        # Arc in YZ plane (rotation around X)
        r_start = trans_start[1:] - hinge_pos[1:]
        r_end = trans_end[1:] - hinge_pos[1:]

        radius_start = np.linalg.norm(r_start)
        radius_end = np.linalg.norm(r_end)
        radii = np.linspace(radius_start, radius_end, len(alphas) + 1)[1:]

        theta_start = np.arctan2(r_start[1], r_start[0])
        theta_end = np.arctan2(r_end[1], r_end[0])

        delta_theta = theta_end - theta_start
        if delta_theta > np.pi:
            delta_theta -= 2 * np.pi
        elif delta_theta < -np.pi:
            delta_theta += 2 * np.pi

        thetas = np.linspace(theta_start, theta_start + delta_theta, len(alphas) + 1)[1:]

        for alpha, radius, theta, interp_rot in zip(alphas, radii, thetas, interp_rots):
            trans_interp = np.array([
                alpha * trans_end[0] + (1 - alpha) * trans_start[0],
                hinge_pos[1] + radius * np.cos(theta),
                hinge_pos[2] + radius * np.sin(theta),
            ])
            action_list.append((trans_interp, interp_rot))

        return action_list
