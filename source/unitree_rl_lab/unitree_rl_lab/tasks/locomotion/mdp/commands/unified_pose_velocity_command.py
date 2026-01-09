"""Unified command generator that combines pose (roll, pitch, height) and velocity (lin_vel_xy, ang_vel_z) commands."""

from __future__ import annotations

import torch
from collections.abc import Sequence
from dataclasses import MISSING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG, GREEN_ARROW_X_MARKER_CFG, RED_ARROW_X_MARKER_CFG

from isaaclab.utils import configclass


class UnifiedPoseVelocityCommand(CommandTerm):
    """Unified command generator for simultaneous pose and velocity control.
    
    This command generator produces 6D commands:
    - lin_vel_x, lin_vel_y, ang_vel_z (velocity commands)
    - roll, pitch, height (pose commands)
    
    The commands are sampled uniformly within the configured ranges.
    """

    cfg: UnifiedPoseVelocityCommandCfg
    """Configuration for the command generator."""

    def __init__(self, cfg: UnifiedPoseVelocityCommandCfg, env: ManagerBasedEnv):
        """Initialize the command generator.

        Args:
            cfg: The configuration for the command generator.
            env: The environment instance.
        """
        super().__init__(cfg, env)

        # Obtain the robot asset
        self.robot: Articulation = env.scene[cfg.asset_name]

        # Initialize command buffers
        # Command format: [lin_vel_x, lin_vel_y, ang_vel_z, roll, pitch, height]
        self.pose_command_w = torch.zeros(self._env.num_envs, 6, device=self._env.device)
        self.pose_command_b = torch.zeros_like(self.pose_command_w)

        # Metrics tracking
        self.metrics["error_lin_vel_xy"] = torch.zeros(self._env.num_envs, device=self._env.device)
        self.metrics["error_ang_vel_z"] = torch.zeros(self._env.num_envs, device=self._env.device)
        self.metrics["error_roll"] = torch.zeros(self._env.num_envs, device=self._env.device)
        self.metrics["error_pitch"] = torch.zeros(self._env.num_envs, device=self._env.device)
        self.metrics["error_height"] = torch.zeros(self._env.num_envs, device=self._env.device)

        # Standing environment tracking
        self.is_standing_env = torch.zeros(self._env.num_envs, dtype=torch.bool, device=self._env.device)

        # Command history for conditional penalties
        self.command_history = torch.zeros(self._env.num_envs, 6, device=self._env.device)

    def __str__(self) -> str:
        """Return a string representation of the command generator."""
        msg = "UnifiedPoseVelocityCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        msg += f"\tStanding envs: {self.cfg.rel_standing_envs * 100:.1f}%"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired unified command in base frame. Shape is (num_envs, 6)."""
        return self.pose_command_b

    """
    Implementation specific functions.
    """

    def _resample_command(self, env_ids: Sequence[int]):
        """Resample commands for specified environments.

        Args:
            env_ids: Environment indices to resample.
        """
        # Sample random values within configured ranges
        r = self.cfg.ranges

        # Velocity commands
        self.pose_command_w[env_ids, 0] = torch.empty(len(env_ids), device=self._env.device).uniform_(
            *r.lin_vel_x
        )
        self.pose_command_w[env_ids, 1] = torch.empty(len(env_ids), device=self._env.device).uniform_(
            *r.lin_vel_y
        )
        self.pose_command_w[env_ids, 2] = torch.empty(len(env_ids), device=self._env.device).uniform_(
            *r.ang_vel_z
        )

        # Pose commands
        self.pose_command_w[env_ids, 3] = torch.empty(len(env_ids), device=self._env.device).uniform_(
            *r.roll
        )
        self.pose_command_w[env_ids, 4] = torch.empty(len(env_ids), device=self._env.device).uniform_(
            *r.pitch
        )
        self.pose_command_w[env_ids, 5] = torch.empty(len(env_ids), device=self._env.device).uniform_(
            *r.height
        )

        # Handle standing environments
        num_standing = int(len(env_ids) * self.cfg.rel_standing_envs)
        if num_standing > 0:
            standing_indices = env_ids[:num_standing]
            self.pose_command_w[standing_indices, :3] = 0.0  # Zero velocity
            self.is_standing_env[standing_indices] = True
            self.is_standing_env[env_ids[num_standing:]] = False

        # Update command history
        self.command_history[env_ids] = self.pose_command_w[env_ids].clone()

    def _update_command(self):
        """Post-process the command signal before sending to the environment."""
        # Convert velocity commands from world to base frame
        base_quat_w = self.robot.data.root_quat_w
        
        # Rotate velocity commands to base frame
        self.pose_command_b[:, :2] = math_utils.quat_apply_inverse(
            base_quat_w, 
            torch.cat([self.pose_command_w[:, :2], torch.zeros(self._env.num_envs, 1, device=self._env.device)], dim=1)
        )[:, :2]
        
        # Angular velocity (yaw) and pose commands remain the same
        self.pose_command_b[:, 2:] = self.pose_command_w[:, 2:]

    def _update_metrics(self):
        """Update metrics for the command generator."""
        # Get current state
        root_quat_w = self.robot.data.root_quat_w
        base_lin_vel = self.robot.data.root_lin_vel_b
        base_ang_vel = self.robot.data.root_ang_vel_b
        base_pos_w = self.robot.data.root_pos_w

        # Velocity tracking errors
        self.metrics["error_lin_vel_xy"] = torch.norm(
            base_lin_vel[:, :2] - self.pose_command_b[:, :2], dim=1
        )
        self.metrics["error_ang_vel_z"] = torch.abs(base_ang_vel[:, 2] - self.pose_command_b[:, 2])

        # Pose tracking errors
        # Extract current roll and pitch from quaternion
        current_roll, current_pitch, _ = math_utils.euler_xyz_from_quat(root_quat_w)
        self.metrics["error_roll"] = torch.abs(current_roll - self.pose_command_b[:, 3])
        self.metrics["error_pitch"] = torch.abs(current_pitch - self.pose_command_b[:, 4])
        
        # Height error
        self.metrics["error_height"] = torch.abs(base_pos_w[:, 2] - self.pose_command_b[:, 5])

    def _set_debug_vis_impl(self, debug_vis: bool):
        """Configure debug visualization markers."""
        # Create markers if debug visualization is enabled
        if debug_vis:
            if not hasattr(self, "arrow_goal_visualizer"):
                marker_cfg = GREEN_ARROW_X_MARKER_CFG.copy()
                marker_cfg.prim_path = "/Visuals/Command/velocity_goal"
                marker_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)
                self.arrow_goal_visualizer = VisualizationMarkers(marker_cfg)
                
            if not hasattr(self, "arrow_current_visualizer"):
                marker_cfg = BLUE_ARROW_X_MARKER_CFG.copy()
                marker_cfg.prim_path = "/Visuals/Command/velocity_current"
                marker_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)
                self.arrow_current_visualizer = VisualizationMarkers(marker_cfg)
                
            # Pose visualization (height indicator)
            if not hasattr(self, "height_goal_visualizer"):
                marker_cfg = RED_ARROW_X_MARKER_CFG.copy()
                marker_cfg.prim_path = "/Visuals/Command/height_goal"
                marker_cfg.markers["arrow"].scale = (0.2, 0.2, 0.2)
                self.height_goal_visualizer = VisualizationMarkers(marker_cfg)

            # Set visibility
            self.arrow_goal_visualizer.set_visibility(True)
            self.arrow_current_visualizer.set_visibility(True)
            self.height_goal_visualizer.set_visibility(True)
        else:
            if hasattr(self, "arrow_goal_visualizer"):
                self.arrow_goal_visualizer.set_visibility(False)
            if hasattr(self, "arrow_current_visualizer"):
                self.arrow_current_visualizer.set_visibility(False)
            if hasattr(self, "height_goal_visualizer"):
                self.height_goal_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        """Callback for debug visualization."""
        # Get robot base position
        base_pos_w = self.robot.data.root_pos_w.clone()
        base_pos_w[:, 2] += 0.5  # Raise above robot
        
        # Convert velocity commands to arrow quaternions and scales
        vel_des_arrow_scale, vel_des_arrow_quat = self._resolve_xy_velocity_to_arrow(
            self.pose_command_w[:, :2]
        )
        vel_arrow_scale, vel_arrow_quat = self._resolve_xy_velocity_to_arrow(
            self.robot.data.root_lin_vel_b[:, :2]
        )
        
        # Visualize commanded height
        height_pos = base_pos_w.clone()
        height_pos[:, 2] = self.pose_command_w[:, 5]  # Commanded height
        height_arrow_scale = torch.tensor([0.02, 0.02, 0.1], device=self._env.device).repeat(
            self._env.num_envs, 1
        )
        # Identity quaternion (no rotation for vertical arrow)
        height_arrow_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=self._env.device).repeat(
            self._env.num_envs, 1
        )
        
        # Update markers
        self.arrow_goal_visualizer.visualize(base_pos_w, vel_des_arrow_quat, vel_des_arrow_scale)
        self.arrow_current_visualizer.visualize(base_pos_w, vel_arrow_quat, vel_arrow_scale)
        self.height_goal_visualizer.visualize(height_pos, height_arrow_quat, height_arrow_scale)

    def _resolve_xy_velocity_to_arrow(
        self, xy_velocity: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Converts the XY base velocity command to arrow direction rotation.
        
        Args:
            xy_velocity: The XY velocity command in base frame, shape (num_envs, 2).
            
        Returns:
            A tuple of (arrow_scale, arrow_quat) where:
            - arrow_scale: Scale of the arrow marker, shape (num_envs, 3)
            - arrow_quat: Quaternion rotation of the arrow in world frame, shape (num_envs, 4)
        """
        # Obtain default scale of the marker
        default_scale = self.arrow_goal_visualizer.cfg.markers["arrow"].scale
        # Arrow scale
        arrow_scale = torch.tensor(default_scale, device=self.device).repeat(xy_velocity.shape[0], 1)
        arrow_scale[:, 0] *= torch.linalg.norm(xy_velocity, dim=1) * 3.0
        # Arrow direction (rotation around Z axis based on velocity direction)
        heading_angle = torch.atan2(xy_velocity[:, 1], xy_velocity[:, 0])
        zeros = torch.zeros_like(heading_angle)
        arrow_quat = math_utils.quat_from_euler_xyz(zeros, zeros, heading_angle)
        # Convert from base frame to world frame
        base_quat_w = self.robot.data.root_quat_w
        arrow_quat = math_utils.quat_mul(base_quat_w, arrow_quat)
        
        return arrow_scale, arrow_quat


@configclass
class UnifiedPoseVelocityCommandCfg(CommandTermCfg):
    """Configuration for unified pose & velocity command generator."""

    class_type: type = UnifiedPoseVelocityCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    resampling_time_range: tuple[float, float] = MISSING
    """Time range for resampling commands (in seconds)."""

    rel_standing_envs: float = 0.0
    """Relative number of environments that should be standing still."""

    debug_vis: bool = True
    """Whether to visualize debug information."""

    @configclass
    class Ranges:
        """Uniform distribution ranges for unified commands."""
        
        # Velocity command ranges
        lin_vel_x: tuple[float, float] = MISSING  # min/max x-velocity (m/s)
        lin_vel_y: tuple[float, float] = MISSING  # min/max y-velocity (m/s)
        ang_vel_z: tuple[float, float] = MISSING  # min/max yaw rate (rad/s)
        
        # Pose command ranges
        roll: tuple[float, float] = MISSING       # min/max roll angle (rad)
        pitch: tuple[float, float] = MISSING      # min/max pitch angle (rad)
        height: tuple[float, float] = MISSING     # min/max height (m)

    ranges: Ranges = MISSING
    """Distribution ranges for different command components."""
