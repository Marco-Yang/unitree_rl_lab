"""Configuration for Go2 robot velocity & pose control task.

This configuration implements a unified locomotion task where the robot tracks:
- Linear velocity commands (x, y in base frame)
- Angular velocity command (yaw)
- Roll angle command
- Pitch angle command  
- Height command

This combines the original Velocity task with additional pose control capabilities.
Curriculum learning is supported to gradually expand the command ranges.
"""

import math

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG as ROBOT_CFG
from unitree_rl_lab.tasks.locomotion import mdp

# Flat terrain for initial curriculum stage
FLAT_TERRAIN_CFG = terrain_gen.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=1.0),
    },
)


@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=FLAT_TERRAIN_CFG,
        max_init_terrain_level=0,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )
    # robots
    robot: ArticulationCfg = ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # sensors
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    
    # lights
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


@configclass
class EventCfg:
    """Configuration for events."""

    # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.2),
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.15),
            "num_buckets": 64,
        },
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )

    # reset
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),
            "velocity_range": (-1.0, 1.0),
        },
    )

    # interval
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(8.0, 15.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}},
    )


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    # Unified pose & velocity command
    pose_command = mdp.UnifiedPoseVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.0,
        debug_vis=True,
        ranges=mdp.UnifiedPoseVelocityCommandCfg.Ranges(
            # Velocity commands (from velocity task)
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(-1.0, 1.0),
            ang_vel_z=(-1.0, 1.0),
            # Pose commands
            roll=(0.0, 0.0),      # Curriculum start: zero roll
            pitch=(0.0, 0.0),     # Curriculum start: zero pitch  
            height=(0.35, 0.35),  # Curriculum start: default Go2 height
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    JointPositionAction = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=[".*"], 
        scale=0.25, 
        use_default_offset=True, 
        clip={".*": (-100.0, 100.0)}
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Robot state observations
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel, 
            scale=2.0, 
            clip=(-100, 100), 
            noise=Unoise(n_min=-0.1, n_max=0.1)
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, 
            scale=0.2, 
            clip=(-100, 100), 
            noise=Unoise(n_min=-0.2, n_max=0.2)
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity, 
            clip=(-100, 100), 
            noise=Unoise(n_min=-0.05, n_max=0.05)
        )
        
        # Unified commands (velocity + pose: 6-dim total)
        pose_commands = ObsTerm(
            func=mdp.generated_commands, 
            clip=(-100, 100), 
            params={"command_name": "pose_command"}
        )
        
        # Joint state
        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel, 
            clip=(-100, 100), 
            noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel, 
            scale=0.05, 
            clip=(-100, 100), 
            noise=Unoise(n_min=-1.5, n_max=1.5)
        )
        
        # Action history
        last_action = ObsTerm(func=mdp.last_action, clip=(-100, 100))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- Velocity tracking rewards (from original Velocity task)
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, 
        weight=1.5,  # Original velocity task weight
        params={"command_name": "pose_command", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, 
        weight=0.75,  # Original velocity task weight
        params={"command_name": "pose_command", "std": math.sqrt(0.25)}
    )

    # -- Pose tracking rewards (new capabilities)
    track_roll = RewTerm(
        func=mdp.track_roll_exp,
        weight=2.0,
        params={"command_name": "pose_command", "std": math.sqrt(0.25)}
    )
    track_pitch = RewTerm(
        func=mdp.track_pitch_exp,
        weight=2.0,
        params={"command_name": "pose_command", "std": math.sqrt(0.25)}
    )
    track_height = RewTerm(
        func=mdp.track_height_exp,
        weight=3.0,
        params={"command_name": "pose_command", "std": 0.05}
    )

    # -- Base penalties (from original Velocity task)
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)
    
    # -- Regularization penalties (from original Velocity task)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)  # Original weight
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)  # Original weight
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2e-4)  # Original weight
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)  # Original weight
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
    energy = RewTerm(func=mdp.energy, weight=-2e-5)  # Original weight

    # -- Feet rewards (from original Velocity task)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.1,  # Original velocity task weight
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "command_name": "pose_command",
            "threshold": 0.5,
        },
    )
    
    air_time_variance = RewTerm(
        func=mdp.air_time_variance_penalty,
        weight=-1.0,  # Original velocity task weight
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )

    # -- Other penalties
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", 
                body_names=["Head_.*", ".*_hip", ".*_thigh", ".*_calf"]
            ),
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), 
            "threshold": 1.0
        },
    )
    
    bad_orientation = DoneTerm(
        func=mdp.bad_orientation, 
        params={"limit_angle": 1.0}  # More tolerant for pose control
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    
    # Curriculum for expanding pose command ranges
    # You can implement custom curriculum terms here later
    pass


@configclass
class RobotVelocityPoseEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Go2 unified velocity & pose control environment."""

    # Scene settings
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=4096, env_spacing=2.5)
    
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        # update sensor update periods
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt


class RobotVelocityPosePlayEnvCfg(RobotVelocityPoseEnvCfg):
    """Configuration for playing/inference with the Go2 velocity & pose control environment."""
    
    def __post_init__(self):
        super().__post_init__()
        # Reduce number of environments for play
        self.scene.num_envs = 32
        # Simplify terrain for visualization
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1
        # Override command ranges with reasonable limits for play mode
        # Use full command ranges for better visualization and testing
        self.commands.pose_command.ranges.lin_vel_x = (-1.0, 1.0)
        self.commands.pose_command.ranges.lin_vel_y = (-0.4, 0.4)
        self.commands.pose_command.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.pose_command.ranges.roll = (-0.2, 0.2)
        self.commands.pose_command.ranges.pitch = (-0.2, 0.2)
        self.commands.pose_command.ranges.height = (0.25, 0.45)
