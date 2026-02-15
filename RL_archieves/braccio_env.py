# config/braccio_env.py
import math
import torch
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg, ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import GaussianNoiseCfg

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm

from isaaclab.scene import InteractiveSceneCfg
from isaaclab.managers import TerminationTermCfg as TermTerm

import isaaclab.envs.mdp as mdp

# Import the robot hardware config
from .braccio_cfg import BRACCIO_CFG

def position_command_error_tanh(
    env: ManagerBasedRLEnvCfg, command_name: str, std: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Custom Reward: Penalize distance to target using Tanh kernel."""
    # 1. Get the Asset (Robot)
    asset = env.scene[asset_cfg.name]
    
    # 2. Get the Command (Target Position)
    # UniformPoseCommandCfg returns a tensor [num_envs, 7] (Pos + Quat)
    command = env.command_manager.get_command(command_name)
    target_pos_rel = command[:, :3] # Take just X,Y,Z
    
    # 3. Get the End Effector Position
    # We need the index of the body named "gripper_base"
    body_ids, _ = asset.find_bodies(asset_cfg.body_names)
    body_idx = body_ids[0]
    
    # Get physics state
    curr_pos_w = asset.data.body_state_w[:, body_idx, :3] # World Frame
    root_pos_w = asset.data.root_pos_w                    # World Frame
    
    # Calculate Relative Position (End Effector relative to Base)
    # (Assuming base is flat on ground, simple subtraction works)
    curr_pos_rel = curr_pos_w - root_pos_w
    
    # 4. Compute Distance
    distance = torch.norm(curr_pos_rel - target_pos_rel, dim=1)
    
    # 5. Apply Tanh Kernel (1.0 = Perfect, 0.0 = Far away)
    return 1.0 - torch.tanh(distance / std)

# -------------------------------------------------------------------------
# 1. SCENE DEFINITION
# -------------------------------------------------------------------------
@configclass
class SceneCfg(InteractiveSceneCfg):
    """Scene definition for the environment."""
    robot: ArticulationCfg = MISSING
    target: RigidObjectCfg = MISSING
    light: AssetBaseCfg = MISSING

# -------------------------------------------------------------------------
# 2. COMMANDS (The Target Logic)
# -------------------------------------------------------------------------
@configclass
class CommandsCfg:
    """Command specifications for the environment."""
    # This generates the random 3D position (X,Y,Z) for the target
    ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="link5", # The part of the robot trying to reach the target
        resampling_time_range=(2.0, 4.0),
        debug_vis=True, # Shows the target goal as a marker
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.10, 0.40),
            pos_y=(-0.3, 0.3),
            pos_z=(0.02, 0.35),
            roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0),
        ),
    )

# -------------------------------------------------------------------------
# 3. ACTIONS & OBSERVATIONS
# -------------------------------------------------------------------------
@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    # This maps the RL output (-1.0 to 1.0) to the robot's physical joint limits.
    arm_action = mdp.JointPositionToLimitsActionCfg(
        asset_name="robot", 
        joint_names=["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_movable"],
        scale=1.0, # 1.0 means full range of motion defined in USD
        clip={".*": (-1.0, 1.0)}
    )

@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        concatenate_terms = True
        enable_corruption = True # Set to True to enable the noise defined in terms below
        
        # Joint Positions with Noise
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, 
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=GaussianNoiseCfg(mean=0.0, std=0.02), 
            clip=(-5.0, 5.0)  # <--- Safety Clamp
        )
        # Joint Velocities with Noise
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=GaussianNoiseCfg(mean=0.0, std=0.05),
            clip=(-5.0, 5.0)  # <--- Safety Clamp
        )
        # Target Position (Reads from the CommandsCfg above)
        target_pos = ObsTerm(
            func=mdp.generated_commands, 
            params={"command_name": "ee_pose"}, 
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),
            clip=(-5.0, 5.0)  # <--- Safety Clamp
        )
        # Gripper State
        gripper_state = ObsTerm(
            func=mdp.joint_pos_limit_normalized,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["gripper_movable"])},
            clip=(-1.0, 1.0)  # <--- Safety Clamp
        )
        
        # Last Action
        last_action = ObsTerm(func=mdp.last_action)
    
    policy: PolicyCfg = PolicyCfg()

# -------------------------------------------------------------------------
# 4. REWARDS (The Learning Signal)
# -------------------------------------------------------------------------
@configclass
class RewardsCfg:
    """Reward terms for the RL agent."""
    
    # 1. Get close to target
    track_target_pos = RewTerm(
        func=position_command_error_tanh,
        weight=2.5,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="link5"), "command_name": "ee_pose", "std": 0.1},
    )

    # 2. Don't hit joint limits
    # CHANGED: mdp.RewTerm -> RewTerm
    joint_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0, 
    )
    
    # 3. Don't jitter
    # Delete because of too high penalty
    #action_rate = RewTerm(
    #    func=mdp.action_rate_l2,
    #    weight=-0.25,
    #)
    # Penalize high speeds directly. This prevents jitter without exploding.
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

# Termination Condition
@configclass
class TerminationsCfg:
    """Termination terms for the environment."""
    # End the episode when time runs out
    time_out = TermTerm(func=mdp.time_out, params={})

# -------------------------------------------------------------------------
# 5. EVENTS (Domain Randomization)
# -------------------------------------------------------------------------
@configclass
class EventCfg:
    """
    # 1. Randomize Friction
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "static_friction_range": (0.4, 1.0),
            "dynamic_friction_range": (0.4, 0.9),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    # 2. Randomize Servo Strength (The "Worn Out" Effect)
    actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "stiffness_distribution_params": (0.75, 1.25), 
            "damping_distribution_params": (0.5, 1.5), 
            "operation": "scale",
            "distribution": "uniform",
        },
    )
    """
    # 3. Reset Robot Position
    reset_robot = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (-0.1, 0.1),
            "velocity_range": (0.0, 0.0),
        },
    )

# -------------------------------------------------------------------------
# 6. MAIN ENVIRONMENT CONFIG
# -------------------------------------------------------------------------
@configclass
class BraccioReachEnvCfg(ManagerBasedRLEnvCfg):
    """Main Environment Configuration."""
    
    # Settings
    scene: SceneCfg = SceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg() 
    events: EventCfg = EventCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    decimation: int = 4     # Robot 'thinks' every 4 physics steps (25Hz if dt=0.01)
    episode_length_s: float = 5.0  # Reset environment every 5 seconds
    
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=0.005,                  # 200Hz Physics (Standard for small arms)
        render_interval=4,
        physx=sim_utils.PhysxCfg(
            # Improve solver accuracy to prevent explosions
            solver_type=1,         # TGS Solver (More stable)
            min_position_iteration_count=12, # Default is 1 or 4. 8 is much safer.
            min_velocity_iteration_count=1, # Help resolve velocity spikes
            bounce_threshold_velocity=0.2, 
            enable_stabilization=True,      # Enable TGS stabilization
        ),
    )
    
    def __post_init__(self):
        # A. The Robot
        self.scene.robot = BRACCIO_CFG.replace(prim_path="{ENV_REGEX_NS}/Braccio")
        
        for name, actuator in self.scene.robot.actuators.items():
            
            # FORCE LIMIT: 2.0 Newton-meters 
            # (Roughly what a strong hobby servo can do)
            actuator.effort_limit = 0.5   
            
            # SPEED LIMIT: ~280 degrees per second
            # (Prevents instantaneous snapping motion)
            actuator.velocity_limit = 2.0 
            
            # SOFTNESS: Low stiffness prevents mathematical explosions
            actuator.stiffness = 10.0     
            actuator.damping = 2.0
        # -----------------------------------------
        
        # 3. SAFETY SPAWN: Start the robot pointing straight up
        # This prevents it from spawning folded inside itself.
        self.scene.robot.init_state.pos = (0.0, 0.0, 0.0)
        self.scene.robot.init_state.joint_pos = {
            "base": 0.0,
            "shoulder": 1.57,       
            "elbow": 1.57,          # 90 degrees
            "wrist_pitch": 1.57,    # 90 degrees
            "wrist_roll": 0.0,
            "gripper_movable": 0.5, 
        }
        
        # 4. TARGET SETUP (Standard)
        self.scene.target = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Target",
            spawn=sim_utils.SphereCfg(
                radius=0.03,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=True),
                mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.2, 0.0, 0.2)), 
        )
        
        # 5. LIGHT SETUP
        self.scene.light = AssetBaseCfg(
            prim_path="/World/light",
            spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        )