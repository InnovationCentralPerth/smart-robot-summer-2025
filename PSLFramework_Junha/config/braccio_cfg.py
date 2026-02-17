# braccio_cfg.py
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import CameraCfg

BRACCIO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/blancjh/IsaacLab/assets/braccio/braccio.usd",
        
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            disable_gravity=False,
        ),
        
        # 1. HEAVY MASS (Stability)
        mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
        
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,  # Disabled: was preventing elbow from reaching IK targets
            solver_position_iteration_count=12,
            fix_root_link=True,
        ),
        
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.005,
            rest_offset=0.0,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        # 2. VALID POSE (Inside Limits)
        # We go back to 1.57 (Vertical) because 0.0 violates the safety limit [0.26, 2.8]
        joint_pos={
            "base": 0.0,
            "shoulder": 1.57,      # Valid! (Inside 0.26 - 2.8 range)
            "elbow": 1.74,         # Straight
            "wrist_pitch": 1.57,   # Straight
            "wrist_roll": 0.0,
            "gripper_movable": 0.5,   
        },
    ),
    
    # 3. POWERFUL ACTUATOR
    # This prevents the "Folding" behavior by forcing every joint to be a strong motor.
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base", "shoulder", "elbow", 
                "wrist_pitch", "wrist_roll", "gripper_movable"
            ],
            stiffness=4000.0,   # Super Stiff (Freezes robot in place)
            damping=400.0,
            effort_limit=None,   # Infinite Torque
            velocity_limit=100.0,
        ),
    },
)

# 4. SAFETY LIMITS
# We keep these wider than the initial state to avoid crashes.
BRACCIO_CFG.soft_joint_pos_limits = {
    "base":        [-6.28, 6.28],  # Allow full rotation
    "shoulder":    [0.26, 2.88],  # Real limits (15 to 165 deg)
    "elbow":       [-3.14, 3.14],
    "wrist_pitch": [-3.14, 3.14],
    "wrist_roll":  [-3.14, 3.14],
    "gripper_movable": [0.0, 1.5],
}

# --- CAMERA CONFIGURATION ---
BRACCIO_CAMERA_CFG = CameraCfg(
    prim_path="{ENV_REGEX_NS}/braccio/link5/front_cam",
    update_period=0.0, 
    height=480, 
    width=640,
    data_types=["rgb", "distance_to_image_plane"], 
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0, 
        focus_distance=400.0, 
        horizontal_aperture=20.955,
        clipping_range=(0.01, 100.0),
    ),
    offset=CameraCfg.OffsetCfg(
        pos=(0.0, 0.05, 0.06), 
        rot=(1.0, 0.0, 0.0, 0.0),
        convention="ros",
    ),
)