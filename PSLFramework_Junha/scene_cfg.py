"""
Scene Configuration for Braccio Robot in Isaac Sim.

Contains:
- OfficeSceneCfg: Scene definition with robot, cubes, and environment
- apply_gripper_friction(): Apply high-friction physics material to gripper
- fix_base_joint_limits(): Unlock base joint rotation limits at runtime
"""

from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import DomeLightCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.assets as assets_utils
import isaaclab.sim as sim_utils
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from config.braccio_cfg import BRACCIO_CFG

# USD imports for physics material manipulation
from pxr import UsdPhysics, UsdShade
import omni.usd


@configclass
class OfficeSceneCfg(InteractiveSceneCfg):
    room = assets_utils.AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room", 
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Room/simple_room.usd")
    )
    light = assets_utils.AssetBaseCfg(
        prim_path="/World/light", 
        spawn=DomeLightCfg(intensity=800.0, color=(0.9, 0.9, 0.9))
    )
    
    robot = BRACCIO_CFG.replace(prim_path="{ENV_REGEX_NS}/braccio")
    robot.init_state.pos = (0.0, 0.0, 0.02)  # Slight raise
    
    robot.actuators = {
        "arm_servos": ImplicitActuatorCfg(
            joint_names_expr=["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll"],
            stiffness=50000.0,   # Increased to fight cube weight on wrist
            damping=500.0,       # Increased to reduce oscillation
            effort_limit=100000.0,
        ),
        "gripper_servo": ImplicitActuatorCfg(
            joint_names_expr=["gripper_movable"],
            stiffness=2000.0,    # Increased to grip tighter during rotation
            damping=100.0,       # Increased damping
            effort_limit=500.0,  # Increased effort to hold cube during movement
        ),
    }
    
    # Home pose: "candle" - arm pointing up
    robot.init_state.pos = (0.0, 0.0, 0.02)  # Lift base slightly to avoid ground collision
    robot.init_state.joint_pos = {
        "base": 0.0, 
        "shoulder": 1.57,  # Vertical up
        "elbow": 0.0, 
        "wrist_pitch": 1.57, 
        "wrist_roll": 0.0, 
        "gripper_movable": 0.175  # Fully open
    }
    
    # Green cube (original)
    cube = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04), 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.01),  # Very light - 10 grams
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=2.0, dynamic_friction=2.0, restitution=0.0,
                compliant_contact_stiffness=5000.0, compliant_contact_damping=50.0
            )
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.00, -0.20, 0.02))
    )
    
    # Red cube - positioned 45° to the LEFT of green cube (requires negative rotation)
    red_cube = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/RedCube",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04), 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.01),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=2.0, dynamic_friction=2.0, restitution=0.0,
                compliant_contact_stiffness=5000.0, compliant_contact_damping=50.0
            )
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.00, -0.20, 0.06))  # Red cube stacked on green cube
    )
    
    marker = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetMarker",
        spawn=sim_utils.SphereCfg(
            radius=0.01, 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.5, 0.0)),  # Orange marker
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, kinematic_enabled=True),
            collision_props=None
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -1.0))
    )


def apply_gripper_friction(stage, env_path="/World/envs/env_0"):
    """
    Apply high friction physics material to gripper collision surfaces.
    This prevents the cube from slipping when gripped.
    """
    # Create a high-friction physics material
    material_path = f"{env_path}/braccio/GripperFrictionMaterial"
    material_prim = stage.DefinePrim(material_path, "Material")
    
    # Apply physics material API
    physics_material = UsdPhysics.MaterialAPI.Apply(material_prim)
    physics_material.CreateStaticFrictionAttr().Set(100.0)  # Very high friction
    physics_material.CreateDynamicFrictionAttr().Set(100.0)
    physics_material.CreateRestitutionAttr().Set(0.0)  # No bounce
    
    # Find gripper link prims and apply the material
    gripper_paths = [
        f"{env_path}/braccio/link6",  # Gripper base link
        f"{env_path}/braccio/gripper_fixed",
        f"{env_path}/braccio/gripper_movable",
    ]
    
    material_binding = UsdShade.MaterialBindingAPI
    
    for gripper_path in gripper_paths:
        prim = stage.GetPrimAtPath(gripper_path)
        if prim.IsValid():
            # Apply material to all collision meshes under this prim
            for child in prim.GetAllChildren():
                if "collision" in child.GetName().lower() or "Collision" in child.GetName():
                    binding_api = material_binding.Apply(child)
                    binding_api.Bind(
                        UsdShade.Material(material_prim),
                        UsdShade.Tokens.weakerThanDescendants,
                        "physics"
                    )
                    print(f"[FRICTION] Applied high friction to {child.GetPath()}")
            
            # Also try to apply directly to the link
            if prim.IsValid():
                binding_api = material_binding.Apply(prim)
                binding_api.Bind(
                    UsdShade.Material(material_prim),
                    UsdShade.Tokens.weakerThanDescendants,
                    "physics"
                )
                print(f"[FRICTION] Applied high friction to {prim.GetPath()}")
    
    print("[FRICTION] Gripper friction material applied!")


def fix_base_joint_limits():
    """Hack to unlock base joint limits at runtime."""
    stage = omni.usd.get_context().get_stage()
    count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            name = prim.GetName()
            # Braccio base joint usually named "base" or "joint1" or similar
            if "base" in name.lower() or "joint1" in name.lower():
                joint = UsdPhysics.RevoluteJoint(prim)
                # Set wide limits (-360 to 360 degrees) -> effectively unlimited
                joint.GetLowerLimitAttr().Set(-360.0)
                joint.GetUpperLimitAttr().Set(360.0)
                print(f"[FIX] Unlocked base joint limits for: {prim.GetPath()}")
                count += 1
    
    if count == 0:
        print("[FIX] WARNING: Could not find base joint to unlock!")
