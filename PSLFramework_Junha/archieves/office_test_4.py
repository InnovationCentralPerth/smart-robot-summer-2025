import torch
import math
import threading
import queue

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=False, enable_cameras=True) 
simulation_app = app_launcher.app

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext, DomeLightCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.assets as assets_utils
import isaaclab.sim as sim_utils
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from config.braccio_cfg import BRACCIO_CFG

# USD imports for physics material manipulation
from pxr import UsdPhysics, UsdShade, Sdf, Gf
import omni.usd

# ==========================================
#   URDF-CALIBRATED DIMENSIONS (meters)
# ==========================================
# From URDF joint origins:
SHOULDER_HEIGHT = 0.0505   # base to shoulder joint (z)
L_SHOULDER      = 0.1205   # shoulder to elbow
L_ELBOW         = 0.1205   # elbow to wrist_pitch
L_WRIST         = 0.06     # wrist_pitch to wrist_roll
L_GRIPPER       = 0.0065 + 0.007 + 0.04  # wrist_roll to gripper tips (approx)

# Total arm length for gripper tip in 2D plane
L_GRIPPER_TOTAL = L_WRIST + L_GRIPPER  # ~0.0935

# Movement
SPEED_LIMIT = 0.008

# ==========================================
#   BRACCIO 2D IK - Native Coordinate System
# ==========================================
#
# KNOWN WORKING POSES:
#   - Candle (arm up): shoulder=1.57, elbow=0, wrist=1.57
#   - Shoulder at 1.57 = arm vertical (up)
#   - Shoulder decreasing toward 0 = arm tilting forward
#   - Elbow at 0 = straight, increasing = bending "inward"
#
# JOINT LIMITS (from URDF):
#   - shoulder: [0.26, 2.88] rad = [15°, 165°]
#   - elbow: [0, π]
#   - wrist_pitch: [0, π]
#
def solve_2d_ik(distance_2d, height):
    """
    Solve 2D IK for Braccio robot, ensuring all angles are within URDF limits.
    
    Args:
        distance_2d: horizontal distance from robot base to gripper tip  
        height: target height from ground
    
    Returns:
        shoulder_angle, elbow_angle, wrist_angle
    """
    # Account for gripper length
    r = distance_2d - L_GRIPPER_TOTAL
    z = height - SHOULDER_HEIGHT
    
    # Ensure minimum reach values
    r = max(r, 0.01)
    
    # Distance from shoulder to wrist
    h = math.sqrt(r**2 + z**2)
    max_reach = L_SHOULDER + L_ELBOW
    min_reach = abs(L_SHOULDER - L_ELBOW)
    
    # Clamp to reachable
    h = max(min_reach + 0.01, min(h, max_reach * 0.95))
    
    # Recalculate r, z if we clamped h (maintain direction)
    if h > 0:
        orig_h = math.sqrt(r**2 + z**2)
        if orig_h > 0:
            scale = h / orig_h
            r = r * scale
            z = z * scale
    
    # Angle FROM VERTICAL to target point (0 = straight up, positive = forward)
    # Using atan2 where vertical up is 0
    angle_from_vertical = math.atan2(r, z)  # Note: r is "x" (forward), z is "y" (up)
    
    # Law of cosines for the arm triangle
    cos_elbow = (L_SHOULDER**2 + L_ELBOW**2 - h**2) / (2 * L_SHOULDER * L_ELBOW)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_internal = math.acos(cos_elbow)  # Internal angle at elbow
    
    cos_shoulder_offset = (L_SHOULDER**2 + h**2 - L_ELBOW**2) / (2 * L_SHOULDER * h)
    cos_shoulder_offset = max(-1.0, min(1.0, cos_shoulder_offset))
    shoulder_offset = math.acos(cos_shoulder_offset)
    
    # === BRACCIO NATIVE ANGLES ===
    # 
    # Candle pose: shoulder = π/2, pointing straight up
    # To reach forward: subtract from π/2
    # 
    # For elbow-down config (natural for grabbing):
    #   shoulder reaches LESS than the line to target
    #   so shoulder = π/2 - (angle_from_vertical + shoulder_offset)
    #
    # But we want elbow-UP actually (based on the URDF rotation directions):
    #   shoulder = π/2 - (angle_from_vertical - shoulder_offset)
    #   OR we can try: shoulder = π/2 - angle_from_vertical + shoulder_offset
    
    # Let's try the standard "elbow up" which often works better for this URDF
    shoulder_angle = (math.pi / 2) - angle_from_vertical + shoulder_offset
    
    # Elbow: URDF elbow=0 is straight
    elbow_angle = math.pi - elbow_internal
    
    # Wrist: keep gripper pointing toward target (horizontal when z is low)
    # The wrist needs to compensate for shoulder and elbow
    # When arm reaches forward and down, wrist tilts down
    # wrist = π/2 + (shoulder_angle - π/2) - elbow_angle  simplified:
    wrist_angle = shoulder_angle - elbow_angle
    
    # If wrist goes negative, adjust
    if wrist_angle < 0:
        wrist_angle = wrist_angle + math.pi
    
    # Debug
    print(f"[IK] Target: r={r:.3f}, z={z:.3f}, h={h:.3f}")
    print(f"[IK] angle_from_vert={math.degrees(angle_from_vertical):.1f}°")
    print(f"[IK] RAW: shoulder={math.degrees(shoulder_angle):.1f}°, elbow={math.degrees(elbow_angle):.1f}°, wrist={math.degrees(wrist_angle):.1f}°")
    
    # Clamp to URDF limits
    shoulder_angle = max(0.26, min(2.88, shoulder_angle))
    elbow_angle = max(0.0, min(3.14, elbow_angle))
    wrist_angle = max(0.0, min(3.14, wrist_angle))
    
    print(f"[IK] CLAMPED: shoulder={math.degrees(shoulder_angle):.1f}°, elbow={math.degrees(elbow_angle):.1f}°, wrist={math.degrees(wrist_angle):.1f}°")
    
    return shoulder_angle, elbow_angle, wrist_angle


# Offset for gripper center (fixed finger is at center, needs slight rotation for grip)
GRIPPER_CENTER_OFFSET = math.radians(6)  # 3 degrees to the right (reduced for better centering)

def get_base_angle_to_target(robot_x, robot_y, target_x, target_y, apply_offset=True):
    """
    Calculate base rotation to face target.
    Returns angle in radians.
    
    Args:
        apply_offset: If True, apply gripper center offset (for grabbing).
                     If False, skip offset (for placing).
    """
    dx = target_x - robot_x
    dy = target_y - robot_y
    # atan2 gives angle from +X axis, but our robot's forward is -Y when base=0
    # URDF: base axis is -Z, so positive rotation is CW from above
    angle = math.atan2(dx, -dy)  # Swap because forward is -Y
    # Add offset for gripper center (only for grabbing)
    if apply_offset:
        angle += GRIPPER_CENTER_OFFSET
    # Normalize to [-π, π] to stay within joint limits
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
        
    return angle


class CLIController:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self._input_loop, daemon=True)
        self.thread.start()

    def _input_loop(self):
        print("\n" + "="*60)
        print("  BRACCIO GRAB - Auto-detect cube position")
        print("  Commands:")
        print("    grab        -> Find cube, rotate, reach, grab")
        print("    place x y z -> Lift cube, move to (x,y,z), release")
        print("    home        -> Return to home position")
        print("    exit        -> Quit")
        print("="*60)
        while self.running:
            try:
                user_input = input("Cmd > ").strip().lower()
                if user_input == "exit": 
                    self.running = False
                    break
                self.command_queue.put(user_input)
            except: 
                pass

    def get_command(self):
        if not self.command_queue.empty(): 
            return self.command_queue.get()
        return None


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
    robot.init_state.joint_pos = {
        "base": 0.0, 
        "shoulder": 1.57,  # Vertical up
        "elbow": 0.0, 
        "wrist_pitch": 1.57, 
        "wrist_roll": 0.0, 
        "gripper_movable": 0.175  # Fully open
    }
    
    cube = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04), 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.01),  # Very light - 10 grams
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0, restitution=0.0)
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.00, -0.20, 0.02))
    )
    
    marker = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetMarker",
        spawn=sim_utils.SphereCfg(
            radius=0.01, 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, kinematic_enabled=True),
            collision_props=None
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -1.0))
    )


class GrabController:
    """Simple state machine for grab sequence."""
    
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene
        self.device = robot.device
        
        # Joint indices: base=0, shoulder=1, elbow=2, wrist_pitch=3, wrist_roll=4, gripper=5
        self.state = "IDLE"
        self.timer = 0
        
        # Home pose (candle) - gripper fully open (0.17)
        self.home_pose = torch.tensor([[0.0, 1.57, 0.0, 1.57, 0.0, 0.175]], device=self.device)
        
        # Target joint positions
        self.target_joints = self.home_pose.clone()
        
        # Cube target in world coords
        self.cube_x = 0.0
        self.cube_y = 0.0
        self.cube_z = 0.0
        
    def process_command(self, cmd):
        if "grab" in cmd:
            # === AUTO-DETECT CUBE POSITION ===
            cube_pos = self.scene["cube"].data.root_pos_w[0]
            self.cube_x = float(cube_pos[0])
            self.cube_y = float(cube_pos[1])
            self.cube_z = float(cube_pos[2])
            
            print(f"\n[GRAB] Cube detected at: ({self.cube_x:.3f}, {self.cube_y:.3f}, {self.cube_z:.3f})")
            
            # Step 1: Rotate base to face cube
            base_angle = get_base_angle_to_target(0, 0, self.cube_x, self.cube_y)
            self.target_joints[0, 0] = base_angle
            
            # Keep arm in candle pose while rotating
            self.target_joints[0, 1] = 1.57  # shoulder up
            self.target_joints[0, 2] = 0.0   # elbow straight
            self.target_joints[0, 3] = 1.57  # wrist
            self.target_joints[0, 5] = 0.175  # gripper fully open
            
            print(f"[GRAB] Rotating base to {math.degrees(base_angle):.1f}°")
            self.state = "ROTATE"
            self.timer = 0
            
        elif cmd.startswith("place"):
            parts = cmd.split()
            if len(parts) >= 4 and self.state == "HOLDING":
                self.place_x = float(parts[1])
                self.place_y = float(parts[2])
                self.place_z = float(parts[3])
                print(f"\n[PLACE] Target: ({self.place_x:.3f}, {self.place_y:.3f}, {self.place_z:.3f})")
                
                # Step 1: Lift arm to candle pose (physics requires vertical arm to rotate)
                print("[PLACE] Lifting to candle pose...")
                self.target_joints[0, 1] = 1.57  # shoulder up
                self.target_joints[0, 2] = 0.0   # elbow straight
                self.target_joints[0, 3] = 1.57  # wrist level
                
                self.state = "PLACE_LIFT"
                self.timer = 0
            elif self.state != "HOLDING":
                print("[PLACE] Must grab cube first!")
            else:
                print("[PLACE] Usage: place x y z")
                
        elif "home" in cmd:
            self.target_joints = self.home_pose.clone()
            self.state = "HOME"
            print("[HOME] Returning to home position")
    
    def update(self):
        """Update state machine, return target joint positions."""
        current = self.robot.data.joint_pos.clone()
        
        if self.state == "IDLE":
            return self.home_pose
        
        elif self.state == "HOME":
            if self._move_towards_target(current, threshold=0.1):
                print("[HOME] Done")
                self.state = "IDLE"
            return current
        
        elif self.state == "ROTATE":
            # Wait for base to finish rotating
            if self._move_towards_target(current, threshold=0.1):
                print("[GRAB] Base rotation complete. Reaching for cube...")
                
                # Calculate 2D distance and solve IK
                distance_2d = math.sqrt(self.cube_x**2 + self.cube_y**2)
                
                # Hover position (10cm above cube)
                hover_z = self.cube_z + 0.10
                
                shoulder, elbow, wrist = solve_2d_ik(distance_2d, hover_z)
                
                self.target_joints[0, 1] = shoulder
                self.target_joints[0, 2] = elbow
                self.target_joints[0, 3] = wrist
                self.target_joints[0, 5] = 0.175  # Fully open
                
                print(f"[IK] Hover -> dist={distance_2d:.3f}, z={hover_z:.3f}")
                print(f"[IK] shoulder={math.degrees(shoulder):.1f}°, elbow={math.degrees(elbow):.1f}°, wrist={math.degrees(wrist):.1f}°")
                
                self.state = "HOVER"
            return current
        
        elif self.state == "HOVER":
            if self._move_towards_target(current):
                print("[GRAB] At grab position. Closing gripper...")
                self.target_joints[0, 5] = 1.1  # Maximum closed for strong grip
                self.state = "CLOSE"
                self.timer = 0
            return current
        
        elif self.state == "CLOSE":
            # Keep arm steady
            self._move_towards_target(current)
            
            # Close gripper using the gripper control function
            gripper_closed = self._move_gripper(current, target_pos=1.5)
            
            # Transition to HOLDING if:
            # 1. Gripper reached target, OR
            # 2. Timeout (gripper is likely blocked by cube)
            self.timer += 1
            if gripper_closed or self.timer > 450:
                print(f"[GRAB] ✓ Cube grabbed! Use 'place x y z' to move it.")
                self.state = "HOLDING"
            return current
        
        elif self.state == "HOLDING":
            # Keep gripper closed, maintain position
            self._move_towards_target(current)
            current[0, 5] = 1.5
            return current
        
        elif self.state == "PLACE_LIFT":
            # Lift arm to candle pose before rotating (physics requires vertical arm)
            self._move_towards_target(current)
            self.timer += 1
            
            # Use timeout - arm oscillates with cube weight, can't reach exact threshold
            if self.timer > 250:  # ~2.5 seconds
                print("[PLACE] Arm lifted. Now rotating base...")
                
                # Now set base rotation target
                base_angle = get_base_angle_to_target(0, 0, self.place_x, self.place_y, apply_offset=False)
                self.target_joints[0, 0] = base_angle
                
                print(f"[PLACE] Rotating base to {math.degrees(base_angle):.1f}°")
                self.state = "PLACE_ROTATE"
                self.timer = 0
            # Keep gripper closed
            current[0, 5] = 1.5
            return current
        
        elif self.state == "PLACE_ROTATE":
            # Save original position before modification
            base_read = float(current[0, 0])
            
            # Move arm towards target (this moves base too)
            self._move_towards_target(current)
            
            # Position after modification (what we're commanding)
            base_commanded = float(current[0, 0])
            
            # Check BASE rotation only (ignore arm steady-state error from holding)
            base_diff = self.target_joints[0, 0] - current[0, 0]
            # Wrap angle
            if base_diff > math.pi:
                base_diff -= 2 * math.pi
            elif base_diff < -math.pi:
                base_diff += 2 * math.pi
            base_error = abs(base_diff)
            
            # Debug: track base rotation progress
            self.timer += 1
            if self.timer % 100 == 0:
                print(f"[PLACE_ROTATE] READ={math.degrees(base_read):.1f}°, "
                      f"COMMANDED={math.degrees(base_commanded):.1f}°, "
                      f"TARGET={math.degrees(float(self.target_joints[0,0])):.1f}°")
            
            if base_error < 0.1:  # Base within ~6 degrees
                print("[PLACE] Base rotation complete. Reaching for target...")
                
                # Calculate 2D distance and solve IK
                distance_2d = math.sqrt(self.place_x**2 + self.place_y**2)
                
                shoulder, elbow, wrist = solve_2d_ik(distance_2d, self.place_z)
                
                self.target_joints[0, 1] = shoulder
                self.target_joints[0, 2] = elbow
                self.target_joints[0, 3] = wrist
                
                print(f"[IK] Place -> dist={distance_2d:.3f}, z={self.place_z:.3f}")
                
                self.state = "PLACE_REACH"
            # Keep gripper closed
            current[0, 5] = 1.1
            return current
        
        elif self.state == "PLACE_REACH":
            if self._move_towards_target(current, threshold=0.5):
                print("[PLACE] At position. Opening gripper...")
                self.target_joints[0, 5] = 0.175  # Open gripper
                self.state = "RELEASE"
                self.timer = 0
            # Keep gripper closed while moving
            current[0, 5] = 1.1
            return current
        
        elif self.state == "RELEASE":
            self._move_towards_target(current)
            gripper_open = self._move_gripper(current, target_pos=0.175)
            
            self.timer += 1
            if gripper_open or self.timer > 200:
                print("[PLACE] ✓ Cube placed! Waiting 2 seconds...")
                self.state = "PLACE_WAIT"
                self.timer = 0
            return current
        
        elif self.state == "PLACE_WAIT":
            # Wait 2 seconds then return home
            self._move_towards_target(current)
            self.timer += 1
            if self.timer > 200:  # 2 seconds at 100Hz
                print("[PLACE] Returning to home position...")
                self.target_joints = self.home_pose.clone()
                self.state = "HOME"
            return current
        
        return current
    
    def _move_towards_target(self, current, threshold=0.705):
        """
        Smoothly move arm joints (0-4) towards target.
        Returns True when close enough.
        Does NOT control gripper - use _move_gripper for that.
        """
        diff = self.target_joints - current
        
        # Wrap base angle
        if diff[0, 0] > math.pi:
            diff[0, 0] -= 2 * math.pi
        elif diff[0, 0] < -math.pi:
            diff[0, 0] += 2 * math.pi
        
        # Only move arm joints (0-4), not gripper
        arm_diff = diff[:, :5]
        arm_step = torch.clamp(arm_diff, -SPEED_LIMIT, SPEED_LIMIT)
        current[:, :5] += arm_step
        
        # Check if arm is close enough (0.4 threshold because physics/joint limits)
        arm_error = float(torch.norm(arm_diff))
        
        # Debug: show arm error periodically
        self.timer += 1
        if self.timer % 100 == 0:
            print(f"[DEBUG] Arm error: {arm_error:.4f}, State: {self.state}")
        
        return arm_error < threshold
    
    def _move_gripper(self, current, target_pos, speed=0.003):
        """
        Move gripper toward target position.
        Returns True when gripper is at target.
        
        Args:
            current: current joint positions tensor
            target_pos: target gripper position (0.175=open, 1.27=closed)
            speed: max movement per step
        """
        gripper_diff = target_pos - current[0, 5]
        gripper_step = max(-speed, min(speed, gripper_diff))
        current[0, 5] += gripper_step
        
        return abs(gripper_diff) < 0.1


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


def main():
    sim_cfg = SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([0.5, 0.5, 0.5], [0.0, -0.1, 0.1])
    
    scene = InteractiveScene(OfficeSceneCfg(num_envs=1, env_spacing=2.0))
    
    # Apply high friction to gripper before simulation starts
    stage = omni.usd.get_context().get_stage()
    apply_gripper_friction(stage)
    
    sim.reset()
    
    cli = CLIController()
    controller = GrabController(scene["robot"], scene)
    
    print("\n[SIM] Ready. Type 'grab' to pick up the cube.")
    
    while simulation_app.is_running():
        cmd = cli.get_command()
        if cmd:
            controller.process_command(cmd)
        
        targets = controller.update()
        scene["robot"].set_joint_position_target(targets)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=0.01)


if __name__ == "__main__":
    main()