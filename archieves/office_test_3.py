import argparse
import sys
import torch
import math
import threading
import queue
import numpy as np

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

# ==========================================
#        CALIBRATION CONSTANTS
# ==========================================
SPEED_LIMIT = 0.005
BASE_OFFSET   = 1.5708
SHOULDER_FLIP = True
WRIST_INVERT  = True
SHOULDER_HEIGHT = 0.16  
L_SHOULDER = 0.125 
L_ELBOW    = 0.125 
L_HAND     = 0.025  # Distance from wrist to gripper tips (tuned: 0.04 too short, 0.01 too far)
GRIPPER_OFFSET = 0.00  # Offset for gripper tips (in METERS) - set to 0 since L_HAND already handles this

def solve_geometric_ik(x, y, z):
    base_target = math.atan2(y, x) + BASE_OFFSET

    r = math.sqrt(x**2 + y**2)
    r_wrist = r - L_HAND
    z_wrist = z - SHOULDER_HEIGHT 

    h = math.sqrt(r_wrist**2 + z_wrist**2)
    max_reach = L_SHOULDER + L_ELBOW
    if h > max_reach: h = max_reach - 0.001 

    alpha = math.atan2(z_wrist, r_wrist)
    cos_beta = (L_SHOULDER**2 + h**2 - L_ELBOW**2) / (2 * L_SHOULDER * h)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    cos_gamma = (L_SHOULDER**2 + L_ELBOW**2 - h**2) / (2 * L_SHOULDER * L_ELBOW)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)

    s_angle = alpha + beta
    e_angle = math.pi - gamma

    if SHOULDER_FLIP: shoulder_target = 3.14 - s_angle
    else: shoulder_target = s_angle
    
    elbow_target = e_angle

    if WRIST_INVERT: wrist_target = (shoulder_target + elbow_target) - 1.57
    else: wrist_target = -(shoulder_target + elbow_target) + 1.57

    return base_target, shoulder_target, elbow_target, wrist_target

class CLIController:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self._input_loop, daemon=True)
        self.thread.start()

    def _input_loop(self):
        print("\n" + "="*60)
        print("      SUPER BRACCIO")
        print("      grab         -> Hover -> Grab -> Lift")
        print("      grab, 0.02   -> Grab 2cm HIGHER")
        print("="*60)
        while self.running:
            try:
                user_input = input("Cmd > ").strip().lower()
                if user_input == "exit": self.running = False; break
                self.command_queue.put(user_input)
            except: pass

    def get_command(self):
        if not self.command_queue.empty(): return self.command_queue.get()
        return None

@configclass
class OfficeSceneCfg(InteractiveSceneCfg):
    room = assets_utils.AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Room", spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Room/simple_room.usd"))
    light = assets_utils.AssetBaseCfg(prim_path="/World/light", spawn=DomeLightCfg(intensity=800.0, color=(0.9, 0.9, 0.9)))
    
    # --- SUPERCHARGE THE ROBOT ---
    # We modify the config to ensure motors are STRONG
    robot = BRACCIO_CFG.replace(prim_path="{ENV_REGEX_NS}/braccio")
    robot.init_state.pos = (0.0, 0.0, 0.02)
    
    # Force high stiffness limits
    robot.soft_joint_pos_limits = {
        "base": [-6.28, 6.28], "shoulder": [-6.28, 6.28], "elbow": [-6.28, 6.28], 
        "wrist_pitch": [-6.28, 6.28], "wrist_roll": [-6.28, 6.28], "gripper_movable": [-1.0, 2.0], 
    }
    
    robot.actuators = {
        "strong_servos": ImplicitActuatorCfg(
            joint_names_expr=[".*"], # Apply to ALL joints
            stiffness=10000.0,        # P-Gain: Very stiff
            damping=50.0,            # D-Gain: No oscillation
            effort_limit=1000.0,      # Max Torque: 100 Nm (Huge)
        ),
    }

    robot.init_state.joint_pos = {"base": 0.0, "shoulder": 1.57, "elbow": 0.0, "wrist_pitch": 1.57, "wrist_roll": 0.0, "gripper_movable": 0.5}
    
    cube = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(size=(0.04, 0.04, 0.04), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)), rigid_props=sim_utils.RigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=0.05), collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=10.0, dynamic_friction=10.0)),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.00, -0.25, 0.05))
    )
    marker = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetMarker",
        spawn=sim_utils.SphereCfg(radius=0.01, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)), rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, kinematic_enabled=True), collision_props=None),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -1.0))
    )

class GeometricBrain:
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene
        self.state = "IDLE" 
        self.timer = 0
        self.device = self.robot.device 
        self.targets_vec = torch.tensor([0.0, 1.57, 0.0, 1.57, 0.0, 1.0], device=self.device)
        self.home_pose = torch.tensor([[0.0, 1.57, 0.0, 1.57, 0.0, 1.0]], device=self.device)
        
        # PERSISTENT TARGET COORDS
        self.tx, self.ty, self.tz = 0.0, -0.25, 0.05

    def process_command(self, cmd_str):
        if "grab" in cmd_str:
            z_offset = 0.0
            parts = cmd_str.split(',')
            if len(parts) > 1: z_offset = float(parts[1])
            
            # Save Target
            self.tx, self.ty = 0.0, -0.25
            self.tz = 0.05 + z_offset

            print(f"[BRAIN] Target: Z={self.tz:.3f}. HOVERING FIRST...")
            
            # --- PHASE 1: HOVER (Go High) ---
            # We add 0.10m (10cm) to the Z height
            self.solve_and_set(self.tx, self.ty, self.tz + 0.10)
            self.targets_vec[5] = 1.0 # Open
            self.state = "HOVER"

    def solve_and_set(self, x, y, z):
        # Update marker to show target gripper position
        marker_pos = torch.tensor([[x, y, z]], device=self.device)
        marker_pose = torch.cat([marker_pos, torch.tensor([[1,0,0,0]], device=self.device)], dim=1)
        self.scene["marker"].write_root_pose_to_sim(marker_pose)
        
        # Add gripper offset - IK solves for wrist, but we want gripper tips at target
        # The L_HAND in IK already accounts for some offset, but may need tuning
        adjusted_z = z + GRIPPER_OFFSET
        
        try:
            b, s, e, w = solve_geometric_ik(x, y, adjusted_z)
            self.targets_vec[0] = b
            self.targets_vec[1] = s
            self.targets_vec[2] = e
            self.targets_vec[3] = w
            print(f"[IK] Target: ({x:.3f}, {y:.3f}, {z:.3f}) -> Adjusted Z: {adjusted_z:.3f}")
        except: print("[ERROR] IK Failed")

    def update(self):
        current_joints = self.robot.data.joint_pos.clone()
        
        if self.state == "IDLE": return self.home_pose

        # === STATE MACHINE ===
        
        if self.state == "HOVER":
            # Move to high position
            if self.move_to_target(current_joints):
                print("[BRAIN] Hover Reached. Descending...")
                # Set target to cube CENTER height (cube is 4cm tall, so center is 2cm up from base)
                # Cube sits at tz=0.05 (bottom), center at tz=0.05, top at tz=0.07
                # We want gripper tips at cube center height so cube is BETWEEN fingers
                grab_height = 0.02  # Height for gripper tips (cube center)
                self.solve_and_set(self.tx, self.ty, grab_height)
                self.state = "DESCEND"

        elif self.state == "DESCEND":
            # Move to grab position
            if self.move_to_target(current_joints):
                print("[BRAIN] At Cube. Closing...")
                self.state = "CLOSING"
                self.timer = 0

        elif self.state == "CLOSING":
            self.timer += 1
            # Keep moving arm to target position while closing gripper
            self.move_to_target(current_joints)
            # Fully close gripper (0.0 is fully closed based on joint limits)
            current_joints[:, 5] = 0.0
            self.targets_vec[5] = 0.0
            if self.timer > 120:  # More time for secure grip 
                print("[BRAIN] Grasp Complete. Lifting...")
                # Lift up again
                self.solve_and_set(self.tx, self.ty, self.tz + 0.15)
                self.state = "LIFT"

        elif self.state == "LIFT":
            if self.move_to_target(current_joints):
                print("[BRAIN] Lift Complete.")
                self.state = "DONE" # Hold position

        elif self.state == "DONE":
            pass # Stay here

        return current_joints

    def move_to_target(self, current_joints):
        # Calculate diff
        diff = self.targets_vec.unsqueeze(0) - current_joints
        
        # Wrap Base
        if diff[0,0] > 3.14: diff[0,0] -= 6.28
        if diff[0,0] < -3.14: diff[0,0] += 6.28

        # SLOW MOVEMENT - Only move arm joints (0-4), NOT the gripper (5)
        # This prevents gripper commands from being overwritten
        arm_diff = torch.clamp(diff[:, :5], -SPEED_LIMIT, SPEED_LIMIT)
        current_joints[:, :5] += arm_diff
        
        # Gripper is handled separately by the state machine
        current_joints[:, 5] = self.targets_vec[5]
        
        # Return True if close (only check arm joints 0-3)
        return torch.norm(diff[0, :4]) < 0.08

def main():
    sim_cfg = SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([0.8, 0.0, 0.6], [0.0, 0.0, 0.0])
    scene = InteractiveScene(OfficeSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    cli = CLIController()
    brain = GeometricBrain(scene["robot"], scene)
    
    while simulation_app.is_running():
        cmd = cli.get_command()
        if cmd: brain.process_command(cmd)
        targets = brain.update()
        scene["robot"].set_joint_position_target(targets)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=0.01)

if __name__ == "__main__":
    main()