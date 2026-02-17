import argparse
import sys
import torch
import carb
import time
import threading
import queue

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=False, enable_cameras=True) 
simulation_app = app_launcher.app

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext, DomeLightCfg
import isaaclab.assets as assets_utils
import isaaclab.sim as sim_utils
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from config.braccio_cfg import BRACCIO_CFG

# --- CLI LISTENER ---
class CLIController:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self._input_loop, daemon=True)
        self.thread.start()

    def _input_loop(self):
        print("\n[CLI] Ready! Syntax: x,y,z,action")
        while self.running:
            try:
                user_input = input()
                if user_input.lower() == "exit": self.running = False; break
                parts = user_input.split(',')
                if len(parts) == 4:
                    self.command_queue.put((float(parts[0]), float(parts[1]), float(parts[2]), parts[3].strip().lower()))
            except: pass

    def get_command(self):
        if not self.command_queue.empty(): return self.command_queue.get()
        return None

# --- SCENE ---
@configclass
class OfficeSceneCfg(InteractiveSceneCfg):
    room = assets_utils.AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Room", spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Room/simple_room.usd"))
    light = assets_utils.AssetBaseCfg(prim_path="/World/light", spawn=DomeLightCfg(intensity=800.0, color=(0.9, 0.9, 0.9)))
    
    # ROBOT
    robot = BRACCIO_CFG.replace(prim_path="{ENV_REGEX_NS}/braccio")
    robot.init_state.pos = (0.0, 0.0, 0.02)
    
    # LIMITS (Safety)
    robot.soft_joint_pos_limits = {
        "base":        [-3.14, 3.14],
        "shoulder":    [0.52, 2.8],  # Min 30 deg
        "elbow":       [0.52, 2.8],  # Min 30 deg
        "wrist_pitch": [0.0, 3.14],
        "wrist_roll":  [0.0, 3.14],
        "gripper_movable": [0.1, 1.3],
    }

    # START POSE
    robot.init_state.joint_pos = {
        "base": 0.0,
        "shoulder": 1.57,      
        "elbow": 1.57,         
        "wrist_pitch": 1.57,   
        "wrist_roll": 0.0,
        "gripper_movable": 0.5,   
    }
    
    # CUBE (Slightly off-center to TEST ROTATION: Y=0.10)
    cube = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04), 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05), 
            collision_props=sim_utils.CollisionPropertiesCfg(), 
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=10.0, dynamic_friction=10.0),
        ),
        # TRY Y=0.10 to force the base to turn!
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.25, -0.25, 0.025)) 
    )
    
    marker = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetMarker",
        spawn=sim_utils.SphereCfg(radius=0.02, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)), rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, kinematic_enabled=True), collision_props=None),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -1.0))
    )

class RobotBrain:
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene
        self.state = "IDLE" 
        self.target_pos = None
        self.action_type = "place"
        self.timer = 0
        
        self.GRIPPER_OPEN = 0.2  
        self.GRIPPER_CLOSED = 0.8 
        self.current_gripper_val = self.GRIPPER_OPEN
        
        # Use Wrist Roll for IK
        self.gripper_index = 4 

        self.home_pose = torch.tensor([
            [0.0, 1.57, 1.57, 1.57, 0.0, 0.2]
        ], device=self.robot.device)

    def set_target(self, x, y, z, action):
        self.target_pos = torch.tensor([[x, y, z]], device=self.robot.device)
        self.action_type = action 
        if "marker" in self.scene.keys():
            marker_pose = torch.cat([self.target_pos, torch.tensor([[1,0,0,0]], device=self.robot.device)], dim=1)
            self.scene["marker"].write_root_pose_to_sim(marker_pose)
        
        # --- FIX 1: START BY ALIGNING ---
        self.state = "ALIGN"
        print(f"[BRAIN] Target: {x:.2f}, {y:.2f}, {z:.2f} | ALIGNING BASE...")

    def update(self):
        current_joints = self.robot.data.joint_pos

        if self.state == "IDLE":
            return self.home_pose

        # Get Current Base Angle
        current_base = current_joints[:, 0]

        # Calculate Required Angle (Atan2 of Y, X)
        target_yaw = torch.atan2(self.target_pos[:, 1], self.target_pos[:, 0])

        # --- STATE MACHINE ---
        
        # 1. ALIGN (Turn Base Only)
        if self.state == "ALIGN":
            new_targets = current_joints.clone()
            
            # Smoothly rotate base towards target_yaw
            diff = target_yaw - current_base
            new_targets[:, 0] += torch.clamp(diff, -0.05, 0.05)
            
            # Keep arm in Home Pose while turning (Prevent flailing)
            new_targets[:, 1:] = self.home_pose[:, 1:]
            
            # If aligned, start approaching
            if torch.abs(diff) < 0.05:
                self.state = "APPROACH"
                print("[BRAIN] Base Aligned. Approaching...")
            
            return new_targets

        # 2. REACHING LOGIC (Approach/Descend/Act)
        ee_idx = self.gripper_index
        full_jacobian = self.robot.root_physx_view.get_jacobians()
        ee_jacobian = full_jacobian[:, ee_idx, :, :] 
        ee_pos = self.robot.data.body_pos_w[:, ee_idx, :]
        
        # Distance calculation
        target_pos_cmd = ee_pos.clone()
        dist = torch.norm(ee_pos[:, :2] - self.target_pos[:, :2])
        
        if self.state == "APPROACH":
            target_pos_cmd = self.target_pos.clone()
            target_pos_cmd[:, 2] += 0.12 # Hover high
            if dist < 0.10: self.state = "DESCEND"

        elif self.state == "DESCEND":
            target_pos_cmd = self.target_pos.clone()
            target_pos_cmd[:, 2] += 0.06 # Hover just above cube
            
            z_dist = torch.abs(ee_pos[:, 2] - target_pos_cmd[:, 2])
            if dist < 0.02 and z_dist < 0.02: 
                self.state = "ACT"
                self.timer = 0
                print("[BRAIN] Grabbing...")

        elif self.state == "ACT":
            target_pos_cmd = self.target_pos.clone()
            target_pos_cmd[:, 2] += 0.06
            self.timer += 1
            if self.action_type == "grab": self.current_gripper_val = self.GRIPPER_CLOSED
            else: self.current_gripper_val = self.GRIPPER_OPEN
            if self.timer > 50: self.state = "RETRACT"

        elif self.state == "RETRACT":
            target_pos_cmd = self.target_pos.clone()
            target_pos_cmd[:, 2] += 0.20
            if ee_pos[:, 2] > (self.target_pos[:, 2] + 0.15): self.state = "IDLE"

        # 3. IK SOLVER
        J_pos = ee_jacobian[:, :3, :5]
        error_pos = target_pos_cmd - ee_pos 
        
        lmbda = 0.05
        J_T = torch.transpose(J_pos, 1, 2)
        JJT = torch.bmm(J_pos, J_T) 
        identity = torch.eye(3, device=self.robot.device).unsqueeze(0)
        damped_inv = torch.inverse(JJT + lmbda * identity)
        delta_theta = torch.bmm(J_T, torch.bmm(damped_inv, error_pos.unsqueeze(2))).squeeze(2)
        
        delta_theta = torch.clamp(delta_theta, -0.05, 0.05)
        
        new_targets = current_joints.clone()
        new_targets[:, :5] += delta_theta
        new_targets[:, 5] = self.current_gripper_val

        # --- FIX 2: FORCE BASE ANGLE ---
        # During reach, override the IK's base decision with our perfect math.
        # This prevents the "lazy base" drift.
        new_targets[:, 0] = target_yaw 

        # 4. SAFETY LIMITS
        limits_low = self.robot.data.soft_joint_pos_limits[:, :, 0]
        limits_high = self.robot.data.soft_joint_pos_limits[:, :, 1]
        new_targets = torch.max(torch.min(new_targets, limits_high), limits_low)

        return new_targets

def main():
    sim_cfg = SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([0.8, 0.0, 0.6], [0.0, 0.0, 0.0])
    scene = InteractiveScene(OfficeSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    
    cli = CLIController()
    brain = RobotBrain(scene["robot"], scene)
    print("[INFO]: Simulation Running...")
    
    for _ in range(20):
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=0.01)

    cube_pos = scene["cube"].data.root_pos_w[0]
    # Grab height offset
    grab_height = cube_pos[2] + 0.02
    
    print(f"\n[INFO] Cube at: {cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}")
    # I changed the cube pos to Y=0.10, so this command will force a rotation!
    print(f">> TRY THIS: {cube_pos[0]:.3f},{cube_pos[1]:.3f},{grab_height:.3f},grab")

    while simulation_app.is_running():
        cmd = cli.get_command()
        if cmd: brain.set_target(*cmd)
        
        targets = brain.update()
        scene["robot"].set_joint_position_target(targets)
        
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=0.01)

if __name__ == "__main__":
    main()