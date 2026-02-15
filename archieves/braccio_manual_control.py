import argparse
import sys
import torch
import carb

# --- 1. APP LAUNCHER ---
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=False, enable_cameras=True) 
simulation_app = app_launcher.app

# --- 2. IMPORTS ---
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext, DomeLightCfg
import isaaclab.assets as assets_utils
import isaaclab.sim as sim_utils
from isaaclab.utils import configclass

# Import config
from config.braccio_cfg import BRACCIO_CFG, BRACCIO_CAMERA_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# --- 3. CONTROLLER (UPDATED FOR 6 JOINTS) ---
class KeyboardController:
    def __init__(self):
        self._input = carb.input.acquire_input_interface()
        # FIX: Changed size from 5 to 6 to match robot DOF
        self.cmd = torch.zeros(6) 
        
        # Subscribe to keyboard events
        self._input.subscribe_to_keyboard_events(None, self._on_key_event)

    def _on_key_event(self, event, *args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._handle_key(event.input, pressed=True)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._handle_key(event.input, pressed=False)
        return True

    def _handle_key(self, key_id, pressed):
        delta = 0.05
        val = delta if pressed else 0.0
        
        # JOINT 0: BASE
        if key_id == carb.input.KeyboardInput.Y: self.cmd[0] = val
        if key_id == carb.input.KeyboardInput.U: self.cmd[0] = -val
        
        # JOINT 1: SHOULDER
        if key_id == carb.input.KeyboardInput.I: self.cmd[1] = -val
        if key_id == carb.input.KeyboardInput.O: self.cmd[1] = val
        
        # JOINT 2: ELBOW
        if key_id == carb.input.KeyboardInput.K: self.cmd[2] = -val
        if key_id == carb.input.KeyboardInput.L: self.cmd[2] = val
        
        # JOINT 3: WRIST PITCH
        if key_id == carb.input.KeyboardInput.N: self.cmd[3] = -val
        if key_id == carb.input.KeyboardInput.M: self.cmd[3] = val

        # JOINT 4: WRIST ROLL
        if key_id == carb.input.KeyboardInput.Z: self.cmd[4] = -val
        if key_id == carb.input.KeyboardInput.C: self.cmd[4] = val
        
        # JOINT 5: GRIPPER MOVABLE
        if key_id == carb.input.KeyboardInput.LEFT_CONTROL: self.cmd[5] = val * 2
        if key_id == carb.input.KeyboardInput.LEFT_SHIFT: self.cmd[5] = -val * 2

    def get_diff(self):
        return self.cmd

# --- 4. SCENE DEFINITION ---
@configclass
class OfficeSceneCfg(InteractiveSceneCfg):
    # 1. SETUP THE ROOM (The Map)
    room = assets_utils.AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room",
        spawn=sim_utils.UsdFileCfg(
            # The correct path construction for the Simple Room
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Room/simple_room.usd",
            scale=(1.0, 1.0, 1.0),
        ),
        init_state=assets_utils.AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

    # 2. LIGHTING
    light = assets_utils.AssetBaseCfg(
        prim_path="/World/light",
        spawn=DomeLightCfg(intensity=800.0, color=(0.9, 0.9, 0.9)),
    )
    
    # 3. ROBOT ARM
    robot = BRACCIO_CFG.replace(prim_path="{ENV_REGEX_NS}/braccio")
    robot.init_state.pos = (0.0, 0.0, 0.01) 
    robot.init_state.rot = (0.0, 0.0, 0.0, 1.0)
    
    # Wrist Camera
    wrist_cam = BRACCIO_CAMERA_CFG

    # Notebook
    notebook = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Notebook",
        spawn=sim_utils.CuboidCfg(
            size=(0.2, 0.15, 0.02),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 0.8)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.005,
                rest_offset=0.0
            ),
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.7, -0.2, 0.8)),
    )

    # Mug
    mug = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Mug",
        spawn=sim_utils.CylinderCfg(
            radius=0.04, height=0.1,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.0, 0.0)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(pos=(0.7, 0.2, 0.8)),
    )

    # Rubik's Cube (Approximation)
    cube = assets_utils.RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Rubiks_Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.06), # 6cm Cube (standard Rubik's size)
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)), # Green
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1), # ~100g
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=assets_utils.RigidObjectCfg.InitialStateCfg(
            pos=(0.4, 0.0, 0.8), # Drop on table (approx height 0.7-0.8m)
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

def main():
    sim_cfg = SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.5, 1.5, 1.6], [0.5, 0.0, 0.5])

    scene_cfg = OfficeSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    keyboard = KeyboardController()
    sim.reset()
    
    robot = scene["robot"]
    
    # DEBUG: Print the detected number of joints
    print(f"[INFO] Robot detected with {robot.num_joints} joints.")
    
    current_targets = robot.data.default_joint_pos.clone()
    
    print("[INFO]: Running...")
    print("  Controls:")
    print("    Base: Y/U")
    print("    Shoulder: I/O")
    print("    Elbow: K/L")
    print("    Wrist Vertical: N/M")
    print("    Wrist Rotate: Z/C")
    print("    Gripper: CONTROL/Shift")

    while simulation_app.is_running():
        # --- CONTROL ---
        cmd_delta = keyboard.get_diff()
        
        # Ensure devices match
        cmd_delta = cmd_delta.to(sim.device)
        
        current_targets += cmd_delta
        
        # Clamp Gripper (Index 5 now)
        current_targets[:, 5] = torch.clamp(current_targets[:, 5], 0.1, 1.5)
        
        robot.set_joint_position_target(current_targets)
        
        # --- STEP ---
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=0.01)

if __name__ == "__main__":
    main()