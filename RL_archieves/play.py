# projects/braccio/play.py
import argparse
import sys
import os
import torch
import glob

# --- 1. SETUP PATHS ---
current_dir = os.path.dirname(os.path.abspath(__file__))
projects_dir = os.path.dirname(current_dir)
sys.path.append(projects_dir)

# --- 2. LAUNCH APP (MUST BE FIRST) ---
from isaaclab.app import AppLauncher

# Define arguments
parser = argparse.ArgumentParser(description="Play Braccio Policy")
parser.add_argument("--num_envs", type=int, default=1, help="Number of robots to spawn")
parser.add_argument("--run_dir", type=str, required=True, help="Path to the training run (e.g., logs/braccio_reach/...)")
parser.add_argument("--checkpoint", type=str, default="model_*.pt", help="Checkpoint file to load")

# Add standard Isaac Lab args (headless=False by default so we can see it)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch the Simulation (Headless=False forces the GUI to open)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 3. IMPORTS (SAFE NOW) ---
import gymnasium as gym
import braccio # Registers "Isaac-Reach-Braccio-v0"
from rsl_rl.runners import OnPolicyRunner
import yaml

# Correct Import for Wrapper
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from braccio.config.braccio_env import BraccioReachEnvCfg

def main():
    print(f"[INFO] Starting Playback for Braccio...")
    
    # 1. Locate the Configuration
    # We load the config used during training to match settings
    # Try finding it in the log directory first, else fall back to local
    run_cfg_path = os.path.join(args_cli.run_dir, "params", "rsl_rl_cfg.yaml")
    if not os.path.exists(run_cfg_path):
        print(f"[WARN] Could not find config in run dir. Using local default.")
        run_cfg_path = os.path.join(current_dir, "config", "rsl_rl_cfg.yaml")

    with open(run_cfg_path, 'r') as f:
        train_cfg = yaml.safe_load(f)

    # 2. Prepare Environment Configuration
    env_cfg = BraccioReachEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs # Usually 1 for visualization
    env_cfg.sim.render_interval = 1 # Render every step for smooth video

    # 3. Create the Environment
    env = gym.make("Isaac-Reach-Braccio-v0", cfg=env_cfg)
    
    # Wrap for RSL-RL compatibility
    env = RslRlVecEnvWrapper(env)

    # 4. Load the Runner and Policy
    runner = OnPolicyRunner(
        env, 
        train_cfg["runner"], 
        log_dir=args_cli.run_dir, 
        device=env.device
    )
    
    # 1. If the user provided a full file path, use it.
    if os.path.isfile(args_cli.run_dir) and args_cli.run_dir.endswith(".pt"):
        model_path = args_cli.run_dir
    else:
        # 2. Otherwise, search the directory for the newest .pt file
        search_path = os.path.join(args_cli.run_dir, "model_*.pt")
        models = sorted(glob.glob(search_path), key=os.path.getmtime)
        
        if not models:
            raise FileNotFoundError(f"No 'model_*.pt' files found in {args_cli.run_dir}")
            
        model_path = models[-1] # Pick the latest one
        
    print(f"[INFO] Loading checkpoint: {model_path}")
    runner.load(model_path)
    
    # Get the inference policy (the trained brain)
    policy = runner.get_inference_policy(device=env.device)
    
    # 5. Run the Simulation Loop
    obs = env.get_observations()
    
    print(f"[INFO] Playing... Press 'Esc' in the viewer to exit.")
    
    while simulation_app.is_running():
        with torch.inference_mode():
            # Ask the brain: "What should I do based on this observation?"
            actions = policy(obs)
            
            # Execute the action
            obs, _, _, _ = env.step(actions)

    env.close()

if __name__ == "__main__":
    main()