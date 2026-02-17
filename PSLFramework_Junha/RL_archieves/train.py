# /home/blancjh/IsaacLab/projects/braccio/train.py
import argparse
import sys
import os
import torch

# --- 1. SETUP PATHS ---
current_dir = os.path.dirname(os.path.abspath(__file__))
projects_dir = os.path.dirname(current_dir)
sys.path.append(projects_dir)

# --- 2. LAUNCH APP (MUST BE FIRST) ---
from isaaclab.app import AppLauncher

# Define arguments
parser = argparse.ArgumentParser(description="Train Braccio")
parser.add_argument("--num_envs", type=int, default=4096, help="Number of environments")
parser.add_argument("--video", action="store_true", default=False, help="Record video")
parser.add_argument("--seed", type=int, default=42, help="Random seed")

# Add standard Isaac Lab args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch the Simulation
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 3. IMPORTS (SAFE NOW) ---
import gymnasium as gym
import braccio # Registers "Isaac-Reach-Braccio-v0"
from rsl_rl.runners import OnPolicyRunner
import yaml
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

# CRITICAL IMPORT: Import your Config Class
from braccio.config.braccio_env import BraccioReachEnvCfg

def main():
    print(f"[INFO] Starting Training for Braccio...")
    
    # 1. Load RSL-RL Configuration
    cfg_path = os.path.join(current_dir, "config", "rsl_rl_cfg.yaml")
    with open(cfg_path, 'r') as f:
        train_cfg = yaml.safe_load(f)
    
    train_cfg["seed"] = args_cli.seed
    
    # 2. Prepare Environment Configuration
    env_cfg = BraccioReachEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs

    # 3. Create the Environment
    env = gym.make("Isaac-Reach-Braccio-v0", cfg=env_cfg)
    
    # --- FIX 2: WRAP THE ENVIRONMENT ---
    # This adds 'get_observations()' and flattens the dictionary obs for RSL-RL
    env = RslRlVecEnvWrapper(env)
    
    # 4. Setup Logging
    log_dir = os.path.join(current_dir, "logs", "braccio_reach")
    print(f"[INFO] Logging to: {log_dir}")
    
    # 5. Initialize PPO Runner
    # Pass the 'env' (which is now the wrapper), NOT 'env.unwrapped'
    runner = OnPolicyRunner(
        env, 
        train_cfg["runner"], 
        log_dir=log_dir, 
        device=env.device # The wrapper exposes .device
    )
    
    # 6. Train
    runner.learn(num_learning_iterations=train_cfg["runner"]["max_iterations"], init_at_random_ep_len=True)
    
    # 7. Close
    env.close()

if __name__ == "__main__":
    main()