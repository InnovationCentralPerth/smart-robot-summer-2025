# braccio/__init__.py
import gymnasium as gym
import os

from .config.braccio_env import BraccioReachEnvCfg

# Register the environment
gym.register(
    id="Isaac-Reach-Braccio-v0", # The name you use in the terminal
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BraccioReachEnvCfg,
        "rsl_rl_cfg_entry_point": f"{os.path.dirname(__file__)}/config/rsl_rl_cfg.yaml",
    },
)