"""
Braccio Robot - Main Entry Point

Thin entry point that initializes the Isaac Sim application, sets up the scene,
and runs the main simulation loop. All logic is delegated to dedicated modules:

- kinematics: FK/IK solver and robot dimension constants
- scene_cfg: Scene configuration, physics material setup
- grab_controller: State machine for grab/place sequences
- cli_controller: Threaded CLI for natural language input
- llm_planner: LLM-based task planning and decomposition
"""

import torch
import math

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=False, enable_cameras=True) 
simulation_app = app_launcher.app

# These imports MUST come after AppLauncher initialization
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationCfg, SimulationContext

import omni.usd

from scene_cfg import OfficeSceneCfg, apply_gripper_friction, fix_base_joint_limits
from grab_controller import GrabController
from cli_controller import CLIController
from llm_planner import LLMController


def main():
    sim_cfg = SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([0.5, 0.5, 0.5], [0.0, -0.1, 0.1])
    
    scene = InteractiveScene(OfficeSceneCfg(num_envs=1, env_spacing=2.0))
    
    # Apply high friction to gripper before simulation starts
    stage = omni.usd.get_context().get_stage()
    apply_gripper_friction(stage)
    
    # HACK: Fix base joint limits to allow negative rotation
    fix_base_joint_limits()
    
    sim.reset()
    
    cli = CLIController()
    grab_controller = GrabController(scene["robot"], scene)
    
    # Initialize LLM controller
    llm_controller = LLMController(scene, model="mistral")
    
    # Current task being executed
    current_task = None
    task_wait_timer = 0  # Timer to wait between tasks
    
    print("\n[SIM] Ready. Enter natural language commands.")
    print("[SIM] Green cube at (0, -0.20), Red cube at (0.06, -0.20)")
    
    while simulation_app.is_running():
        # Check for new commands
        cmd = cli.get_command()
        if cmd:
            if cmd.lower() == "home":
                grab_controller.process_command("home")
            else:
                # Process natural language command via LLM
                success = llm_controller.process_command(cmd)
                if success:
                    print(f"[MAIN] Task queue ready with {len(llm_controller.task_queue)} tasks")
        
        # Check if current task completed (robot returned to IDLE or HOLDING)
        if current_task and not grab_controller.is_busy():
            print(f"[MAIN] Task completed: {current_task}")
            llm_controller.task_completed()
            current_task = None
            task_wait_timer = 500  # Wait 5 seconds (500 steps at 100Hz) before next task
            
        # If robot is idle and we have pending tasks, start next task (after wait period)
        if not grab_controller.is_busy() and llm_controller.has_pending_tasks():
            if task_wait_timer > 0:
                # Waiting between tasks
                task_wait_timer -= 1
                if task_wait_timer % 100 == 0:
                    print(f"[MAIN] Waiting {task_wait_timer // 100} seconds before next task...")
            else:
                task = llm_controller.get_next_task()
                if task:
                    print(f"\n[MAIN] Starting task: {task}")
                    if task.coordinates:
                        accepted = grab_controller.process_task(task)
                        if accepted:
                            current_task = task
                        else:
                            print(f"[MAIN] Task rejected, skipping...")
                            llm_controller.task_failed("Task rejected by controller")
                    else:
                        print(f"[MAIN] Task has no coordinates, skipping...")
        
        # Update robot
        targets = grab_controller.update()
        scene["robot"].set_joint_position_target(targets)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=0.01)


if __name__ == "__main__":
    main()