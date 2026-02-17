"""
Grab Controller for Braccio Robot.

State machine that manages grab, place, and home sequences.
"""

import torch
import math

from kinematics import (
    forward_kinematics,
    solve_2d_ik,
    get_base_angle_to_target,
    SPEED_LIMIT,
)
from llm_planner import Task


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
        
        # Find gripper link index for debug logging
        # The link is usually named "gripper_movable" or similar in the USD
        try:
            # find_bodies returns (indices, names)
            indices, names = self.robot.find_bodies(".*gripper_movable.*")
            if len(indices) > 0:
                self.gripper_idx = indices[0]
                print(f"[DEBUG] Found gripper link '{names[0]}' at index {self.gripper_idx}")
            else:
                self.gripper_idx = None
                print("[DEBUG] WARNING: Could not find gripper link for debug logging")
        except Exception as e:
            print(f"[DEBUG] Error finding gripper link: {e}")
            self.gripper_idx = None
        
    def process_task(self, task: Task) -> bool:
        """
        Process a Task object from the LLM planner.
        Returns True if task was accepted, False if robot is busy.
        """
        if self.state not in ["IDLE", "HOLDING"]:
            print(f"[GrabController] Robot busy in state {self.state}, cannot accept task")
            return False
        
        if task.action == "grab":
            if not task.coordinates:
                print(f"[GrabController] Grab task has no coordinates!")
                return False
            
            self.cube_x, self.cube_y, self.cube_z = task.coordinates
            print(f"\n[GRAB] Target: {task.target} at ({self.cube_x:.3f}, {self.cube_y:.3f}, {self.cube_z:.3f})")
            
            # Rotate base to face target
            base_angle = get_base_angle_to_target(0, 0, self.cube_x, self.cube_y, target_z=self.cube_z)
            self.target_joints[0, 0] = base_angle
            
            # Keep arm in candle pose while rotating
            self.target_joints[0, 1] = 1.57
            self.target_joints[0, 2] = 0.0
            self.target_joints[0, 3] = 1.57
            self.target_joints[0, 5] = 0.175  # Gripper open
            
            # Store target key for live updates
            if task.target == "green_cube":
                self.target_key = "cube"
            elif task.target == "red_cube":
                self.target_key = "red_cube"
            else:
                self.target_key = None

            print(f"[GRAB] Rotating base to {math.degrees(base_angle):.1f}°")
            self.state = "ROTATE"
            self.timer = 0
            return True
            
        elif task.action == "place":
            if self.state != "HOLDING":
                print(f"[GrabController] Cannot place - not holding anything!")
                return False
            
            if not task.coordinates:
                print(f"[GrabController] Place task has no coordinates!")
                return False
            
            self.place_x, self.place_y, self.place_z = task.coordinates
            print(f"\n[PLACE] Target: ({self.place_x:.3f}, {self.place_y:.3f}, {self.place_z:.3f})")
            
            # Lift arm to candle pose first
            print("[PLACE] Lifting to candle pose...")
            self.target_joints[0, 1] = 1.57
            self.target_joints[0, 2] = 0.0
            self.target_joints[0, 3] = 1.57
            
            self.state = "PLACE_LIFT"
            self.timer = 0
            
            # Update cube_z immediately so threshold logic uses target height
            self.cube_z = self.place_z
            return True
        
        return False
    
    def process_command(self, cmd):
        """Legacy command processing for backward compatibility."""
        if "grab" in cmd.lower():
            # Auto-detect green cube position
            cube_pos = self.scene["cube"].data.root_pos_w[0]
            self.cube_x = float(cube_pos[0])
            self.cube_y = float(cube_pos[1])
            self.cube_z = float(cube_pos[2])
            
            print(f"\n[GRAB] Cube detected at: ({self.cube_x:.3f}, {self.cube_y:.3f}, {self.cube_z:.3f})")
            
            base_angle = get_base_angle_to_target(0, 0, self.cube_x, self.cube_y, target_z=self.cube_z)
            self.target_joints[0, 0] = base_angle
            self.target_joints[0, 1] = 1.57
            self.target_joints[0, 2] = 0.0
            self.target_joints[0, 3] = 1.57
            self.target_joints[0, 5] = 0.175
            
            print(f"[GRAB] Rotating base to {math.degrees(base_angle):.1f}°")
            self.state = "ROTATE"
            self.timer = 0
            
        elif cmd.lower().startswith("place"):
            parts = cmd.split()
            if len(parts) >= 4 and self.state == "HOLDING":
                self.place_x = float(parts[1])
                self.place_y = float(parts[2])
                self.place_z = float(parts[3])
                print(f"\n[PLACE] Target: ({self.place_x:.3f}, {self.place_y:.3f}, {self.place_z:.3f})")
                
                print("[PLACE] Lifting to candle pose...")
                self.target_joints[0, 1] = 1.57
                self.target_joints[0, 2] = 0.0
                self.target_joints[0, 3] = 1.57
                
                self.state = "PLACE_LIFT"
                self.timer = 0
            elif self.state != "HOLDING":
                print("[PLACE] Must grab cube first!")
            else:
                print("[PLACE] Usage: place x y z")
                
        elif "home" in cmd.lower():
            self.target_joints = self.home_pose.clone()
            self.state = "HOME"
            print("[HOME] Returning to home position")
    
    def is_busy(self) -> bool:
        """Check if robot is currently executing a task."""
        return self.state not in ["IDLE", "HOLDING"]
    
    def is_holding(self) -> bool:
        """Check if robot is holding an object."""
        return self.state == "HOLDING"
    
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
                print("[GRAB] Base rotation complete. Moving ABOVE cube...")
                
                # Calculate 2D distance
                distance_2d = math.sqrt(self.cube_x**2 + self.cube_y**2)
                
                # Phase 1: Position arm ABOVE the cube (8cm higher)
                hover_height = self.cube_z + 0.08
                
                shoulder, elbow, wrist = solve_2d_ik(distance_2d, hover_height)
                
                self.target_joints[0, 1] = shoulder
                self.target_joints[0, 2] = elbow
                self.target_joints[0, 3] = wrist
                self.target_joints[0, 5] = 0.175  # Fully open
                
                print(f"[IK] Hover ABOVE -> dist={distance_2d:.3f}, z={hover_height:.3f}")
                print(f"[IK] shoulder={math.degrees(shoulder):.1f}°, elbow={math.degrees(elbow):.1f}°, wrist={math.degrees(wrist):.1f}°")
                
                self.state = "HOVER_ABOVE"
                self.timer = 0
            return current
        
        elif self.state == "HOVER_ABOVE":
            if self._move_towards_target(current):
                print("[GRAB] Above cube. Lowering to grab height...")
                
                # === BODY POSITION DEBUG ===
                try:
                    body_pos = self.robot.data.body_pos_w
                    body_names = self.robot.data.body_names
                    print(f"[BODY] === LINK POSITIONS (arm settled above cube) ===")
                    for i, name in enumerate(body_names):
                        pos = body_pos[0, i]
                        print(f"[BODY] {name}: x={float(pos[0]):.4f}, y={float(pos[1]):.4f}, z={float(pos[2]):.4f}")
                    print(f"[BODY] === DISTANCES ===")
                    for i in range(len(body_names) - 1):
                        d = float(torch.norm(body_pos[0, i+1] - body_pos[0, i]))
                        print(f"[BODY] {body_names[i]} -> {body_names[i+1]}: {d:.4f}m")
                    print(f"[BODY] Cube target: x={self.cube_x:.4f}, y={self.cube_y:.4f}, z={self.cube_z:.4f}")
                except Exception as e:
                    print(f"[BODY] Error: {e}")
                
                # Refresh coordinates just before lowering (in case cube moved)
                if hasattr(self, 'target_key') and self.target_key:
                    try:
                        # Get latest position from simulation
                        pos = self.scene[self.target_key].data.root_pos_w[0]
                        self.cube_x = float(pos[0])
                        self.cube_y = float(pos[1])
                        self.cube_z = float(pos[2])
                        print(f"[GRAB] Refreshed target coordinates: ({self.cube_x:.3f}, {self.cube_y:.3f}, {self.cube_z:.3f})")
                    except Exception as e:
                        print(f"[GRAB] Failed to refresh coordinates for {self.target_key}: {e}")

                # Phase 2: Lower to grab height
                # Target slightly ABOVE top of cube to account for tilt/error
                distance_2d = math.sqrt(self.cube_x**2 + self.cube_y**2)
                
                # For bottom cubes: target IK directly at cube height → gives E≈108°
                # For top cubes: add offset → gives E≈86° (similar to hover, settles fast)
                if self.cube_z < 0.045:
                    grab_z = self.cube_z +0.03 # Bottom cube: target directly
                else:
                    grab_z = self.cube_z + 0.04  # Top cube: grab a bit higher
                print(f"[GRAB] cube_z={self.cube_z:.3f}, grab_z={grab_z:.3f}")
                
                shoulder, elbow, wrist = solve_2d_ik(distance_2d, grab_z)
                
                self.target_joints[0, 1] = shoulder
                self.target_joints[0, 2] = elbow
                self.target_joints[0, 3] = wrist
                
                print(f"[IK] LOWER -> dist={distance_2d:.3f}, z={self.cube_z:.3f}")
                print(f"[IK] shoulder={math.degrees(shoulder):.1f}°, elbow={math.degrees(elbow):.1f}°, wrist={math.degrees(wrist):.1f}°")
                
                self.state = "LOWER"
                self.timer = 0


            return current
        
        elif self.state == "LOWER":
            # Keep commanding the arm toward IK target
            self._move_towards_target(current, threshold=0.01)
            
            # Check ACTUAL gripper body position from physics simulation
            gripper_close_enough = False
            timeout = self.timer > 1500  # Safety timeout (~15 seconds at 100Hz)
            
            if self.gripper_idx is not None:
                gripper_pos = self.robot.data.body_pos_w[0, self.gripper_idx]
                gripper_z = float(gripper_pos[2])
                gripper_r = math.sqrt(float(gripper_pos[0])**2 + float(gripper_pos[1])**2)
                target_r = math.sqrt(self.cube_x**2 + self.cube_y**2)
                
                # Target: gripper_movable should be close to cube
                # Successful red cube grab had gripper_gm 5.9cm above cube center.
                # Use +0.04 offset (4cm above cube center) as target.
                desired_gripper_z = self.cube_z + 0.04
                z_error = gripper_z - desired_gripper_z  # Positive = too high
                r_error = abs(gripper_r - target_r)
                
                # Log periodically
                if self.timer % 50 == 0:
                    print(f"[LOWER] Gripper body: r={gripper_r:.3f}, z={gripper_z:.3f} | "
                          f"DesiredZ: {desired_gripper_z:.3f} | "
                          f"z_err={z_error:+.3f}, r_err={r_error:.3f} | Timer={self.timer}")
                
                # DIRECT PROPORTIONAL CONTROL: Nudge ELBOW up to lower gripper.
                # Wait 200 steps for IK to settle, then nudge every 50 steps
                # so the arm has time to respond between adjustments.
                if self.timer > 200 and self.timer % 50 == 0 and z_error > 0.05:
                    if not hasattr(self, '_elbow_nudge_total'):
                        self._elbow_nudge_total = 0.0
                    
                    max_elbow_nudge = math.radians(20.0)
                    
                    if self._elbow_nudge_total < max_elbow_nudge:
                        # Proportional nudge, capped per step
                        elbow_nudge = min(z_error * 0.2, math.radians(2.0))
                        elbow_nudge = min(elbow_nudge, max_elbow_nudge - self._elbow_nudge_total)
                        
                        new_elbow = float(self.target_joints[0, 2]) + elbow_nudge
                        new_elbow = min(new_elbow, 3.14)
                        self.target_joints[0, 2] = new_elbow
                        self._elbow_nudge_total += elbow_nudge
                        
                        print(f"[LOWER] Elbow nudge: E={math.degrees(new_elbow):.1f}°, "
                              f"total={math.degrees(self._elbow_nudge_total):.1f}°, "
                              f"z_err={z_error:+.3f}")
                
                # Accept gripper position when within 5cm of desired Z
                # MUST wait for arm to settle (timer > 200) to avoid false convergence
                # during the HOVER→LOWER transition
                if self.timer > 200 and abs(z_error) < 0.05:
                    gripper_close_enough = True
                    print(f"[LOWER] ✓ Gripper reached cube! z_err={z_error:+.3f}, r_err={r_error:.3f}")
            
            if gripper_close_enough or timeout:
                if timeout:
                    gripper_pos = self.robot.data.body_pos_w[0, self.gripper_idx] if self.gripper_idx else None
                    gz = float(gripper_pos[2]) if gripper_pos is not None else -1
                    print(f"[LOWER] ⚠ Timeout! Gripper at z={gz:.3f}, cube at z={self.cube_z:.3f}")
                
                # Clean up nudge state
                if hasattr(self, '_elbow_nudge_total'):
                    del self._elbow_nudge_total
                
                # Read ACTUAL body positions for debug
                try:
                    body_pos = self.robot.data.body_pos_w
                    body_names = self.robot.data.body_names
                    print(f"[BODY DEBUG] === ACTUAL LINK POSITIONS ===")
                    for i, name in enumerate(body_names):
                        pos = body_pos[0, i]
                        print(f"[BODY DEBUG] {name}: x={float(pos[0]):.4f}, y={float(pos[1]):.4f}, z={float(pos[2]):.4f}")
                except Exception as e:
                    print(f"[BODY DEBUG] Error: {e}")
                
                print(f"[BODY DEBUG] Target cube: x={self.cube_x:.4f}, y={self.cube_y:.4f}, z={self.cube_z:.4f}")
                
                print("[GRAB] At grab position. Closing gripper...")
                self.target_joints[0, 5] = 1.1
                self.state = "CLOSE"
                self.timer = 0
            return current
        
        elif self.state == "CLOSE":
            # Keep arm steady
            self._move_towards_target(current)
            
            # Close gripper using the gripper control function
            gripper_closed = self._move_gripper(current, target_pos=1.1, speed=0.001)
            
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
            current[0, 5] = 1.1
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
            current[0, 5] = 1.1
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
                
                # Update cube_z for dynamic threshold calculation (already done in PLACE_LIFT, but ensuring)
                self.cube_z = self.place_z
                
                self.state = "PLACE_REACH"
            # Keep gripper closed
            current[0, 5] = 1.1
            return current
        
        elif self.state == "PLACE_REACH":
            # Strict distance check: only release if physically close to target (r, z)
            # Calculate distance in 2D plane (r)
            r_current, z_current, _, _ = forward_kinematics(
                float(current[0, 1]), float(current[0, 2]), float(current[0, 3]), float(current[0, 0])
            )
            dist_error = abs(r_current - (getattr(self, 'place_r', 0.25)))
            
            # Use cube_z as target height (should be same as place_z but tracked better)
            target_z = getattr(self, 'cube_z', 0.02)
            z_error = abs(z_current - target_z)
            
            # Check joint convergence
            joints_converged = self._move_towards_target(current, threshold=0.05)
            
            if joints_converged:
                if dist_error < 0.05 and z_error < 0.05:
                    print(f"[DEBUG] PLACE_REACH complete! JointErr={float(torch.norm(self.target_joints[:, :5] - current[:, :5])):.4f}, DistErr={dist_error:.4f}, ZErr={z_error:.4f}")
                    print("[PLACE] At position. Tilting wrist to release...")
                    # Tilt wrist UP (towards 0 if coming from positive) so cube can slide out
                    # Previous logic subtracted, which might have tilted down depending on usage
                    self.target_joints[0, 3] = max(0.0, float(self.target_joints[0, 3]) - 0.3)  # Tilt UP ~17°
                    
                    self.state = "PLACE_TILT"
                    self.timer = 0
                else:
                    # Joints matched but physical distance is wrong (IK ambiguity?)
                    if self.timer % 50 == 0:
                        print(f"[DEBUG] Joints converged but DistErr {dist_error:.4f} or ZErr {z_error:.4f} > 0.05. Waiting...")
            
            # Keep gripper closed while moving
            current[0, 5] = 1.1
            return current
        
        elif self.state == "PLACE_TILT":
            self._move_towards_target(current)
            self.timer += 1
            if self.timer > 50:  # Wait 0.5s for tilt
                print("[PLACE] Opening gripper...")
                self.state = "RELEASE"
                self.timer = 0
            # Keep gripper closed while tilting
            current[0, 5] = 1.1
            return current
        
        elif self.state == "RELEASE":
            self._move_towards_target(current)
            # Increase speed to open quickly (0.05 instead of default 0.001)
            target_open = 0.175  # Fully open
            gripper_open = self._move_gripper(current, target_pos=target_open, speed=0.05)
            
            # Also update target_joints so future states know we want it open
            self.target_joints[0, 5] = target_open
            
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
    
    def _move_towards_target(self, current, threshold=None):
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
        
        # Check if arm is close enough
        arm_error = float(torch.norm(arm_diff))
        
        # Dynamic threshold logic if not specified
        if threshold is None:
            # Horizontal threshold: depends on base angle (error peaks at 45, 135 degrees)
            base_angle = float(current[0, 0])
            sin_factor = abs(math.sin(2 * base_angle))
            horizontal_threshold = 0.705 + (0.035 * sin_factor)
            
            # Vertical threshold: higher targets = arm more vertical = less error
            z_target = getattr(self, 'cube_z', 0.02)  # Default to low z if not set
            
            # HARDCODED FOR TESTING: use 0.4 threshold for stacked cubes (z > 0.05)
            if z_target > 0.05:
                threshold = 0.1  # Lenient for testing high-z grabs
                if self.timer % 50 == 0:
                    print(f"[DEBUG HIGH-Z] z={z_target:.3f}, arm_error={arm_error:.4f}, threshold={threshold:.3f}")
            else:
                vertical_threshold = max(0, (z_target - 0.02)) * 1.2
                # Use stricter threshold for low Z placements (0.1 instead of 0.3 fallback)
                threshold = max(0.1, horizontal_threshold - vertical_threshold)
            
        # Debug: show arm error periodically
        self.timer += 1
        if self.timer % 100 == 0:
            print(f"[DEBUG] Arm error: {arm_error:.4f}, Threshold: {threshold:.3f}, State: {self.state}")
            self._log_ground_truth_state()
        
        return arm_error < threshold
    
    def _log_ground_truth_state(self):
        """Log actual physical state of the robot from simulation physics."""
        if self.gripper_idx is None:
            return
            
        # Get actual joint positions (degrees)
        joints = self.robot.data.joint_pos[0]
        # joint_names = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"]
        
        # Get actual gripper position (world coords)
        # body_pos_w is [num_envs, num_bodies, 3]
        gripper_pos = self.robot.data.body_pos_w[0, self.gripper_idx]
        
        print(f"[DEBUG GT] Joints: S={math.degrees(joints[1]):.1f}, E={math.degrees(joints[2]):.1f}, W={math.degrees(joints[3]):.1f}")
        print(f"[DEBUG GT] Gripper Tip (World): ({gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f})")
        
        # Calculate derived FK for comparison
        # radius = sqrt(x^2 + y^2)
        r_actual = math.sqrt(gripper_pos[0]**2 + gripper_pos[1]**2)
        z_actual = gripper_pos[2]
        print(f"[DEBUG GT] Gripper Tip (Polar): r={r_actual:.3f}, z={z_actual:.3f}")
    
    def _move_gripper(self, current, target_pos, speed=0.001):
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
