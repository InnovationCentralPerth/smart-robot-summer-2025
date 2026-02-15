"""
Kinematics Module for Braccio Robot.

Contains:
- URDF-calibrated dimension constants
- Forward kinematics (FK)
- 2D Inverse kinematics (IK) solver
- Base angle calculation for target alignment
"""

import math

# ==========================================
#   URDF-CALIBRATED DIMENSIONS (meters)
# ==========================================
# From URDF joint origins:
ROBOT_BASE_Z = 0.02  # Robot is lifted slightly off ground
SHOULDER_HEIGHT = 0.0505 + ROBOT_BASE_Z  # Effective shoulder height from ground = 0.0705m
L_BASE          = 0.05
L_SHOULDER      = 0.1205  # Calibrated: 0.1205m
L_ELBOW         = 0.1205  # Calibrated: 0.1205m
L_WRIST         = 0.06     # wrist_pitch to wrist_roll
L_GRIPPER       = 0.07 # 0.02m to gripper pivot + 0.03m to finger contact point

# Total arm length for gripper tip in 2D plane
L_GRIPPER_TOTAL = L_WRIST + L_GRIPPER

# Reach compensation - negative to bring gripper closer to target
IK_REACH_OFFSET = 0.0  # Extend reach 1cm past cube center for better grip

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
def forward_kinematics(shoulder, elbow, wrist_pitch, base_angle=0.0):
    """
    Compute gripper tip position from joint angles using our link lengths.
    Returns (r, z) in 2D plane and (x, y, z) in 3D.
    """
    # Shoulder angle from vertical
    # shoulder=pi/2 means vertical up
    # Decreasing shoulder = tilting forward
    
    # Shoulder link
    # shoulder=1.57 (pi/2) is vertical. cos(1.57)=0 (r=0), sin(1.57)=1 (z=L).
    # So we use standard polar coordinates: r = L*cos(theta), z = L*sin(theta).
    
    elbow_r = L_SHOULDER * math.cos(shoulder)
    elbow_z = L_SHOULDER * math.sin(shoulder)
    
    # Elbow adds to the angle. If elbow=0 is straight, then forearm angle = shoulder.
    # But IK usually produces elbow relative to shoulder.
    # IK: theta_elbow = shoulder - elbow_angle (if elbow bends "in")?
    # Let's verify with IK logic: 
    #   gripper_angle = pi/2 + shoulder - elbow - wrist.
    # This implies angles SUBTRACT.
    
    forearm_angle = shoulder - elbow
    wrist_r = elbow_r + L_ELBOW * math.cos(forearm_angle)
    wrist_z = elbow_z + L_ELBOW * math.sin(forearm_angle)
    
    # Wrist pitch
    # gripper_angle = forearm_angle - wrist_pitch?
    # IK says: wrist = pi/2 + shoulder - elbow - gripper_abs.
    # so gripper_abs = pi/2 + shoulder - elbow - wrist.
    # Let's use that global angle:
    gripper_abs_angle = (math.pi / 2) + shoulder - elbow - wrist_pitch
    
    # But wait, is 'wrist_pitch' the joint angle or the absolute angle?
    # IK returns joint angles.
    # Let's assume standard chain: global = sum of relatives?
    # If the IK formula is correct: `wrist_angle = (math.pi / 2) + shoulder_angle - elbow_angle - angle_L3_abs`
    # Then `angle_L3_abs = (math.pi / 2) + shoulder - elbow - wrist`.
    # Let's use this angle_L3_abs for the gripper vector.
    
    gripper_angle = (math.pi / 2) + shoulder - elbow - wrist_pitch
    
    tip_r = wrist_r + L_GRIPPER_TOTAL * math.cos(gripper_angle)
    tip_z = wrist_z + L_GRIPPER_TOTAL * math.sin(gripper_angle)
    
    # Convert to world coordinates
    tip_z_world = tip_z + SHOULDER_HEIGHT
    
    # Convert 2D to 3D using base angle
    # Robot faces -Y.
    # r is distance from Z axis.
    # x = r * sin(base) -> if base=0, x=0.
    # y = -r * cos(base) -> if base=0, y=-r (forward in -Y).
    tip_x = tip_r * math.sin(base_angle)
    tip_y = -tip_r * math.cos(base_angle)
    
    return tip_r, tip_z_world, tip_x, tip_y

def solve_2d_ik(distance_2d, height):
    """
    Solve 2D IK for Braccio robot with iterative gripper offset correction.
    
    Args:
        distance_2d: horizontal distance from robot base to gripper tip  
        height: target height from ground
    
    Returns:
        shoulder_angle, elbow_angle, wrist_angle
    """
    
    def _solve_for_wrist_pos(r, h, target_tilt=-45.0):
        """
        Given wrist joint position (r, h) relative to shoulder,
        solve for shoulder and elbow angles.
        Also returns wrist angle to maintain `target_tilt`.
        
        Args:
            r, h: Wrist joint position (meters) relative to shoulder.
            target_tilt: Desired gripper tilt in degrees (from horizontal).
                         Default -45.0 (pointing down-forward).
        """
        # Distance from shoulder to wrist
        dist = math.sqrt(r**2 + h**2)
        
        # Check if reachable
        if dist > (L_SHOULDER + L_ELBOW):
            # Target too far, maximal reach
            # Force arm to point towards target
            angle_from_vertical = math.atan2(r, h)  # Angle of vector (r, h) from vertical
            
            shoulder = (math.pi / 2) - angle_from_vertical
            elbow = 0.0 # Straight arm
            
            # Wrist compensates to reach tilt?
            # If arm is straight at angle 'shoulder', gripper angle = ?
            # We want gripper_angle = target_tilt
            # gripper_angle = (pi/2) + shoulder - elbow - wrist
            # wrist = (pi/2) + shoulder - elbow - gripper_angle
            wrist = (math.pi / 2) + shoulder - 0.0 - math.radians(target_tilt)
            
            return shoulder, elbow, wrist
            
        else:
            # Standard analytical solution
            # Triangle: Side1=L_SHOULDER, Side2=L_ELBOW, Side3=dist
            
            # Angle of vector (r, h) from vertical z-axis
            # h is up, r is horizontal
            # atan2(r, h) gives angle from vertical (0,1) towards (r, h)
            angle_from_vertical = math.atan2(r, h)
            
            # Law of cosines
            cos_elbow = (L_SHOULDER**2 + L_ELBOW**2 - dist**2) / (2 * L_SHOULDER * L_ELBOW)
            cos_elbow = max(-1.0, min(1.0, cos_elbow))
            elbow_internal = math.acos(cos_elbow)
            
            cos_shoulder_offset = (L_SHOULDER**2 + dist**2 - L_ELBOW**2) / (2 * L_SHOULDER * dist)
            cos_shoulder_offset = max(-1.0, min(1.0, cos_shoulder_offset))
            shoulder_offset = math.acos(cos_shoulder_offset)
            
            # Compute angles
            shoulder = (math.pi / 2) - angle_from_vertical + shoulder_offset
            elbow = math.pi - elbow_internal
            
            # Gripper tilt from horizontal (negative = downward)
            GRIPPER_TILT = math.radians(target_tilt)
            
            # FK chain: gripper_from_horiz = pi/2 + shoulder - elbow - wrist
            # Solve for wrist: wrist = pi/2 + shoulder - elbow - TILT
            wrist = (math.pi / 2) + shoulder - elbow - GRIPPER_TILT
            if wrist < 0:
                wrist += math.pi
            
            return shoulder, elbow, wrist



    # Target height relative to shoulder
    # Shoulder is at z ~ 0.08m?
    # Let's check URDF or config. Assuming shoulder is at (0, 0, SHOULDER_HEIGHT)
    
    # Braccio dimensions (approx)
    # Base height?
    
    # Use -45° tilt for all heights.
    # -45° gives the best tradeoff of reach and height for all cube positions.
    # We rely on body-position feedback in LOWER state for fine convergence.
    tilt_deg = -45.0
        
    # ITERATION 1: Assume gripper points toward target (initial guess)
    target_r = distance_2d + IK_REACH_OFFSET  # Extend reach by offset
    target_z = height - SHOULDER_HEIGHT
    
    dist_to_target = math.sqrt(target_r**2 + target_z**2)
    if dist_to_target > 0.01:
        scale = (dist_to_target - L_GRIPPER_TOTAL) / dist_to_target
        wrist_r = target_r * scale
        wrist_z = target_z * scale
    else:
        wrist_r = 0.01
        wrist_z = target_z
    
    wrist_r = max(0.01, wrist_r)
    shoulder_angle, elbow_angle, wrist_angle = _solve_for_wrist_pos(wrist_r, wrist_z, target_tilt=tilt_deg)
    
    # ITERATION 2: Use actual wrist angle to correct gripper offset
    # FK chain: α₃ = (π/2 - shoulder) + elbow + (wrist - π/2)
    # gripper_from_horiz = π/2 - α₃ = π/2 + shoulder - elbow - wrist
    gripper_angle_from_horiz = (math.pi / 2) + shoulder_angle - elbow_angle - wrist_angle
    
    # Gripper offset components
    gripper_horiz = L_GRIPPER_TOTAL * math.cos(gripper_angle_from_horiz)
    gripper_vert = L_GRIPPER_TOTAL * math.sin(gripper_angle_from_horiz)
    
    # Corrected wrist position
    wrist_r = target_r - gripper_horiz
    wrist_z = target_z - gripper_vert
    wrist_r = max(0.01, wrist_r)
    
    # Recalculate with corrected position
    shoulder_angle, elbow_angle, wrist_angle = _solve_for_wrist_pos(wrist_r, wrist_z, target_tilt=tilt_deg)
    
    # Debug - calculate where gripper tip would actually end up (forward kinematics)
    # From shoulder, the arm extends:
    # - shoulder link at angle (pi/2 - shoulder_angle) from horizontal
    # - elbow link at additional bend
    # For simplicity, compute expected tip position:
    arm_angle_from_horiz = (math.pi/2) - shoulder_angle  # Shoulder tilt from horizontal
    elbow_bend = elbow_angle  # Additional bend at elbow
    
    # Approximate wrist position (ignoring exact kinematics for debug)
    # Just trace the target vs what we compute
    print(f"[IK] === DEBUG ===")
    print(f"[IK] Target gripper tip: r={distance_2d:.3f}m, z={height:.3f}m")
    print(f"[IK] Computed wrist pos: r={wrist_r:.3f}m, z={wrist_z:.3f}m")
    print(f"[IK] Gripper offset used: horiz={gripper_horiz:.3f}m, vert={gripper_vert:.3f}m")
    print(f"[IK] Gripper angle from horiz: {math.degrees(gripper_angle_from_horiz):.1f}°")
    print(f"[IK] RAW: shoulder={math.degrees(shoulder_angle):.1f}°, elbow={math.degrees(elbow_angle):.1f}°, wrist={math.degrees(wrist_angle):.1f}°")
    
    # Check if shoulder is below physical limit (~15 degrees = 0.26 rad)
    # If so, we must clamp it AND re-solve the other joints to maintain position at the cost of gripper orientation
    SHOULDER_LIMIT = 0.26
    if shoulder_angle < SHOULDER_LIMIT:
        print(f"[IK] Shoulder angle {math.degrees(shoulder_angle):.1f}° is below limit {math.degrees(SHOULDER_LIMIT):.1f}°")
        print(f"[IK] Applying FALLBACK IK: Fixing shoulder at limit and solving for Elbow/Wrist...")
        
        # PROSTRATE MODE: The arm is too low. We fix shoulder at limit.
        shoulder_angle = SHOULDER_LIMIT
        
        # New geometry:
        # We need to reach wrist potition (wrist_r, wrist_z)
        # Shoulder is fixed at angle `alpha` = pi/2 - SHOULDER_LIMIT from vertical (or SHOULDER_LIMIT from horiz)
        # Elbow joint position (E) is known:
        # Ex = L_SHOULDER * cos(pi/2 - SHOULDER_LIMIT) = L_SHOULDER * sin(SHOULDER_LIMIT)
        # Ez = L_SHOULDER * sin(pi/2 - SHOULDER_LIMIT) = L_SHOULDER * cos(SHOULDER_LIMIT)
        
        # Let's work in the frame relative to shoulder (0,0)
        # Elbow position relative to shoulder:
        # The shoulder joint rotates from vertical.
        # 0 = vertical
        # pi/2 = horizontal forward
        # Angle from vertical = pi/2 - shoulder_angle
        theta_s = math.pi/2 - shoulder_angle
        Ex = L_SHOULDER * math.sin(theta_s)
        Ez = L_SHOULDER * math.cos(theta_s)
        
        # Vector from Elbow to Wrist
        Wx = wrist_r
        Wz = wrist_z 
        # Vector d = W - E
        dx = Wx - Ex
        dz = Wz - Ez
        dist_EW = math.sqrt(dx**2 + dz**2)
        
        # We need to span this distance with Elbow link + Wrist link? No, wait.
        # The IK chain is: Shoulder -> (L_SHOULDER) -> Elbow -> (L_ELBOW) -> Wrist -> (L_WRIST+L_GRIPPER) -> Tip
        # We are solving for Wrist Center position (wrist_r, wrist_z).
        # We fixed Shoulder angle. 
        # We need to find Elbow angle such that the Elbow link reaches the Wrist Center?
        # WAIT. The standard IK solves for Wrist Center (joint 4).
        # Distance from Elbow to Wrist Center is simply L_ELBOW.
        # So we need dist_EW == L_ELBOW.
        
        print(f"[IK] Fallback: W=({Wx:.3f}, {Wz:.3f}), E=({Ex:.3f}, {Ez:.3f}), dist={dist_EW:.3f}, L_ELBOW={L_ELBOW:.3f}")
        
        if abs(dist_EW - L_ELBOW) > 0.01:
            # If we fixed shoulder, we might NOT be able to reach the wrist target anymore with just the elbow angle.
            # The elbow angle only controls the direction of the elbow link, not length.
            # If dist_EW != L_ELBOW, it means the wrist target is unattainable with fixed shoulder.
            # However, we have one more degree of freedom: the Gripper Angle.
            # In the main IK, we iterated to find a wrist pos that satisfies gripper angle.
            # Here, we must relax the gripper angle constraint.
            
            # Re-strategy: TARGET IS GRIPPER TIP (distance_2d, height).
            # We fixed Shoulder.
            # We have links: L_ELBOW, (L_WRIST + L_GRIPPER).
            # We treat (L_WRIST + L_GRIPPER) as a single link for simplicity? 
            # Or we can just point the gripper at the target?
            
            # Let's try 3-link IK with fixed first angle?
            # Link 1 (Shoulder) is fixed. Tip of Link 1 is E=(Ex, Ez).
            # Target is T=(target_r, target_z).
            # We need to reach T from E using Link 2 (Elbow) and Link 3 (Wrist+Gripper).
            # L2 = L_ELBOW
            # L3 = L_GRIPPER_TOTAL
            # Distance needed = dist(T - E)
            dist_ET = math.sqrt((target_r - Ex)**2 + (target_z - Ez)**2)
            
            # Law of cosines for the triangle formed by L2, L3, and dist_ET
            # angle at Elbow (internal)
            # dist_ET^2 = L2^2 + L3^2 - 2*L2*L3*cos(angle_between_links)
            
            cos_gamma = (L_ELBOW**2 + L_GRIPPER_TOTAL**2 - dist_ET**2) / (2 * L_ELBOW * L_GRIPPER_TOTAL)
            # Clamp
            cos_gamma = max(-1.0, min(1.0, cos_gamma))
            gamma = math.acos(cos_gamma) # Angle between Elbow link and Gripper link (0 = folded back)
            
            # Angle of vector ET relative to vertical? or horizontal?
            # Let's use standard trig functions atan2(dz, dx)
            angle_ET = math.atan2(target_z - Ez, target_r - Ex) # relative to horizontal x-axis
            
            # Now we need angle of Elbow link.
            # In triangle formed by E, Wrist, T (Wait, Wrist is joint, T is tip)
            # Triangle sides: L_ELBOW, dist_ET, L_GRIPPER_TOTAL
            # Angle at E (between ET and L_ELBOW)
            cos_alpha = (L_ELBOW**2 + dist_ET**2 - L_GRIPPER_TOTAL**2) / (2 * L_ELBOW * dist_ET)
            cos_alpha = max(-1.0, min(1.0, cos_alpha))
            alpha = math.acos(cos_alpha)
            
            # Elbow link angle (absolute from horizontal)
            # Two solutions: elbow up or elbow down. We want elbow up usually?
            # Geometry: Shoulder is high, target is low. 
            # Elbow link goes DOWN from shoulder?
            # Standard config: Shoulder link goes UP/FWD, Elbow link goes FWD/DOWN
            
            # Our angle_ET is likely negative (pointing down to target)
            # Solution 1: angle_elbow_abs = angle_ET + alpha
            # Solution 2: angle_elbow_abs = angle_ET - alpha
            
            # Let's guess Solution 1 for "convex" shape
            angle_elbow_abs = angle_ET + alpha
            
            # Now convert to robot joint angles.
            # Shoulder is fixed at shoulder_angle (from vertical)
            # Elbow joint angle is relative to shoulder link?
            # URDF definition: 0 means straight? Or standard DH?
            # In our FK: elbow_angle is passed to sin/cos. 
            # Previous calculation: elbow = pi - elbow_internal
            
            # Let's derive from geometry.
            # Shoulder link angle from horiz = (pi/2 - shoulder_angle)
            # Elbow link angle from horiz = angle_elbow_abs
            # Angle difference = (pi/2 - shoulder_angle) - angle_elbow_abs
            # This is the "bend" at the elbow?
            # Robot Elbow joint (elbow_angle):
            # If 0, arm is straight?
            # If pi, arm is fully bent back?
            # In `solve_2d_ik`: elbow = pi - elbow_internal.
            # elbow_internal was angle inside the triangle of Side1, Side2, Reach.
            
            # Let's stick to the vector logic:
            # Vec_Shoulder = [cos(theta_s), sin(theta_s)] (theta_s from horiz)
            # Vec_Elbow = [cos(angle_elbow_abs), sin(angle_elbow_abs)]
            # Angle between them?
            # We want the exterior angle?
            
            # Based on standard 2-link planar logic:
            # theta2 = atan2(...)
            
            # Let's map back to the robot's specific constraints.
            # Shoulder link absolute angle: Theta1_abs = pi/2 - shoulder_angle
            Theta1_abs = math.pi/2 - shoulder_angle
            Theta2_abs = angle_elbow_abs
            
            # Knee/Elbow bend:
            # If Theta2_abs == Theta1_abs, joint is 0? Or pi?
            # Usually joint = Theta1_abs - Theta2_abs
            
            # Let's sanity check with the standard solver's output:
            # elbow = pi - elbow_internal
            # This implies elbow=0 is fully bent? No, elbow=pi is fully straight?
            # Wait, standard cos rule gives internal angle.
            # if cos_elbow is -1 (dist=max), elbow_internal=pi, elbow=0. So 0 is straight.
            # if cos_elbow is 1 (dist=min), elbow_internal=0, elbow=pi. So pi is folded.
            
            # So elbow_angle = angle between link1 extension and link2.
            # link1 extension angle = Theta1_abs
            # link2 angle = Theta2_abs
            # elbow_angle = Theta1_abs - Theta2_abs?
            # Let's re-verify direction.
            # If arm is horizontal straight: Theta1=0, Theta2=0. elbow=0.
            # If arm is 90 deg bent (forearm down): Theta1=0, Theta2=-pi/2. elbow = 0 - (-pi/2) = pi/2.
            # So yes, elbow_angle = Theta1_abs - Theta2_abs.
            
            elbow_angle = Theta1_abs - Theta2_abs
            
            # Now Wrist.
            # We have Link 2 (Elbow) and Link 3 (Wrist/Gripper).
            # We treated L3 as a single rigid link from Elbow to Tip.
            # So the wrist joint must align the physical Gripper link with this virtual L3 line?
            # No, the Wrist joint is BETWEEN Link 2 and Link 3.
            # Link 3 aboslute angle: Theta3_abs
            # In our triangle calculation, angle of L3 (from E to T) relative to horizontal?
            # In the triangle (L_ELBOW, L_GRIPPER_TOTAL, dist_ET), the angle at E was 'alpha'.
            # The angle at T (tip) ? No we need angle of L3.
            # Angle of L3 relative to ET?
            # Angle at 'Tip' (or virtual wrist) in triangle:
            # cos_beta = (L3^2 + dist_ET^2 - L2^2) / (2*L3*dist_ET)
            # beta = acos(cos_beta)
            # Angle of L3_abs = angle_ET - beta (assuming "convex" down configuration)
            
            # Let's compute angle_L3_abs
            cos_beta = (L_GRIPPER_TOTAL**2 + dist_ET**2 - L_ELBOW**2) / (2 * L_GRIPPER_TOTAL * dist_ET)
            cos_beta = max(-1.0, min(1.0, cos_beta))
            beta = math.acos(cos_beta)
            
            angle_L3_abs = angle_ET - beta
            
            # Wrist joint (wrist_angle):
            # Controls angle between L_ELBOW and L_WRIST (which is L3 for us).
            # Same logic: wrist_angle = Theta2_abs - Theta3_abs ?
            # Wait, previous formula: wrist = pi/2 + shoulder - elbow - TILT
            # That was: wrist = (pi/2) + (pi/2 - Theta1_abs) - (Theta1_abs - Theta2_abs) - TILT
            # wrist = pi - 2*Theta1_abs + Theta1_abs - Theta2_abs - TILT ??? No.
            
            # Let's rely on absolute angles.
            # Robot Wrist:
            # If wrist=0, is it straight?
            # From `solve_for_wrist_pos`:
            # wrist = pi/2 + shoulder - elbow - TILT
            # If we assume TILT is the resulting absolute angle of gripper.
            # TILT_abs = Theta3_abs.
            
            # Re-arranging:
            # wrist = pi/2 + shoulder - elbow - Theta3_abs
            # Let's check signs. 
            # If shoulder=0 (vertical up), elbow=0 (straight up), wrist=0.
            # Gripper angle? 
            # wrist=0 => 0 = pi/2 + 0 - 0 - TILT => TILT = pi/2.
            # So gripper points back horizontal?
            # Wait.
            
            # Let's just use the relative angle logic.
            # wrist_angle is deviation from straight line of forearm?
            # If forearm is at Theta2_abs.
            # Gripper is at Theta3_abs.
            # wrist_angle ~ Theta2_abs - Theta3_abs + offset?
            # Let's deduce from standard FK:
            # global_gripper = (pi/2 - shoulder) + elbow + (wrist - pi/2)
            # global_gripper = Theta3_abs
            # Theta3_abs = (pi/2 - shoulder) - elbow + wrist - pi/2  (Wait, sign of elbow?)
            # In standard IK: `gripper_angle_from_horiz = (math.pi / 2) + shoulder_angle - elbow_angle - wrist_angle`
            # Note signs: +shoulder, -elbow, -wrist.
            # So: Theta3_abs = pi/2 + shoulder - elbow - wrist
            # => wrist = pi/2 + shoulder - elbow - Theta3_abs
            
            wrist_angle = (math.pi / 2) + shoulder_angle - elbow_angle - angle_L3_abs
            
    print(f"[IK] RAW: shoulder={math.degrees(shoulder_angle):.1f}°, elbow={math.degrees(elbow_angle):.1f}°, wrist={math.degrees(wrist_angle):.1f}°")
    
    # Check if shoulder is below physical limit (~15 degrees = 0.26 rad)
    # If so, we must clamp it AND re-solve the other joints to maintain position at the cost of gripper orientation
    SHOULDER_LIMIT = 0.26
    if shoulder_angle < SHOULDER_LIMIT:
        # ... (fallback code) ...
        pass

    # Clamp to URDF limits
    orig_shoulder = shoulder_angle
    orig_elbow = elbow_angle
    orig_wrist = wrist_angle
    
    shoulder_angle = max(0.26, min(2.88, shoulder_angle))
    elbow_angle = max(0.0, min(3.14, elbow_angle))
    wrist_angle = max(0.0, min(3.14, wrist_angle))
    
    if abs(wrist_angle - orig_wrist) > 0.01:
        print(f"[IK] WARNING: Wrist clamped from {math.degrees(orig_wrist):.1f}° to {math.degrees(wrist_angle):.1f}°")
    
    print(f"[IK] CLAMPED: shoulder={math.degrees(shoulder_angle):.1f}°, elbow={math.degrees(elbow_angle):.1f}°, wrist={math.degrees(wrist_angle):.1f}°")
    
    return shoulder_angle, elbow_angle, wrist_angle


# Formula for base rotation moved to get_base_angle_to_target (dynamic)

def get_base_angle_to_target(robot_x, robot_y, target_x, target_y, target_z=0.02, apply_offset=True):
    """
    Calculate base rotation to face target.
    Returns angle in radians.
    
    Args:
        target_z: Target Z height (affects lateral offset due to arm geometry)
        apply_offset: If True, apply gripper center offset (for grabbing).
                     If False, skip offset (for placing).
    """
    dx = target_x - robot_x
    dy = target_y - robot_y
    # atan2 gives angle from +X axis, but our robot's forward is -Y when base=0
    # URDF: base axis is -Z, so positive rotation is CW from above
    # Negate dx to match Isaac Sim visual coordinates (Right/Left swap)
    angle = math.atan2(-dx, -dy)  # INVERTED dx handles frame mismatch
    
    # Debug: show coordinate calculation
    print(f"[BASE_ANGLE] target=({target_x:.3f}, {target_y:.3f}), dx={dx:.3f}, dy={dy:.3f}, angle={math.degrees(angle):.1f}°")
    
    # Add offset for gripper center (only for grabbing)
    if apply_offset:
        # Calculate 2D distance from robot base to target
        r = math.sqrt(dx**2 + dy**2)

        # 1. Reach-dependent offset: atan2(lateral offset, reach)
        # Note: arctan(0.021 / 0.20) is exactly +6.0 degrees.
        # This rotates the arm LEFT to center fingers that are physically on the RIGHT.
        
        # Determine lateral offset based on Z height
        # Lower cubes need slightly more rotation to align properly
        lateral_offset = 0.021
        if target_z < 0.045:
             # Bottom cube: needs slightly more curve
             lateral_offset = 0.021 # Increased from 0.021/0.024
             
        reach_correction = math.atan2(lateral_offset, max(0.01, r))
        print(f"[BASE_ANGLE] LateralOffset={lateral_offset:.3f}, ReachCorr={math.degrees(reach_correction):.1f}°")
        
        # 2. Periodic compensation: compensates for base misalignment at diagonals
        periodic_correction = math.radians(1.2) * math.sin(2 * angle)
        
        angle += (reach_correction + periodic_correction)
        
        # Debug: show coordinate calculation
        print(f"[BASE_ANGLE] target=({target_x:.3f}, {target_y:.3f}), r={r:.3f}m")
        print(f"[BASE_ANGLE] ReachCorr={math.degrees(reach_correction):.1f}°, PeriodicCorr={math.degrees(periodic_correction):.1f}°")
        print(f"[BASE_ANGLE] final_angle={math.degrees(angle):.1f}°")
    # Normalize to [-π, π] to stay within joint limits
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
        
    return angle
