import pybullet as p
import pybullet_data
import pandas as pd
import math
import time
import numpy as np
import pygame

# ============================================================
# GAIT DATA
# ============================================================
FILE_PATH = "gait_params.csv"
df_master = pd.read_csv(FILE_PATH, skipinitialspace=True)
df_master.columns = df_master.columns.str.strip()
df_master["direction"] = df_master["direction"].astype(str).str.strip().str.lower()

# ============================================================
# ROBOT CONSTANTS & GLOBAL TUNING
# ============================================================
L1, L2 = 0.1000, 0.1000
BODY_OFFSET = 0.0544

# Restored high-performance motor parameters from Script 1
GLOBAL_MAX_TORQUE = 1.8
GLOBAL_MAX_VELOCITY = 6.0
KNEE_ASSEMBLY_OFFSET = math.pi / 2

DT = 1.0 / 240.0

# --- HEIGHT ADJUSTMENT CONFIGURATION ---
MIN_HEIGHT = 0.14           # Minimum allowed body height
MAX_HEIGHT = 0.24           # Maximum allowed body height
HEIGHT_SPEED = 0.06         # How fast the height changes per second (meters/sec)
current_body_height = 0.20  # Starting target position

# Logitech F710 X-Mode Windows Axis Layout
AXIS_L_STICK_X = 0
AXIS_L_STICK_Y = 1
AXIS_R_STICK_X = 3           
AXIS_L_TRIGGER = 2           
AXIS_R_TRIGGER = 5           

JOYSTICK_DEADZONE = 0.12     
TRIGGER_THRESHOLD = 0.05     

LEG_ORDER = ["FL", "ML", "RL", "FR", "MR", "RR"]
LEG_PHASES = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5] # Tripod Gait

LEG_ROOTS = {
    "FL": [0.12, 0.10], "ML": [0.00, 0.14], "RL": [-0.12, 0.10],
    "FR": [0.12, -0.10], "MR": [0.00, -0.14], "RR": [-0.12, -0.10]
}

# ------------------------------------------------------------
# DIRECTION-SPECIFIC CONFIGURATION (VARIABLE SETTINGS)
# ------------------------------------------------------------
DIRECTION_PARAMS = {
    "straight": {
        "step_height": 0.010,
        "urdf_x_offset": -0.015
    },
    "sideways": {
        "step_height": 0.020,   
        "urdf_x_offset": 0.0058
    },
    "diagonal": {
        "step_height": 0.010,   
        "urdf_x_offset": -0.015  
    },
    "spin": {
        "step_height": 0.020,   
        "urdf_x_offset": 0.020 
    }
}

# ============================================================
# GAIT SELECTION & PARAMETER BLENDING
# ============================================================
def safe_pick_row(mode, normalized_mag):
    subset = df_master[df_master["direction"] == mode]
    if subset.empty:
        subset = df_master[df_master["direction"] == "straight"]
    subset = subset.reset_index(drop=True)

    max_csv_speed = subset["speed"].max()
    min_csv_speed = subset["speed"].min()

    target = min_csv_speed + (normalized_mag * (max_csv_speed - min_csv_speed))
    idx = (subset["speed"] - target).abs().argmin()
    return subset.iloc[idx]

def get_blended_gait_params(joy_fwd, joy_side, joy_spin, norm_mag):
    """
    Blends parameters proportionally based on joystick input weights.
    Yields 100% pure values when driving unmixed axes.
    """
    f = abs(joy_fwd)
    s = abs(joy_side)
    r = abs(joy_spin)

    # Deconstruct mixed inputs into analytical weights
    diag_w = min(f, s) * 2.0             
    straight_w = max(0.0, f - diag_w / 2.0)
    sideways_w = max(0.0, s - diag_w / 2.0)
    spin_w = r
    
    total_w = straight_w + sideways_w + diag_w + spin_w

    # Fetch lookup properties from library slices
    row_str = safe_pick_row("straight", norm_mag)
    row_sid = safe_pick_row("sideways", norm_mag)
    row_dia = safe_pick_row("diagonal", norm_mag)
    row_spi = safe_pick_row("spin", r)

    if total_w > 0.001:
        # Weighted normalized combination blends seamlessly without dampening limits
        freq = (straight_w * row_str["frequency"] + sideways_w * row_sid["frequency"] + 
                diag_w * row_dia["frequency"] + spin_w * row_spi["frequency"]) / total_w
                
        step_amplitude = (straight_w * row_str["step_amplitude"] + sideways_w * row_sid["step_amplitude"] + 
                          diag_w * row_dia["step_amplitude"] + spin_w * row_spi["step_amplitude"]) / total_w
                          
        step_h = (straight_w * DIRECTION_PARAMS["straight"]["step_height"] +
                  sideways_w * DIRECTION_PARAMS["sideways"]["step_height"] +
                  diag_w     * DIRECTION_PARAMS["diagonal"]["step_height"] +
                  spin_w     * DIRECTION_PARAMS["spin"]["step_height"]) / total_w

        urdf_x = (straight_w * DIRECTION_PARAMS["straight"]["urdf_x_offset"] +
                  sideways_w * DIRECTION_PARAMS["sideways"]["urdf_x_offset"] +
                  diag_w     * DIRECTION_PARAMS["diagonal"]["urdf_x_offset"] +
                  spin_w     * DIRECTION_PARAMS["spin"]["urdf_x_offset"]) / total_w
    else:
        freq = row_str["frequency"]
        step_amplitude = row_str["step_amplitude"]
        step_h = DIRECTION_PARAMS["straight"]["step_height"]
        urdf_x = DIRECTION_PARAMS["straight"]["urdf_x_offset"]

    return {"freq": freq, "step_amplitude": step_amplitude, "step_h": step_h, "urdf_x": urdf_x}

# ============================================================
# INVERSE KINEMATICS
# ============================================================
def solve_leg_ik_3dof(tx, ty, tz, urdf_x_offset):
    x = tx + urdf_x_offset
    y = ty
    z_from_thigh = -(tz - BODY_OFFSET)

    hip = math.atan2(y, -z_from_thigh)
    z_sag = -math.sqrt(y*y + z_from_thigh*z_from_thigh)

    dist_sq = x*x + z_sag*z_sag
    dist = math.sqrt(dist_sq)

    if dist > (L1 + L2) * 0.99 or dist < abs(L1 - L2):
        return None, None, None

    cos_phi = (L1*L1 + L2*L2 - dist_sq) / (2*L1*L2)
    knee = math.pi - math.acos(np.clip(cos_phi, -1, 1))

    alpha = math.atan2(z_sag, x)
    cos_beta = (L1*L1 + dist_sq - L2*L2) / (2*L1*dist)
    beta = math.acos(np.clip(cos_beta, -1, 1))

    thigh = alpha - beta + math.pi/2
    return hip, thigh, knee

# ============================================================
# INIT SIMULATION
# ============================================================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")
p.changeDynamics(plane_id, -1, lateralFriction=1.0)

robot_id = p.loadURDF(
    "HexaDog_ZBD.urdf", 
    [0, 0, 0.05],
    flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
)
p.changeDynamics(robot_id, -1, lateralFriction=1.0)

joint_map = {
    p.getJointInfo(robot_id, j)[1].decode(): j
    for j in range(p.getNumJoints(robot_id))
}

for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, lateralFriction=1.0)

pygame.init()
joystick = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
if joystick:
    joystick.init()

phase_accumulator = 0.0
physics_step = 0

# --- METRICS TRACKING ---
pos, _ = p.getBasePositionAndOrientation(robot_id)
last_cycle_x, last_cycle_y = pos[0], pos[1]
cycle_time_elapsed = 0.0
cycle_count = 0

# ============================================================
# MAIN RUNTIME LOOP
# ============================================================
try:
    while True:
        pygame.event.pump()

        if joystick:
            raw_fwd = -joystick.get_axis(AXIS_L_STICK_Y)
            raw_side = -joystick.get_axis(AXIS_L_STICK_X)
            raw_spin = joystick.get_axis(AXIS_R_STICK_X)

            joy_fwd = raw_fwd if abs(raw_fwd) > JOYSTICK_DEADZONE else 0.0
            joy_side = raw_side if abs(raw_side) > JOYSTICK_DEADZONE else 0.0
            joy_spin = raw_spin if abs(raw_spin) > JOYSTICK_DEADZONE else 0.0

            joy_mag = math.sqrt(joy_fwd**2 + joy_side**2)
            norm_mag = min(1.0, joy_mag)

            # --- DYNAMIC TRIGGER-BASED HEIGHT SCALING ---
            raw_lt = joystick.get_axis(AXIS_L_TRIGGER)
            raw_rt = joystick.get_axis(AXIS_R_TRIGGER)
            norm_lt = (raw_lt + 1.0) / 2.0
            norm_rt = (raw_rt + 1.0) / 2.0

            if norm_rt > TRIGGER_THRESHOLD:
                current_body_height += HEIGHT_SPEED * norm_rt * DT
            if norm_lt > TRIGGER_THRESHOLD:
                current_body_height -= HEIGHT_SPEED * norm_lt * DT

            current_body_height = np.clip(current_body_height, MIN_HEIGHT, MAX_HEIGHT)
        else:
            # --- KEYBOARD FALLBACK MODE ---
            keys = p.getKeyboardEvents()
            
            # Forward / Backward
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                joy_fwd = 0.5
            elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                joy_fwd = -0.5
            else:
                joy_fwd = 0.0

            # Sideways Strafe
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                joy_side = -0.5
            elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                joy_side = 0.5
            else:
                joy_side = 0.0

            # Spin (Q = CCW, E = CW)
            if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
                joy_spin = -0.4
            elif ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
                joy_spin = 0.4
            else:
                joy_spin = 0.0

            joy_mag = math.sqrt(joy_fwd**2 + joy_side**2)
            norm_mag = min(1.0, joy_mag)

        active = norm_mag > 0.02 or abs(joy_spin) > 0.02
        direction_rad = math.atan2(joy_side, joy_fwd) if norm_mag > 0.001 else 0.0

        # Retrieve smoothly blended parameters
        g = get_blended_gait_params(joy_fwd, joy_side, joy_spin, norm_mag)

        # --------------------------------------------------------
        # PHASE UPDATE & CYCLE DETECTION
        # --------------------------------------------------------
        if active:
            phase_accumulator = (phase_accumulator + g["freq"] * DT)
        else:
            phase_accumulator = 0.0

        cycle_time_elapsed += DT

        if phase_accumulator >= 1.0:
            current_pos, _ = p.getBasePositionAndOrientation(robot_id)
            dx = current_pos[0] - last_cycle_x
            dy = current_pos[1] - last_cycle_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if cycle_time_elapsed > 0:
                avg_speed_cms = (distance / cycle_time_elapsed) * 100.0
                cycle_count += 1
                print(f"[Cycle {cycle_count:03d}] Speed: {avg_speed_cms:6.2f} cm/s | Freq: {g['freq']:.2f}Hz | Target Body H: {current_body_height:.3f}m")
            
            last_cycle_x, last_cycle_y = current_pos[0], current_pos[1]
            cycle_time_elapsed = 0.0

        phase_accumulator = phase_accumulator % 1.0

        # --------------------------------------------------------
        # LEG TRAJECTORY GENERATION
        # --------------------------------------------------------
        for idx, leg in enumerate(LEG_ORDER):
            lx_root, ly_root = LEG_ROOTS[leg]

            phase = (phase_accumulator + LEG_PHASES[idx]) % 1.0
            s_phase = (phase if phase < 0.5 else phase - 0.5) * 2.0

            cycloid_factor = s_phase - (math.sin(2.0 * math.pi * s_phase) / (2.0 * math.pi))

            if phase < 0.5:  # Swing phase
                phase_mult = (1.0 - 2.0 * cycloid_factor)
                tz = current_body_height + g["step_h"] * 0.5 * (1.0 - math.cos(2.0 * math.pi * s_phase))
            else:  # Stance phase
                phase_mult = (-1.0 + 2.0 * cycloid_factor)
                tz = current_body_height


            sign_x = np.sign(math.cos(direction_rad))
            sign_y = np.sign(math.sin(direction_rad))
            
            tx_trans = g["step_amplitude"] * phase_mult * abs(math.cos(direction_rad)) * sign_x * norm_mag
            ty_trans = g["step_amplitude"] * phase_mult * abs(math.sin(direction_rad)) * sign_y * norm_mag


            omega = -joy_spin * g["step_amplitude"] * 2.0
            tx_spin = -ly_root * omega * phase_mult
            ty_spin = lx_root * omega * phase_mult


            tx = tx_trans + tx_spin
            ty = ty_trans + ty_spin


            h, th, kn = solve_leg_ik_3dof(tx, ty, tz, g["urdf_x"])
            if h is None:
                continue

            is_left = leg.endswith("L")
            is_right = leg.endswith("R")

            actual_th = -th if is_left else th
            actual_kn = -kn if is_left else kn
            
            if is_right:
                actual_th = th


            p.setJointMotorControl2(
                robot_id, joint_map[f"{leg}3"], p.POSITION_CONTROL, 
                targetPosition=h, force=GLOBAL_MAX_TORQUE, maxVelocity=GLOBAL_MAX_VELOCITY
            )
            p.setJointMotorControl2(
                robot_id, joint_map[f"{leg}2"], p.POSITION_CONTROL, 
                targetPosition=actual_th, force=GLOBAL_MAX_TORQUE, maxVelocity=GLOBAL_MAX_VELOCITY
            )
            p.setJointMotorControl2(
                robot_id, joint_map[f"{leg}1"], p.POSITION_CONTROL, 
                targetPosition=actual_kn, force=GLOBAL_MAX_TORQUE, maxVelocity=GLOBAL_MAX_VELOCITY
            )

        p.stepSimulation()
        physics_step += 1
        time.sleep(DT)

except KeyboardInterrupt:
    p.disconnect()
