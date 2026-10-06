import time
from controller import Controller

motor = Controller()

# ===== Robot constants =====
DT = 0.03                      #33hz
MAX_POWER = 127
STALL_POWER = 27
TICKS_PER_MM = 1400 / 93.0    # 14 ticks per mm
WHEEL_BASE_MM = 119          # wheel separation
HALF_BASE_MM = WHEEL_BASE_MM / 2.0

# ===== PI + D gains =====
KI = 0.5        # integrator gain
KD = 0.0007       # derivative gain (motor acceleration damping)

KSYNC_DIST = 0.5
errL=0
errR=0
def velocity_step(target_left_mm_s, target_right_mm_s,
                  last_vL, last_vR, iL, iR,
                  progress_L, progress_R):
    # Convert mm/s → ticks/s
    target_left_ticks = target_left_mm_s * TICKS_PER_MM
    target_right_ticks = target_right_mm_s * TICKS_PER_MM

    # --- Encoder measurement ---
    dL, dR = motor.get_encoder_ticks()
    vL = dL / DT
    vR = dR / DT

    # --- Velocity error ---
    errL = target_left_ticks - vL
    errR = target_right_ticks - vR

    # --- Integrator ---
    iL += KI * errL * DT
    iR += KI * errR * DT
    iL = max(min(iL, 180), -80)
    iR = max(min(iR, 180), -80)
    # --- Bias curve ---
    baseL = (27 + 0.3333 * abs(target_left_mm_s))*(target_left_mm_s/abs(target_left_mm_s))
    baseR = (27 + 0.3333 * abs(target_right_mm_s))*(target_right_mm_s/abs(target_right_mm_s))

    # --- Derivative ---
    accelL = (vL - last_vL) / DT
    accelR = (vR - last_vR) / DT

    dTermL = KD * accelL
    dTermR = KD * accelR

    # --- Distance sync (use progress only, no target mutation) ---
    dist_error = (progress_L-progress_R)
    sync = KSYNC_DIST * dist_error

    uL = baseL + iL - dTermL - sync #* baseL/abs(baseL)
    uR = baseR + iR - dTermR - sync #* baseR/abs(baseR)

    uL = max(min(uL, MAX_POWER), -MAX_POWER)
    uR = max(min(uR, MAX_POWER), -MAX_POWER)

    #if abs(uL) < STALL_POWER: uL = 0
   # if abs(uR) < STALL_POWER: uR = 0

    motor.set_raw_motor_speed(int(uL), int(uR))

    return vL, vR, iL, iR
def drive_distance_mm(distance_mm, base_speed_mm_s=100):
    # Initial remaining distance in ticks
    remaining_ticks = distance_mm * TICKS_PER_MM

    # Relative encoder reference
    rel = motor.new_relative()

    # Velocity controller state
    last_vL = 0.0
    last_vR = 0.0
    iL = 0.0
    iR = 0.0

    print("\n=== DISTANCE CONTROL START ===\n")
    progress_L =0
    progress_R =0
    while True:
        time.sleep(DT)

        # --- Encoder measurement ---
        dL, dR = motor.get_relative_encoder_ticks(rel)

        # --- YOUR INCREMENTOR (correct) ---
        progress_ticks = (dL + dR) / 2.0
        remaining_ticks -= progress_ticks

        # --- Stop condition ---
        if abs(remaining_ticks) <= 8:
            motor.set_raw_motor_speed(0, 0)
            print("\n=== TARGET REACHED ===\n")
            break

        # Convert remaining to mm
        remaining_mm = remaining_ticks / TICKS_PER_MM

        # --- SIGN ONLY (no slowdown, no crawl) ---
        cmd_speed = base_speed_mm_s * (remaining_mm / max(abs(remaining_mm), base_speed_mm_s))

        # --- Distance sync uses raw wheel progress ---
        progress_L = dL + progress_L
        progress_R = dR + progress_R

        # --- Run velocity controller once ---
        last_vL, last_vR, iL, iR = velocity_step(
            cmd_speed, cmd_speed,
            last_vL, last_vR,
            iL, iR,
            progress_L, progress_R
        )

        print(
            f"REMAINING: {remaining_mm:.1f} mm\n"
            f"CMD SPEED: {cmd_speed:.1f} mm/s\n"
            f"progress: L={progress_L/TICKS_PER_MM:.1f} R={progress_R/TICKS_PER_MM:.1f}\n"
        )
def drive_angle_deg(angle_deg, base_speed_mm_s=100):
    # Convert degrees → radians
    theta = angle_deg * 3.1415926535 / 180.0

    # Wheel travel distances in mm
    dL_mm = -HALF_BASE_MM * theta
    dR_mm = +HALF_BASE_MM * theta

    # Convert mm → ticks
    remaining_L = dL_mm * TICKS_PER_MM
    remaining_R = dR_mm * TICKS_PER_MM

    # Relative encoder reference
    rel = motor.new_relative()

    # Velocity controller state
    last_vL = 0.0
    last_vR = 0.0
    iL = 0.0
    iR = 0.0

    print("\n=== ANGLE CONTROL START ===\n")

    # Progress accumulators (your working pattern)
    progress_L = 0
    progress_R = 0

    while True:
        time.sleep(DT)

        # --- Encoder measurement ---
        dL, dR = motor.get_relative_encoder_ticks(rel)

        # --- YOUR INCREMENTOR (correct) ---
        remaining_L -= dL
        remaining_R -= dR

        # --- Stop condition ---
        if abs(remaining_L) <= 4 and abs(remaining_R) <= 4:
            motor.set_raw_motor_speed(0, 0)
            print("\n=== TARGET ANGLE REACHED ===\n")
            break

        # --- SIGN ONLY (no slowdown, no crawl) ---
        # Geometry signs ONLY
        cmd_L = base_speed_mm_s * (remaining_L / max(abs(remaining_L), base_speed_mm_s*2))
        cmd_R = base_speed_mm_s * (remaining_R / max(abs(remaining_R), base_speed_mm_s*2))

        # --- Distance sync uses accumulated progress ---
        progress_L = dL + progress_L
        progress_R = dR + progress_R

        # --- Run velocity controller once ---
        last_vL, last_vR, iL, iR = velocity_step(
            cmd_L, cmd_R,
            last_vL, last_vR,
            iL, iR,
            progress_L/2, -progress_R/2
        )

        print(
            f"REMAIN L: {remaining_L/TICKS_PER_MM:.1f} mm\n"
            f"REMAIN R: {remaining_R/TICKS_PER_MM:.1f} mm\n"
            f"CMD L: {cmd_L:.1f} mm/s   CMD R: {cmd_R:.1f} mm/s\n"
            f"VEL: L={last_vL:.1f} R={last_vR:.1f}\n"
        )
# Example tests
drive_distance_mm(285,250)
drive_angle_deg(-180,200)
#drive_distance_mm(100,200)
#drive_angle_deg(-180,250)
#drive_distance_mm(285*1)