import math
from controller import Robot, Keyboard

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ── Motors ──
rw_motor = robot.getDevice('RW Motor')
lw_motor = robot.getDevice('LW Motor')
rw_motor.setPosition(float('inf'))
lw_motor.setPosition(float('inf'))
rw_motor.setVelocity(0.0)
lw_motor.setVelocity(0.0)

# ── Sensors ──
imu = robot.getDevice('imu')
imu.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)

# ── Keyboard ──
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# ── PD gains ──
Kp = 6.0
Kd = 1.0

# ── Steering ──
FORWARD_LEAN = 5.0 / 57.0
TURN_TORQUE = 1.0
PITCH_OFFSET = 16.7 / 57.0
MAX_TORQUE = 2.0

# ── Logging ──
LOG_INTERVAL = 20  
step_count = 0

HEADER = (
    f"{'time':>8} │"
    f"{'pitch':>8} {'rate':>8} │"
    f"{'target':>8} {'error':>8} │"
    f"{'torque':>8} {'L_trq':>8} {'R_trq':>8} │"
    f"{'steer':>6}"
)
SEPARATOR = "─" * len(HEADER)

print("PD balance controller active. WASD to steer.")
print(SEPARATOR)
print(HEADER)
print(SEPARATOR)

while robot.step(timestep) != -1:
    sim_time = robot.getTime()

    # ── Read sensors ──
    pitch = imu.getRollPitchYaw()[1]
    pitch_rate = gyro.getValues()[1]

    # ── Keyboard ──
    target_pitch = PITCH_OFFSET
    turn = 0.0
    steer_label = "  ──"

    key = keyboard.getKey()
    while key != -1:
        if key == ord('W'):
            target_pitch = PITCH_OFFSET + FORWARD_LEAN
            steer_label = "  FW"
        elif key == ord('S'):
            target_pitch = PITCH_OFFSET - FORWARD_LEAN
            steer_label = "  BK"
        elif key == ord('A'):
            turn = -TURN_TORQUE
            steer_label = "  ←"
        elif key == ord('D'):
            turn = TURN_TORQUE
            steer_label = "  →"
        key = keyboard.getKey()

    # ── PD control ──
    error = pitch - target_pitch
    torque = (Kp * error + Kd * pitch_rate)
    torque = max(-MAX_TORQUE, min(MAX_TORQUE, torque))

    left_torque = torque + turn
    right_torque = torque - turn

    # ── Apply ──
    lw_motor.setTorque(left_torque)
    rw_motor.setTorque(right_torque)

    # ── Log ──
    step_count += 1
    if step_count % LOG_INTERVAL == 0:
        pitch_deg = math.degrees(pitch)
        rate_deg = math.degrees(pitch_rate)
        target_deg = math.degrees(target_pitch)
        error_deg = math.degrees(error)

        print(
            f"{sim_time:8.2f} │"
            f"{pitch_deg:+8.2f} {rate_deg:+8.2f} │"
            f"{target_deg:+8.2f} {error_deg:+8.2f} │"
            f"{torque:+8.3f} {left_torque:+8.3f} {right_torque:+8.3f} │"
            f"{steer_label:>6}"
        )

        # Reprint header periodically
        if step_count % (LOG_INTERVAL * 20) == 0:
            print(SEPARATOR)
            print(HEADER)
            print(SEPARATOR)