import sys
import time
import serial
import serial.tools.list_ports
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

BAUD = 115200
HISTORY = 500 
UPDATE_INTERVAL = 25
PORT = None

if len(sys.argv) > 1:
    PORT = sys.argv[1]
else:
    ports = [p for p in serial.tools.list_ports.comports() if any(k in p.description for k in ("USB", "Teensy", "Serial"))]
    if ports:
        PORT = ports[0].device
    else:
        print("No serial port found")
        sys.exit(1)

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.5)
ser.reset_input_buffer()

pitch = deque(maxlen=HISTORY)
roll  = deque(maxlen=HISTORY)
yaw   = deque(maxlen=HISTORY)
gp    = deque(maxlen=HISTORY)
gr    = deque(maxlen=HISTORY)
gy    = deque(maxlen=HISTORY)
ax    = deque(maxlen=HISTORY)
ay    = deque(maxlen=HISTORY)
az    = deque(maxlen=HISTORY)
cal   = [0, 0, 0, 0] 
sample_timestamps = deque(maxlen=100)

fig = plt.figure(figsize=(13, 9))
fig.canvas.manager.set_window_title("BNO-055 IMU Dashboard")

gs = gridspec.GridSpec(4, 1, height_ratios=[3, 3, 3, 1], hspace=0.35)

ax_euler = fig.add_subplot(gs[0])
ax_gyro  = fig.add_subplot(gs[1])
ax_accel = fig.add_subplot(gs[2])
ax_cal   = fig.add_subplot(gs[3])

COLORS = {"r": "#e74c3c", "g": "#2ecc71", "b": "#3498db"}

# Orientation
ax_euler.set_title("Orientation", fontsize=11, fontweight="bold", loc="left")
ax_euler.set_ylabel("Degrees")
ln_pitch, = ax_euler.plot([], [], label="Pitch", color=COLORS["r"], linewidth=1.2)
ln_roll,  = ax_euler.plot([], [], label="Roll",  color=COLORS["g"], linewidth=1.2)
ln_yaw,   = ax_euler.plot([], [], label="Yaw",   color=COLORS["b"], linewidth=1.2)
ax_euler.legend(loc="upper left", fontsize=8)
ax_euler.set_xlim(0, HISTORY)
ax_euler.grid(True, alpha=0.2)
euler_readout = ax_euler.text(0.99, 0.95, "", transform=ax_euler.transAxes,
                               ha="right", va="top", fontsize=9, fontfamily="monospace")

# Gyroscope
ax_gyro.set_title("Gyroscope", fontsize=11, fontweight="bold", loc="left")
ax_gyro.set_ylabel("deg/s")
ln_gp, = ax_gyro.plot([], [], label="Pitch rate", color=COLORS["r"], linewidth=1.2)
ln_gr, = ax_gyro.plot([], [], label="Roll rate",  color=COLORS["g"], linewidth=1.2)
ln_gy, = ax_gyro.plot([], [], label="Yaw rate",   color=COLORS["b"], linewidth=1.2)
ax_gyro.legend(loc="upper left", fontsize=8)
ax_gyro.set_xlim(0, HISTORY)
ax_gyro.grid(True, alpha=0.2)
gyro_readout = ax_gyro.text(0.99, 0.95, "", transform=ax_gyro.transAxes,
                             ha="right", va="top", fontsize=9, fontfamily="monospace")

# Linear Acceleration
ax_accel.set_title("Linear Acceleration", fontsize=11, fontweight="bold", loc="left")
ax_accel.set_ylabel("m/s\u00b2")
ax_accel.set_xlabel("Sample")
ln_ax, = ax_accel.plot([], [], label="X", color=COLORS["r"], linewidth=1.2)
ln_ay, = ax_accel.plot([], [], label="Y", color=COLORS["g"], linewidth=1.2)
ln_az, = ax_accel.plot([], [], label="Z", color=COLORS["b"], linewidth=1.2)
ax_accel.legend(loc="upper left", fontsize=8)
ax_accel.set_xlim(0, HISTORY)
ax_accel.grid(True, alpha=0.2)
accel_readout = ax_accel.text(0.99, 0.95, "", transform=ax_accel.transAxes,
                               ha="right", va="top", fontsize=9, fontfamily="monospace")

# Sensor Calibration
ax_cal.set_xlim(0, 4)
ax_cal.set_ylim(0, 1)
ax_cal.set_xticks([])
ax_cal.set_yticks([])
ax_cal.set_frame_on(False)
ax_cal.set_title("Sensor Calibration", fontsize=11, fontweight="bold", loc="left")

CAL_NAMES = ["System", "Gyro", "Accel", "Mag"]
CAL_COLORS = {0: "#e74c3c", 1: "#e67e22", 2: "#f1c40f", 3: "#2ecc71"}
CAL_STATUS_LABELS = {
    "System": {0: "Not ready", 1: "Initializing", 2: "Partial fusion", 3: "Fully fused"},
    "Gyro":   {0: "Uncalibrated", 1: "Low accuracy", 2: "Medium accuracy", 3: "Calibrated"},
    "Accel":  {0: "Uncalibrated", 1: "Low accuracy", 2: "Medium accuracy", 3: "Calibrated"},
    "Mag":    {0: "Uncalibrated", 1: "Low accuracy", 2: "Medium accuracy", 3: "Calibrated"},
}
CAL_ACTIONS = {
    "System": {0: "Calibrate all sensors first", 1: "Keep calibrating sensors", 2: "Almost there", 3: ""},
    "Gyro":   {0: "Place sensor flat and hold still", 1: "Keep holding still", 2: "Hold still a bit longer", 3: ""},
    "Accel":  {0: "Place sensor on each of its 6 faces", 1: "Continue rotating to each face", 2: "One more orientation", 3: ""},
    "Mag":    {0: "Move sensor in a figure-8 pattern", 1: "Keep moving in figure-8", 2: "Almost calibrated", 3: ""},
}

cal_name_texts = []
cal_status_texts = []
cal_action_texts = []

for i in range(4):
    x = i + 0.5
    name_t = ax_cal.text(x, 0.85, CAL_NAMES[i], ha="center", va="top",
                         fontsize=10, fontweight="bold")
    status_t = ax_cal.text(x, 0.50, "", ha="center", va="center",
                           fontsize=9, fontfamily="monospace", fontweight="bold")
    action_t = ax_cal.text(x, 0.15, "", ha="center", va="bottom",
                           fontsize=7, color="#666666", style="italic",
                           wrap=True)
    cal_name_texts.append(name_t)
    cal_status_texts.append(status_t)
    cal_action_texts.append(action_t)

status_text = fig.text(0.01, 0.01, "", fontsize=9, fontfamily="monospace", color="#666666")
keys_text = fig.text(0.99, 0.01, "Z = zero   Q = quit", fontsize=9,
                     fontfamily="monospace", color="#999999", ha="right")

plt.subplots_adjust(left=0.08, right=0.97, top=0.96, bottom=0.06)

def autoscale(axis, *buffers):
    all_vals = []
    for b in buffers:
        all_vals.extend(b)
    if not all_vals:
        return
    lo, hi = min(all_vals), max(all_vals)
    span = hi - lo
    margin = max(0.5, span * 0.15)
    axis.set_ylim(lo - margin, hi + margin)

def on_key(event):
    if event.key == 'z':
        try:
            ser.write(b'z')
        except Exception:
            pass
    elif event.key == 'q':
        plt.close(fig)

fig.canvas.mpl_connect('key_press_event', on_key)

def update(frame):
    global ser

    try:
        lines_read = 0
        while ser.in_waiting and lines_read < 80:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            lines_read += 1

            if not line.startswith("D,"):
                if line:
                    print(f"  [{line}]")
                continue

            parts = line[2:].split(",")
            if len(parts) != 13:
                continue
            try:
                vals = [float(v) for v in parts]
            except ValueError:
                continue

            pitch.append(vals[0])
            roll.append(vals[1])
            yaw.append(vals[2])
            gp.append(vals[3])
            gr.append(vals[4])
            gy.append(vals[5])
            ax.append(vals[6])
            ay.append(vals[7])
            az.append(vals[8])
            cal[:] = [int(vals[9]), int(vals[10]), int(vals[11]), int(vals[12])]
            sample_timestamps.append(time.monotonic())

    except (serial.SerialException, OSError):
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(1)
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.05)
            print("  [Reconnected]")
        except Exception:
            pass
        return

    if not pitch:
        return

    x = list(range(len(pitch)))

    ln_pitch.set_data(x, list(pitch))
    ln_roll.set_data(x, list(roll))
    ln_yaw.set_data(x, list(yaw))
    autoscale(ax_euler, pitch, roll, yaw)
    euler_readout.set_text(f"P:{pitch[-1]:+6.1f}  R:{roll[-1]:+6.1f}  Y:{yaw[-1]:+6.1f}")

    gp_deg = [v * 57.2958 for v in gp]
    gr_deg = [v * 57.2958 for v in gr]
    gy_deg = [v * 57.2958 for v in gy]
    ln_gp.set_data(x, gp_deg)
    ln_gr.set_data(x, gr_deg)
    ln_gy.set_data(x, gy_deg)
    autoscale(ax_gyro, gp_deg, gr_deg, gy_deg)
    gyro_readout.set_text(f"P:{gp_deg[-1]:+6.1f}  R:{gr_deg[-1]:+6.1f}  Y:{gy_deg[-1]:+6.1f}")

    ln_ax.set_data(x, list(ax))
    ln_ay.set_data(x, list(ay))
    ln_az.set_data(x, list(az))
    autoscale(ax_accel, ax, ay, az)
    accel_readout.set_text(f"X:{ax[-1]:+5.2f}  Y:{ay[-1]:+5.2f}  Z:{az[-1]:+5.2f}")

    for i in range(4):
        level = cal[i]
        name = CAL_NAMES[i]
        color = CAL_COLORS[level]
        cal_name_texts[i].set_color(color)
        cal_status_texts[i].set_text(CAL_STATUS_LABELS[name][level])
        cal_status_texts[i].set_color(color)
        cal_action_texts[i].set_text(CAL_ACTIONS[name][level])

    if len(sample_timestamps) >= 2:
        dt = sample_timestamps[-1] - sample_timestamps[0]
        if dt > 0:
            hz = (len(sample_timestamps) - 1) / dt
            status_text.set_text(f"{hz:.0f} Hz  |  {len(pitch)} samples")
        else:
            status_text.set_text(f"-- Hz  |  {len(pitch)} samples")
    else:
        status_text.set_text("Waiting for data...")


ani = animation.FuncAnimation(fig, update, interval=UPDATE_INTERVAL,
                              blit=False, cache_frame_data=False)
plt.show()
ser.close()
