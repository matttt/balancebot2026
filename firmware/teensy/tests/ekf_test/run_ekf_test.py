"""
EKF Dashboard — full state + sensor-health visualization.

Usage:  python run_ekf_test.py [COM_PORT]
Keys:
  Z = zero pitch
  1 = IMU pitch fault
  2 = wheel fault
  3 = L/R wheel divergence
  4 = IMU stall (freeze raw reads)
  N = clear faults
  Q = quit
"""

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
UPDATE_MS = 25
NFIELDS = 28

# Serial
PORT = None
if len(sys.argv) > 1:
    PORT = sys.argv[1]
else:
    ports = [p for p in serial.tools.list_ports.comports()
             if any(k in p.description for k in ("USB", "Teensy", "Serial"))]
    if ports:
        PORT = ports[0].device
        print(f"Auto-detected: {PORT}")
    else:
        print("No serial port found. Usage: python run_ekf_test.py COM3")
        sys.exit(1)

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.5)
ser.reset_input_buffer()

# Numeric (time-series) channels
NUM_IDX = list(range(14)) + [26, 27]   # 0..13 plus accel_imu, accel_wheel
buf = {i: deque(maxlen=HISTORY) for i in NUM_IDX}

state = {
    "accepted": 1,
    "fault_imu": 0, "fault_wheel": 0,
    "cal": [0]*4,
    "imu_stale": 0, "wheel_diverged": 0, "xval_bad": 0, "fallen": 0,
    "imu_fault_mask": 0,
}
timestamps = deque(maxlen=100)

# Layout
fig = plt.figure(figsize=(14, 10.5))
fig.canvas.manager.set_window_title("EKF Dashboard")
gs = gridspec.GridSpec(7, 2, height_ratios=[3, 3, 3, 3, 3, 1, 1], hspace=0.45, wspace=0.25)

C_raw  = "#bbbbbb"
C_ekf  = "#e74c3c"
C_sim  = "#bbbbbb"
C_ekf2 = "#2ecc71"
C_ekf3 = "#3498db"
C_diag1 = "#e67e22"
C_diag2 = "#3498db"


def make_plot(gs_pos, title, ylabel):
    ax = fig.add_subplot(gs_pos)
    ax.set_title(title, fontsize=10, fontweight="bold", loc="left")
    ax.set_ylabel(ylabel, fontsize=8)
    ax.set_xlim(0, HISTORY)
    ax.grid(True, alpha=0.2)
    ax.tick_params(labelsize=7)
    readout = ax.text(0.99, 0.95, "", transform=ax.transAxes,
                      ha="right", va="top", fontsize=8, fontfamily="monospace")
    return ax, readout


# Row 0: Pitch / Pitch rate
ax_p, rd_p = make_plot(gs[0, 0], "Pitch", "deg")
ln_raw_p, = ax_p.plot([], [], color=C_raw, lw=1.0, label="Raw")
ln_ekf_p, = ax_p.plot([], [], color=C_ekf, lw=1.5, label="EKF")
ax_p.legend(loc="upper left", fontsize=7)

ax_pr, rd_pr = make_plot(gs[0, 1], "Pitch Rate", "deg/s")
ln_raw_pr, = ax_pr.plot([], [], color=C_raw, lw=1.0, label="Raw")
ln_ekf_pr, = ax_pr.plot([], [], color=C_ekf, lw=1.5, label="EKF")
ax_pr.legend(loc="upper left", fontsize=7)

# Row 1: Yaw rate / Forward velocity
ax_yr, rd_yr = make_plot(gs[1, 0], "Yaw Rate", "rad/s")
ln_raw_yr, = ax_yr.plot([], [], color=C_raw, lw=1.0, label="Raw")
ln_ekf_yr, = ax_yr.plot([], [], color=C_ekf3, lw=1.5, label="EKF")
ax_yr.legend(loc="upper left", fontsize=7)

ax_xv, rd_xv = make_plot(gs[1, 1], "Forward Velocity", "m/s")
ln_xv, = ax_xv.plot([], [], color=C_ekf2, lw=1.5, label="EKF x_vel")
ax_xv.legend(loc="upper left", fontsize=7)

# Row 2: Wheel position / velocity
ax_wp, rd_wp = make_plot(gs[2, 0], "Wheel Position (mean L/R)", "rad")
ln_sim_wp, = ax_wp.plot([], [], color=C_sim, lw=1.0, label="Simulated")
ln_ekf_wp, = ax_wp.plot([], [], color=C_ekf2, lw=1.5, label="EKF")
ax_wp.legend(loc="upper left", fontsize=7)

ax_wv, rd_wv = make_plot(gs[2, 1], "Wheel Velocity (mean L/R)", "rad/s")
ln_sim_wv, = ax_wv.plot([], [], color=C_sim, lw=1.0, label="Simulated")
ln_ekf_wv, = ax_wv.plot([], [], color=C_ekf2, lw=1.5, label="EKF")
ax_wv.legend(loc="upper left", fontsize=7)

# Row 3: Forward position / EKF diagnostics
ax_xp, rd_xp = make_plot(gs[3, 0], "Forward Position", "m")
ln_xp, = ax_xp.plot([], [], color=C_ekf3, lw=1.5, label="EKF x_pos")
ax_xp.legend(loc="upper left", fontsize=7)

ax_dg, rd_dg = make_plot(gs[3, 1], "EKF Diagnostics", "")
ln_pv, = ax_dg.plot([], [], color=C_diag1, lw=1.2, label="Pitch variance")
ln_kp, = ax_dg.plot([], [], color=C_diag2, lw=1.2, label="Kalman gain (pitch)")
ax_dg.legend(loc="upper left", fontsize=7)
ax_dg.set_ylim(-0.05, 1.1)

# Row 4: IMU/wheel accel cross-validation
ax_xa, rd_xa = make_plot(gs[4, :], "Acceleration cross-check (IMU vs wheel-derived)", "m/s²")
ln_ax_imu,   = ax_xa.plot([], [], color=C_diag2, lw=1.2, label="IMU lin_accel_x")
ln_ax_wheel, = ax_xa.plot([], [], color=C_diag1, lw=1.2, label="d(wheel_vel)/dt · r")
ax_xa.legend(loc="upper left", fontsize=7)

# Row 5: state vector
ax_st = fig.add_subplot(gs[5, :])
ax_st.set_xlim(0, 1); ax_st.set_ylim(0, 1)
ax_st.set_xticks([]); ax_st.set_yticks([]); ax_st.set_frame_on(False)
ax_st.set_title("State Vector", fontsize=10, fontweight="bold", loc="left")
state_text = ax_st.text(0.02, 0.5, "", va="center", fontsize=9, fontfamily="monospace")

# Row 6: status bar
ax_sb = fig.add_subplot(gs[6, :])
ax_sb.set_xlim(0, 1); ax_sb.set_ylim(0, 1)
ax_sb.set_xticks([]); ax_sb.set_yticks([]); ax_sb.set_frame_on(False)

health_text = ax_sb.text(0.0, 0.7, "", va="center", fontsize=10,
                         fontweight="bold", fontfamily="monospace")
flags_text  = ax_sb.text(0.0, 0.2, "", va="center", fontsize=9,
                         fontfamily="monospace")
fault_text  = ax_sb.text(0.5, 0.7, "", va="center", ha="center", fontsize=10,
                         fontfamily="monospace")
rate_text   = ax_sb.text(1.0, 0.7, "", va="center", ha="right", fontsize=9,
                         fontfamily="monospace", color="#666666")

fig.text(0.5, 0.005,
         "Z=zero  1=imu_fault  2=wheel_fault  3=L/R_diverge  4=imu_stall  N=clear  Q=quit",
         fontsize=9, fontfamily="monospace", color="#999999", ha="center")

plt.subplots_adjust(left=0.07, right=0.97, top=0.97, bottom=0.04)


def autoscale(ax, *bufs):
    vals = []
    for b in bufs:
        vals.extend(b)
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    m = max(0.3, (hi - lo) * 0.15)
    ax.set_ylim(lo - m, hi + m)


def on_key(event):
    try:
        if   event.key == 'z': ser.write(b'z')
        elif event.key == '1': ser.write(b'1')
        elif event.key == '2': ser.write(b'2')
        elif event.key == '3': ser.write(b'3')
        elif event.key == '4': ser.write(b'4')
        elif event.key == 'n': ser.write(b'n')
        elif event.key == 'q': plt.close(fig)
    except Exception:
        pass

fig.canvas.mpl_connect('key_press_event', on_key)


def update(_frame):
    global ser
    try:
        n = 0
        while ser.in_waiting and n < 80:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            n += 1
            if not line.startswith("D,"):
                if line:
                    print(f"  [{line}]")
                continue
            parts = line[2:].split(",")
            if len(parts) != NFIELDS:
                continue
            try:
                v = [float(x) for x in parts]
            except ValueError:
                continue
            for i in NUM_IDX:
                buf[i].append(v[i])
            state["accepted"]       = int(v[14])
            state["fault_imu"]      = int(v[15])
            state["fault_wheel"]    = int(v[16])
            state["cal"]            = [int(v[17]), int(v[18]), int(v[19]), int(v[20])]
            state["imu_stale"]      = int(v[21])
            state["wheel_diverged"] = int(v[22])
            state["xval_bad"]       = int(v[23])
            state["fallen"]         = int(v[24])
            state["imu_fault_mask"] = int(v[25])
            timestamps.append(time.monotonic())
    except (serial.SerialException, OSError):
        try: ser.close()
        except Exception: pass
        time.sleep(1)
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.05)
            print("  [Reconnected]")
        except Exception: pass
        return

    if not buf[0]:
        return

    x = list(range(len(buf[0])))

    ln_raw_p.set_data(x, list(buf[0]));   ln_ekf_p.set_data(x, list(buf[1]))
    autoscale(ax_p, buf[0], buf[1])
    rd_p.set_text(f"Raw:{buf[0][-1]:+6.1f}  EKF:{buf[1][-1]:+6.1f}")

    pr_raw_deg = [v * 57.2958 for v in buf[2]]
    pr_ekf_deg = [v * 57.2958 for v in buf[3]]
    ln_raw_pr.set_data(x, pr_raw_deg); ln_ekf_pr.set_data(x, pr_ekf_deg)
    autoscale(ax_pr, pr_raw_deg, pr_ekf_deg)
    rd_pr.set_text(f"Raw:{pr_raw_deg[-1]:+6.1f}  EKF:{pr_ekf_deg[-1]:+6.1f}")

    ln_raw_yr.set_data(x, list(buf[4])); ln_ekf_yr.set_data(x, list(buf[5]))
    autoscale(ax_yr, buf[4], buf[5])
    rd_yr.set_text(f"Raw:{buf[4][-1]:+5.2f}  EKF:{buf[5][-1]:+5.2f}")

    ln_xv.set_data(x, list(buf[11])); autoscale(ax_xv, buf[11])
    rd_xv.set_text(f"{buf[11][-1]:+.3f} m/s")

    ln_sim_wp.set_data(x, list(buf[6])); ln_ekf_wp.set_data(x, list(buf[7]))
    autoscale(ax_wp, buf[6], buf[7])
    rd_wp.set_text(f"Sim:{buf[6][-1]:+6.2f}  EKF:{buf[7][-1]:+6.2f}")

    ln_sim_wv.set_data(x, list(buf[8])); ln_ekf_wv.set_data(x, list(buf[9]))
    autoscale(ax_wv, buf[8], buf[9])
    rd_wv.set_text(f"Sim:{buf[8][-1]:+5.2f}  EKF:{buf[9][-1]:+5.2f}")

    ln_xp.set_data(x, list(buf[10])); autoscale(ax_xp, buf[10])
    rd_xp.set_text(f"{buf[10][-1]:+.3f} m")

    x_dg = list(range(len(buf[12])))
    ln_pv.set_data(x_dg, list(buf[12])); ln_kp.set_data(x_dg, list(buf[13]))
    rd_dg.set_text(f"Pvar:{buf[12][-1]:.5f}  Kp:{buf[13][-1]:.4f}")

    x_xa = list(range(len(buf[26])))
    ln_ax_imu.set_data(x_xa, list(buf[26]))
    ln_ax_wheel.set_data(x_xa, list(buf[27]))
    autoscale(ax_xa, buf[26], buf[27])
    rd_xa.set_text(f"IMU:{buf[26][-1]:+5.2f}  Wheel:{buf[27][-1]:+5.2f}")

    state_text.set_text(
        f"pitch={buf[1][-1]:+6.2f}deg  pitch_rate={buf[3][-1]:+5.3f}  "
        f"yaw_rate={buf[5][-1]:+5.3f}  "
        f"wheel_pos={buf[7][-1]:+6.2f}  wheel_vel={buf[9][-1]:+5.2f}  "
        f"x_pos={buf[10][-1]:+6.3f}m  x_vel={buf[11][-1]:+5.3f}m/s"
    )

    if state["accepted"]:
        health_text.set_text("EKF: accepting")
        health_text.set_color("#2ecc71")
    else:
        health_text.set_text("EKF: rejected")
        health_text.set_color("#e74c3c")

    # Latched/derived flags row
    flag_items = []
    if state["fallen"]:         flag_items.append(("FALLEN",         "#e74c3c"))
    if state["imu_stale"]:      flag_items.append(("IMU STALE",      "#e74c3c"))
    if state["wheel_diverged"]: flag_items.append(("L/R DIVERGED",   "#e74c3c"))
    if state["xval_bad"]:       flag_items.append(("ACCEL X-CHECK",  "#e67e22"))
    if state["imu_fault_mask"]: flag_items.append(
        (f"imu_mask=0x{state['imu_fault_mask']:02X}", "#e67e22"))
    if flag_items:
        flags_text.set_text("  |  ".join(s for s, _ in flag_items))
        flags_text.set_color(flag_items[0][1])
    else:
        flags_text.set_text("All sensors nominal")
        flags_text.set_color("#2ecc71")

    faults = []
    if state["fault_imu"]:   faults.append("IMU injected")
    if state["fault_wheel"]: faults.append("WHEEL injected")
    fault_text.set_text("  |  ".join(faults) if faults else "")
    fault_text.set_color("#e74c3c" if faults else "#999999")

    if len(timestamps) >= 2:
        dt = timestamps[-1] - timestamps[0]
        if dt > 0:
            rate_text.set_text(f"{(len(timestamps)-1)/dt:.0f} Hz")


ani = animation.FuncAnimation(fig, update, interval=UPDATE_MS,
                              blit=False, cache_frame_data=False)
plt.show()
ser.close()
