"""
Generic telemetry dashboard for any teensy demo app.

Usage:  python dashboard.py [COM_PORT]

The firmware describes itself: it sends a one-time header line
  H,<chan>,<chan>,...
followed by data rows
  D,<val>,<val>,...
This tool reads the header and builds plots automatically — no per-app code.

Channel names use a "group.series" convention: channels that share a group
(text before the '.') are drawn on the same axis as separate lines; a name with
no '.' gets its own subplot. Any other serial line is printed to the console.

Keys: any single key is forwarded to the firmware (e.g. z=zero, 1-4=faults,
n=clear), q quits.
"""

import sys
import time
import math
from collections import deque, OrderedDict

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation

BAUD = 115200
HISTORY = 500
UPDATE_MS = 25
PALETTE = ["#e74c3c", "#2ecc71", "#3498db", "#e67e22", "#9b59b6", "#1abc9c"]


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = [p for p in serial.tools.list_ports.comports()
             if any(k in p.description for k in ("USB", "Teensy", "Serial"))]
    if ports:
        print(f"Auto-detected: {ports[0].device}")
        return ports[0].device
    print("No serial port found. Usage: python dashboard.py COM3")
    sys.exit(1)


def open_serial(port):
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()
    return ser


def read_header(ser):
    """Block (echoing other lines) until an 'H,...' header arrives; return channel names."""
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("H,"):
            return line[2:].split(",")
        elif line:
            print(f"  [{line}]")


def autoscale(ax, *series):
    vals = [v for s in series for v in s]
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    margin = max(0.3, (hi - lo) * 0.15)
    ax.set_ylim(lo - margin, hi + margin)


def build_figure(channels):
    """Group channels by prefix before '.', one subplot per group."""
    groups = OrderedDict()
    for ch in channels:
        group = ch.split(".", 1)[0] if "." in ch else ch
        groups.setdefault(group, []).append(ch)

    n = len(groups)
    cols = 1 if n <= 1 else (2 if n <= 8 else 3)
    rows = math.ceil(n / cols)

    fig = plt.figure(figsize=(7 * cols, 2.4 * rows))
    fig.canvas.manager.set_window_title("Telemetry Dashboard")

    buf = {ch: deque(maxlen=HISTORY) for ch in channels}
    lines = {}
    axes = []
    for i, (group, chans) in enumerate(groups.items()):
        ax = fig.add_subplot(rows, cols, i + 1)
        ax.set_title(group, fontsize=10, fontweight="bold", loc="left")
        ax.set_xlim(0, HISTORY)
        ax.grid(True, alpha=0.2)
        ax.tick_params(labelsize=7)
        for j, ch in enumerate(chans):
            label = ch.split(".", 1)[1] if "." in ch else ch
            (lines[ch],) = ax.plot([], [], lw=1.3, color=PALETTE[j % len(PALETTE)],
                                    label=label)
        if len(chans) > 1 or "." in chans[0]:
            ax.legend(loc="upper left", fontsize=7, ncol=2)
        axes.append((ax, chans))

    rate_text = fig.text(0.99, 0.01, "", ha="right", fontsize=9,
                         fontfamily="monospace", color="#666666")
    fig.text(0.01, 0.01, "keys forwarded to firmware  |  q = quit",
             fontsize=8, fontfamily="monospace", color="#999999")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    return fig, buf, lines, axes, rate_text


def main():
    port = find_port()
    ser = open_serial(port)

    print("Waiting for telemetry header...")
    channels = read_header(ser)
    print(f"Channels: {', '.join(channels)}")

    fig, buf, lines, axes, rate_text = build_figure(channels)
    stamps = deque(maxlen=100)

    def on_key(event):
        if event.key == "q":
            plt.close(fig)
        elif event.key and len(event.key) == 1:
            try:
                ser.write(event.key.encode())
            except Exception:
                pass

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        nonlocal ser
        try:
            n = 0
            while ser.in_waiting and n < 200:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                n += 1
                if not line.startswith("D,"):
                    if line and not line.startswith("H,"):
                        print(f"  [{line}]")
                    continue
                parts = line[2:].split(",")
                if len(parts) != len(channels):
                    continue
                try:
                    vals = [float(x) for x in parts]
                except ValueError:
                    continue
                for ch, v in zip(channels, vals):
                    buf[ch].append(v)
                stamps.append(time.monotonic())
        except (serial.SerialException, OSError):
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1)
            try:
                ser = open_serial(port)
                print("  [Reconnected]")
            except Exception:
                pass
            return

        for ax, chans in axes:
            for ch in chans:
                x = range(len(buf[ch]))
                lines[ch].set_data(x, list(buf[ch]))
            autoscale(ax, *(buf[ch] for ch in chans))

        if len(stamps) >= 2:
            dt = stamps[-1] - stamps[0]
            if dt > 0:
                rate_text.set_text(f"{(len(stamps) - 1) / dt:.0f} Hz")

    ani = animation.FuncAnimation(fig, update, interval=UPDATE_MS,
                                  blit=False, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == "__main__":
    main()
