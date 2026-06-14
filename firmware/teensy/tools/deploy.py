"""
One-command deploy: build + upload a firmware env, then run its dashboard.

Usage:  python tools/deploy.py <env>

  <env> is any environment from platformio.ini (main, ekf_demo, imu_demo,
  hip_demo, balance_demo, ...). After upload the generic tools/dashboard.py is
  launched: it auto-builds plots from the firmware's telemetry header, or just
  echoes raw serial for apps that don't stream telemetry.

Board and serial port come from platformio_local.ini (see the .example file).
"""

import sys
import configparser
import subprocess
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent   # firmware/teensy/
LOCAL_INI = PROJECT / "platformio_local.ini"
EXAMPLE_INI = PROJECT / "platformio_local.ini.example"


def read_port():
    """Return monitor_port (or upload_port) from platformio_local.ini, else None."""
    if not LOCAL_INI.exists():
        return None
    cfg = configparser.ConfigParser()
    cfg.read(LOCAL_INI)
    if not cfg.has_section("env"):
        return None
    return cfg.get("env", "monitor_port", fallback=cfg.get("env", "upload_port", fallback=None))


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    env = sys.argv[1]

    if not LOCAL_INI.exists():
        if EXAMPLE_INI.exists():
            LOCAL_INI.write_text(EXAMPLE_INI.read_text())
            print(f"Created {LOCAL_INI.name} from the example.")
        print(f"Edit {LOCAL_INI.name} for your board and serial port, then re-run.")
        sys.exit(1)

    # Build + upload
    upload = subprocess.run(["pio", "run", "-e", env, "-t", "upload"], cwd=PROJECT)
    if upload.returncode != 0:
        sys.exit(upload.returncode)

    # Generic dashboard: auto-builds from the firmware's telemetry header, and
    # falls back to echoing raw serial for apps that don't stream telemetry.
    port = read_port()
    cmd = [sys.executable, str(PROJECT / "tools" / "dashboard.py")] + ([port] if port else [])
    subprocess.run(cmd, cwd=PROJECT)


if __name__ == "__main__":
    main()
