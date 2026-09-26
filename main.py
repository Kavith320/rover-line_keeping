#!/usr/bin/env python3
"""
Autonomous Agricultural Rover - Master Production Controller & Web Platform
=============================================================================
Unified, single-file production entry point optimized for Raspberry Pi (3B / 4B / Zero 2W),
ASUS Tinker Board, and embedded SBC field deployments.

Key Capabilities:
  - Failsafe Safety: Motors remain strictly stopped & disabled on startup until
    explicitly enabled via the "RUN TRACKING" web interface button.
  - Serial Protocol: Transmits `<TRACKING_DISABLED>` and `<0,0>` upon initialization / stop,
    and `<TRACKING_ENABLED>` when tracking starts.
  - High-Accuracy Line Tracking: Renders exact plant-aligned crop rows, drivable navigation
    corridor cone, center guidance track, and steering error vectors with 0% coordinate offset
    across any camera resolution (320x240, 480x360, 640x480, 1280x720).
  - Headless Operation: Fully decoupled from desktop GUI / X11 / Tkinter dependencies,
    running smoothly over Wi-Fi / SSH systemd service on Raspberry Pi.
  - Automatic Platform Optimization: Tailors downscaled CV processing (320x240 @ 18-20 FPS)
    to keep ARM Cortex-A53 CPU usage < 45% and prevent thermal throttling.

Usage:
  python3 main.py
  python3 main.py --video data/good.mp4
  python3 main.py --profile rpi3b --port 5001
  python3 main.py --serial-port /dev/ttyUSB0 --baudrate 115200
"""

import os
import sys
import time
import socket
import argparse
import threading

import web_rover_dashboard as wrd
import sbc_optimizer as sbc
import serial_motor_controller as smc
import video_device_manager as vdm


def is_port_in_use(port, host='127.0.0.1'):
    """Checks if a local TCP network port is currently occupied."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex((host, port)) == 0


def print_banner(host_ip, port, profile_name, serial_port, baudrate, current_video):
    """Displays a clean ASCII welcome banner in the terminal."""
    border = "=" * 68
    print("\n" + border)
    print("  🌾  AUTONOMOUS AGRICULTURAL ROVER - PRODUCTION MASTER CONTROLLER  🌾")
    print(border)
    print(f"  📱 Mobile / Tablet Cockpit : http://{host_ip}:{port}")
    print(f"  💻 Local Web Dashboard     : http://localhost:{port}")
    print("-" * 68)
    print(f"  ⚡ SBC Hardware Profile    : {profile_name}")
    print(f"  🔌 Serial Motor Driver     : {serial_port} @ {baudrate} baud")
    print(f"  📷 Active Vision Source    : {os.path.basename(str(current_video))}")
    print(f"  🛡️ Startup Safety State    : MOTORS DISABLED (Press RUN in Web UI)")
    print(border)
    print("  Press Ctrl+C in terminal to shut down.\n")


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Agricultural Rover - Master Controller (Raspberry Pi & SBC)"
    )
    parser.add_argument(
        "--video", "--camera", type=str, default=None,
        help="Initial video file or camera device node (e.g. 0, /dev/video0, or data/good.mp4)"
    )
    parser.add_argument(
        "--profile", type=str, default=None,
        choices=["rpi3b", "rpi4", "tinker_board", "balanced", "desktop"],
        help="SBC compute optimization profile (default: auto-detected or rpi3b)"
    )
    parser.add_argument(
        "--serial-port", type=str, default=None,
        help="Serial port connecting to motor driver (e.g. /dev/ttyUSB0, /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baudrate", type=int, default=115200,
        help="Serial link baudrate (default: 115200)"
    )
    parser.add_argument(
        "--auto-connect", action="store_true", default=None,
        help="Automatically scan and connect to active serial motor driver on startup"
    )
    parser.add_argument(
        "--port", type=int, default=5001,
        help="Web server HTTP listening port (default: 5001)"
    )
    parser.add_argument(
        "--host", type=str, default="0.0.0.0",
        help="Web server network interface binding (default: 0.0.0.0)"
    )
    args = parser.parse_args()

    # Determine default SBC optimization profile
    if args.profile:
        chosen_profile = args.profile
    else:
        chosen_profile = sbc.detect_sbc_profile()

    # Handle automatic port fallback if port is already bound
    target_port = args.port
    if is_port_in_use(target_port):
        for candidate in [5001, 5002, 5000, 8080, 8081, 8888]:
            if not is_port_in_use(candidate):
                print(f"[INFO] Port {target_port} was busy. Switched to port {candidate}.")
                target_port = candidate
                break

    if args.auto_connect is not None:
        smc.get_motor_controller().cfg["auto_connect"] = args.auto_connect

    # Initialize Master Rover Vision & Hardware Engine
    wrd.engine = wrd.RoverVisionEngine(
        initial_video=args.video,
        sbc_profile=chosen_profile,
        serial_port=args.serial_port,
        baudrate=args.baudrate
    )

    # Launch background vision pipeline worker thread
    worker = threading.Thread(target=wrd.engine.run_worker, daemon=True)
    worker.start()

    local_ip = wrd.get_local_ip()
    print_banner(
        host_ip=local_ip,
        port=target_port,
        profile_name=wrd.engine.sbc.profile["name"],
        serial_port=wrd.engine.motor_controller.cfg["port"],
        baudrate=wrd.engine.motor_controller.cfg["baudrate"],
        current_video=wrd.engine.current_source
    )

    # Launch Flask HTTP Server
    wrd.app.run(host=args.host, port=target_port, threaded=True, debug=False)


if __name__ == "__main__":
    main()
