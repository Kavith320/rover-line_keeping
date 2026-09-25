# Autonomous Agricultural Rover Vision Platform

An educational computer vision and robotics simulation platform for autonomous crop row navigation. The platform processes camera streams (both real field footage and synthetic 3D field simulations), isolates crop rows, calculates drivable free space, computes steering error, and displays live differential drive telemetry on a digital twin rover.

---

## Table of Contents
1. [Overview & Features](#overview--features)
2. [Platform Architecture](#platform-architecture)
3. [Installation & Setup](#installation--setup)
4. [How to Run (Master Production App)](#how-to-run-master-production-app)
5. [Startup Motor Safety & Tracking Toggle](#startup-motor-safety--tracking-toggle)
6. [Line Tracking & Drivable Corridor Visualization](#line-tracking--drivable-corridor-visualization)
7. [Interactive Controls & Web Cockpit](#interactive-controls--web-cockpit)
8. [Detection Area (ROI) Calibration](#detection-area-roi-calibration)
9. [PID Steering Optimization](#pid-steering-optimization)
10. [Field Edge & End-of-Row Detection](#field-edge--end-of-row-detection)
11. [Project File Structure](#project-file-structure)
12. [Hardware Integration & Serial Link Guide](#hardware-integration--serial-link-guide)
13. [SBC Optimization (Raspberry Pi 3B / 4B & ASUS Tinker Board)](#sbc-optimization-raspberry-pi-3b--4b--asus-tinker-board)
14. [Microcontroller Firmware (Arduino / ESP32)](#microcontroller-firmware-arduino--esp32)

---

## Overview & Features

- **No AI / Deep Learning Required**: Runs entirely on fast, deterministic classical computer vision using Python, OpenCV, and NumPy.
- **Works on Real & Simulated Footage**: Robust against real-world field videos (including straddle rovers, corn fields, and raised beds) as well as 3D synthetic simulations.
- **Adaptive Lighting & Shadow Invariance**:
  - **CLAHE (Contrast Limited Adaptive Histogram Equalization)**: Local contrast equalization on the $L$ (Luminance) channel in LAB color space prevents blowout from direct overhead sun glare.
  - **Excess Green Index ($\text{ExG} = 2G - R - B$)**: Precision agriculture index invariant to illumination shifts and deep chassis or tree shadows ($V < 30$).
  - **Dynamic HSV Thresholding**: Real-time ambient scene brightness tracking automatically adapts $S_{\text{min}}$ and $V_{\text{min}}$ thresholds.
  - **Live Toggle Hotkey**: Instant comparison using the <kbd>l</kbd> key between Adaptive Mode and Static HSV, with live HUD telemetry.
- **PID Steering Optimization**:
  - Full Proportional-Integral-Derivative control loop with anti-windup clamping.
  - Low-pass filtered derivative to eliminate visual jitter.
  - Live on-screen gain tuning sliders ($K_p$, $K_i$, $K_d$, and Base RPM).
  - Real-time numerical telemetry breakdown of active $P$, $I$, and $D$ terms.
  - Visual torque balance gauge showing steering effort.
- **Field Edge & End-of-Row Detection**:
  - Forward lookahead plant density monitoring in the upper 35% of the ROI.
  - Temporal persistence filtering to prevent false triggers from crop gaps.
  - Three operational states: `🟢 ROW_FOLLOWING`, `🟡 APPROACHING_EDGE`, and `🔴 FIELD_EDGE_DETECTED`.
  - Automated headland turn readiness alert and track deceleration.
- **Interactive ROI Selection**: Drag a box with your mouse or use live sliders to crop out rover chassis parts, struts, and tires.
- **Per-Video Memory**: Automatically saves and restores your custom detection area, tuned PID parameters, and CLAHE clip in `roi_config.json`.
- **Digital Twin Telemetry**:
  - Live animated differential tracks with scrolling rubber treads.
  - Dynamic steering vector arrow and status beacon.
  - Left & Right track RPM gauges with dual-color power meters.
  - Action command badges (**DRIVE STRAIGHT**, **STEER LEFT**, **STEER RIGHT**, **HEADLAND TURN READY**).

---

## Platform Architecture

```
[ Camera Stream / Video Feed ]
              │
              ▼
[ HSV Color Space Conversion ] ──► Isolates Green Vegetation
              │
              ▼
[ Morphological Noise Filter ] ──► Eliminates Weeds & Soil Artifacts
              │
              ▼
[ User-Defined ROI Masking   ] ──► Crops Out Chassis & Distant Sky
              │
              ▼
[ Sliding Window Row Tracker ] ──► Fits Left (Red) & Right (Blue) Crop Lines
              │
              ▼
[ Path Centerline Calculation] ──► Finds Drivable Center Corridor
              │
              ▼
[ Steering Error (ΔX) Compute] ──► Compares Path Center vs Camera Axis
              │
              ▼
[ Rover Digital Twin Control ] ──► Animates Differential Tracks & Commands
```

---

## Installation & Setup

### 1. Requirements
- Python 3.10, 3.11, 3.12, or 3.13
- macOS, Linux, or Windows

### 2. Setup Virtual Environment
Open your terminal in the project directory:

```bash
cd /Users/kavithudapola/Documents/Rover/Image_processing
```

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

*(On Windows PowerShell, use: `.venv\Scripts\Activate.ps1`)*

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

---

## How to Run (Master Production App)

### Option 1: Master Production Application (Recommended for Raspberry Pi & Embedded SBCs)
Run the consolidated, production-ready master application:
```bash
# Default launch (Auto-detects Raspberry Pi 3B / 4B / SBC & optimizes):
python3 main.py

# Launch with a specific video file:
python3 main.py --video data/good.mp4

# Launch with live USB or CSI camera on Raspberry Pi:
python3 main.py --video 0

# Set explicit SBC performance profile (rpi3b, rpi4, tinker_board, desktop):
python3 main.py --profile rpi3b --serial-port /dev/ttyUSB0 --baudrate 115200
```

> **🛡️ Safety Notice:** On startup, the rover starts in **SAFE STANDBY** with all motor commands disabled (`PWM=0`, `<TRACKING_DISABLED>` transmitted over serial). To start autonomous movement, click the green **`▶ RUN TRACKING`** button in the web cockpit!

When launched, the terminal displays your network address:
```
====================================================================
  🌾  AUTONOMOUS AGRICULTURAL ROVER - PRODUCTION MASTER CONTROLLER  🌾
====================================================================
  📱 Mobile / Tablet Cockpit : http://192.168.1.3:5001
  💻 Local Web Dashboard     : http://localhost:5001
--------------------------------------------------------------------
  ⚡ SBC Hardware Profile    : 🍓 Raspberry Pi 3B / Zero 2W (320x240 @ 18 FPS)
  🔌 Serial Motor Driver     : /dev/ttyUSB0 @ 115200 baud
  📷 Active Vision Source    : good.mp4
  🛡️ Startup Safety State    : MOTORS DISABLED (Press RUN in Web UI)
====================================================================
```
Open the URL in **Safari on your iPhone/iPad**, **Chrome on Android**, or your laptop. You get live sub-50ms MJPEG video streaming, real-time PID & track RPM gauges, and touch sliders to calibrate detection bounds and gains directly in the field!

### Option 2: Run via web_rover_dashboard.py
```bash
python3 web_rover_dashboard.py --port 5001
```

### Option 3: Run the Legacy Desktop OpenCV Simulation GUI
*(Requires an attached HDMI monitor or X11 desktop environment)*:
```bash
python3 step4_rover_simulation_dashboard.py --video data/good.mp4
```

### Option 4: Run the Modular Educational Stages
Each stage is modular and can be run independently:

- **Stage 1 (Video Playback)**: `python3 step1_load_video.py`
- **Stage 2 (Green Plant Masking)**: `python3 step2_plant_detection.py`
- **Stage 3 (Crop Row Tracking)**: `python3 step3_crop_row_detection.py`
- **Generate Fresh 3D Simulation Video**: `python3 generate_simulation_video.py`

---

## Startup Motor Safety & Tracking Toggle

In agricultural robotics, uncontrolled motion upon system startup or camera initialization can cause crop damage or safety hazards. This platform implements a strict **Fail-Safe Startup Architecture**:

```
[ System Startup / Reset ]
          │
          ▼
[ State: SAFE STANDBY ] ──► Motors Disabled (PWM L:0, R:0)
          │             ──► Serial Transmits: <TRACKING_DISABLED>
          │             ──► Microcontroller: Emergency Stop Engaged
          │
[ Operator Presses RUN ] (Web UI button or 'Space' / 'R' key)
          │
          ▼
[ State: TRACKING ACTIVE] ──► Serial Transmits: <TRACKING_ENABLED>
          │              ──► Microcontroller: Motors Armed
          │              ──► Differential PID Steering Packets Streamed (<L,R>)
          │
[ Operator Presses STOP / E-STOP ] (Web UI or 'E' key / Watchdog timeout)
          │
          ▼
[ State: SAFE STANDBY ] ──► Serial Transmits: <TRACKING_DISABLED> & <0,0>
```

- **Default Disabled**: Neither Python nor the microcontroller will spin motors until explicitly activated.
- **Bi-directional State Protocol**:
  - Start / Stop commands: `<TRACKING_ENABLED>` and `<TRACKING_DISABLED>`.
  - Microcontroller acknowledges: `ACK:TRACKING_ENABLED` and `ACK:TRACKING_DISABLED`.
- **Emergency Stop (E-Stop)**: Immediate cut-off accessible via the UI or keyboard shortcut <kbd>E</kbd>.

---

## Line Tracking & Drivable Corridor Visualization

The vision pipeline accurately segments the field into navigable free space and crop row boundaries, rendered on the cockpit stream:

- **Green Drivable Navigation Corridor**: A semi-transparent green polygonal cone rendered directly between the left and right crop rows, representing safe clearance for rover wheels.
- **Crop Row Boundary Lines**:
  - **Red Curve**: Left crop row 2nd-degree polynomial fit ($x = ay^2 + by + c$).
  - **Blue Curve**: Right crop row 2nd-degree polynomial fit.
- **Center Guidance Track**: Yellow dashed centerline calculated equidistant between left and right crop rows.
- **Dynamic Steering Vector**: Directional arrow originating from the rover center toward the prospective lookahead target.
- **Resolution-Invariant Rendering**: All polynomial curves, centroids, and polygon cones are evaluated and overlaid directly at the camera's native processing grid before display scaling. This guarantees **zero coordinate drift or offset** regardless of whether running at 320×240 (RPi 3B), 480×360 (RPi 4B), or 1080p.

---

## Interactive Controls & Web Cockpit

### Web Cockpit Hotkeys & Controls:

| Control | Action |
| :--- | :--- |
| **`▶ RUN` / `⏹ STOP`** | Master toggle to arm/disarm autonomous tracking & motor output |
| **`🛑 EMERGENCY STOP`** | Immediate hardware motor stop and failsafe engagement |
| <kbd>Space</kbd> or <kbd>R</kbd> | Toggle Tracking **RUN / STOP** |
| <kbd>E</kbd> | Trigger **EMERGENCY STOP** |
| **ROI Sliders** | Real-time adjustment of Top %, Bottom %, Left %, and Right % crop bounds |
| **PID Sliders** | On-the-fly tuning of $K_p$, $K_i$, $K_d$, and Base RPM |
| **D-Pad Bench Test** | Manual jog controls (Forward, Reverse, Left, Right, Stop) |

### Desktop OpenCV GUI Hotkeys (When running step4):

| Key | Action |
| :--- | :--- |
| **Mouse Click & Drag** | Draw a custom detection box directly on the camera view |
| <kbd>l</kbd> | **Toggle Adaptive Lighting** (CLAHE + ExG + Dynamic HSV vs Static HSV) |
| <kbd>v</kbd> | **Switch video source** on the fly between loaded videos |
| <kbd>Space</kbd> | **Pause / Resume** video playback |
| <kbd>r</kbd> | **Reset** ROI, PID, and Light parameters back to defaults |
| <kbd>q</kbd> or <kbd>Esc</kbd> | **Save settings and quit** the application |

---

## Detection Area (ROI) Calibration

Different camera mountings capture different view angles. For example, in straddle rovers (like `data/aaa.mp4`), the top 50% of the frame is the rover's own metal chassis.

### How to Calibrate a New Video:
1. Launch your video: `python step4_rover_simulation_dashboard.py --video data/your_video.mp4`
2. **Click and drag your mouse** across the ground area containing only the crop rows and the soil path.
3. Use the trackbars at the top of the window to fine-tune the edges:
   - **`ROI Top %`**: Lower the top slider until the rover frame and horizon disappear.
   - **`ROI Bottom %`**: Raise the slider if wheel hubs or rover bumpers appear at the bottom.
   - **`ROI Left %` / `Right %`**: Adjust to exclude wheels or struts on the sides.
4. Press <kbd>q</kbd> when finished. Your settings are **automatically saved** in `roi_config.json` and will automatically load every time you run that video.

---

## PID Steering Optimization

Rather than abrupt on/off bang-bang thresholding, the rover utilizes a continuous **Proportional-Integral-Derivative (PID)** closed-loop controller:

$$\text{Output}(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$

### Component Roles:
1. **Proportional Term ($P = K_p \cdot e$)**:
   Directly proportional to the offset from the crop corridor center. Drives the rover back toward the centerline.
2. **Integral Term ($I = K_i \int e\,dt$)**:
   Accumulates lingering offset over time to eliminate steady-state error caused by mechanical drift, wheel slip, or terrain slope. Includes **anti-windup clamping** to prevent overshoot.
3. **Derivative Term ($D = K_d \frac{de}{dt}$)**:
   Anticipates future error by measuring rate-of-change. It actively counteracts high turning velocity, providing smooth damping that prevents the rover from oscillating back and forth across crop rows. Uses a **low-pass filter** to reject vision pixel noise.

### Differential Skid-Steer Track Equations:
$$\text{Speed}_{\text{Left}} = \text{Base RPM} + \text{PID Output}$$
$$\text{Speed}_{\text{Right}} = \text{Base RPM} - \text{PID Output}$$

- **Positive Error (Corridor to the Right)** $\rightarrow$ Positive PID output $\rightarrow$ Left track accelerates, Right track decelerates $\rightarrow$ Rover steers **RIGHT**.
- **Negative Error (Corridor to the Left)** $\rightarrow$ Negative PID output $\rightarrow$ Left track decelerates, Right track accelerates $\rightarrow$ Rover steers **LEFT**.

### Live Tuning Guidelines:
- If the rover reacts too sluggishly $\rightarrow$ Increase **`Kp`**.
- If the rover oscillates or wiggles across rows $\rightarrow$ Increase **`Kd`** for stronger damping.
- If the rover tracks steadily slightly off-center $\rightarrow$ Increase **`Ki`** slightly.

---

## Field Edge & End-of-Row Detection

In real precision agriculture, crop rows end at the **headland** (cleared turnaround boundary at the perimeter of the field). The rover must never drive blindly when rows terminate.

### Detection Mechanism:
1. **Forward Lookahead Zone**: The upper 35% of the ROI (ahead of the rover towards the vanishing point) is continuously sampled for green plant density.
2. **Row Continuity Check**: Evaluates if sliding window centroids are present in the upper third of the detection corridor.
3. **Temporal Persistence**: Requires 4 consecutive frames of row termination to prevent false triggers from occasional dead crop gaps.

### Operational States:
- **`🟢 ROW_FOLLOWING`**: Green density $> 6\%$ and rows tracked ahead $\rightarrow$ Normal PID autonomous tracking.
- **`🟡 APPROACHING_EDGE`**: Green density drops below $6\%$ $\rightarrow$ Warning alert displayed, rover prepares for boundary.
- **`🔴 FIELD_EDGE_DETECTED`**: Green density $< 2\%$ for 4+ consecutive frames $\rightarrow$ Flashing Red Alert:
  - Action Command switches to **`HEADLAND TURN READY`**.
  - Track speeds safely decelerate to a controlled crawling speed ($35\text{ RPM}$).
  - Full-width warning banner displayed across camera HUD.

---

## Project File Structure

```
Image_processing/
├── main.py                             # 🌟 MASTER PRODUCTION APPLICATION (Raspberry Pi 3B/4B, Tinker Board, Web Cockpit)
├── web_rover_dashboard.py              # Web remote telemetry cockpit server with hardware controls
├── serial_motor_controller.py          # Serial communication & PWM motor driver controller (Safe startup standby)
├── video_device_manager.py             # Dynamic video capture device discovery & zero-latency capture
├── sbc_optimizer.py                    # Performance tuning profiles for Raspberry Pi 3B/4B & ASUS Tinker Board
├── arduino_rover_motor_controller.ino  # Ready-to-flash microcontroller firmware with TRACKING_DISABLED/ENABLED support
├── hardware_config.json                # Saved serial port, baudrate, and PWM calibration
├── roi_config.json                     # Saved per-video detection boundaries
├── requirements.txt                    # Project dependencies (opencv-python, numpy, flask, pyserial)
├── templates/index.html                # Responsive web cockpit UI with Master RUN/STOP control
├── static/                             # Web styling, JavaScript, and 3D digital twin assets
│   ├── app.js                          # Client-side telemetry polling & remote control logic
│   ├── style.css                       # Premium responsive cockpit styling & micro-animations
│   ├── rover_model.png                 # 3D digital twin rover chassis asset
│   └── wheel_rim.png                   # Rotating CNC wheel rim asset
├── data/                               # Video storage (crop row videos & simulations)
│   ├── good.mp4                        # Clean field footage
│   ├── simulated_field.mp4             # 3D synthetic field simulation video
│   ├── real_field_video.mp4            # Real corn row field footage
│   └── aaa.mp4                         # Real straddle agricultural robot footage
├── step4_rover_simulation_dashboard.py # Complete navigation platform (with Serial & SBC flags)
└── README.md                           # Documentation manual (this file)
```

---

## Hardware Integration & Serial Link Guide

### 1. Serial Motor Driver Communication & PWM Control
The rover transforms differential steering errors and cruising speeds into hardware **PWM** (Pulse Width Modulation) commands sent across a serial link (USB or hardware UART) to your motor driver microcontroller (Arduino Uno/Nano/Mega, ESP32, STM32, Teensy, Cytron, BTS7960, or Sabertooth).

- **Standard Packet Protocol**: `<PWM_LEFT,PWM_RIGHT>\n`
  - Example: `<180,180>\n` (Drive forward at ~70% duty cycle)
  - Example: `<225,135>\n` (Steer right: Left track accelerates, Right decelerates)
  - Example: `<-140,140>\n` (Spin pivot turn in place)
  - Example: `<0,0>\n` (Emergency Stop or Headland Stop)
- **Alternate Protocols**: Key-Value (`L:180,R:180\n`) and JSON (`{"l":180,"r":180}\n`) selectable in the UI.
- **Deadband Min PWM & Max Limit**: Overcomes static friction on heavy rover wheels (default: min 35 PWM, max 255 PWM).
- **Direction Inversion**: Independent Left/Right motor inversion toggles to correct field wiring without rewiring.
- **Hardware Watchdog Failsafe**: If vision frames stop arriving for >500ms (e.g. camera disconnect), the system and Arduino firmware automatically cut PWM to `0` to prevent runaway rovers.
- **Emergency Stop (E-STOP)**: Prominent red E-STOP button in the Web Cockpit and desktop GUI immediately cuts all motor power.

### 2. Video Capture Device Selection
Switch effortlessly between any video source without restarting the platform:
- **USB Webcams**: Camera indices `0`, `1`, `2` or device paths like `/dev/video0`, `/dev/video1`.
- **CSI Cameras (MIPI)**: V4L2 device nodes or GStreamer pipelines on the ASUS Tinker Board.
- **Recorded Test Footage**: Video files in `data/` (`good.mp4`, `aaa.mp4`, `simulated_field.mp4`, etc.).
- **Zero-Latency Threaded Capture**: Uses a dedicated background frame-draining thread that eliminates the 3-5 frame V4L2 queue delay common on Linux USB camera drivers.

### 3. Customizable Serial Port & Baudrate
- **Dynamic Port Scanning**: Auto-detects `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/ttyS1` (Tinker Board UART), or Windows `COM*` ports.
- **Baud Rates**: Selectable from `9600`, `19200`, `38400`, `57600`, `115200` (default standard), `230400`, `460800`, `921600` baud.
- **Benchtop Motor Testing**: Directional D-Pad pad (Forward ⬆️, Reverse ⬇️, Spin Left ↩️, Spin Right ↪️, Stop 🛑) allows bench testing motor direction before setting the rover in the field.

---

## SBC Optimization (Raspberry Pi 3B / 4B & ASUS Tinker Board)

The platform is engineered specifically for energy-efficient edge Single Board Computers (SBCs), with tuned presets for Raspberry Pi and ASUS Tinker Board architectures:

### 1. Pre-Tuned SBC Performance Profiles

| Profile | Target Hardware | Processing Resolution | Target FPS | CLAHE Grid | Use Case |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`rpi3b`** | Raspberry Pi 3B, 3B+, Zero 2W | **320 × 240** | **18 FPS** | 4 × 4 | High thermal efficiency, quad-core Cortex-A53 |
| **`rpi4`** | Raspberry Pi 4B, 5 | **480 × 360** | **25 FPS** | 6 × 6 | Quad-core Cortex-A72 / A76 high responsiveness |
| **`tinker_board`** | ASUS Tinker Board / S (RK3288) | **320 × 240** | **20 FPS** | 6 × 6 | Rockchip ARM Cortex-A17 32-bit architecture |
| **`desktop`** | PC / Mac / Jetson Xavier | **640 × 480** | **30 FPS** | 8 × 8 | Full-resolution desktop simulation |

- **Auto-Detection**: Running `python3 main.py` automatically checks `/proc/cpuinfo` and `uname` on Linux. If Raspberry Pi 3B hardware is detected, the `rpi3b` profile is automatically engaged.
- **Thermal & Compute Optimizations**:
  - Vision processing runs on scaled frames, cutting pixel throughput by **75% to 85%** and CPU utilization from ~100% to under 28%.
  - Frame pacer prevents CPU spinning and heat buildup inside sealed rover enclosures.
  - Zero X11/Tkinter dependencies in headless mode ensures 100% stability over SSH.

### 2. Linux Setup & Permissions on Raspberry Pi / Tinker Board
Add your Linux user to the `dialout` (serial) and `video` (camera) groups to access hardware without requiring `sudo`:
```bash
sudo usermod -a -G dialout,video $USER
```
*(Log out and back in for group permissions to apply).*

### 3. Launching on Raspberry Pi 3B
```bash
# Recommended: Automatic detection & optimization:
python3 main.py

# Or explicitly specify the RPi 3B profile and serial driver:
python3 main.py --profile rpi3b --serial-port /dev/ttyUSB0 --baudrate 115200
```

---

## Microcontroller Firmware (Arduino / ESP32)

Upload the included [`arduino_rover_motor_controller.ino`](file:///Users/kavithudapola/Documents/Rover/Image_processing/arduino_rover_motor_controller.ino) to your microcontroller:
1. Open [`arduino_rover_motor_controller.ino`](file:///Users/kavithudapola/Documents/Rover/Image_processing/arduino_rover_motor_controller.ino) in the Arduino IDE.
2. Verify pin assignments for your motor driver (L298N, BTS7960, Cytron, etc.):
   - Left Motor: `PIN_PWM_LEFT = 5`, `PIN_DIR_LEFT_A = 7`, `PIN_DIR_LEFT_B = 8`
   - Right Motor: `PIN_PWM_RIGHT = 6`, `PIN_DIR_RIGHT_A = 9`, `PIN_DIR_RIGHT_B = 10`
   - Status LED: `PIN_LED_STATUS = 13`
3. Select your board (Arduino Uno, Nano, Mega, or ESP32) and upload.
4. Connect the USB cable between the microcontroller and the Raspberry Pi / SBC USB port (or connect TX/RX to hardware UART).

### Supported Firmware Commands:
- `<TRACKING_DISABLED>`, `<DISABLE>`, or `<STOP>`:
  - Immediately disengages motors (`emergencyStop()`).
  - Extinguishes Status LED.
  - Responds with `ACK:TRACKING_DISABLED`.
- `<TRACKING_ENABLED>`, `<ENABLE>`, or `<RUN>`:
  - Arms motor driver and illuminates Status LED.
  - Responds with `ACK:TRACKING_ENABLED`.
- `<PWM_LEFT,PWM_RIGHT>`:
  - Sets differential drive speed and direction.
  - Ignored if tracking has not been enabled or failsafe watchdog expires (>500ms without packet).
- `<EMERGENCY_STOP>`:
  - Immediate fail-safe cutoff.
