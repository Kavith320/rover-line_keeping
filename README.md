# Autonomous Agricultural Rover Vision Platform

An educational computer vision and robotics simulation platform for autonomous crop row navigation. The platform processes camera streams (both real field footage and synthetic 3D field simulations), isolates crop rows, calculates drivable free space, computes steering error, and displays live differential drive telemetry on a digital twin rover.

---

## Table of Contents
1. [Overview & Features](#overview--features)
2. [Platform Architecture](#platform-architecture)
3. [Installation & Setup](#installation--setup)
4. [How to Run](#how-to-run)
5. [Interactive Controls](#interactive-controls)
6. [Detection Area (ROI) Calibration](#detection-area-roi-calibration)
7. [Steering Logic & Track Physics](#steering-logic--track-physics)
8. [Project File Structure](#project-file-structure)
9. [Hardware Transition Roadmap](#hardware-transition-roadmap)

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

## How to Run

### Option 1: Run the Web Remote Cockpit (View on Any Device / Phone / Tablet)
```bash
# Launch the web platform (accessible on phone, tablet, laptop over Wi-Fi):
python web_rover_dashboard.py

# Or launch with a specific video:
python web_rover_dashboard.py --video data/good.mp4
# Or via dashboard flag:
python step4_rover_simulation_dashboard.py --video data/good.mp4 --web
```
When launched, the terminal displays your network address:
```
==============================================================
  🌾 Autonomous Agricultural Rover - Web Remote Platform 🌾
==============================================================
  📱 Phone / Tablet Access : http://192.168.1.105:5000
  💻 Local Computer Access : http://localhost:5000
==============================================================
```
Open the URL in **Safari on your iPhone/iPad**, **Chrome on Android**, or your laptop. You get live sub-50ms MJPEG video streaming, real-time PID & track RPM gauges, and touch sliders to calibrate detection bounds and gains directly in the field!

### Option 2: Run the Local Desktop OpenCV Window
```bash
python step4_rover_simulation_dashboard.py
```

### Option 3: Run Desktop with a Custom Video
Point to any video file inside the `data/` folder:

```bash
# Run with real straddle robot footage:
python step4_rover_simulation_dashboard.py --video data/aaa.mp4

# Run with corn row footage:
python step4_rover_simulation_dashboard.py --video data/real_field_video.mp4

# Run with other field videos:
python step4_rover_simulation_dashboard.py --video data/good.mp4
python step4_rover_simulation_dashboard.py --video data/s.mp4
```

### Option 3: Run the Educational Stages
Each stage is modular and can be run independently:

- **Stage 1 (Video Playback)**:
  ```bash
  python step1_load_video.py
  ```
- **Stage 2 (Green Plant Masking)**:
  ```bash
  python step2_plant_detection.py
  ```
- **Stage 3 (Crop Row Tracking)**:
  ```bash
  python step3_crop_row_detection.py
  ```
- **Generate Fresh 3D Simulation Video**:
  ```bash
  python generate_simulation_video.py
  ```

---

## Interactive Controls

While the dashboard window is active:

| Control | Action |
| :--- | :--- |
| **Mouse Click & Drag** | Draw a custom detection box directly on the camera view |
| **ROI Sliders** | Fine-tune `ROI Top %`, `Bottom %`, `Left %`, and `Right %` |
| **PID Sliders** | Live adjust `Kp (x100)`, `Ki (x100)`, and `Kd (x100)` gains |
| **Base RPM Slider** | Adjust baseline cruising speed ($30 - 160\text{ RPM}$) |
| **CLAHE (x10) Slider** | Contrast limit for local shadow/glare equalization ($0.1 - 5.0$) |
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
├── data/                               # Video storage
│   ├── crop_row_video.mp4              # Default active video
│   ├── simulated_field.mp4             # 3D synthetic field simulation video
│   ├── real_field_video.mp4            # Real corn row field footage
│   ├── aaa.mp4                         # Real straddle agricultural robot footage
│   ├── s.mp4                           # Real field sample
│   └── cr.mp4                          # Real field sample
├── .venv/                              # Isolated Python virtual environment
├── roi_config.json                     # Saved per-video detection boundaries
├── requirements.txt                    # Project dependencies (opencv-python, numpy)
├── generate_simulation_video.py        # 3D synthetic field video generator
├── step1_load_video.py                 # Stage 1: Basic OpenCV stream loader
├── step2_plant_detection.py            # Stage 2: HSV plant color segmentation
├── step3_crop_row_detection.py         # Stage 3: Sliding window crop row tracker
├── step4_rover_simulation_dashboard.py # Stage 4 & 5: Complete navigation platform (with Serial & SBC flags)
├── web_rover_dashboard.py              # Web remote telemetry cockpit with hardware controls
├── serial_motor_controller.py          # Serial communication & PWM motor driver controller
├── video_device_manager.py             # Dynamic video capture device discovery & zero-latency capture
├── sbc_optimizer.py                    # Performance tuning profiles for ASUS Tinker Board
├── arduino_rover_motor_controller.ino  # Ready-to-flash microcontroller firmware for motor drivers
├── hardware_config.json                # Saved serial port, baudrate, and PWM calibration
├── roi_config.json                     # Saved per-video detection boundaries
├── requirements.txt                    # Project dependencies (opencv-python, numpy, flask, pyserial)
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

## ASUS Tinker Board (Rockchip RK3288) Optimization

The platform is purpose-built to run smoothly and efficiently on Single Board Computers (SBCs) like the **ASUS Tinker Board** (Quad-Core ARM Cortex-A17 @ 1.8GHz, 2GB/4GB RAM running TinkerOS or Armbian):

### 1. Thermal & Compute Optimizations
- **Vision Resolution Downscaling**: Scales frames to **320x240** (or 480x360) for computer vision processing (CLAHE, ExG, sliding window). Reduces pixel throughput by **75% to 85%**, cutting CPU load from ~100% to under 25% while maintaining identical steering accuracy.
- **Hardware MJPEG Negotiation**: Requests `cv2.VideoWriter_fourcc(*'MJPG')` directly from USB UVC cameras, avoiding high-bandwidth YUYV USB 2.0 bus bottlenecks.
- **Adaptive Frame Pacer**: Enforces a target rate (e.g. 20 FPS) with dynamic sleeping, preventing thermal throttling in sealed field enclosures.
- **Headless Field Mode**: Eliminates X11 desktop display overhead. Run the web cockpit or background CLI service; access the dashboard from your smartphone, tablet, or laptop over Wi-Fi!

### 2. Linux Setup & Permissions on Tinker Board
On your ASUS Tinker Board terminal, add your user to the `dialout` and `video` groups to access serial ports and cameras without root:
```bash
sudo usermod -a -G dialout,video $USER
```
*(Log out and back in for group permissions to apply).*

### 3. Launching on ASUS Tinker Board
```bash
# Launch Web Cockpit with Tinker Board optimization and USB Serial:
python web_rover_dashboard.py --sbc-profile tinker_board --serial-port /dev/ttyUSB0 --baudrate 115200

# Or launch with a live USB camera:
python web_rover_dashboard.py --video 0 --serial-port /dev/ttyUSB0

# Or run desktop mode with Tinker Board optimizations:
python step4_rover_simulation_dashboard.py --camera 0 --sbc-mode --serial-port /dev/ttyUSB0
```

---

## Microcontroller Firmware (Arduino / ESP32)

Upload the included [`arduino_rover_motor_controller.ino`](file:///Users/kavithudapola/Documents/Rover/Image_processing/arduino_rover_motor_controller.ino) to your microcontroller:
1. Open [`arduino_rover_motor_controller.ino`](file:///Users/kavithudapola/Documents/Rover/Image_processing/arduino_rover_motor_controller.ino) in the Arduino IDE.
2. Verify pin assignments for your motor driver:
   - Left Motor: `PIN_PWM_LEFT = 5`, `PIN_DIR_LEFT_A = 7`, `PIN_DIR_LEFT_B = 8`
   - Right Motor: `PIN_PWM_RIGHT = 6`, `PIN_DIR_RIGHT_A = 9`, `PIN_DIR_RIGHT_B = 10`
3. Select your board (Arduino Uno, Nano, Mega, or ESP32) and upload.
4. Connect the USB cable between the microcontroller and the ASUS Tinker Board's USB port (or connect TX/RX to Tinker Board UART1 `/dev/ttyS1`).
