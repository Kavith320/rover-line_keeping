import cv2
import numpy as np
import math
import os
import sys
import json
import argparse
import serial_motor_controller as smc
import video_device_manager as vdm
import sbc_optimizer as sbc

# ==============================================================================
# Agricultural Rover Simulation - Stage 4 & 5
# Features:
#   - Real-time Plant & Crop Row Perception
#   - Interactive Detection Area (ROI) Selection (Mouse Drag + Sliders)
#   - Field Edge & End-of-Row Detection (Active Boundary Line + Stop Line)
#   - Elimination of Ghost Lines when crops terminate
#   - PID Steering Controller with Live On-Screen Optimization Sliders
#   - Top-Down Digital Twin with Animated Differential Tracks & Telemetry
# ==============================================================================

LOWER_GREEN = np.array([25, 40, 30], dtype=np.uint8)
UPPER_GREEN = np.array([85, 255, 255], dtype=np.uint8)
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
CONFIG_FILE = "roi_config.json"


class PIDController:
    """
    Proportional-Integral-Derivative (PID) Controller for Rover Steering.
    
    Formula:
        Output = Kp * e(t) + Ki * ∫ e(t) dt + Kd * (de(t)/dt)
        
    Features:
        - Proportional (P): Drives rover toward path center.
        - Integral (I): Eliminates steady-state mechanical/terrain drift (with anti-windup clamping).
        - Derivative (D): Damps oscillations using low-pass filtered rate-of-change.
        - Real-time telemetry tracking of individual P, I, D components.
    """
    def __init__(self, kp=0.45, ki=0.03, kd=0.15, out_min=-85.0, out_max=85.0, integral_limit=60.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_deriv = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0

    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_deriv = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0

    def compute(self, error, dt=0.033):
        if dt <= 0.0:
            dt = 0.033

        # 1. Proportional term
        self.p_term = self.kp * error

        # 2. Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))
        self.i_term = self.ki * self.integral

        # 3. Derivative term with low-pass filter to reject vision jitter
        raw_deriv = (error - self.prev_error) / dt
        self.filtered_deriv = 0.35 * raw_deriv + 0.65 * self.filtered_deriv
        self.d_term = self.kd * self.filtered_deriv
        self.prev_error = error

        # 4. Total saturated output
        raw_output = self.p_term + self.i_term + self.d_term
        self.output = float(np.clip(raw_output, self.out_min, self.out_max))
        return self.output


class EdgeDetector:
    """
    Monitors forward crop row continuity and plant density ahead.
    Identifies when the rover reaches the end of the crop row (headland / field boundary).
    Computes the exact pixel coordinate Y_edge where the crop rows terminate.
    
    States:
        - 'ROW_FOLLOWING'       : Crop rows healthy ahead. Normal PID steering.
        - 'APPROACHING_EDGE'    : Forward vegetation ends at edge_y. Draws active boundary line.
        - 'FIELD_EDGE_DETECTED' : Crops terminated ahead. Draws Stop Line & triggers headland turnaround.
    """
    def __init__(self, persistence_threshold=4):
        self.persistence = persistence_threshold
        self.edge_streak = 0
        self.status = "ROW_FOLLOWING"
        self.fwd_density = 0.0
        self.edge_y = None

    def update(self, mask, roi_bounds, left_pts, right_pts):
        roi_top, roi_bottom, roi_left, roi_right = roi_bounds
        roi_h = roi_bottom - roi_top

        # 1. Forward lookahead zone: upper 35% of the ROI
        lookahead_top = roi_top
        lookahead_bottom = roi_top + int(0.35 * roi_h)
        fwd_zone = mask[lookahead_top:lookahead_bottom, roi_left:roi_right]
        total_px = max(1, fwd_zone.size)
        green_px = np.count_nonzero(fwd_zone)
        self.fwd_density = (green_px / total_px) * 100.0

        # 2. Row-by-row vegetation scan inside ROI
        roi_mask = mask[roi_top:roi_bottom, roi_left:roi_right]
        row_sums = np.sum(roi_mask > 0, axis=1)
        valid_rows = np.where(row_sums > 15)[0]

        if len(valid_rows) == 0:
            # Completely out of the crops (bare soil)
            self.edge_y = roi_bottom
            self.edge_streak += 1
        else:
            min_y_rel = np.min(valid_rows)
            furthest_y = roi_top + min_y_rel

            # If the furthest plant is below the top 15% of ROI, we see the end of the row
            if min_y_rel > int(0.15 * roi_h):
                self.edge_y = furthest_y
                self.edge_streak += 1
            else:
                self.edge_y = None
                self.edge_streak = max(0, self.edge_streak - 1)

        # 3. State Determination
        if len(valid_rows) == 0 or (len(left_pts) == 0 and len(right_pts) == 0 and self.edge_streak >= self.persistence):
            self.status = "FIELD_EDGE_DETECTED"
        elif self.edge_y is not None or self.fwd_density < 6.0:
            self.status = "APPROACHING_EDGE"
        else:
            self.status = "ROW_FOLLOWING"

        return self.status, self.fwd_density, self.edge_y


def load_config(video_filename):
    """Loads saved ROI, PID, and Lighting settings for the current video, or returns sensible defaults."""
    base_name = os.path.basename(video_filename)
    defaults = {
        "top": 45,
        "bottom": 95,
        "left": 0,
        "right": 100,
        "kp": 45,       # Kp = 0.45
        "ki": 3,        # Ki = 0.03
        "kd": 15,       # Kd = 0.15
        "base_rpm": 100,
        "clahe_clip": 25,  # 2.5
        "adapt_light": 1   # 1 = ON, 0 = OFF
    }
    if "aaa" in base_name.lower():
        defaults.update({"top": 64, "bottom": 95, "left": 1, "right": 100, "kp": 50, "kd": 20})
    elif "good" in base_name.lower():
        defaults.update({"top": 38, "bottom": 83, "left": 7, "right": 100, "clahe_clip": 25})
    elif "real_field" in base_name.lower():
        defaults.update({"top": 45, "bottom": 100, "left": 23, "right": 78, "base_rpm": 38})
    elif "s.mp4" in base_name.lower():
        defaults.update({"top": 52, "bottom": 80, "left": 0, "right": 100})

    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                if base_name in data:
                    defaults.update(data[base_name])
        except Exception:
            pass
    return defaults


def save_config(video_filename, cfg_dict):
    """Saves customized ROI, PID, and Lighting settings to roi_config.json keyed by video filename."""
    base_name = os.path.basename(video_filename)
    data = {}
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
        except Exception:
            data = {}
    data[base_name] = cfg_dict
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        print(f"[WARN] Could not save config: {e}")


def apply_clahe_enhancement(bgr_frame, clip_limit=2.5, tile_grid=(8, 8)):
    """Applies Contrast Limited Adaptive Histogram Equalization on Luminance channel."""
    lab = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid)
    l_clahe = clahe.apply(l)
    lab_clahe = cv2.merge([l_clahe, a, b])
    return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)


def compute_exg_mask(bgr_frame, threshold=18):
    """
    Excess Green Index (ExG): 2*G - R - B
    Standard agricultural index invariant to uniform illumination shifts and shadows.
    """
    b = bgr_frame[:, :, 0].astype(np.float32)
    g = bgr_frame[:, :, 1].astype(np.float32)
    r = bgr_frame[:, :, 2].astype(np.float32)
    exg = 2.0 * g - r - b
    exg_clipped = np.clip(exg, 0, 255).astype(np.uint8)
    _, exg_mask = cv2.threshold(exg_clipped, threshold, 255, cv2.THRESH_BINARY)
    return exg_mask


def get_green_mask(frame, use_adaptive=True, clahe_clip=2.5):
    """
    Filters green plants using either Adaptive Lighting (CLAHE + ExG + Dynamic HSV)
    or Standard static HSV thresholding.
    Returns: (clean_mask, ambient_brightness, (s_min, v_min))
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ambient_brightness = float(np.mean(gray))

    if use_adaptive and clahe_clip > 0.1:
        proc_frame = apply_clahe_enhancement(frame, clip_limit=clahe_clip)
        blurred = cv2.GaussianBlur(proc_frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Dynamic S_min and V_min based on ambient lighting
        v_min = int(np.clip(30 - (128 - ambient_brightness) * 0.15, 15, 45))
        s_min = int(np.clip(40 + (ambient_brightness - 128) * 0.10, 25, 60))

        lower_green = np.array([25, s_min, v_min], dtype=np.uint8)
        upper_green = np.array([85, 255, 255], dtype=np.uint8)
        hsv_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Fuse with ExG mask (with hue safety constraint)
        exg_mask = compute_exg_mask(blurred, threshold=18)
        fused = cv2.bitwise_or(hsv_mask, exg_mask)
        valid_hue = (hsv[:, :, 0] >= 22) & (hsv[:, :, 0] <= 90)
        fused[~valid_hue] = 0
    else:
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        s_min, v_min = 40, 30
        fused = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    clean = cv2.morphologyEx(fused, cv2.MORPH_OPEN, KERNEL)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, KERNEL)
    return clean, ambient_brightness, (s_min, v_min)


def detect_crop_rows(mask, prev_left, prev_right, roi_bounds, n_windows=6, window_width=150):
    """
    Detects left and right crop rows strictly within the user-selected ROI bounds.
    If crops disappear (field edge), fits are set to None to prevent ghost lines.
    """
    roi_top, roi_bottom, roi_left, roi_right = roi_bounds
    win_h = max(10, (roi_bottom - roi_top) // n_windows)
    mid_x = (roi_left + roi_right) // 2

    # Bottom slice histogram
    bottom_slice = mask[roi_bottom - win_h:roi_bottom, :]
    hist_left = np.sum(bottom_slice[:, roi_left:mid_x], axis=0)
    hist_right = np.sum(bottom_slice[:, mid_x:roi_right], axis=0)

    left_base = (roi_left + np.argmax(hist_left)) if len(hist_left) > 0 and np.max(hist_left) > 0 else (roi_left + (mid_x - roi_left) // 2)
    right_base = (mid_x + np.argmax(hist_right)) if len(hist_right) > 0 and np.max(hist_right) > 0 else (mid_x + (roi_right - mid_x) // 2)

    left_curr = int(np.polyval(prev_left, roi_bottom)) if prev_left is not None else left_base
    right_curr = int(np.polyval(prev_right, roi_bottom)) if prev_right is not None else right_base

    left_curr = np.clip(left_curr, roi_left + 10, mid_x - 10)
    right_curr = np.clip(right_curr, mid_x + 10, roi_right - 10)

    left_pts, right_pts = [], []

    for i in range(n_windows):
        y_low = roi_bottom - (i + 1) * win_h
        y_high = roi_bottom - i * win_h
        y_center = (y_low + y_high) // 2

        xl_1 = max(roi_left, left_curr - window_width // 2)
        xl_2 = min(mid_x, left_curr + window_width // 2)
        xr_1 = max(mid_x, right_curr - window_width // 2)
        xr_2 = min(roi_right, right_curr + window_width // 2)

        patch_l = mask[y_low:y_high, xl_1:xl_2]
        if np.sum(patch_l > 0) > 25:
            M_l = cv2.moments(patch_l)
            if M_l['m00'] > 0:
                left_curr = xl_1 + int(M_l['m10'] / M_l['m00'])
                left_pts.append((left_curr, y_center))

        patch_r = mask[y_low:y_high, xr_1:xr_2]
        if np.sum(patch_r > 0) > 25:
            M_r = cv2.moments(patch_r)
            if M_r['m00'] > 0:
                right_curr = xr_1 + int(M_r['m10'] / M_r['m00'])
                right_pts.append((right_curr, y_center))

    # Fit degree-1 lines (x = m*y + c).
    # Crucial fix: Clear fits to None if points are absent to prevent ghost lines!
    left_fit = None
    if len(left_pts) >= 2:
        fit = np.polyfit(np.array(left_pts)[:, 1], np.array(left_pts)[:, 0], 1)
        left_fit = 0.75 * fit + 0.25 * prev_left if prev_left is not None else fit
    elif len(left_pts) == 1 and prev_left is not None:
        left_fit = prev_left
    else:
        left_fit = None

    right_fit = None
    if len(right_pts) >= 2:
        fit = np.polyfit(np.array(right_pts)[:, 1], np.array(right_pts)[:, 0], 1)
        right_fit = 0.75 * fit + 0.25 * prev_right if prev_right is not None else fit
    elif len(right_pts) == 1 and prev_right is not None:
        right_fit = prev_right
    else:
        right_fit = None

    return left_fit, right_fit, left_pts, right_pts
# Assets for 3D Photorealistic Rover Digital Twin
ROVER_DIR = os.path.dirname(os.path.abspath(__file__))
ROVER_SPRITE_PATH = os.path.join(ROVER_DIR, "static", "rover_model.png")
WHEEL_RIM_PATH = os.path.join(ROVER_DIR, "static", "wheel_rim.png")

_cached_rover_sprite = None
_cached_wheel_rim = None

def get_rover_assets(target_w=240):
    global _cached_rover_sprite, _cached_wheel_rim
    if _cached_rover_sprite is None and os.path.exists(ROVER_SPRITE_PATH):
        raw = cv2.imread(ROVER_SPRITE_PATH, cv2.IMREAD_UNCHANGED)
        if raw is not None:
            h, w = raw.shape[:2]
            target_h = int(h * (target_w / float(w)))
            _cached_rover_sprite = cv2.resize(raw, (target_w, target_h), interpolation=cv2.INTER_AREA)

    if _cached_wheel_rim is None and os.path.exists(WHEEL_RIM_PATH):
        raw_rim = cv2.imread(WHEEL_RIM_PATH, cv2.IMREAD_UNCHANGED)
        if raw_rim is not None:
            scale = target_w / 1269.0
            rw = max(4, int(raw_rim.shape[1] * scale))
            rh = max(4, int(raw_rim.shape[0] * scale))
            _cached_wheel_rim = cv2.resize(raw_rim, (rw, rh), interpolation=cv2.INTER_AREA)

    return _cached_rover_sprite, _cached_wheel_rim

def overlay_rgba(background, overlay, x, y, angle=0.0):
    """Alpha blends an RGBA image onto a BGR background with optional rotation."""
    oh, ow = overlay.shape[:2]
    bh, bw = background.shape[:2]

    if angle != 0.0:
        M = cv2.getRotationMatrix2D((ow / 2.0, oh / 2.0), angle, 1.0)
        overlay = cv2.warpAffine(overlay, M, (ow, oh), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))

    x1, y1 = max(0, x), max(0, y)
    x2, y2 = min(bw, x + ow), min(bh, y + oh)

    ox1, oy1 = max(0, -x), max(0, -y)
    ox2, oy2 = ox1 + (x2 - x1), oy1 + (y2 - y1)

    if x1 >= x2 or y1 >= y2 or ox1 >= ox2 or oy1 >= oy2:
        return background

    sub_bg = background[y1:y2, x1:x2]
    sub_ov = overlay[oy1:oy2, ox1:ox2]

    alpha = sub_ov[:, :, 3].astype(np.float32) / 255.0
    alpha = np.expand_dims(alpha, axis=2)

    sub_bg[:] = (sub_ov[:, :, :3].astype(np.float32) * alpha +
                 sub_bg.astype(np.float32) * (1.0 - alpha)).astype(np.uint8)
    return background

def render_top_down_panel_3d(panel_w, panel_h, decision, dec_color, error_px,
                          speed_l, speed_r, offset_l, offset_r,
                          pid, edge_status, fwd_density, base_rpm, frame_idx):
    """
    Renders the exact photorealistic 3D digital twin of the user's agricultural weeding rover:
      - Photorealistic 3D render of physical rover (exact hoverboard wheels, white controller box,
        battery pack with wiring, white twin-lobe fuel tank, engine & muffler, tiller blades, rear spring suspension)
      - Dynamic engine rumble / combustion vibration micro-jitter
      - Chassis steering roll/lean when turning
      - Rotating hoverboard wheel hubcap with animated 6-star spokes
      - Rotating rotary cultivator tines underneath with soil churn particles
      - Glowing active green & red pulsing LEDs on the controller box
      - High-power LED projector headlights casting forward illumination cones
      - Real-time steering trajectory arrow & telemetry clusters
    """
    panel = np.full((panel_h, panel_w, 3), (16, 20, 26), dtype=np.uint8)

    # 1. Engineering Telemetry Grid
    for gx in range(0, panel_w, 28):
        cv2.line(panel, (gx, 0), (gx, panel_h), (24, 30, 38), 1)
    for gy in range(0, panel_h, 28):
        cv2.line(panel, (0, gy), (panel_w, gy), (24, 30, 38), 1)

    # Outer Panel Border
    cv2.rectangle(panel, (6, 6), (panel_w - 6, panel_h - 6), (45, 58, 72), 2)
    cv2.putText(panel, 'AGRI-ROVER DIGITAL TWIN', (16, 26), cv2.FONT_HERSHEY_DUPLEX, 0.48, (0, 240, 255), 1)

    # Status Beacon Badge (Top Right)
    if edge_status == 'FIELD_EDGE_DETECTED':
        flash = (frame_idx // 6) % 2 == 0
        badge_col = (0, 0, 255) if flash else (0, 140, 255)
        badge_txt = "! FIELD EDGE !"
    elif edge_status == 'APPROACHING_EDGE':
        badge_col = (0, 215, 255)
        badge_txt = "EDGE NEARING"
    else:
        badge_col = (0, 255, 0)
        badge_txt = "ROW TRACKING"

    cv2.rectangle(panel, (panel_w - 148, 10), (panel_w - 12, 32), (26, 32, 40), -1)
    cv2.rectangle(panel, (panel_w - 148, 10), (panel_w - 12, 32), badge_col, 1)
    cv2.circle(panel, (panel_w - 138, 21), 4, badge_col, -1)
    cv2.putText(panel, badge_txt, (panel_w - 128, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.36, badge_col, 1)

    # =========================================================================
    # 3D ROVER DIGITAL TWIN INTEGRATION
    # =========================================================================
    sprite, rim_sprite = get_rover_assets(target_w=240)
    sw, sh = (sprite.shape[1], sprite.shape[0]) if sprite is not None else (240, 208)

    # Center position of rover
    rcx = panel_w // 2 - 4
    rcy = 136

    # 1. Engine Rumble Vibration (combustion micro-jitter)
    avg_speed = (speed_l + speed_r) / 2.0
    if abs(avg_speed) > 10:
        vib_x = int(math.sin(frame_idx * 1.8) * 1.2)
        vib_y = int(math.cos(frame_idx * 2.3) * 0.8)
    else:
        vib_x = int(math.sin(frame_idx * 0.6) * 0.5)
        vib_y = 0

    # 2. Dynamic Steering Lean (Chassis tilts slightly into turn)
    steer_deg = float(np.clip(error_px * 0.22, -35, 35))
    chassis_lean = -steer_deg * 0.12  # Subtle lean in degrees

    # 3. Ground Projector Headlights
    front_deck_x = rcx - 45 + vib_x
    front_deck_y = rcy + 25 + vib_y

    hl_overlay = panel.copy()
    hl1_pts = np.array([
        [front_deck_x - 15, front_deck_y],
        [front_deck_x - 85, front_deck_y - 85],
        [front_deck_x - 10, front_deck_y - 85]
    ], np.int32)
    hl2_pts = np.array([
        [front_deck_x + 35, front_deck_y + 10],
        [front_deck_x + 20, front_deck_y - 75],
        [front_deck_x + 95, front_deck_y - 75]
    ], np.int32)
    cv2.fillPoly(hl_overlay, [hl1_pts, hl2_pts], (65, 88, 105))
    cv2.addWeighted(hl_overlay, 0.25, panel, 0.75, 0, panel)

    # 4. Draw 3D Photorealistic Rover Model
    top_left_x = rcx - sw // 2 + vib_x
    top_left_y = rcy - sh // 2 + vib_y
    if sprite is not None:
        overlay_rgba(panel, sprite, top_left_x, top_left_y, angle=chassis_lean)

    # 5. Seamlessly Rotate Front Hoverboard Wheel Rim
    if rim_sprite is not None:
        rw, rh = rim_sprite.shape[1], rim_sprite.shape[0]
        # In the 240-wide sprite, the wheel rim center is at:
        rim_cx = top_left_x + int(sw * 0.495)
        rim_cy = top_left_y + int(sh * 0.835)
        # Rotation angle driven by right wheel speed
        rim_angle = (offset_r * 28.0) % 360.0
        overlay_rgba(panel, rim_sprite, rim_cx - rw // 2, rim_cy - rh // 2, angle=rim_angle)

    # 6. Active Rotary Cultivator / Tiller Soil Churn
    if abs(avg_speed) > 15:
        tiller_rel_x = int(sw * 0.63)
        tiller_rel_y = int(sh * 0.54)
        tx = top_left_x + tiller_rel_x
        ty = top_left_y + tiller_rel_y

        np.random.seed((frame_idx // 2) % 30)
        for _ in range(6):
            px = tx + np.random.randint(-16, 16)
            py = ty + np.random.randint(4, 18)
            cv2.circle(panel, (px, py), np.random.randint(1, 3), (35, 52, 75), -1)

    # 7. Glowing Status LEDs on Controller Enclosure
    box_led_x = top_left_x + int(sw * 0.21)
    box_led_y = top_left_y + int(sh * 0.56)

    # Bright Green Power/Active LED with Soft Radial Bloom
    cv2.circle(panel, (box_led_x, box_led_y), 5, (0, 200, 90), -1)
    cv2.circle(panel, (box_led_x, box_led_y), 2, (180, 255, 210), -1)

    # Pulsing Red Heartbeat LED
    red_flash = (frame_idx // 8) % 2 == 0
    red_col = (0, 0, 255) if red_flash else (0, 0, 140)
    cv2.circle(panel, (box_led_x + 12, box_led_y - 4), 4, red_col, -1)
    cv2.circle(panel, (box_led_x + 12, box_led_y - 4), 1, (200, 200, 255), -1)

    # 8. Dynamic Steering Vector & Direction Indicator
    arrow_origin_x = rcx
    arrow_origin_y = rcy - 92
    if edge_status == 'FIELD_EDGE_DETECTED':
        # 180° Turnaround Arc
        cv2.ellipse(panel, (arrow_origin_x, arrow_origin_y), (36, 22), 0, 180, 360, (0, 215, 255), 3)
        cv2.arrowedLine(panel, (arrow_origin_x + 36, arrow_origin_y), (arrow_origin_x + 36, arrow_origin_y + 14), (0, 215, 255), 3, tipLength=0.35)
    else:
        arrow_len = 38
        rad = math.radians(steer_deg)
        ax = int(arrow_origin_x + arrow_len * math.sin(rad))
        ay = int(arrow_origin_y - arrow_len * math.cos(rad))
        cv2.arrowedLine(panel, (arrow_origin_x, arrow_origin_y + 6), (ax, ay), dec_color, 3, tipLength=0.32)

    # =========================================================================
    # TELEMETRY DASHBOARD CLUSTERS (RPM Gauges, Tiller Status, PID, Action)
    # =========================================================================

    # 1. Differential Hoverboard Wheel Speeds & Rotary Tiller Status
    bar_w = 115
    cv2.putText(panel, f'L WHEEL: {speed_l} RPM', (18, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (225, 225, 225), 1)
    cv2.rectangle(panel, (18, 251), (18 + bar_w, 259), (32, 38, 45), -1)
    bar_l = int(bar_w * (abs(speed_l) / 160.0))
    cv2.rectangle(panel, (18, 251), (18 + max(0, min(bar_w, bar_l)), 259), (0, 220, 255), -1)

    cv2.putText(panel, f'R WHEEL: {speed_r} RPM', (panel_w - 18 - bar_w, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (225, 225, 225), 1)
    cv2.rectangle(panel, (panel_w - 18 - bar_w, 251), (panel_w - 18, 259), (32, 38, 45), -1)
    bar_r = int(bar_w * (abs(speed_r) / 160.0))
    cv2.rectangle(panel, (panel_w - 18 - bar_w, 251), (panel_w - 18 - bar_w + max(0, min(bar_w, bar_r)), 259), (255, 180, 0), -1)

    # Tiller Status Center Badge
    tiller_active = abs(avg_speed) > 15
    tiller_txt = "TILLER: ACTIVE" if tiller_active else "TILLER: IDLE"
    tiller_col = (0, 255, 120) if tiller_active else (150, 160, 170)
    t_sz = cv2.getTextSize(tiller_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
    cv2.rectangle(panel, (rcx - t_sz[0]//2 - 6, 243), (rcx + t_sz[0]//2 + 6, 259), (26, 32, 38), -1)
    cv2.rectangle(panel, (rcx - t_sz[0]//2 - 6, 243), (rcx + t_sz[0]//2 + 6, 259), tiller_col, 1)
    cv2.putText(panel, tiller_txt, (rcx - t_sz[0]//2, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.35, tiller_col, 1)

    # 2. PID Telemetry Box (y=274 to 412)
    box_y1, box_y2 = 274, 412
    cv2.rectangle(panel, (14, box_y1), (panel_w - 14, box_y2), (15, 18, 24), -1)
    cv2.rectangle(panel, (14, box_y1), (panel_w - 14, box_y2), (48, 62, 78), 1)
    cv2.putText(panel, 'PID STEERING OPTIMIZATION', (24, box_y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)

    gains_str = f"Kp={pid.kp:.2f}  Ki={pid.ki:.3f}  Kd={pid.kd:.2f} | Base={base_rpm} RPM"
    cv2.putText(panel, gains_str, (24, box_y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 180, 200), 1)

    # P, I, D Terms with clean color coding
    p_col = (0, 230, 255)
    i_col = (255, 130, 240)
    d_col = (100, 255, 120)
    cv2.putText(panel, f"P-Term (Offset)  : {pid.p_term:+5.1f} RPM", (24, box_y1 + 54), cv2.FONT_HERSHEY_SIMPLEX, 0.40, p_col, 1)
    cv2.putText(panel, f"I-Term (Steady)  : {pid.i_term:+5.1f} RPM", (24, box_y1 + 71), cv2.FONT_HERSHEY_SIMPLEX, 0.40, i_col, 1)
    cv2.putText(panel, f"D-Term (Damping) : {pid.d_term:+5.1f} RPM", (24, box_y1 + 88), cv2.FONT_HERSHEY_SIMPLEX, 0.40, d_col, 1)

    net_col = (0, 255, 0) if abs(pid.output) < 8 else ((0, 200, 255) if pid.output < 0 else (255, 180, 0))
    cv2.putText(panel, f"Net Correction   : {pid.output:+5.1f} RPM", (24, box_y1 + 107), cv2.FONT_HERSHEY_SIMPLEX, 0.43, net_col, 1)

    # PID Steering Balance Meter (-60 to +60 RPM)
    bm_y = box_y1 + 124
    bm_w = panel_w - 48
    bm_x1 = 24
    cv2.rectangle(panel, (bm_x1, bm_y), (bm_x1 + bm_w, bm_y + 8), (30, 36, 45), -1)
    center_notch = bm_x1 + bm_w // 2
    cv2.line(panel, (center_notch, bm_y - 2), (center_notch, bm_y + 10), (180, 180, 180), 1)

    ratio = np.clip(pid.output / 60.0, -1.0, 1.0)
    tick_x = int(center_notch + ratio * (bm_w // 2))
    tick_col = (0, 200, 255) if pid.output < 0 else (255, 180, 0)
    if abs(pid.output) < 3:
        tick_col = (0, 255, 0)
    if ratio < 0:
        cv2.rectangle(panel, (tick_x, bm_y + 1), (center_notch, bm_y + 7), tick_col, -1)
    else:
        cv2.rectangle(panel, (center_notch, bm_y + 1), (tick_x, bm_y + 7), tick_col, -1)
    cv2.circle(panel, (tick_x, bm_y + 4), 4, (255, 255, 255), -1)

    # 3. Action Command Box (y=422 to 486)
    cmd_y1, cmd_y2 = 422, 486
    cv2.rectangle(panel, (14, cmd_y1), (panel_w - 14, cmd_y2), (14, 18, 24), -1)
    cv2.rectangle(panel, (14, cmd_y1), (panel_w - 14, cmd_y2), dec_color, 2)
    cv2.putText(panel, 'ACTION COMMAND:', (26, cmd_y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
    cv2.putText(panel, f'{decision}', (26, cmd_y1 + 50), cv2.FONT_HERSHEY_DUPLEX, 0.70, dec_color, 2)

    return panel




def draw_realistic_front_wheel(panel, cx, cy, w, h, angle_rad, speed=100, is_left=True):
    """
    Draws a photo-realistic hoverboard hub wheel from top-down perspective:
    - 3D cylindrical rubber tire with rounded shoulders
    - Smooth cosine-curved scrolling chevron knobby tractor treads (zero pop-in/pop-out)
    - 3D embossed rubber lug highlights and shadow crevices
    - CNC-machined silver aluminum rim faceplate
    - 6-spoke machined star pattern that rotates continuously and smoothly
    - Center hex axle nut and hub bevel
    """
    half_w = w // 2
    half_h = h // 2
    R = half_h - 2

    # 1. Soft Ground Contact Shadow
    shadow = panel.copy()
    cv2.ellipse(shadow, (cx, cy + 3), (half_w + 3, half_h + 3), 0, 0, 360, (10, 11, 14), -1)
    cv2.addWeighted(shadow, 0.40, panel, 0.60, 0, panel)

    # 2. Main Tire Rubber Body (Charcoal with cylindrical curvature gradient)
    tx1, tx2 = cx - half_w, cx + half_w
    ty1, ty2 = cy - half_h, cy + half_h

    # Rounded shoulder capsule
    cv2.rectangle(panel, (tx1, ty1 + 4), (tx2, ty2 - 4), (18, 20, 23), -1)
    cv2.ellipse(panel, (cx, ty1 + 4), (half_w, 4), 0, 180, 360, (18, 20, 23), -1)
    cv2.ellipse(panel, (cx, ty2 - 4), (half_w, 4), 0, 0, 180, (18, 20, 23), -1)

    # Shaded tire crown (brighter along center Y, darker near ends)
    for dy in range(-half_h + 1, half_h):
        phi = (dy / float(half_h)) * (math.pi / 2.0)
        cos_f = max(0.0, math.cos(phi))
        lum = int(20 + 26 * (cos_f ** 1.4))
        cur_hw = int(half_w * (1.0 - 0.08 * (abs(dy) / float(half_h)) ** 4))
        py = cy + dy
        cv2.line(panel, (cx - cur_hw, py), (cx + cur_hw, py), (lum, lum + 2, lum + 4), 1)

    # 3. Knobby Tractor Treads rolling over 3D cylindrical surface
    num_lugs = 12
    for i in range(num_lugs):
        lug_ang = angle_rad + i * (2.0 * math.pi / num_lugs)
        cos_a = math.cos(lug_ang)
        sin_a = math.sin(lug_ang)

        if cos_a > 0.05:
            py = int(cy - R * sin_a)
            th = max(1, int(3 * (cos_a ** 0.85)))
            bright = int(75 + 105 * (cos_a ** 1.2))
            dark = int(12 + 18 * cos_a)

            # Chevron V-point offset (points forward/upward in direction of travel)
            chev_dy = int(-4 * (cos_a ** 0.9))

            p_l_out = (cx - half_w + 1, py)
            p_center = (cx, py + chev_dy)
            p_r_out = (cx + half_w - 1, py)

            # Dark shadow groove behind lug
            cv2.line(panel, (p_l_out[0], p_l_out[1] + 1), (p_center[0], p_center[1] + 1), (dark, dark, dark), th)
            cv2.line(panel, (p_r_out[0], p_r_out[1] + 1), (p_center[0], p_center[1] + 1), (dark, dark, dark), th)

            # Bright highlighted rubber lug ridge
            cv2.line(panel, p_l_out, p_center, (bright, bright + 3, bright + 6), th)
            cv2.line(panel, p_r_out, p_center, (bright, bright + 3, bright + 6), th)

            # Side biting traction blocks
            bite_w = max(2, int(4 * cos_a))
            cv2.line(panel, (cx - half_w, py), (cx - half_w + bite_w, py), (bright + 15, bright + 18, bright + 22), th)
            cv2.line(panel, (cx + half_w - bite_w, py), (cx + half_w, py), (bright + 15, bright + 18, bright + 22), th)

    # 4. Outer Tire Sidewall Borders
    cv2.line(panel, (cx - half_w, ty1 + 6), (cx - half_w, ty2 - 6), (55, 60, 68), 1)
    cv2.line(panel, (cx + half_w, ty1 + 6), (cx + half_w, ty2 - 6), (55, 60, 68), 1)

    # 5. CNC Machined Aluminum Rim Faceplate
    rim_w = half_w - 4
    rim_h = half_h - 8
    rx1, rx2 = cx - rim_w, cx + rim_w
    ry1, ry2 = cy - rim_h, cy + rim_h

    cv2.rectangle(panel, (rx1, ry1), (rx2, ry2), (210, 218, 226), -1)
    cv2.rectangle(panel, (rx1, ry1), (rx2, ry2), (145, 155, 165), 1)
    cv2.rectangle(panel, (rx1 + 2, ry1 + 3), (rx2 - 2, ry2 - 3), (26, 28, 32), -1)

    # 6. Continuous Smooth 6-Spoke Machined Aluminum Star
    spoke_rx = rim_w - 3
    spoke_ry = rim_h - 5

    for s in range(6):
        sp_ang = angle_rad + s * (math.pi / 3.0)
        sx = int(cx + spoke_rx * math.sin(sp_ang))
        sy = int(cy - spoke_ry * math.cos(sp_ang))

        sheen = int(195 + 60 * math.sin(sp_ang * 2.0))

        cv2.line(panel, (cx, cy), (sx, sy), (40, 44, 50), 3)
        cv2.line(panel, (cx, cy), (sx, sy), (sheen, sheen + 5, sheen + 10), 2)
        cv2.circle(panel, (sx, sy), 2, (235, 242, 250), -1)

    # 7. Center Axle Cap & Rotating Hex Nut
    cv2.circle(panel, (cx, cy), 5, (170, 178, 188), -1)
    cv2.circle(panel, (cx, cy), 5, (240, 245, 252), 1)

    hex_pts = []
    for h_i in range(6):
        ha = angle_rad + h_i * (math.pi / 3.0)
        hx = int(cx + 3 * math.cos(ha))
        hy = int(cy + 3 * math.sin(ha))
        hex_pts.append([hx, hy])
    cv2.fillPoly(panel, [np.array(hex_pts, np.int32)], (48, 52, 58))
    cv2.polylines(panel, [np.array(hex_pts, np.int32)], True, (230, 238, 246), 1)
    cv2.circle(panel, (cx, cy), 1, (16, 18, 20), -1)


def draw_realistic_rear_wheel(panel, cx, cy, w, h, angle_rad):
    """
    Draws realistic rear trailing wheel with scrolling tread ribs and cream dish rim.
    """
    half_w = w // 2
    half_h = h // 2
    R = half_h - 2

    # Tire Body
    tx1, tx2 = cx - half_w, cx + half_w
    ty1, ty2 = cy - half_h, cy + half_h
    cv2.rectangle(panel, (tx1, ty1 + 4), (tx2, ty2 - 4), (18, 20, 24), -1)
    cv2.ellipse(panel, (cx, ty1 + 4), (half_w, 4), 0, 180, 360, (18, 20, 24), -1)
    cv2.ellipse(panel, (cx, ty2 - 4), (half_w, 4), 0, 0, 180, (18, 20, 24), -1)

    # Curvature shading
    for dy in range(-half_h + 1, half_h):
        phi = (dy / float(half_h)) * (math.pi / 2.0)
        cos_f = max(0.0, math.cos(phi))
        lum = int(20 + 22 * (cos_f ** 1.5))
        cur_hw = int(half_w * (1.0 - 0.08 * (abs(dy) / float(half_h)) ** 4))
        py = cy + dy
        cv2.line(panel, (cx - cur_hw, py), (cx + cur_hw, py), (lum, lum + 2, lum + 3), 1)

    # 10 Rolling tread ribs over cylindrical curve
    num_ribs = 10
    for i in range(num_ribs):
        rib_ang = angle_rad + i * (2.0 * math.pi / num_ribs)
        cos_a = math.cos(rib_ang)
        sin_a = math.sin(rib_ang)
        if cos_a > 0.05:
            py = int(cy - R * sin_a)
            th = max(1, int(3 * (cos_a ** 0.85)))
            bright = int(70 + 85 * (cos_a ** 1.2))
            dark = int(12 + 16 * cos_a)
            cv2.line(panel, (cx - half_w + 1, py + 1), (cx + half_w - 1, py + 1), (dark, dark, dark), th)
            cv2.line(panel, (cx - half_w + 1, py), (cx + half_w - 1, py), (bright, bright, bright), th)

    # Cream / Beige Solid Dish Rim
    rim_w = half_w - 3
    rim_h = half_h - 7
    cv2.rectangle(panel, (cx - rim_w, cy - rim_h), (cx + rim_w, cy + rim_h), (205, 228, 238), -1)
    cv2.rectangle(panel, (cx - rim_w, cy - rim_h), (cx + rim_w, cy + rim_h), (145, 170, 180), 1)

    # 3 Rotating cream dish vent slots
    for slot_i in range(3):
        slot_ang = angle_rad + slot_i * (2.0 * math.pi / 3.0)
        slot_x = int(cx + (rim_w - 3) * math.sin(slot_ang))
        slot_y = int(cy - (rim_h - 5) * math.cos(slot_ang))
        cv2.circle(panel, (slot_x, slot_y), 2, (42, 45, 52), -1)
        cv2.circle(panel, (slot_x, slot_y), 2, (120, 135, 145), 1)

    # Center Axle Nut
    cv2.circle(panel, (cx, cy), 3, (80, 85, 90), -1)
    cv2.circle(panel, (cx, cy), 3, (160, 168, 175), 1)
    cv2.circle(panel, (cx, cy), 1, (240, 245, 250), -1)


def render_top_down_panel_2d(
    panel_w, panel_h, decision, dec_color, error_px,
    speed_l, speed_r, offset_l, offset_r,
    pid, edge_status, fwd_density, base_rpm, frame_idx
):
    panel = np.full((panel_h, panel_w, 3), (18, 22, 28), dtype=np.uint8)

    # 1. Engineering Telemetry Grid
    for gx in range(0, panel_w, 28):
        cv2.line(panel, (gx, 0), (gx, panel_h), (26, 32, 40), 1)
    for gy in range(0, panel_h, 28):
        cv2.line(panel, (0, gy), (panel_w, gy), (26, 32, 40), 1)

    # Outer Panel Border
    cv2.rectangle(panel, (6, 6), (panel_w - 6, panel_h - 6), (50, 62, 75), 2)
    cv2.putText(panel, 'AGRI-ROVER DIGITAL TWIN', (16, 26), cv2.FONT_HERSHEY_DUPLEX, 0.48, (0, 240, 255), 1)

    # Status Beacon Badge (Top Right)
    if edge_status == 'FIELD_EDGE_DETECTED':
        flash = (frame_idx // 6) % 2 == 0
        badge_col = (0, 0, 255) if flash else (0, 140, 255)
        badge_txt = "! FIELD EDGE !"
    elif edge_status == 'APPROACHING_EDGE':
        badge_col = (0, 215, 255)
        badge_txt = "EDGE NEARING"
    else:
        badge_col = (0, 255, 0)
        badge_txt = "ROW TRACKING"

    cv2.rectangle(panel, (panel_w - 148, 10), (panel_w - 12, 32), (28, 34, 42), -1)
    cv2.rectangle(panel, (panel_w - 148, 10), (panel_w - 12, 32), badge_col, 1)
    cv2.circle(panel, (panel_w - 138, 21), 4, badge_col, -1)
    cv2.putText(panel, badge_txt, (panel_w - 128, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.36, badge_col, 1)

    rcx, rcy = panel_w // 2, 138

    # -------------------------------------------------------------------------
    # LAYER 1: HEADLIGHT PROJECTOR BEAMS
    # -------------------------------------------------------------------------
    hl_l = (rcx - 30, rcy - 52)
    hl_r = (rcx + 30, rcy - 52)

    beam_overlay = panel.copy()
    outer_l = np.array([hl_l, [rcx - 90, rcy - 118], [rcx - 2, rcy - 118]], np.int32)
    outer_r = np.array([hl_r, [rcx + 2, rcy - 118], [rcx + 90, rcy - 118]], np.int32)
    cv2.fillPoly(beam_overlay, [outer_l, outer_r], (55, 75, 88))
    inner_l = np.array([hl_l, [rcx - 65, rcy - 118], [rcx - 12, rcy - 118]], np.int32)
    inner_r = np.array([hl_r, [rcx + 12, rcy - 118], [rcx + 65, rcy - 118]], np.int32)
    cv2.fillPoly(beam_overlay, [inner_l, inner_r], (90, 120, 140))
    cv2.addWeighted(beam_overlay, 0.28, panel, 0.72, 0, panel)

    # -------------------------------------------------------------------------
    # LAYER 2: REAR SUSPENSION & REALISTIC REAR TRAILING WHEELS
    # -------------------------------------------------------------------------
    rear_l_x = rcx - 58
    rear_r_x = rcx + 58
    rear_y   = rcy + 58
    rw_w, rw_h = 16, 48

    # Rear Transverse Crossbar & Axle
    cv2.line(panel, (rear_l_x - 4, rear_y - 12), (rear_r_x + 4, rear_y - 12), (32, 34, 38), 5)
    cv2.line(panel, (rear_l_x - 4, rear_y - 12), (rear_r_x + 4, rear_y - 12), (80, 85, 92), 1)

    # Rear wheels rotation angle (continuous from offset)
    rot_rear = ((offset_l + offset_r) * 0.5 * 0.45) % (2.0 * math.pi)

    # Draw realistic rotating rear wheels
    draw_realistic_rear_wheel(panel, rear_l_x, rear_y, rw_w, rw_h, rot_rear)
    draw_realistic_rear_wheel(panel, rear_r_x, rear_y, rw_w, rw_h, rot_rear)

    for rx_pos in [rear_l_x, rear_r_x]:
        # Vertical steel suspension rod
        cv2.line(panel, (rx_pos, rear_y - 28), (rx_pos, rear_y + 14), (160, 165, 175), 2)
        cv2.circle(panel, (rx_pos, rear_y - 28), 3, (200, 205, 215), -1)

        # Helical Compression Coil Spring (3D shaded coils)
        coils = 7
        sp_start_y = rear_y - 26
        sp_end_y   = rear_y + 2
        sp_step = (sp_end_y - sp_start_y) / coils
        for i in range(coils):
            cy_top = int(sp_start_y + i * sp_step)
            cy_bot = int(sp_start_y + (i + 1) * sp_step)
            cv2.line(panel, (rx_pos - 6, cy_top), (rx_pos + 6, cy_bot), (140, 145, 155), 3)
            cv2.line(panel, (rx_pos - 6, cy_top), (rx_pos + 6, cy_bot), (240, 245, 250), 1)

    # -------------------------------------------------------------------------
    # LAYER 3: MAIN BLACK TUBULAR STEEL CHASSIS FRAME
    # -------------------------------------------------------------------------
    fx1 = rcx - 47
    fx2 = rcx + 47
    fy1 = rcy - 52
    fy2 = rcy + 68

    cv2.rectangle(panel, (fx1 - 2, fy1 - 2), (fx2 + 2, fy2 + 2), (10, 11, 14), 5)
    cv2.rectangle(panel, (fx1, fy1), (fx2, fy2), (20, 22, 26), -1)
    cv2.rectangle(panel, (fx1, fy1), (fx2, fy2), (65, 72, 80), 2)
    cv2.rectangle(panel, (fx1 + 3, fy1 + 3), (fx2 - 3, fy2 - 3), (12, 13, 16), 1)

    mid_y = rcy - 2
    cv2.line(panel, (fx1, mid_y), (fx2, mid_y), (20, 22, 26), 6)
    cv2.line(panel, (fx1, mid_y), (fx2, mid_y), (65, 72, 80), 1)

    rear_cross_y = rcy + 42
    cv2.line(panel, (fx1, rear_cross_y), (fx2, rear_cross_y), (20, 22, 26), 6)
    cv2.line(panel, (fx1, rear_cross_y), (fx2, rear_cross_y), (65, 72, 80), 1)

    for bx, by in [(fx1 + 2, fy1 + 2), (fx2 - 2, fy1 + 2),
                   (fx1 + 2, fy2 - 2), (fx2 - 2, fy2 - 2),
                   (fx1 + 2, mid_y),   (fx2 - 2, mid_y)]:
        cv2.circle(panel, (bx, by), 3, (175, 180, 190), -1)
        cv2.circle(panel, (bx, by), 1, (255, 255, 255), -1)

    # -------------------------------------------------------------------------
    # LAYER 4: FRONT WOODEN PAYLOAD DECK (Wood Grain Texture)
    # -------------------------------------------------------------------------
    wx1 = fx1 + 4
    wx2 = fx2 - 4
    wy1 = fy1 + 4
    wy2 = mid_y - 4

    cv2.rectangle(panel, (wx1, wy1), (wx2, wy2), (135, 165, 202), -1)
    cv2.rectangle(panel, (wx1, wy1), (wx2, wy2), (105, 135, 170), 1)

    for gy in range(wy1 + 3, wy2 - 2, 5):
        w_offset = int(math.sin(gy * 0.9) * 2)
        cv2.line(panel, (wx1 + 2, gy), (wx2 - 2, gy + w_offset), (120, 150, 185), 1)

    # -------------------------------------------------------------------------
    # LAYER 5: ACTIVE ROTARY CULTIVATOR / TILLER TINES
    # -------------------------------------------------------------------------
    tiller_cy = rcy + 35
    tiller_w  = 76

    cv2.line(panel, (rcx - tiller_w // 2, tiller_cy),
             (rcx + tiller_w // 2, tiller_cy), (35, 38, 42), 6)
    cv2.line(panel, (rcx - tiller_w // 2, tiller_cy),
             (rcx + tiller_w // 2, tiller_cy), (140, 148, 160), 2)

    cv2.circle(panel, (rcx - 26, tiller_cy), 3, (210, 215, 220), -1)
    cv2.circle(panel, (rcx + 26, tiller_cy), 3, (210, 215, 220), -1)

    avg_speed = (speed_l + speed_r) / 2.0
    tine_rot = (frame_idx * 0.40 + (offset_l + offset_r) * 0.75) % (2 * math.pi)

    for t_hub_x in [rcx - 26, rcx - 12, rcx + 12, rcx + 26]:
        cv2.circle(panel, (t_hub_x, tiller_cy), 5, (45, 48, 54), -1)
        cv2.circle(panel, (t_hub_x, tiller_cy), 5, (160, 165, 172), 1)

        for arm in range(4):
            ang = tine_rot + arm * (math.pi / 2)
            r_inner = 12
            r_outer = 20
            bx1 = int(t_hub_x + r_inner * math.cos(ang))
            by1 = int(tiller_cy + r_inner * math.sin(ang))
            bx2 = int(t_hub_x + r_outer * math.cos(ang + 0.38))
            by2 = int(tiller_cy + r_outer * math.sin(ang + 0.38))

            cv2.line(panel, (t_hub_x, tiller_cy), (bx1, by1), (65, 70, 78), 3)
            cv2.line(panel, (bx1, by1), (bx2, by2), (205, 215, 228), 2)
            cv2.circle(panel, (bx2, by2), 2, (230, 235, 245), -1)

    if abs(avg_speed) > 15:
        np.random.seed((frame_idx // 2) % 40)
        for _ in range(8):
            px = rcx + np.random.randint(-30, 30)
            py = tiller_cy + np.random.randint(4, 24)
            cv2.circle(panel, (px, py), np.random.randint(1, 3), (38, 55, 78), -1)

    # -------------------------------------------------------------------------
    # LAYER 6: DIAGONAL A-FRAME TOWER & MINI GAS ENGINE
    # -------------------------------------------------------------------------
    cv2.line(panel, (fx1 + 10, rcy + 24), (rcx - 22, mid_y + 2), (18, 20, 24), 6)
    cv2.line(panel, (fx1 + 10, rcy + 24), (rcx - 22, mid_y + 2), (75, 82, 92), 2)
    cv2.line(panel, (fx2 - 10, rcy + 24), (rcx + 22, mid_y + 2), (18, 20, 24), 6)
    cv2.line(panel, (fx2 - 10, rcy + 24), (rcx + 22, mid_y + 2), (75, 82, 92), 2)

    eng_x1 = rcx - 25
    eng_x2 = rcx + 25
    eng_y1 = mid_y + 8
    eng_y2 = mid_y + 44
    cv2.rectangle(panel, (eng_x1, eng_y1), (eng_x2, eng_y2), (24, 26, 30), -1)
    cv2.rectangle(panel, (eng_x1, eng_y1), (eng_x2, eng_y2), (58, 64, 72), 2)

    for fy in range(eng_y1 + 4, eng_y2 - 6, 6):
        cv2.line(panel, (eng_x1 + 4, fy), (eng_x2 - 4, fy), (42, 46, 52), 1)

    cv2.circle(panel, (rcx - 4, mid_y + 24), 11, (16, 17, 20), -1)
    cv2.circle(panel, (rcx - 4, mid_y + 24), 11, (65, 72, 80), 1)
    cv2.rectangle(panel, (rcx - 13, mid_y + 13), (rcx - 7, mid_y + 18), (10, 10, 12), -1)
    cv2.rectangle(panel, (rcx - 13, mid_y + 13), (rcx - 7, mid_y + 18), (140, 145, 150), 1)

    muf_x1 = rcx + 8
    muf_x2 = rcx + 24
    muf_y1 = mid_y + 10
    muf_y2 = mid_y + 40
    cv2.rectangle(panel, (muf_x1, muf_y1), (muf_x2, muf_y2), (36, 38, 42), -1)
    cv2.rectangle(panel, (muf_x1, muf_y1), (muf_x2, muf_y2), (75, 80, 88), 1)
    for lx in range(muf_x1 + 3, muf_x2 - 1, 3):
        cv2.line(panel, (lx, muf_y1 + 4), (lx, muf_y2 - 4), (16, 17, 19), 1)

    cable_pts = np.array([
        [rcx + 19, mid_y + 12],
        [rcx + 34, mid_y - 6],
        [rcx + 39, mid_y - 20]
    ], np.int32)
    cv2.polylines(panel, [cable_pts], False, (14, 14, 16), 2)

    # -------------------------------------------------------------------------
    # LAYER 7: SCULPTED TRANSLUCENT HDPE FUEL TANK
    # -------------------------------------------------------------------------
    tank_cx = rcx
    tank_cy = mid_y + 11
    tw_val, th_val = 52, 22

    cv2.rectangle(panel, (tank_cx - tw_val//2 - 1, tank_cy - th_val//2 - 1),
                  (tank_cx + tw_val//2 + 1, tank_cy + th_val//2 + 2), (14, 15, 18), 3)

    tx1 = tank_cx - tw_val // 2
    tx2 = tank_cx + tw_val // 2
    ty1 = tank_cy - th_val // 2
    ty2 = tank_cy + th_val // 2
    tank_pts = np.array([
        [tx1 + 5, ty1], [tx2 - 5, ty1],
        [tx2, ty1 + 5], [tx2, ty2 - 4],
        [tx2 - 6, ty2], [tx1 + 6, ty2],
        [tx1, ty2 - 4], [tx1, ty1 + 5]
    ], np.int32)
    cv2.fillPoly(panel, [tank_pts], (234, 238, 244))
    cv2.polylines(panel, [tank_pts], True, (160, 168, 178), 2)

    cv2.line(panel, (tx1 + 14, ty1 + 3), (tx1 + 22, ty1 + 3), (255, 255, 255), 1)
    cv2.line(panel, (tx2 - 22, ty1 + 3), (tx2 - 14, ty1 + 3), (255, 255, 255), 1)

    fuel_pts = np.array([
        [tx1 + 3, ty1 + 12], [tx2 - 3, ty1 + 12],
        [tx2 - 1, ty2 - 4], [tx2 - 6, ty2 - 1],
        [tx1 + 6, ty2 - 1], [tx1 + 1, ty2 - 4]
    ], np.int32)
    fuel_layer = panel.copy()
    cv2.fillPoly(fuel_layer, [fuel_pts], (65, 195, 165))
    cv2.addWeighted(fuel_layer, 0.42, panel, 0.58, 0, panel)
    cv2.line(panel, (tx1 + 4, ty1 + 12), (tx2 - 4, ty1 + 12), (90, 215, 185), 1)

    for gx_pos in [tank_cx - 13, tank_cx + 13]:
        cv2.circle(panel, (gx_pos, tank_cy + 2), 5, (22, 23, 27), -1)
        cv2.circle(panel, (gx_pos, tank_cy + 2), 5, (60, 65, 72), 1)
        cv2.circle(panel, (gx_pos, tank_cy + 2), 2, (38, 42, 48), -1)
        cv2.circle(panel, (gx_pos, tank_cy + 2), 1, (110, 115, 122), -1)

    neck_pts = np.array([
        [tx1 + 4, ty1 + 5],
        [tx1 - 5, ty1 - 6],
        [tx1 - 1, ty1 - 9],
        [tx1 + 8, ty1 + 2]
    ], np.int32)
    cv2.fillPoly(panel, [neck_pts], (225, 230, 236))
    cv2.polylines(panel, [neck_pts], True, (155, 165, 175), 1)
    cv2.circle(panel, (tx1 - 3, ty1 - 7), 5, (18, 19, 22), -1)
    cv2.circle(panel, (tx1 - 3, ty1 - 7), 5, (85, 92, 100), 1)

    # -------------------------------------------------------------------------
    # LAYER 8: FRONT ELECTRONICS TRAY PAYLOAD
    # -------------------------------------------------------------------------
    eb_x1 = wx1 + 2
    eb_x2 = rcx - 5
    eb_y1 = wy1 + 2
    eb_y2 = wy2 - 2

    cv2.rectangle(panel, (eb_x1, eb_y1), (eb_x2, eb_y2), (236, 240, 246), -1)
    cv2.rectangle(panel, (eb_x1, eb_y1), (eb_x2, eb_y2), (165, 175, 185), 1)

    for ry in range(eb_y1 + 8, eb_y2 - 13, 5):
        cv2.line(panel, (eb_x1 + 6, ry), (eb_x2 - 6, ry), (210, 218, 225), 1)

    for scx, scy in [(eb_x1 + 3, eb_y1 + 3), (eb_x2 - 3, eb_y1 + 3),
                     (eb_x1 + 3, eb_y2 - 3), (eb_x2 - 3, eb_y2 - 3)]:
        cv2.circle(panel, (scx, scy), 1, (130, 135, 140), -1)

    fan_y = eb_y2 - 7
    fan_r = 5
    for fan_x in [eb_x1 + 9, eb_x1 + 23]:
        cv2.circle(panel, (fan_x, fan_y), fan_r, (155, 162, 170), -1)
        cv2.circle(panel, (fan_x, fan_y), fan_r, (70, 75, 80), 1)
        cv2.line(panel, (fan_x - 3, fan_y), (fan_x + 3, fan_y), (55, 60, 65), 1)
        cv2.line(panel, (fan_x, fan_y - 3), (fan_x, fan_y + 3), (55, 60, 65), 1)

    led_flash = (frame_idx // 10) % 2 == 0
    cv2.circle(panel, (eb_x1 + 8, eb_y1 + 9), 5, (0, 180, 80), -1)
    cv2.circle(panel, (eb_x1 + 8, eb_y1 + 9), 2, (160, 255, 190), -1)
    red_col = (0, 0, 255) if led_flash else (0, 0, 160)
    cv2.circle(panel, (eb_x2 - 8, eb_y1 + 9), 4, red_col, -1)
    cv2.circle(panel, (eb_x2 - 8, eb_y1 + 9), 1, (200, 200, 255), -1)

    bat_x1 = rcx + 5
    bat_x2 = wx2 - 2
    bat_y1 = wy1 + 2
    bat_y2 = wy2 - 2

    cv2.rectangle(panel, (bat_x1, bat_y1), (bat_x2, bat_y2), (22, 23, 26), -1)
    cv2.rectangle(panel, (bat_x1, bat_y1), (bat_x2, bat_y2), (58, 62, 68), 1)

    strap_y1 = (bat_y1 + bat_y2) // 2 - 4
    strap_y2 = strap_y1 + 8
    cv2.rectangle(panel, (bat_x1, strap_y1), (bat_x2, strap_y2), (80, 85, 90), -1)
    cv2.rectangle(panel, (bat_x1, strap_y1), (bat_x2, strap_y2), (115, 120, 126), 1)

    cv2.line(panel, (bat_x1 + 6, bat_y1 + 5), (rcx - 2, bat_y1 + 5), (30, 35, 225), 2)
    cv2.line(panel, (bat_x1 + 6, bat_y1 + 8), (rcx - 2, bat_y1 + 8), (12, 13, 15), 2)
    cv2.line(panel, (bat_x1 + 6, bat_y2 - 6), (rcx - 2, bat_y2 - 6), (190, 170, 35), 2)
    cv2.line(panel, (bat_x1 + 6, bat_y2 - 9), (rcx - 2, bat_y2 - 9), (45, 195, 85), 2)

    cv2.rectangle(panel, (rcx - 3, bat_y1 + 3), (rcx + 3, bat_y1 + 10), (0, 215, 255), -1)
    cv2.rectangle(panel, (rcx - 3, bat_y1 + 3), (rcx + 3, bat_y1 + 10), (0, 160, 200), 1)

    # -------------------------------------------------------------------------
    # LAYER 9: REALISTIC FRONT HOVERBOARD WHEELS (Knobby Chevrons + Smooth 6-Spoke Rims)
    # -------------------------------------------------------------------------
    hw_w, hw_h = 24, 58
    front_l_x = rcx - 61
    front_r_x = rcx + 61
    front_y   = rcy - 38

    # Smooth continuous rotation angles:
    rot_l = (offset_l * 0.45) % (2.0 * math.pi)
    rot_r = (offset_r * 0.45) % (2.0 * math.pi)

    draw_realistic_front_wheel(panel, front_l_x, front_y, hw_w, hw_h, rot_l, speed=speed_l, is_left=True)
    draw_realistic_front_wheel(panel, front_r_x, front_y, hw_w, hw_h, rot_r, speed=speed_r, is_left=False)

    # -------------------------------------------------------------------------
    # LAYER 10: FRONT HEADLIGHT PROJECTORS & BUMPER MOUNTS
    # -------------------------------------------------------------------------
    for hx in [hl_l[0], hl_r[0]]:
        cv2.circle(panel, (hx, fy1), 4, (40, 45, 50), -1)
        cv2.circle(panel, (hx, fy1), 3, (255, 255, 210), -1)
        cv2.circle(panel, (hx, fy1), 1, (255, 255, 255), -1)

    # -------------------------------------------------------------------------
    # LAYER 11: DYNAMIC STEERING DIRECTION VECTOR
    # -------------------------------------------------------------------------
    if edge_status == 'FIELD_EDGE_DETECTED':
        cv2.ellipse(panel, (rcx, fy1 - 16), (36, 24), 0, 180, 360, (0, 215, 255), 3)
        cv2.arrowedLine(panel, (rcx + 36, fy1 - 16), (rcx + 36, fy1 + 2), (0, 215, 255), 3, tipLength=0.35)
    else:
        steer_deg = np.clip(error_px * 0.22, -35, 35)
        arrow_len = 36
        rad = math.radians(steer_deg)
        ax = int(rcx + arrow_len * math.sin(rad))
        ay = int((fy1 - 10) - arrow_len * math.cos(rad))
        cv2.arrowedLine(panel, (rcx, fy1 - 8), (ax, ay), dec_color, 3, tipLength=0.32)

    # =========================================================================
    # TELEMETRY CLUSTERS
    # =========================================================================
    bar_w = 115
    cv2.putText(panel, f'L WHEEL: {speed_l} RPM', (18, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (225, 225, 225), 1)
    cv2.rectangle(panel, (18, 251), (18 + bar_w, 259), (32, 38, 45), -1)
    bar_l = int(bar_w * (abs(speed_l) / 160.0))
    cv2.rectangle(panel, (18, 251), (18 + max(0, min(bar_w, bar_l)), 259), (0, 220, 255), -1)

    cv2.putText(panel, f'R WHEEL: {speed_r} RPM', (panel_w - 18 - bar_w, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (225, 225, 225), 1)
    cv2.rectangle(panel, (panel_w - 18 - bar_w, 251), (panel_w - 18, 259), (32, 38, 45), -1)
    bar_r = int(bar_w * (abs(speed_r) / 160.0))
    cv2.rectangle(panel, (panel_w - 18 - bar_w, 251), (panel_w - 18 - bar_w + max(0, min(bar_w, bar_r)), 259), (255, 180, 0), -1)

    tiller_active = abs(avg_speed) > 15
    tiller_txt = "TILLER: ACTIVE" if tiller_active else "TILLER: IDLE"
    tiller_col = (0, 255, 120) if tiller_active else (150, 160, 170)
    t_sz = cv2.getTextSize(tiller_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
    cv2.rectangle(panel, (rcx - t_sz[0]//2 - 6, 243), (rcx + t_sz[0]//2 + 6, 259), (26, 32, 38), -1)
    cv2.rectangle(panel, (rcx - t_sz[0]//2 - 6, 243), (rcx + t_sz[0]//2 + 6, 259), tiller_col, 1)
    cv2.putText(panel, tiller_txt, (rcx - t_sz[0]//2, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.35, tiller_col, 1)

    box_y1, box_y2 = 274, 412
    cv2.rectangle(panel, (14, box_y1), (panel_w - 14, box_y2), (15, 18, 24), -1)
    cv2.rectangle(panel, (14, box_y1), (panel_w - 14, box_y2), (48, 62, 78), 1)
    cv2.putText(panel, 'PID STEERING OPTIMIZATION', (24, box_y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)

    gains_str = f"Kp={pid.kp:.2f}  Ki={pid.ki:.3f}  Kd={pid.kd:.2f} | Base={base_rpm} RPM"
    cv2.putText(panel, gains_str, (24, box_y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 180, 200), 1)

    p_col = (0, 230, 255)
    i_col = (255, 130, 240)
    d_col = (100, 255, 120)
    cv2.putText(panel, f"P-Term (Offset)  : {pid.p_term:+5.1f} RPM", (24, box_y1 + 54), cv2.FONT_HERSHEY_SIMPLEX, 0.40, p_col, 1)
    cv2.putText(panel, f"I-Term (Steady)  : {pid.i_term:+5.1f} RPM", (24, box_y1 + 71), cv2.FONT_HERSHEY_SIMPLEX, 0.40, i_col, 1)
    cv2.putText(panel, f"D-Term (Damping) : {pid.d_term:+5.1f} RPM", (24, box_y1 + 88), cv2.FONT_HERSHEY_SIMPLEX, 0.40, d_col, 1)

    net_col = (0, 255, 0) if abs(pid.output) < 8 else ((0, 200, 255) if pid.output < 0 else (255, 180, 0))
    cv2.putText(panel, f"Net Correction   : {pid.output:+5.1f} RPM", (24, box_y1 + 107), cv2.FONT_HERSHEY_SIMPLEX, 0.43, net_col, 1)

    bm_y = box_y1 + 124
    bm_w = panel_w - 48
    bm_x1 = 24
    cv2.rectangle(panel, (bm_x1, bm_y), (bm_x1 + bm_w, bm_y + 8), (30, 36, 45), -1)
    center_notch = bm_x1 + bm_w // 2
    cv2.line(panel, (center_notch, bm_y - 2), (center_notch, bm_y + 10), (180, 180, 180), 1)

    ratio = np.clip(pid.output / 60.0, -1.0, 1.0)
    tick_x = int(center_notch + ratio * (bm_w // 2))
    tick_col = (0, 200, 255) if pid.output < 0 else (255, 180, 0)
    if abs(pid.output) < 3:
        tick_col = (0, 255, 0)
    if ratio < 0:
        cv2.rectangle(panel, (tick_x, bm_y + 1), (center_notch, bm_y + 7), tick_col, -1)
    else:
        cv2.rectangle(panel, (center_notch, bm_y + 1), (tick_x, bm_y + 7), tick_col, -1)
    cv2.circle(panel, (tick_x, bm_y + 4), 4, (255, 255, 255), -1)

    cmd_y1, cmd_y2 = 422, 486
    cv2.rectangle(panel, (14, cmd_y1), (panel_w - 14, cmd_y2), (14, 18, 24), -1)
    cv2.rectangle(panel, (14, cmd_y1), (panel_w - 14, cmd_y2), dec_color, 2)
    cv2.putText(panel, 'ACTION COMMAND:', (26, cmd_y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
    cv2.putText(panel, f'{decision}', (26, cmd_y1 + 50), cv2.FONT_HERSHEY_DUPLEX, 0.70, dec_color, 2)

    return panel




def render_top_down_panel(panel_w, panel_h, decision, dec_color, error_px,
                          speed_l, speed_r, offset_l, offset_r,
                          pid, edge_status, fwd_density, base_rpm, frame_idx, mode='2d'):
    """
    Renders digital twin panel. Defaults to the clean 2D top-down view.
    """
    if mode == '3d':
        return render_top_down_panel_3d(panel_w, panel_h, decision, dec_color, error_px,
                                         speed_l, speed_r, offset_l, offset_r,
                                         pid, edge_status, fwd_density, base_rpm, frame_idx)
    return render_top_down_panel_2d(panel_w, panel_h, decision, dec_color, error_px,
                                     speed_l, speed_r, offset_l, offset_r,
                                     pid, edge_status, fwd_density, base_rpm, frame_idx)


# Global variables for interactive mouse drag ROI selection
mouse_dragging = False
mouse_start = (0, 0)
mouse_current = (0, 0)
new_mouse_box = None

def mouse_callback(event, x, y, flags, param):
    """Handles mouse click-and-drag inside the camera view to select detection area."""
    global mouse_dragging, mouse_start, mouse_current, new_mouse_box
    disp_cam_w = 640

    if x < disp_cam_w:
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_dragging = True
            mouse_start = (x, y)
            mouse_current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and mouse_dragging:
            mouse_current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and mouse_dragging:
            mouse_dragging = False
            x1 = min(mouse_start[0], x)
            x2 = max(mouse_start[0], x)
            y1 = min(mouse_start[1], y)
            y2 = max(mouse_start[1], y)
            if (x2 - x1) > 20 and (y2 - y1) > 20:
                new_mouse_box = (x1, y1, x2, y2)


def main():
    parser = argparse.ArgumentParser(description="Autonomous Agricultural Rover Simulation with PID & Edge Detection")
    parser.add_argument("--video", type=str, default="data/simulated_field.mp4",
                        help="Path to video file or camera index (e.g. 0, /dev/video0)")
    parser.add_argument("--camera", type=str, default=None,
                        help="Camera device index or V4L2 device (e.g. 0, 1, /dev/video0)")
    parser.add_argument("--serial-port", type=str, default=None,
                        help="Serial port for motor driver (e.g. /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyS1)")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="Serial baudrate (default: 115200)")
    parser.add_argument("--enable-serial", action="store_true",
                        help="Automatically connect to serial motor controller on launch")
    parser.add_argument("--sbc-mode", action="store_true",
                        help="Enable ASUS Tinker Board SBC low-power performance optimization")
    parser.add_argument("--headless", action="store_true",
                        help="Run headless without OpenCV GUI window (for Tinker Board background service)")
    parser.add_argument("--web", action="store_true",
                        help="Launch web remote cockpit server for browser/phone access")
    parser.add_argument("--port", type=int, default=5000,
                        help="Port for web server (default: 5000)")
    args = parser.parse_args()

    if args.web:
        import subprocess
        cmd = [sys.executable, "web_rover_dashboard.py", "--video", args.camera or args.video, "--port", str(args.port)]
        if args.serial_port:
            cmd.extend(["--serial-port", args.serial_port, "--baudrate", str(args.baudrate)])
        if args.sbc_mode:
            cmd.extend(["--sbc-profile", "tinker_board"])
        sys.exit(subprocess.call(cmd))

    # Initialize Serial Motor Driver Controller
    motor_controller = smc.get_motor_controller()
    if args.serial_port or args.enable_serial:
        target_port = args.serial_port or motor_controller.cfg.get("port", "/dev/ttyUSB0")
        motor_controller.connect(target_port, args.baudrate)

    # Initialize SBC Optimizer
    sbc_mode = args.sbc_mode
    sbc_opt = sbc.SBCOptimizer("tinker_board" if sbc_mode else "desktop")

    # Discover devices (hardware cameras + videos)
    detected_devices = vdm.scan_available_devices()
    device_sources = [d["id"] for d in detected_devices]

    initial_src = args.camera if args.camera is not None else args.video
    if initial_src not in device_sources:
        device_sources.insert(0, initial_src)

    current_video_idx = 0
    if initial_src in device_sources:
        current_video_idx = device_sources.index(initial_src)

    current_video = device_sources[current_video_idx]
    
    # Check if numeric camera
    is_cam = (isinstance(current_video, str) and current_video.isdigit()) or (isinstance(current_video, str) and current_video.startswith("/dev/video"))
    cap_src = int(current_video) if (isinstance(current_video, str) and current_video.isdigit()) else current_video
    
    cap = cv2.VideoCapture(cap_src)
    if not cap.isOpened():
        print(f"[ERROR] Could not open video or camera: {current_video}")
        sys.exit(1)

    if is_cam:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    fps = cap.get(cv2.CAP_PROP_FPS)
    delay = int(1000 / fps) if fps > 0 else 33

    window_name = "Agricultural Rover - Autonomous Navigation & Motor Link"
    if not args.headless:
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, mouse_callback)

    # Load initial ROI and PID configs for this video
    cfg = load_config(str(current_video))

    def nothing(x):
        pass

    if not args.headless:
        # 1. ROI Sliders
        cv2.createTrackbar("ROI Top %", window_name, int(cfg.get("top", 45)), 100, nothing)
        cv2.createTrackbar("ROI Bottom %", window_name, int(cfg.get("bottom", 95)), 100, nothing)
        cv2.createTrackbar("ROI Left %", window_name, int(cfg.get("left", 0)), 100, nothing)
        cv2.createTrackbar("ROI Right %", window_name, int(cfg.get("right", 100)), 100, nothing)

        # 2. Live PID Optimization Sliders
        cv2.createTrackbar("Kp (x100)", window_name, int(cfg.get("kp", 45)), 200, nothing)
        cv2.createTrackbar("Ki (x100)", window_name, int(cfg.get("ki", 3)), 50, nothing)
        cv2.createTrackbar("Kd (x100)", window_name, int(cfg.get("kd", 15)), 150, nothing)
        cv2.createTrackbar("Base RPM", window_name, int(cfg.get("base_rpm", 100)), 160, nothing)

        # 3. Adaptive Lighting Sliders
        cv2.createTrackbar("CLAHE (x10)", window_name, int(cfg.get("clahe_clip", 25)), 50, nothing)

    ser_st = motor_controller.get_status()
    print("==========================================================")
    print("  Agricultural Rover Simulation - Stage 4 & 5")
    print("  PID Steering & Serial Motor Controller (PWM Output)")
    print("==========================================================")
    print(f"Active Capture Source : {current_video}")
    print(f"Serial Motor Status   : {ser_st['status_text']} ({ser_st['port']} @ {ser_st['baudrate']})")
    print(f"SBC Optimization Mode : {'ACTIVE (ASUS Tinker Board)' if sbc_mode else 'DESKTOP HIGH RES'}")
    print("----------------------------------------------------------")
    print("Interactive Controls:")
    print("  - ROI Sliders      : Move 'ROI Top %', 'Bottom %', 'Left %', 'Right %'")
    print("  - PID Sliders      : Live adjust 'Kp (x100)', 'Ki (x100)', 'Kd (x100)'")
    print("  - Base Speed       : Move 'Base RPM' slider")
    print("  - [s]              : Connect / Disconnect Serial Motor Driver")
    print("  - [e]              : Emergency Stop (E-Stop) toggle")
    print("  - [o]              : Toggle SBC (ASUS Tinker Board) Optimization")
    print("  - [c] / [v]        : Cycle Video / Camera capture device")
    print("  - [l]              : Toggle Adaptive Lighting (CLAHE+ExG) vs Static HSV")
    print("  - [Space]          : Pause / Resume")
    print("  - [q] / [Esc]      : Save settings and Quit")
    print("==========================================================")

    use_adaptive = bool(cfg.get("adapt_light", 1))

    pid = PIDController(
        kp=cfg.get("kp", 45) / 100.0,
        ki=cfg.get("ki", 3) / 100.0,
        kd=cfg.get("kd", 15) / 100.0
    )
    edge_detector = EdgeDetector(persistence_threshold=4)

    paused = False
    prev_left = None
    prev_right = None
    offset_l = 0.0
    offset_r = 0.0
    tread_gap = 13
    frame_count = 0

    global new_mouse_box

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                # Video ended / loop back to start: reset state cleanly
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                pid.reset()
                edge_detector.edge_streak = 0
                edge_detector.status = "ROW_FOLLOWING"
                edge_detector.edge_y = None
                prev_left = None
                prev_right = None
                continue

            frame_count += 1
            h, w = frame.shape[:2]
            disp_cam_w, disp_cam_h = 640, 500
            panel_w, panel_h = 420, 500

            # If user dragged with mouse on camera view, sync to ROI trackbars
            if new_mouse_box is not None:
                x1, y1, x2, y2 = new_mouse_box
                new_top = int((y1 / disp_cam_h) * 100)
                new_bottom = int((y2 / disp_cam_h) * 100)
                new_left = int((x1 / disp_cam_w) * 100)
                new_right = int((x2 / disp_cam_w) * 100)

                cv2.setTrackbarPos("ROI Top %", window_name, np.clip(new_top, 0, 95))
                cv2.setTrackbarPos("ROI Bottom %", window_name, np.clip(new_bottom, 5, 100))
                cv2.setTrackbarPos("ROI Left %", window_name, np.clip(new_left, 0, 95))
                cv2.setTrackbarPos("ROI Right %", window_name, np.clip(new_right, 5, 100))
                new_mouse_box = None

            # Read current trackbar positions
            top_pct = cv2.getTrackbarPos("ROI Top %", window_name)
            bottom_pct = cv2.getTrackbarPos("ROI Bottom %", window_name)
            left_pct = cv2.getTrackbarPos("ROI Left %", window_name)
            right_pct = cv2.getTrackbarPos("ROI Right %", window_name)

            tk_kp = cv2.getTrackbarPos("Kp (x100)", window_name)
            tk_ki = cv2.getTrackbarPos("Ki (x100)", window_name)
            tk_kd = cv2.getTrackbarPos("Kd (x100)", window_name)
            base_rpm = max(20, cv2.getTrackbarPos("Base RPM", window_name))
            clahe_clip = max(0.1, cv2.getTrackbarPos("CLAHE (x10)", window_name) / 10.0)

            # Update PID gains live from sliders
            pid.update_gains(tk_kp / 100.0, tk_ki / 100.0, tk_kd / 100.0)

            # Ensure valid bounds
            if bottom_pct <= top_pct:
                bottom_pct = min(100, top_pct + 5)
            if right_pct <= left_pct:
                right_pct = min(100, left_pct + 5)

            # Convert percentage to actual frame pixel coordinates
            roi_top = int((top_pct / 100.0) * h)
            roi_bottom = int((bottom_pct / 100.0) * h)
            roi_left = int((left_pct / 100.0) * w)
            roi_right = int((right_pct / 100.0) * w)
            roi_bounds = (roi_top, roi_bottom, roi_left, roi_right)

            # 1. Detect plants & crop rows within selected ROI
            mask, ambient_val, (dyn_s, dyn_v) = get_green_mask(
                frame, use_adaptive=use_adaptive, clahe_clip=clahe_clip
            )
            left_fit, right_fit, left_pts, right_pts = detect_crop_rows(
                mask, prev_left, prev_right, roi_bounds=roi_bounds
            )
            prev_left = left_fit
            prev_right = right_fit

            # 2. Field Edge / End-of-Row Detection (Active boundary horizon)
            edge_status, fwd_density, edge_y = edge_detector.update(mask, roi_bounds, left_pts, right_pts)

            # 3. Path Centering & PID Steering Computation
            cam_center_x = w // 2
            eval_y = int(roi_top + 0.70 * (roi_bottom - roi_top))

            if edge_status == "FIELD_EDGE_DETECTED":
                # Clear rows on bare headland so zero ghost lines are drawn
                left_fit = None
                right_fit = None
                prev_left = None
                prev_right = None
                error_px = 0
                pid.reset()
                decision = "HEADLAND TURNAROUND"
                dec_color = (0, 0, 255)
                # Differential pivot speed for 180° turnaround
                speed_l = 45
                speed_r = -45
            else:
                curr_xl = int(np.polyval(left_fit, eval_y)) if left_fit is not None else cam_center_x - 150
                curr_xr = int(np.polyval(right_fit, eval_y)) if right_fit is not None else cam_center_x + 150
                path_cx = (curr_xl + curr_xr) // 2
                error_px = path_cx - cam_center_x

                # Compute PID output (dt based on video fps)
                pid_output = pid.compute(error_px, dt=1.0 / (fps if fps > 0 else 30))

                if error_px < -15:
                    decision = "STEER LEFT"
                    dec_color = (0, 200, 255)
                elif error_px > 15:
                    decision = "STEER RIGHT"
                    dec_color = (255, 180, 0)
                else:
                    decision = "DRIVE STRAIGHT"
                    dec_color = (0, 255, 0)

                # Differential skid-steer formula
                speed_l = int(np.clip(base_rpm + pid_output, -160, 160))
                speed_r = int(np.clip(base_rpm - pid_output, -160, 160))

            # Transmit differential drive PWM over serial link to motor driver
            pwm_l, pwm_r, tx_packet = motor_controller.send_differential_drive(speed_l, speed_r)

            # Wheel continuous angular rotation (radians)
            offset_l = (offset_l + speed_l * 0.035) % (2.0 * math.pi)
            offset_r = (offset_r + speed_r * 0.035) % (2.0 * math.pi)

            # 4. Camera View Visuals
            cam_vis = frame.copy()

            # Dim area outside selected ROI
            dim_overlay = cam_vis.copy()
            outside_mask = np.ones((h, w), dtype=bool)
            outside_mask[roi_top:roi_bottom, roi_left:roi_right] = False
            dim_overlay[outside_mask] = (dim_overlay[outside_mask] * 0.45).astype(np.uint8)
            cam_vis = dim_overlay

            # Case A: FIELD EDGE DETECTED (Headland Boundary Reached)
            if edge_status == "FIELD_EDGE_DETECTED":
                # Draw prominent red Stop Line across the ground
                stop_y = int(roi_bottom - 45)
                cv2.line(cam_vis, (roi_left, stop_y), (roi_right, stop_y), (0, 0, 255), 4)
                for x in range(roi_left + 15, roi_right - 15, 36):
                    cv2.line(cam_vis, (x, stop_y - 8), (x + 14, stop_y + 8), (255, 255, 255), 2)
                cv2.putText(cam_vis, "--- HEADLAND / FIELD BOUNDARY STOP LINE ---",
                            (roi_left + 35, stop_y - 14), cv2.FONT_HERSHEY_DUPLEX, 0.52, (0, 0, 255), 2)

                # Draw 180° Turnaround Curved Arrow
                center_pt = (cam_center_x, stop_y - 65)
                axes = (85, 52)
                cv2.ellipse(cam_vis, center_pt, axes, 0, 180, 360, (0, 215, 255), 3)
                cv2.arrowedLine(cam_vis, (center_pt[0] + 85, center_pt[1]),
                                (center_pt[0] + 85, center_pt[1] + 35), (0, 215, 255), 3, tipLength=0.4)
                cv2.putText(cam_vis, "180 DEG PIVOT TURNAROUND", (cam_center_x - 115, stop_y - 125),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 215, 255), 1)

            # Case B: APPROACHING EDGE or NORMAL ROW TRACKING
            else:
                top_limit = edge_y if (edge_status == "APPROACHING_EDGE" and edge_y is not None) else roi_top
                
                # Draw active Field Edge Line at edge_y
                if edge_status == "APPROACHING_EDGE" and edge_y is not None:
                    cv2.line(cam_vis, (roi_left, edge_y), (roi_right, edge_y), (0, 140, 255), 3)
                    for x in range(roi_left, roi_right, 20):
                        cv2.line(cam_vis, (x, edge_y), (x + 10, edge_y), (0, 255, 255), 3)
                    cv2.putText(cam_vis, f"<<< FIELD EDGE DETECTED (Y={edge_y}) >>>",
                                (roi_left + 20, edge_y - 10), cv2.FONT_HERSHEY_DUPLEX, 0.50, (0, 140, 255), 2)

                # Draw green drivable corridor up to top_limit
                y_range = np.linspace(top_limit, roi_bottom, 25).astype(int)
                if left_fit is not None and right_fit is not None:
                    xl_pts = np.polyval(left_fit, y_range).astype(int)
                    xr_pts = np.polyval(right_fit, y_range).astype(int)

                    corridor_pts = np.vstack([
                        np.column_stack([xl_pts, y_range]),
                        np.column_stack([xr_pts[::-1], y_range[::-1]])
                    ])

                    overlay = cam_vis.copy()
                    cv2.fillPoly(overlay, [corridor_pts], (40, 180, 60))
                    cv2.addWeighted(overlay, 0.35, cam_vis, 0.65, 0, cam_vis)

                    path_mid_x = ((xl_pts + xr_pts) // 2).astype(int)
                    for i in range(len(y_range) - 1):
                        if i % 2 == 0:
                            cv2.line(cam_vis, (path_mid_x[i], y_range[i]), (path_mid_x[i+1], y_range[i+1]), (0, 255, 0), 2)

                # Left Row Line (Red)
                if left_fit is not None:
                    y_eval = np.array([roi_bottom, top_limit])
                    xl = np.polyval(left_fit, y_eval).astype(int)
                    cv2.line(cam_vis, (xl[0], y_eval[0]), (xl[1], y_eval[1]), (0, 0, 255), 3)

                # Right Row Line (Blue)
                if right_fit is not None:
                    y_eval = np.array([roi_bottom, top_limit])
                    xr = np.polyval(right_fit, y_eval).astype(int)
                    cv2.line(cam_vis, (xr[0], y_eval[0]), (xr[1], y_eval[1]), (255, 0, 0), 3)

                # Draw detected row points
                for pt in left_pts:
                    cv2.circle(cam_vis, pt, 4, (0, 0, 255), -1)
                for pt in right_pts:
                    cv2.circle(cam_vis, pt, 4, (255, 0, 0), -1)

                # Steering vector arrow and target points
                cv2.arrowedLine(cam_vis, (cam_center_x, eval_y), (path_cx, eval_y), (0, 255, 255), 3, tipLength=0.25)
                cv2.circle(cam_vis, (path_cx, eval_y), 7, (0, 255, 0), -1)
                cv2.circle(cam_vis, (cam_center_x, eval_y), 6, (0, 0, 255), -1)

            # Draw ROI boundary rectangle (Yellow border)
            cv2.rectangle(cam_vis, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 255), 2)

            # Camera center line (rover forward axis)
            for cy in range(roi_top, roi_bottom, 16):
                cv2.line(cam_vis, (cam_center_x, cy), (cam_center_x, cy + 8), (220, 220, 220), 2)

            # Resize Camera View to dashboard dimensions
            disp_cam = cv2.resize(cam_vis, (disp_cam_w, disp_cam_h))
            cv2.rectangle(disp_cam, (0, 0), (disp_cam_w, 42), (18, 20, 24), -1)
            base_disp_name = os.path.basename(str(current_video))
            ser_st = motor_controller.get_status()
            ser_tag = f"PWM: L{pwm_l} R{pwm_r}" if ser_st["connected"] else "SERIAL: OFF [S]"
            cv2.putText(disp_cam, f"ERR: {error_px:+d}px | {base_disp_name} | {ser_tag}",
                        (14, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

            sbc_tag = "SBC: ON [O]" if sbc_mode else "SBC: OFF [O]"
            light_mode_str = f"ADAPTIVE [L] ({ambient_val:.0f} LUX) | {sbc_tag}" if use_adaptive else f"STATIC HSV [L] | {sbc_tag}"
            light_col = (80, 255, 120) if use_adaptive else (120, 180, 255)
            cv2.putText(disp_cam, f"LIGHT: {light_mode_str}", (14, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, light_col, 1)

            # Draw in-progress mouse drag rectangle
            if mouse_dragging and mouse_current[0] < disp_cam_w:
                cv2.rectangle(disp_cam, mouse_start, mouse_current, (0, 255, 255), 2)
                cv2.putText(disp_cam, "Selecting New Area...", (mouse_start[0] + 5, mouse_start[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            # 5. Render Top-Down Digital Twin Panel
            panel = render_top_down_panel(panel_w, panel_h, decision, dec_color, error_px,
                                          speed_l, speed_r, offset_l, offset_r,
                                          pid, edge_status, fwd_density, base_rpm, frame_count)

            # 6. Assemble Combined Dashboard: [640x500] + [420x500] = 1060x500
            dashboard = np.hstack([disp_cam, panel])

        if not args.headless:
            cv2.imshow(window_name, dashboard)
            key = cv2.waitKey(delay if not paused else 50) & 0xFF
        else:
            time.sleep(1.0 / (fps if fps > 0 else 30))
            key = 255

        if key == ord('q') or key == 27:
            save_config(str(current_video), {
                "top": top_pct, "bottom": bottom_pct, "left": left_pct, "right": right_pct,
                "kp": tk_kp, "ki": tk_ki, "kd": tk_kd, "base_rpm": base_rpm,
                "clahe_clip": int(round(clahe_clip * 10)), "adapt_light": int(use_adaptive)
            })
            print(f"\n[INFO] Saved configuration for {current_video}")
            print(f"       ROI: Top={top_pct}%, Bottom={bottom_pct}%, Left={left_pct}%, Right={right_pct}%")
            print(f"       PID: Kp={tk_kp/100.0:.2f}, Ki={tk_ki/100.0:.3f}, Kd={tk_kd/100.0:.2f} | Base={base_rpm} RPM")
            print(f"       LIGHT: Mode={'ADAPTIVE' if use_adaptive else 'STATIC'} | CLAHE Clip={clahe_clip:.1f}")
            print("[INFO] Exiting Simulation...")
            break

        elif key == ord(' '):
            paused = not paused
            print("[INFO] Paused" if paused else "[INFO] Resumed")

        elif key == ord('l') or key == ord('L'):
            use_adaptive = not use_adaptive
            print(f"[INFO] Light Adaptation Mode: {'ON (CLAHE + ExG + Dynamic HSV)' if use_adaptive else 'OFF (Static HSV)'}")

        elif key == ord('s') or key == ord('S'):
            if motor_controller.is_open:
                motor_controller.disconnect()
                print("[INFO] Serial motor link disconnected.")
            else:
                port = motor_controller.cfg.get("port", "/dev/ttyUSB0")
                baud = motor_controller.cfg.get("baudrate", 115200)
                motor_controller.connect(port, baud)

        elif key == ord('e') or key == ord('E'):
            if motor_controller.emergency_stopped:
                motor_controller.reset_estop()
            else:
                motor_controller.emergency_stop()

        elif key == ord('o') or key == ord('O'):
            sbc_mode = not sbc_mode
            sbc_opt.set_profile("tinker_board" if sbc_mode else "desktop")
            print(f"[INFO] SBC Optimization Mode toggled: {'ON (ASUS Tinker Board)' if sbc_mode else 'OFF (Desktop)'}")

        elif key == ord('r'):
            if not args.headless:
                cv2.setTrackbarPos("ROI Top %", window_name, 45)
                cv2.setTrackbarPos("ROI Bottom %", window_name, 95)
                cv2.setTrackbarPos("ROI Left %", window_name, 0)
                cv2.setTrackbarPos("ROI Right %", window_name, 100)
                cv2.setTrackbarPos("Kp (x100)", window_name, 45)
                cv2.setTrackbarPos("Ki (x100)", window_name, 3)
                cv2.setTrackbarPos("Kd (x100)", window_name, 15)
                cv2.setTrackbarPos("Base RPM", window_name, 100)
                cv2.setTrackbarPos("CLAHE (x10)", window_name, 25)
            use_adaptive = True
            pid.reset()
            print("[INFO] ROI, PID, and Light parameters reset to defaults.")

        elif key in (ord('v'), ord('V'), ord('c'), ord('C')):
            save_config(str(current_video), {
                "top": top_pct, "bottom": bottom_pct, "left": left_pct, "right": right_pct,
                "kp": tk_kp, "ki": tk_ki, "kd": tk_kd, "base_rpm": base_rpm,
                "clahe_clip": int(round(clahe_clip * 10)), "adapt_light": int(use_adaptive)
            })

            current_video_idx = (current_video_idx + 1) % len(device_sources)
            current_video = device_sources[current_video_idx]
            cap.release()
            
            is_cam = (isinstance(current_video, str) and current_video.isdigit()) or (isinstance(current_video, str) and current_video.startswith("/dev/video"))
            cap_src = int(current_video) if (isinstance(current_video, str) and current_video.isdigit()) else current_video
            cap = cv2.VideoCapture(cap_src)
            if is_cam:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            prev_left = None
            prev_right = None
            pid.reset()
            edge_detector.edge_streak = 0
            edge_detector.status = "ROW_FOLLOWING"
            edge_detector.edge_y = None

            new_cfg = load_config(str(current_video))
            if not args.headless:
                cv2.setTrackbarPos("ROI Top %", window_name, int(new_cfg.get("top", 45)))
                cv2.setTrackbarPos("ROI Bottom %", window_name, int(new_cfg.get("bottom", 95)))
                cv2.setTrackbarPos("ROI Left %", window_name, int(new_cfg.get("left", 0)))
                cv2.setTrackbarPos("ROI Right %", window_name, int(new_cfg.get("right", 100)))
                cv2.setTrackbarPos("Kp (x100)", window_name, int(new_cfg.get("kp", 45)))
                cv2.setTrackbarPos("Ki (x100)", window_name, int(new_cfg.get("ki", 3)))
                cv2.setTrackbarPos("Kd (x100)", window_name, int(new_cfg.get("kd", 15)))
                cv2.setTrackbarPos("Base RPM", window_name, int(new_cfg.get("base_rpm", 100)))
                cv2.setTrackbarPos("CLAHE (x10)", window_name, int(new_cfg.get("clahe_clip", 25)))
            use_adaptive = bool(new_cfg.get("adapt_light", 1))
            print(f"\n[INFO] Switched capture source to: {current_video}")


    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Cleaned up and closed window.")


if __name__ == "__main__":
    main()
