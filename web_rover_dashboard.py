#!/usr/bin/env python3
"""
Autonomous Agricultural Rover - Web Remote Telemetry & Control Platform
=======================================================================
Enables real-time remote monitoring, live video streaming, and parameter
calibration across any device (smartphone, tablet, laptop) over Wi-Fi / LAN.

Optimized for Single Board Computers (ASUS Tinker Board, Raspberry Pi)
and headless field operation:
  - Video capture device selection (USB webcams, CSI camera /dev/video*, test videos)
  - Full serial link communication with motor driver controller (PWM differential drive)
  - Customizable serial port and baudrate (9600 to 921600 baud)
  - ASUS Tinker Board (Rockchip RK3288) compute & thermal optimization
  - Watchdog failsafe and Emergency Stop (E-Stop)
"""

import os
import sys
import time
import json
import socket
import argparse
import threading
import cv2
import numpy as np
import math
from flask import Flask, render_template, Response, request, jsonify

# Import core vision and control algorithms from Stage 4 & 5
import step4_rover_simulation_dashboard as s4
import serial_motor_controller as smc
import video_device_manager as vdm
import sbc_optimizer as sbc

app = Flask(__name__)


def get_local_ip():
    """Detects the rover's primary LAN IP address on the local Wi-Fi network."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


class RoverVisionEngine:
    """
    Threaded vision processing pipeline. Continuously ingests camera frames,
    computes adaptive lighting, fits crop rows, executes PID steering calculations,
    transmits PWM motor commands over serial, and produces synchronized MJPEG JPEG buffers.
    """
    def __init__(self, initial_video=None, sbc_profile="tinker_board", serial_port=None, baudrate=115200):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        
        # Initialize SBC Performance Optimizer
        self.sbc = sbc.SBCOptimizer(sbc_profile)

        # Initialize Serial Motor Driver Controller
        self.motor_controller = smc.get_motor_controller()
        if serial_port:
            self.motor_controller.connect(serial_port, baudrate)

        # Discover available capture devices (live cameras + recorded videos)
        self.devices = vdm.scan_available_devices()
        
        # Determine initial video source
        if initial_video:
            self.current_source = initial_video
        elif self.devices:
            # Prefer recorded simulation video if available, or first camera
            file_devs = [d["id"] for d in self.devices if not d.get("is_live", False)]
            self.current_source = file_devs[0] if file_devs else self.devices[0]["id"]
        else:
            self.current_source = "data/simulated_field.mp4"

        # Initialize Zero-Latency Threaded Video Capture
        self.cap = vdm.ZeroLatencyCapture(
            self.current_source,
            width=640,
            height=480,
            fps=self.sbc.target_fps,
            is_sbc=(self.sbc.profile_key == "tinker_board")
        )
        
        # Load calibration config for this source
        self.cfg = s4.load_config(self.current_source)
        self.top_pct = int(self.cfg.get("top", 45))
        self.bottom_pct = int(self.cfg.get("bottom", 95))
        self.left_pct = int(self.cfg.get("left", 0))
        self.right_pct = int(self.cfg.get("right", 100))
        
        self.tk_kp = int(self.cfg.get("kp", 45))
        self.tk_ki = int(self.cfg.get("ki", 3))
        self.tk_kd = int(self.cfg.get("kd", 15))
        self.base_rpm = int(self.cfg.get("base_rpm", 100))
        
        self.clahe_clip = float(self.cfg.get("clahe_clip", 25)) / 10.0
        self.use_adaptive = bool(self.cfg.get("adapt_light", 1))
        
        self.pid = s4.PIDController(
            kp=self.tk_kp / 100.0,
            ki=self.tk_ki / 100.0,
            kd=self.tk_kd / 100.0
        )
        self.edge_detector = s4.EdgeDetector(persistence_threshold=4)
        
        self.paused = False
        self.running = True
        
        # Telemetry state
        self.telemetry = {
            "status": "ROW_FOLLOWING",
            "decision": "INITIALIZING",
            "decision_color": [0, 255, 0],
            "error_px": 0,
            "speed_l": 100,
            "speed_r": 100,
            "base_rpm": self.base_rpm,
            "pid": {
                "kp": self.tk_kp / 100.0,
                "ki": self.tk_ki / 100.0,
                "kd": self.tk_kd / 100.0,
                "p_term": 0.0,
                "i_term": 0.0,
                "d_term": 0.0,
                "output": 0.0
            },
            "lighting": {
                "mode": "ADAPTIVE" if self.use_adaptive else "STATIC",
                "ambient": 128.0,
                "clahe_clip": self.clahe_clip,
                "s_min": 40,
                "v_min": 30
            },
            "edge": {
                "status": "ROW_FOLLOWING",
                "density": 0.0,
                "edge_y": None
            },
            "roi": {
                "top": self.top_pct,
                "bottom": self.bottom_pct,
                "left": self.left_pct,
                "right": self.right_pct
            },
            "video": os.path.basename(str(self.current_source)),
            "active_device": str(self.current_source),
            "available_devices": self.devices,
            "paused": self.paused,
            "fps": 20.0,
            "serial": self.motor_controller.get_status(),
            "sbc": {
                "profile": self.sbc.profile_key,
                "name": self.sbc.profile["name"],
                "proc_w": self.sbc.proc_w,
                "proc_h": self.sbc.proc_h,
                "target_fps": self.sbc.target_fps,
                "desc": self.sbc.profile["desc"]
            }
        }
        
        # Frame buffers (encoded JPEG bytes)
        self.jpeg_combined = None
        self.jpeg_camera = None
        self.jpeg_mask = None
        
        self.prev_left = None
        self.prev_right = None
        self.offset_l = 0.0
        self.offset_r = 0.0
        self.frame_count = 0

    def switch_device(self, new_source):
        """Switches the active video capture device or video file on the fly."""
        with self.lock:
            # Save config for old video
            s4.save_config(str(self.current_source), {
                "top": self.top_pct, "bottom": self.bottom_pct,
                "left": self.left_pct, "right": self.right_pct,
                "kp": self.tk_kp, "ki": self.tk_ki, "kd": self.tk_kd,
                "base_rpm": self.base_rpm,
                "clahe_clip": int(round(self.clahe_clip * 10)),
                "adapt_light": int(self.use_adaptive)
            })
            
            if self.cap is not None:
                self.cap.release()
                
            self.current_source = str(new_source).strip()
            print(f"[INFO] Switching video capture device to: {self.current_source}")
            
            self.cap = vdm.ZeroLatencyCapture(
                self.current_source,
                width=640,
                height=480,
                fps=self.sbc.target_fps,
                is_sbc=(self.sbc.profile_key == "tinker_board")
            )
            
            self.prev_left = None
            self.prev_right = None
            self.pid.reset()
            self.edge_detector.edge_streak = 0
            self.edge_detector.status = "ROW_FOLLOWING"
            self.edge_detector.edge_y = None
            
            # Load config for new source
            new_cfg = s4.load_config(str(self.current_source))
            self.top_pct = int(new_cfg.get("top", 45))
            self.bottom_pct = int(new_cfg.get("bottom", 95))
            self.left_pct = int(new_cfg.get("left", 0))
            self.right_pct = int(new_cfg.get("right", 100))
            self.tk_kp = int(new_cfg.get("kp", 45))
            self.tk_ki = int(new_cfg.get("ki", 3))
            self.tk_kd = int(new_cfg.get("kd", 15))
            self.base_rpm = int(new_cfg.get("base_rpm", 100))
            self.clahe_clip = float(new_cfg.get("clahe_clip", 25)) / 10.0
            self.use_adaptive = bool(new_cfg.get("adapt_light", 1))
            self.pid.update_gains(self.tk_kp / 100.0, self.tk_ki / 100.0, self.tk_kd / 100.0)
            
            # Refresh devices list
            self.devices = vdm.scan_available_devices()
            self.telemetry["video"] = os.path.basename(str(self.current_source))
            self.telemetry["active_device"] = str(self.current_source)
            self.telemetry["available_devices"] = self.devices

    def set_sbc_profile(self, profile_key):
        """Switches compute profile (tinker_board, balanced, desktop)."""
        with self.lock:
            self.sbc.set_profile(profile_key)
            self.telemetry["sbc"] = {
                "profile": self.sbc.profile_key,
                "name": self.sbc.profile["name"],
                "proc_w": self.sbc.proc_w,
                "proc_h": self.sbc.proc_h,
                "target_fps": self.sbc.target_fps,
                "desc": self.sbc.profile["desc"]
            }

    def update_control(self, key, val):
        """Dynamically applies remote control changes sent from the web client."""
        with self.lock:
            if key == "kp":
                self.tk_kp = int(np.clip(val, 0, 200))
                self.pid.update_gains(self.tk_kp / 100.0, self.tk_ki / 100.0, self.tk_kd / 100.0)
            elif key == "ki":
                self.tk_ki = int(np.clip(val, 0, 50))
                self.pid.update_gains(self.tk_kp / 100.0, self.tk_ki / 100.0, self.tk_kd / 100.0)
            elif key == "kd":
                self.tk_kd = int(np.clip(val, 0, 150))
                self.pid.update_gains(self.tk_kp / 100.0, self.tk_ki / 100.0, self.tk_kd / 100.0)
            elif key == "base_rpm":
                self.base_rpm = int(np.clip(val, 20, 160))
            elif key == "clahe_clip":
                self.clahe_clip = float(np.clip(val, 0.1, 5.0))
            elif key == "use_adaptive":
                self.use_adaptive = bool(val)
            elif key == "top_pct":
                self.top_pct = int(np.clip(val, 0, self.bottom_pct - 5))
            elif key == "bottom_pct":
                self.bottom_pct = int(np.clip(val, self.top_pct + 5, 100))
            elif key == "left_pct":
                self.left_pct = int(np.clip(val, 0, self.right_pct - 5))
            elif key == "right_pct":
                self.right_pct = int(np.clip(val, self.left_pct + 5, 100))
            elif key == "paused":
                self.paused = bool(val)
            elif key == "reset":
                self.top_pct, self.bottom_pct, self.left_pct, self.right_pct = 45, 95, 0, 100
                self.tk_kp, self.tk_ki, self.tk_kd, self.base_rpm = 45, 3, 15, 100
                self.clahe_clip, self.use_adaptive = 2.5, True
                self.pid.reset()
                self.pid.update_gains(0.45, 0.03, 0.15)
            elif key == "save":
                s4.save_config(str(self.current_source), {
                    "top": self.top_pct, "bottom": self.bottom_pct,
                    "left": self.left_pct, "right": self.right_pct,
                    "kp": self.tk_kp, "ki": self.tk_ki, "kd": self.tk_kd,
                    "base_rpm": self.base_rpm,
                    "clahe_clip": int(round(self.clahe_clip * 10)),
                    "adapt_light": int(self.use_adaptive)
                })
                print(f"[INFO] Saved configuration via Web UI for {self.current_source}")

            # Keep telemetry dictionary in sync immediately
            self.telemetry["pid"]["kp"] = float(self.pid.kp)
            self.telemetry["pid"]["ki"] = float(self.pid.ki)
            self.telemetry["pid"]["kd"] = float(self.pid.kd)
            self.telemetry["base_rpm"] = int(self.base_rpm)
            self.telemetry["lighting"]["clahe_clip"] = float(self.clahe_clip)
            self.telemetry["lighting"]["mode"] = "ADAPTIVE" if self.use_adaptive else "STATIC"
            self.telemetry["roi"] = {
                "top": self.top_pct, "bottom": self.bottom_pct,
                "left": self.left_pct, "right": self.right_pct
            }
            self.telemetry["paused"] = self.paused

    def run_worker(self):
        """Worker thread processing camera frames and transmitting serial motor commands."""
        disp_cam_w, disp_cam_h = 640, 500
        panel_w, panel_h = 420, 500
        
        while self.running:
            if not self.paused:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    time.sleep(0.04)
                    continue

                with self.lock:
                    self.frame_count += 1
                    
                    # 1. SBC Optimization: Downscale frame for fast vision pipeline
                    proc_frame, (scale_x, scale_y), (orig_w, orig_h) = self.sbc.prepare_frame(frame)
                    proc_h, proc_w = proc_frame.shape[:2]
                    
                    # Compute ROI pixel coordinates in processing frame
                    roi_top = int((self.top_pct / 100.0) * proc_h)
                    roi_bottom = int((self.bottom_pct / 100.0) * proc_h)
                    roi_left = int((self.left_pct / 100.0) * proc_w)
                    roi_right = int((self.right_pct / 100.0) * proc_w)
                    roi_bounds = (roi_top, roi_bottom, roi_left, roi_right)
                    
                    # 2. Detect plants with Adaptive Lighting (using SBC CLAHE grid)
                    mask, ambient_val, (dyn_s, dyn_v) = s4.get_green_mask(
                        proc_frame, use_adaptive=self.use_adaptive, clahe_clip=self.clahe_clip
                    )
                    
                    # 3. Sliding window crop row tracker
                    left_fit, right_fit, left_pts, right_pts = s4.detect_crop_rows(
                        mask, self.prev_left, self.prev_right, roi_bounds=roi_bounds
                    )
                    self.prev_left = left_fit
                    self.prev_right = right_fit
                    
                    # 4. Field Edge Detection
                    edge_status, fwd_density, edge_y = self.edge_detector.update(
                        mask, roi_bounds, left_pts, right_pts
                    )
                    
                    # 5. Steering and Differential Track Dynamics
                    cam_center_x = proc_w // 2
                    eval_y = int(roi_top + 0.70 * (roi_bottom - roi_top))
                    
                    if edge_status == "FIELD_EDGE_DETECTED":
                        left_fit, right_fit = None, None
                        self.prev_left, self.prev_right = None, None
                        error_px = 0
                        self.pid.reset()
                        decision = "HEADLAND TURNAROUND"
                        dec_color = (0, 0, 255)
                        speed_l, speed_r = 45, -45
                    else:
                        curr_xl = int(np.polyval(left_fit, eval_y)) if left_fit is not None else cam_center_x - int(proc_w * 0.22)
                        curr_xr = int(np.polyval(right_fit, eval_y)) if right_fit is not None else cam_center_x + int(proc_w * 0.22)
                        path_cx = (curr_xl + curr_xr) // 2
                        
                        # Scale-invariant error normalized to standard 640px camera view
                        raw_error = path_cx - cam_center_x
                        error_px = int(raw_error * (disp_cam_w / float(proc_w)))
                        
                        pid_output = self.pid.compute(error_px, dt=1.0 / max(10, self.sbc.target_fps))
                        
                        if error_px < -15:
                            decision = "STEER LEFT"
                            dec_color = (0, 200, 255)
                        elif error_px > 15:
                            decision = "STEER RIGHT"
                            dec_color = (255, 180, 0)
                        else:
                            decision = "DRIVE STRAIGHT"
                            dec_color = (0, 255, 0)
                            
                        speed_l = int(np.clip(self.base_rpm + pid_output, -160, 160))
                        speed_r = int(np.clip(self.base_rpm - pid_output, -160, 160))
                    
                    # 6. HARDWARE TRANSMISSION: Send PWM commands over serial to motor driver
                    pwm_l, pwm_r, tx_packet = self.motor_controller.send_differential_drive(speed_l, speed_r)
                        
                    # Advance continuous wheel angular rotation (radians)
                    self.offset_l = (self.offset_l + speed_l * 0.035) % (2.0 * math.pi)
                    self.offset_r = (self.offset_r + speed_r * 0.035) % (2.0 * math.pi)
                    
                    # 7. Render crisp Camera View with Overlays
                    disp_cam = cv2.resize(proc_frame, (disp_cam_w, disp_cam_h))
                    
                    # Scale coordinates for display
                    d_roi_top = int((self.top_pct / 100.0) * disp_cam_h)
                    d_roi_bottom = int((self.bottom_pct / 100.0) * disp_cam_h)
                    d_roi_left = int((self.left_pct / 100.0) * disp_cam_w)
                    d_roi_right = int((self.right_pct / 100.0) * disp_cam_w)
                    d_cam_cx = disp_cam_w // 2
                    d_eval_y = int(d_roi_top + 0.70 * (d_roi_bottom - d_roi_top))
                    d_path_cx = int(d_cam_cx + error_px)
                    
                    if edge_status == "FIELD_EDGE_DETECTED":
                        stop_y = int(d_roi_top + 0.50 * (d_roi_bottom - d_roi_top))
                        cv2.line(disp_cam, (d_roi_left, stop_y), (d_roi_right, stop_y), (0, 0, 255), 4)
                        cv2.putText(disp_cam, ">>> HEADLAND REACHED - ROW TERMINATION <<<",
                                    (d_roi_left + 15, stop_y - 12), cv2.FONT_HERSHEY_DUPLEX, 0.52, (0, 0, 255), 2)
                    elif edge_status == "APPROACHING_EDGE":
                        top_limit = int(edge_y * (disp_cam_h / float(proc_h))) if edge_y is not None else d_roi_top
                        cv2.line(disp_cam, (d_roi_left, top_limit), (d_roi_right, top_limit), (0, 165, 255), 2)
                    else:
                        top_limit = d_roi_top
                        
                    if edge_status != "FIELD_EDGE_DETECTED":
                        # Rescale polynomial fits to display resolution
                        if left_fit is not None:
                            y_eval = np.array([d_roi_bottom, top_limit])
                            y_proc = y_eval * (float(proc_h) / disp_cam_h)
                            xl_proc = np.polyval(left_fit, y_proc)
                            xl_disp = (xl_proc * (float(disp_cam_w) / proc_w)).astype(int)
                            cv2.line(disp_cam, (xl_disp[0], y_eval[0]), (xl_disp[1], y_eval[1]), (0, 0, 255), 3)
                            
                        if right_fit is not None:
                            y_eval = np.array([d_roi_bottom, top_limit])
                            y_proc = y_eval * (float(proc_h) / proc_h)
                            xr_proc = np.polyval(right_fit, y_proc)
                            xr_disp = (xr_proc * (float(disp_cam_w) / proc_w)).astype(int)
                            cv2.line(disp_cam, (xr_disp[0], y_eval[0]), (xr_disp[1], y_eval[1]), (255, 0, 0), 3)
                            
                        for pt in left_pts:
                            px = int(pt[0] * (disp_cam_w / float(proc_w)))
                            py = int(pt[1] * (disp_cam_h / float(proc_h)))
                            cv2.circle(disp_cam, (px, py), 4, (0, 0, 255), -1)
                        for pt in right_pts:
                            px = int(pt[0] * (disp_cam_w / float(proc_w)))
                            py = int(pt[1] * (disp_cam_h / float(proc_h)))
                            cv2.circle(disp_cam, (px, py), 4, (255, 0, 0), -1)
                            
                        cv2.arrowedLine(disp_cam, (d_cam_cx, d_eval_y), (d_path_cx, d_eval_y), (0, 255, 255), 3, tipLength=0.25)
                        cv2.circle(disp_cam, (d_path_cx, d_eval_y), 7, (0, 255, 0), -1)
                        cv2.circle(disp_cam, (d_cam_cx, d_eval_y), 6, (0, 0, 255), -1)
                        
                    # ROI Boundary Box
                    cv2.rectangle(disp_cam, (d_roi_left, d_roi_top), (d_roi_right, d_roi_bottom), (0, 255, 255), 2)
                    for cy in range(d_roi_top, d_roi_bottom, 16):
                        cv2.line(disp_cam, (d_cam_cx, cy), (d_cam_cx, cy + 8), (220, 220, 220), 2)
                        
                    # HUD Status Banner
                    cv2.rectangle(disp_cam, (0, 0), (disp_cam_w, 42), (18, 20, 24), -1)
                    source_name = os.path.basename(str(self.current_source))
                    ser_st = self.motor_controller.get_status()
                    ser_tag = f"PWM: L{pwm_l} R{pwm_r}" if ser_st["connected"] else "SERIAL: OFF"
                    
                    cv2.putText(disp_cam, f"ERR: {error_px:+d}px | {source_name} | {ser_tag}",
                                (14, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
                    
                    light_str = f"ADAPTIVE (Amb:{ambient_val:.0f} Clip:{self.clahe_clip:.1f} Smin:{dyn_s})" if self.use_adaptive else "STATIC HSV"
                    light_col = (80, 255, 120) if self.use_adaptive else (120, 180, 255)
                    cv2.putText(disp_cam, f"LIGHT: {light_str} | SBC: {self.sbc.proc_w}x{self.sbc.proc_h}",
                                (14, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.38, light_col, 1)
                                
                    # 8. Render Digital Twin Panel
                    panel = s4.render_top_down_panel(
                        panel_w, panel_h, decision, dec_color, error_px,
                        speed_l, speed_r, self.offset_l, self.offset_r,
                        self.pid, edge_status, fwd_density, self.base_rpm, self.frame_count
                    )
                    
                    # 9. Render Vegetation Mask View
                    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    cv2.rectangle(mask_colored, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 255), 2)
                    disp_mask = cv2.resize(mask_colored, (disp_cam_w, disp_cam_h))
                    cv2.putText(disp_mask, f"VEGETATION MASK [{light_str}]", (14, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 1)
                                
                    # 10. Combined Dashboard View
                    dashboard = np.hstack([disp_cam, panel])
                    
                    # 11. Pace execution to prevent thermal throttling on Tinker Board
                    actual_fps = self.sbc.pace_frame()
                    
                    # 12. Update Telemetry State
                    self.telemetry = {
                        "status": edge_status,
                        "decision": decision,
                        "decision_color": [int(dec_color[2]), int(dec_color[1]), int(dec_color[0])], # RGB for CSS
                        "error_px": int(error_px),
                        "speed_l": int(speed_l),
                        "speed_r": int(speed_r),
                        "base_rpm": int(self.base_rpm),
                        "pid": {
                            "kp": float(self.pid.kp),
                            "ki": float(self.pid.ki),
                            "kd": float(self.pid.kd),
                            "p_term": float(round(self.pid.p_term, 1)),
                            "i_term": float(round(self.pid.i_term, 1)),
                            "d_term": float(round(self.pid.d_term, 1)),
                            "output": float(round(self.pid.output, 1))
                        },
                        "lighting": {
                            "mode": "ADAPTIVE" if self.use_adaptive else "STATIC",
                            "ambient": float(round(ambient_val, 1)),
                            "clahe_clip": float(round(self.clahe_clip, 1)),
                            "s_min": int(dyn_s),
                            "v_min": int(dyn_v)
                        },
                        "edge": {
                            "status": edge_status,
                            "density": float(round(fwd_density, 1)),
                            "edge_y": int(edge_y) if edge_y is not None else None
                        },
                        "tiller": {
                            "active": bool(abs((speed_l + speed_r) / 2.0) > 15),
                            "rpm": int(abs((speed_l + speed_r) / 2.0) * 4.2) if abs((speed_l + speed_r) / 2.0) > 15 else 0
                        },
                        "roi": {
                            "top": self.top_pct,
                            "bottom": self.bottom_pct,
                            "left": self.left_pct,
                            "right": self.right_pct
                        },
                        "video": os.path.basename(str(self.current_source)),
                        "active_device": str(self.current_source),
                        "available_devices": self.devices,
                        "paused": self.paused,
                        "fps": float(round(actual_fps, 1)),
                        "serial": self.motor_controller.get_status(),
                        "sbc": {
                            "profile": self.sbc.profile_key,
                            "name": self.sbc.profile["name"],
                            "proc_w": self.sbc.proc_w,
                            "proc_h": self.sbc.proc_h,
                            "target_fps": self.sbc.target_fps,
                            "desc": self.sbc.profile["desc"]
                        }
                    }
                    
                    # 13. Encode JPEGs with optimized compression
                    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 72]
                    _, self.jpeg_combined = cv2.imencode('.jpg', dashboard, encode_params)
                    _, self.jpeg_camera = cv2.imencode('.jpg', disp_cam, encode_params)
                    _, self.jpeg_mask = cv2.imencode('.jpg', disp_mask, encode_params)
                    
                    # Notify streaming clients
                    self.condition.notify_all()
            else:
                time.sleep(0.08)


# Global Engine Instance
engine = None

@app.route('/')
def index():
    """Serves the main mobile-friendly rover dashboard."""
    return render_template('index.html')

def frame_generator(view='combined'):
    """Generates an HTTP multipart MJPEG stream for client browsers."""
    global engine
    while True:
        with engine.condition:
            engine.condition.wait(timeout=0.2)
            if view == 'camera':
                buf = engine.jpeg_camera
            elif view == 'mask':
                buf = engine.jpeg_mask
            else:
                buf = engine.jpeg_combined
                
        if buf is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Returns low-latency MJPEG video stream."""
    view = request.args.get('view', 'combined')
    return Response(frame_generator(view),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/telemetry')
def get_telemetry():
    """REST JSON endpoint providing high-frequency vehicle telemetry."""
    global engine
    with engine.lock:
        data = dict(engine.telemetry)
    return jsonify(data)

@app.route('/api/control', methods=['POST'])
def update_control():
    """Receives remote control commands and parameter adjustments."""
    global engine
    payload = request.get_json(force=True)
    if not payload:
        return jsonify({"status": "error", "message": "No JSON payload"}), 400
        
    for k, v in payload.items():
        if k == "select_device":
            engine.switch_device(v)
        elif k == "sbc_profile":
            engine.set_sbc_profile(v)
        else:
            engine.update_control(k, v)
            
    return jsonify({"status": "ok", "telemetry": engine.telemetry})

# --- HARDWARE & SERIAL LINK API ENDPOINTS ---

@app.route('/api/devices', methods=['GET'])
def get_devices():
    """Returns all available video capture devices (cameras and test videos)."""
    global engine
    devs = vdm.scan_available_devices()
    return jsonify({
        "status": "ok",
        "devices": devs,
        "active_device": str(engine.current_source) if engine else None
    })

@app.route('/api/device/select', methods=['POST'])
def select_device():
    """Switches active capture device to specified camera index or video file."""
    global engine
    payload = request.get_json(force=True) or {}
    device_id = payload.get("device_id")
    if device_id is not None:
        engine.switch_device(device_id)
        return jsonify({"status": "ok", "active_device": str(engine.current_source)})
    return jsonify({"status": "error", "message": "Missing device_id"}), 400

@app.route('/api/serial/ports', methods=['GET'])
def get_serial_ports():
    """Returns list of detected serial ports on the host system."""
    ports = smc.list_serial_ports()
    return jsonify({"status": "ok", "ports": ports})

@app.route('/api/serial/connect', methods=['POST'])
def connect_serial():
    """Connects to motor controller on specified port and baudrate."""
    global engine
    payload = request.get_json(force=True) or {}
    port = payload.get("port", "/dev/ttyUSB0")
    baudrate = payload.get("baudrate", 115200)
    success, msg = engine.motor_controller.connect(port, baudrate)
    return jsonify({
        "status": "ok" if success else "error",
        "message": msg,
        "serial": engine.motor_controller.get_status()
    })

@app.route('/api/serial/disconnect', methods=['POST'])
def disconnect_serial():
    """Safely closes serial motor link."""
    global engine
    engine.motor_controller.disconnect()
    return jsonify({"status": "ok", "serial": engine.motor_controller.get_status()})

@app.route('/api/serial/config', methods=['POST'])
def update_serial_config():
    """Updates motor driver configuration (PWM min/max, invert, protocol)."""
    global engine
    payload = request.get_json(force=True) or {}
    engine.motor_controller.update_config(payload)
    return jsonify({"status": "ok", "serial": engine.motor_controller.get_status()})

@app.route('/api/serial/estop', methods=['POST'])
def emergency_stop():
    """Emergency Stop cutoff."""
    global engine
    payload = request.get_json(force=True) or {}
    action = payload.get("action", "trigger")
    if action == "reset":
        engine.motor_controller.reset_estop()
    else:
        engine.motor_controller.emergency_stop()
    return jsonify({"status": "ok", "serial": engine.motor_controller.get_status()})

@app.route('/api/serial/test_drive', methods=['POST'])
def test_drive():
    """Executes momentary benchtop motor test motion."""
    global engine
    payload = request.get_json(force=True) or {}
    command = payload.get("command", "stop")
    pwml, pwmr, packet = engine.motor_controller.manual_test_drive(command)
    return jsonify({
        "status": "ok",
        "command": command,
        "pwm_l": pwml,
        "pwm_r": pwmr,
        "packet": packet.strip()
    })

@app.route('/api/sbc/profile', methods=['POST'])
def set_sbc_profile():
    """Switches SBC optimization profile."""
    global engine
    payload = request.get_json(force=True) or {}
    profile = payload.get("profile", "tinker_board")
    engine.set_sbc_profile(profile)
    return jsonify({"status": "ok", "sbc": engine.telemetry["sbc"]})


def is_port_in_use(port, host='127.0.0.1'):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex((host, port)) == 0


def main():
    parser = argparse.ArgumentParser(description="Autonomous Agricultural Rover - Web Remote Control")
    parser.add_argument("--video", type=str, default=None, help="Initial video file or camera device index (e.g. 0, /dev/video0)")
    parser.add_argument("--serial-port", type=str, default=None, help="Serial port for motor driver (e.g. /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyS1)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument("--sbc-profile", type=str, default="tinker_board", choices=["tinker_board", "balanced", "desktop"],
                        help="Performance optimization profile (default: tinker_board)")
    parser.add_argument("--port", type=int, default=5001, help="Web server listening port (default: 5001)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Web server host binding (default: 0.0.0.0)")
    args = parser.parse_args()

    target_port = args.port
    if is_port_in_use(target_port):
        for alt_port in [5001, 5000, 8080, 8081, 8888]:
            if not is_port_in_use(alt_port):
                print(f"[INFO] Port {target_port} is busy. Automatically using port {alt_port} instead.")
                target_port = alt_port
                break

    global engine
    engine = RoverVisionEngine(
        initial_video=args.video,
        sbc_profile=args.sbc_profile,
        serial_port=args.serial_port,
        baudrate=args.baudrate
    )
    
    # Launch vision processing in a background worker thread
    worker_thread = threading.Thread(target=engine.run_worker, daemon=True)
    worker_thread.start()
    
    local_ip = get_local_ip()
    
    print("\n" + "=" * 64)
    print("  🌾 Autonomous Agricultural Rover - Web Cockpit & Motor Link 🌾")
    print("=" * 64)
    print(f"  📱 Phone / Tablet Access : http://{local_ip}:{target_port}")
    print(f"  💻 Local Computer Access : http://localhost:{target_port}")
    print("-" * 64)
    print(f"  ⚡ SBC Profile           : {engine.sbc.profile['name']}")
    print(f"  🔌 Serial Motor Driver   : {engine.motor_controller.cfg['port']} @ {engine.motor_controller.cfg['baudrate']} baud")
    print("  Press Ctrl+C in terminal to stop server.")
    print("=" * 64 + "\n")
    
    # Run Flask server (threaded=True for concurrent stream + telemetry clients)
    app.run(host=args.host, port=target_port, threaded=True, debug=False)


if __name__ == "__main__":
    main()
