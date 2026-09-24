"""
Agricultural Rover - Single Board Computer (SBC) Optimizer
===========================================================
Dedicated performance tuning and compute optimization suite designed
specifically for the ASUS Tinker Board (Rockchip RK3288 Quad-Core ARM Cortex-A17),
Raspberry Pi, and other embedded Linux platforms.

Key Techniques:
  1. Resolution Downscaling: Scales vision processing down to 320x240 or 480x360
     for scale-invariant crop row extraction, reducing memory & CPU load by 75-85%.
  2. Lightweight Morphological Filters: Fast 3x3 structuring elements to avoid
     cache thrashing on 32-bit ARM architectures.
  3. Efficient CLAHE Grid: 4x4 or 6x6 local tile histogram equalization.
  4. Adaptive Frame Pacer: Precise sleep timing to prevent thermal throttling
     on passively-cooled SBC enclosures.
  5. Coordinate Projection: Seamlessly transforms ROI and row polynomials between
     low-power processing coordinates and high-res display coordinates.
"""

import cv2
import numpy as np
import time


PROFILES = {
    "tinker_board": {
        "name": "🚀 ASUS Tinker Board (Low Power / Thermal Safe)",
        "proc_w": 320,
        "proc_h": 240,
        "target_fps": 20,
        "clahe_grid": (4, 4),
        "kernel_size": (3, 3),
        "desc": "Ultra-low CPU overhead. Keeps RK3288 cool in field enclosures."
    },
    "balanced": {
        "name": "⚡ Balanced Embedded (480x360 @ 25 FPS)",
        "proc_w": 480,
        "proc_h": 360,
        "target_fps": 25,
        "clahe_grid": (6, 6),
        "kernel_size": (3, 3),
        "desc": "Ideal for active cooling setups or modern SBCs."
    },
    "desktop": {
        "name": "🖥️ Desktop / Full Power (640x480 @ 30 FPS)",
        "proc_w": 640,
        "proc_h": 480,
        "target_fps": 30,
        "clahe_grid": (8, 8),
        "kernel_size": (5, 5),
        "desc": "Maximum precision rendering for development laptops."
    }
}


class SBCOptimizer:
    def __init__(self, profile_key="tinker_board"):
        self.set_profile(profile_key)
        self.last_frame_time = time.time()
        self.actual_fps = 20.0

    def set_profile(self, profile_key):
        if profile_key not in PROFILES:
            profile_key = "tinker_board"
        self.profile_key = profile_key
        self.profile = PROFILES[profile_key]
        self.proc_w = self.profile["proc_w"]
        self.proc_h = self.profile["proc_h"]
        self.target_fps = self.profile["target_fps"]
        self.clahe_grid = self.profile["clahe_grid"]
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.profile["kernel_size"])
        print(f"[INFO] SBC Optimizer active: {self.profile['name']}")

    def prepare_frame(self, frame):
        """
        Downscales the incoming camera frame to the optimized resolution
        for fast classical computer vision processing.
        Returns: (proc_frame, (scale_x, scale_y), (orig_w, orig_h))
        """
        orig_h, orig_w = frame.shape[:2]
        if orig_w == self.proc_w and orig_h == self.proc_h:
            return frame, (1.0, 1.0), (orig_w, orig_h)
            
        proc_frame = cv2.resize(frame, (self.proc_w, self.proc_h), interpolation=cv2.INTER_LINEAR)
        scale_x = orig_w / float(self.proc_w)
        scale_y = orig_h / float(self.proc_h)
        return proc_frame, (scale_x, scale_y), (orig_w, orig_h)

    def pace_frame(self):
        """
        Paces execution to maintain the target frame rate.
        Prevents CPU running at 100% spinlock and keeps SBC thermals safe.
        """
        now = time.time()
        elapsed = now - self.last_frame_time
        target_interval = 1.0 / max(5, self.target_fps)
        sleep_needed = target_interval - elapsed
        if sleep_needed > 0.002:
            time.sleep(sleep_needed)
            
        now_after = time.time()
        dt = max(1e-4, now_after - self.last_frame_time)
        self.actual_fps = 0.9 * self.actual_fps + 0.1 * (1.0 / dt)
        self.last_frame_time = now_after
        return self.actual_fps
