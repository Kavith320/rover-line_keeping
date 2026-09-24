"""
Agricultural Rover - Video Capture Device Manager
=================================================
Manages camera devices and video sources for the autonomous navigation platform.
Optimized for Single Board Computers like the ASUS Tinker Board (Rockchip RK3288),
Raspberry Pi, and desktop environments.

Key Capabilities:
  - Scans and detects all connected hardware camera devices (/dev/video*, indices 0, 1, 2...)
  - Discovers simulation and recorded test videos in data/
  - Zero-Latency Threaded Video Capture: Eliminates the 3-5 frame V4L2 hardware queue
    latency on Linux/Tinker Board so navigation operates strictly on fresh real-time frames
  - Hardware MJPEG format negotiation for fast USB camera streaming without CPU choking
  - Dynamic capture source hot-swapping on the fly
"""

import os
import sys
import glob
import time
import threading
import cv2


def scan_available_devices(data_dir="data"):
    """
    Scans system for both live cameras and simulation video files.
    Returns a unified list of selectable sources.
    """
    devices = []
    
    # 1. Scan for Linux / Tinker Board V4L2 video nodes
    v4l2_nodes = sorted(glob.glob("/dev/video*"))
    for node in v4l2_nodes:
        devices.append({
            "id": node,
            "name": f"📷 {node} (Linux V4L2 / CSI)",
            "type": "camera",
            "is_live": True
        })

    # Add standard hardware camera indices for USB/CSI cameras
    if not v4l2_nodes:
        devices.append({
            "id": "0",
            "name": "📷 Camera 0 (Primary USB / Webcam)",
            "type": "camera",
            "is_live": True
        })
        devices.append({
            "id": "1",
            "name": "📷 Camera 1 (Secondary USB / CSI)",
            "type": "camera",
            "is_live": True
        })

    # 2. Scan recorded test video files in data/
    if os.path.exists(data_dir):
        video_exts = (".mp4", ".avi", ".mov", ".mkv")
        for f in sorted(os.listdir(data_dir)):
            if f.lower().endswith(video_exts):
                path = os.path.join(data_dir, f)
                devices.append({
                    "id": path,
                    "name": f"🎥 {f} (Recorded)",
                    "type": "file",
                    "is_live": False
                })

    return devices


class ZeroLatencyCapture:
    """
    Dedicated threaded camera reader.
    Continuously drains the camera hardware queue in a background thread
    so the main computer vision loop always receives the freshest available frame
    with ZERO queue lag.
    """
    def __init__(self, source, width=640, height=480, fps=30, is_sbc=True):
        self.source = source
        self.width = width
        self.height = height
        self.target_fps = fps
        self.is_sbc = is_sbc
        
        self.lock = threading.Lock()
        self.cap = None
        self.latest_frame = None
        self.latest_ret = False
        self.running = False
        self.thread = None
        self.is_camera = False

        self._open_capture()

    def _open_capture(self):
        """Initializes the underlying cv2.VideoCapture."""
        # Convert numeric string to integer for camera index
        source_val = self.source
        if isinstance(source_val, str) and source_val.isdigit():
            source_val = int(source_val)

        self.is_camera = isinstance(source_val, int) or (isinstance(source_val, str) and source_val.startswith("/dev/video"))

        # Open capture
        if sys.platform.startswith("linux") and self.is_camera:
            self.cap = cv2.VideoCapture(source_val, cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(source_val)

        if not self.cap or not self.cap.isOpened():
            print(f"[WARN] Failed to open capture source: {self.source}")
            return False

        # If it is a live camera, configure resolution and MJPEG for low SBC CPU usage
        if self.is_camera:
            try:
                # Ask for hardware MJPEG compression to relieve USB bus and CPU
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                # Keep internal buffer small
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception as e:
                print(f"[DEBUG] VideoCapture properties set note: {e}")

        # Read first frame
        self.latest_ret, self.latest_frame = self.cap.read()

        # Start thread
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        return True

    def _reader_loop(self):
        """Continuously pulls frames as fast as hardware delivers."""
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.05)
                continue

            ret, frame = self.cap.read()
            if not ret:
                if not self.is_camera:
                    # Loop video file for continuous simulation
                    with self.lock:
                        if self.cap is not None:
                            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    time.sleep(0.01)
                    continue
                else:
                    time.sleep(0.02)
                    continue

            with self.lock:
                self.latest_ret = ret
                self.latest_frame = frame

            if not self.is_camera:
                # Pace recorded video to avoid runaway playback
                time.sleep(1.0 / max(10, self.target_fps))

    def read(self):
        """Returns the freshest frame with zero latency."""
        with self.lock:
            if self.latest_frame is None:
                return False, None
            return self.latest_ret, self.latest_frame.copy()

    def get_fps(self):
        """Returns hardware reported or configured FPS."""
        if self.cap and self.cap.isOpened():
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            if fps > 0:
                return fps
        return float(self.target_fps)

    def release(self):
        """Stops background reader thread and releases camera."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
        self.latest_frame = None
