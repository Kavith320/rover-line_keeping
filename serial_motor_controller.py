"""
Agricultural Rover - Serial Motor Driver Controller
===================================================
Manages serial communication with motor driver microcontrollers
(Arduino, ESP32, STM32, Teensy, Sabertooth, Hoverboard controller)
over USB-Serial (/dev/ttyUSB0, /dev/ttyACM0) or hardware UART (/dev/ttyS1)
on Single Board Computers like the ASUS Tinker Board or Raspberry Pi.

Features:
  - Dynamic serial port discovery and custom port/baudrate selection
  - Differential drive speed to hardware PWM conversion
  - Configurable PWM limits (min deadband, max limit) and motor direction inverting
  - Multiple packet protocols: Delimited bracket <L,R>, Key-Value L:...,R:..., JSON
  - Real-time watchdog & failsafe (stops motors if vision frames stall)
  - Emergency Stop (E-Stop) cutoff
  - Benchtop test drive functions (Forward, Reverse, Spin, Stop)
  - Simulated fallback mode when no serial hardware is connected
  - Persistent configuration storage in hardware_config.json
"""

import os
import sys
import time
import json
import threading
import glob

# Gracefully import pyserial with simulated fallback
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False


HARDWARE_CONFIG_FILE = "hardware_config.json"


def list_serial_ports():
    """
    Scans the host system for available serial ports.
    Supports Linux (Asus Tinker Board, Raspberry Pi), macOS, and Windows.
    Returns a list of dicts: [{"port": "/dev/ttyUSB0", "description": "..."}, ...]
    """
    detected_ports = []
    
    if SERIAL_AVAILABLE and serial and hasattr(serial, 'tools') and hasattr(serial.tools, 'list_ports'):
        try:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                desc = p.description if p.description else "Serial Device"
                detected_ports.append({
                    "port": p.device,
                    "description": f"{p.device} ({desc})"
                })
        except Exception as e:
            print(f"[WARN] Error scanning serial ports via pyserial: {e}")

    # Check for Raspberry Pi GPIO primary UART symlinks
    for rpi_port in ["/dev/serial0", "/dev/serial1"]:
        if os.path.exists(rpi_port) and not any(dp["port"] == rpi_port for dp in detected_ports):
            detected_ports.append({
                "port": rpi_port,
                "description": f"{rpi_port} (Raspberry Pi GPIO UART)"
            })

    # Fallback scanning on Linux / Tinker Board / macOS if pyserial list_ports returned empty
    if not detected_ports:
        # Common Linux / Tinker Board / Raspberry Pi device nodes
        patterns = [
            "/dev/serial*",
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/ttyS[1-4]*",    # Tinker Board UART1-UART4
            "/dev/ttyAMA*",
            "/dev/cu.usb*",
            "/dev/cu.wch*",
            "/dev/tty.usb*"
        ]
        found_paths = []
        for pat in patterns:
            found_paths.extend(glob.glob(pat))
        
        for path in sorted(found_paths):
            detected_ports.append({
                "port": path,
                "description": f"{path} (Hardware Port)"
            })

    # If still empty, add common device names for user convenience
    if not detected_ports:
        detected_ports = [
            {"port": "/dev/serial0", "description": "/dev/serial0 (Raspberry Pi GPIO UART)"},
            {"port": "/dev/ttyUSB0", "description": "/dev/ttyUSB0 (Standard USB-Serial)"},
            {"port": "/dev/ttyACM0", "description": "/dev/ttyACM0 (Arduino Uno/Mega/Micro)"},
            {"port": "/dev/ttyS1", "description": "/dev/ttyS1 (Tinker Board UART1)"},
        ]
        
    return detected_ports


def load_hardware_config():
    """Loads serial and motor hardware configuration from JSON file."""
    defaults = {
        "port": "/dev/serial0",
        "baudrate": 115200,
        "auto_connect": True,
        "pwm_min": 35,          # Minimum PWM to overcome static friction / deadband
        "pwm_max": 255,         # Maximum allowable PWM (0-255)
        "invert_left": False,   # Invert Left motor direction
        "invert_right": False,  # Invert Right motor direction
        "bidirectional": True,  # True allows negative PWM (-255 to +255 for reverse)
        "protocol": "bracket",  # 'bracket' (<L,R>), 'text' (L:...,R:...), 'json'
        "watchdog_timeout": 0.5 # Seconds before stopping if no fresh command
    }
    if os.path.exists(HARDWARE_CONFIG_FILE):
        try:
            with open(HARDWARE_CONFIG_FILE, "r") as f:
                saved = json.load(f)
                defaults.update(saved)
        except Exception as e:
            print(f"[WARN] Could not read {HARDWARE_CONFIG_FILE}: {e}")
    return defaults


def save_hardware_config(cfg):
    """Saves serial and motor hardware configuration to JSON file."""
    try:
        with open(HARDWARE_CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
        print(f"[INFO] Hardware configuration saved to {HARDWARE_CONFIG_FILE}")
        return True
    except Exception as e:
        print(f"[ERROR] Failed to save {HARDWARE_CONFIG_FILE}: {e}")
        return False


class SerialMotorController:
    """
    Thread-safe controller that communicates with motor driver hardware.
    Handles speed-to-PWM conversion, safety watchdogs, and packet transmission.
    """
    def __init__(self, port=None, baudrate=115200, auto_connect=False):
        self.lock = threading.Lock()
        self.cfg = load_hardware_config()
        
        if port:
            self.cfg["port"] = port
        if baudrate:
            self.cfg["baudrate"] = int(baudrate)
        if auto_connect:
            self.cfg["auto_connect"] = bool(auto_connect)

        self.ser = None
        self.is_open = False
        self.simulated_mode = not SERIAL_AVAILABLE
        self.last_tx_time = time.time()
        self.last_tx_packet = "<0,0>"
        self.last_pwm_l = 0
        self.last_pwm_r = 0
        self.emergency_stopped = False
        # SAFETY FIRST: Motors and autonomous tracking are DISABLED by default on application start
        self.motors_enabled = False
        self.tracking_enabled = False

        # Bidirectional Telemetry Feedback from Microcontroller
        self.last_rx_packet = ""
        self.last_rx_time = 0
        self.handshake_verified = False
        self.rx_thread = None

        # Watchdog monitor thread
        self.running = True
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        if self.cfg.get("auto_connect", False):
            self.auto_connect_hardware()

    def auto_connect_hardware(self, preferred_port=None, preferred_baud=None):
        """
        Intelligently scans available hardware serial ports (Raspberry Pi GPIO UART /dev/serial0,
        USB Serial /dev/ttyUSB0, /dev/ttyACM0, etc.) and connects to the first valid port.
        Saves the successful port to config.
        """
        target_baud = int(preferred_baud) if preferred_baud else self.cfg.get("baudrate", 115200)
        target_port = preferred_port if preferred_port else self.cfg.get("port", "/dev/serial0")

        # 1. Try preferred or configured port first
        if target_port:
            success, msg = self.connect(target_port, target_baud)
            if success and not self.simulated_mode:
                print(f"[AUTO-CONNECT] Connected successfully to target port: {target_port} @ {target_baud} baud.")
                return True, msg

        # 2. If configured port failed and pyserial is available, scan available hardware ports
        if SERIAL_AVAILABLE:
            detected = list_serial_ports()
            # Prioritize Raspberry Pi GPIO /dev/serial0 and USB ports
            candidate_ports = []
            for p in detected:
                dev = p["port"]
                if dev != target_port and dev not in candidate_ports:
                    candidate_ports.append(dev)

            for port in candidate_ports:
                print(f"[AUTO-CONNECT] Probing serial port candidate: {port}...")
                success, msg = self.connect(port, target_baud)
                if success and not self.simulated_mode:
                    print(f"[AUTO-CONNECT] Successfully auto-connected to {port} @ {target_baud} baud!")
                    self.cfg["port"] = port
                    save_hardware_config(self.cfg)
                    return True, f"Auto-connected to {port}"

        # 3. Fallback to configured port in simulated mode if hardware port couldn't be opened
        return self.connect(target_port or "/dev/serial0", target_baud)

    def connect(self, port, baudrate):
        """Attempts to open serial connection to the motor controller."""
        with self.lock:
            self.disconnect_internal()
            self.cfg["port"] = str(port).strip()
            self.cfg["baudrate"] = int(baudrate)
            self.emergency_stopped = False

            if not SERIAL_AVAILABLE:
                self.simulated_mode = True
                self.is_open = True
                print(f"[INFO] PySerial not installed. Running in SIMULATED Serial mode on {self.cfg['port']} @ {self.cfg['baudrate']} baud.")
                # Transmit initial safe disabled state
                self._raw_send("<0,0>\n")
                self._raw_send("<TRACKING_DISABLED>\n")
                return True, "Simulated serial mode (pyserial not installed)"

            try:
                self.ser = serial.Serial(
                    port=self.cfg["port"],
                    baudrate=self.cfg["baudrate"],
                    timeout=0.1,
                    write_timeout=0.1
                )
                # CRITICAL FOR ESP32 & ARDUINO:
                # Explicitly clear DTR and RTS so the ESP32 is not held in reset or bootloader
                try:
                    self.ser.dtr = False
                    self.ser.rts = False
                except Exception:
                    pass

                self.is_open = True
                self.simulated_mode = False
                self.handshake_verified = False

                # Launch asynchronous RX reader thread immediately to capture boot messages
                if self.rx_thread is None or not self.rx_thread.is_alive():
                    self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                    self.rx_thread.start()

                # Allow ESP32 FreeRTOS / Arduino bootloader settling (1.5 seconds)
                time.sleep(1.5)

                # Clear any startup bootloader garbage
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass

                print(f"[SUCCESS] Connected to Motor Controller on {self.cfg['port']} @ {self.cfg['baudrate']} baud.")
                # Transmit initial safe disabled status to microcontroller
                self._raw_send("<0,0>\n")
                if not self.tracking_enabled:
                    self._raw_send("<TRACKING_DISABLED>\n")
                else:
                    self._raw_send("<TRACKING_ENABLED>\n")
                return True, f"Connected to {self.cfg['port']}"
            except Exception as e:
                self.ser = None
                self.is_open = False
                self.simulated_mode = True
                self.handshake_verified = False
                err_msg = f"Failed to open {self.cfg['port']}: {e}. Switched to SIMULATED mode."
                print(f"[WARN] {err_msg}")
                return False, err_msg

    def _rx_loop(self):
        """
        Dedicated background thread continuously reading serial responses & telemetry from microcontroller.
        Parses acknowledgments (ACK:...), status handshakes, and diagnostic warnings.
        """
        while self.running:
            if self.ser is not None and self.is_open and not self.simulated_mode:
                try:
                    raw_line = self.ser.readline()
                    if raw_line:
                        line = raw_line.decode("utf-8", errors="ignore").strip()
                        if line:
                            self.last_rx_time = time.time()
                            self.last_rx_packet = line
                            if any(keyword in line for keyword in ["ACK:", "AGRI-ROVER", "STATUS:", "PONG", "READY"]):
                                self.handshake_verified = True
                            if "ERR:" in line or "WARN:" in line:
                                print(f"[MICROCONTROLLER ALERT] << {line}")
                            elif "AGRI-ROVER" in line or "ACK:TRACKING" in line:
                                print(f"[MICROCONTROLLER CONFIRM] << {line}")
                except Exception:
                    pass
            time.sleep(0.01)

    def ping_controller(self):
        """Sends a ping handshake to verify bidirectional connection with microcontroller."""
        with self.lock:
            if not self.is_open or self.simulated_mode:
                return False, "Not connected to physical hardware"
            self.last_rx_packet = ""
            self._raw_send("<PING>\n")
        
        # Wait up to 300ms for pong or ack
        start_wait = time.time()
        while time.time() - start_wait < 0.3:
            if "PONG" in self.last_rx_packet or "ACK:" in self.last_rx_packet:
                self.handshake_verified = True
                return True, f"Microcontroller verified: {self.last_rx_packet}"
            time.sleep(0.02)
        return False, "No response to PING within 300ms"

    def disconnect_internal(self):
        """Internal helper to close port without acquiring lock again."""
        self.handshake_verified = False
        if self.ser is not None:
            try:
                # Send stop before closing
                self._raw_send("<0,0>\n")
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.is_open = False

    def disconnect(self):
        """Safely disconnects from the serial port."""
        with self.lock:
            self.disconnect_internal()
            print("[INFO] Serial motor link disconnected.")
            return True

    def get_status(self):
        """Returns the current state of the serial motor controller."""
        with self.lock:
            base_state = "CONNECTED" if (self.is_open and not self.simulated_mode) else ("SIMULATED" if self.is_open else "DISCONNECTED")
            if not self.tracking_enabled or not self.motors_enabled:
                state = f"{base_state} (TRACKING DISABLED)"
            else:
                state = f"{base_state} (ACTIVE)"

            return {
                "connected": self.is_open,
                "simulated": self.simulated_mode,
                "status_text": state,
                "port": self.cfg.get("port", "/dev/serial0"),
                "baudrate": self.cfg.get("baudrate", 115200),
                "auto_connect": self.cfg.get("auto_connect", True),
                "last_tx": self.last_tx_packet,
                "last_rx": self.last_rx_packet,
                "last_rx_time": self.last_rx_time,
                "handshake_verified": self.handshake_verified,
                "pwm_l": self.last_pwm_l,
                "pwm_r": self.last_pwm_r,
                "motors_enabled": self.motors_enabled,
                "tracking_enabled": self.tracking_enabled,
                "emergency_stopped": self.emergency_stopped,
                "pwm_min": self.cfg.get("pwm_min", 35),
                "pwm_max": self.cfg.get("pwm_max", 255),
                "invert_l": self.cfg.get("invert_left", False),
                "invert_r": self.cfg.get("invert_right", False),
                "protocol": self.cfg.get("protocol", "bracket"),
                "pyserial_installed": SERIAL_AVAILABLE
            }

    def update_config(self, new_cfg):
        """Updates motor controller parameters and saves to disk."""
        with self.lock:
            for k, v in new_cfg.items():
                if k in self.cfg:
                    self.cfg[k] = v
            save_hardware_config(self.cfg)

    def speed_to_pwm(self, speed_rpm, is_left=True):
        """
        Converts RPM differential speed (-160 to +160) to a scaled hardware PWM value.
        Applies deadband minimum PWM and maximum clamp.
        """
        if not self.motors_enabled or self.emergency_stopped:
            return 0

        # Sign and magnitude
        direction = 1 if speed_rpm >= 0 else -1
        mag = abs(speed_rpm)

        if mag < 5:
            # Below idle noise threshold
            return 0

        # Scale speed from [0, 160] RPM to [pwm_min, pwm_max]
        pwm_min = self.cfg.get("pwm_min", 35)
        pwm_max = self.cfg.get("pwm_max", 255)
        
        ratio = min(1.0, mag / 160.0)
        scaled_pwm = int(pwm_min + ratio * (pwm_max - pwm_min))
        scaled_pwm = max(0, min(pwm_max, scaled_pwm))

        # Check inversion
        invert = self.cfg.get("invert_left", False) if is_left else self.cfg.get("invert_right", False)
        if invert:
            direction = -direction

        if not self.cfg.get("bidirectional", True):
            # If unidirectional, clamp to positive only
            return scaled_pwm if direction > 0 else 0

        return direction * scaled_pwm

    def format_packet(self, pwm_l, pwm_r):
        """Formats the serial packet according to the chosen protocol."""
        proto = self.cfg.get("protocol", "bracket")
        if proto == "text":
            return f"L:{pwm_l},R:{pwm_r}\n"
        elif proto == "json":
            return json.dumps({"l": pwm_l, "r": pwm_r}) + "\n"
        else: # 'bracket' default: <pwm_l,pwm_r>\n
            return f"<{pwm_l},{pwm_r}>\n"

    def format_status_packet(self, status_str):
        """Formats high-level status / mode commands to microcontroller."""
        proto = self.cfg.get("protocol", "bracket")
        if proto == "json":
            return json.dumps({"cmd": status_str}) + "\n"
        elif proto == "text":
            return f"CMD:{status_str}\n"
        else: # 'bracket' default: <status_str>\n
            return f"<{status_str}>\n"

    def _raw_send(self, packet_str):
        """Sends raw string over serial or updates simulation buffer."""
        self.last_tx_time = time.time()
        self.last_tx_packet = packet_str.strip()
        
        if self.ser is not None and self.is_open and not self.simulated_mode:
            try:
                self.ser.write(packet_str.encode("utf-8"))
                self.ser.flush()
            except Exception as e:
                print(f"[WARN] Serial write error: {e}")
                self.simulated_mode = True

    def enable_tracking(self):
        """Enables autonomous tracking and active motor commanding."""
        with self.lock:
            self.tracking_enabled = True
            self.motors_enabled = True
            self.emergency_stopped = False
            packet = self.format_status_packet("TRACKING_ENABLED")
            self._raw_send(packet)
            print("[SERIAL] >> TRACKING ENABLED: Motor commanding is now active.")
            return True

    def disable_tracking(self):
        """Disables autonomous tracking and safely halts motors."""
        with self.lock:
            self.tracking_enabled = False
            self.motors_enabled = False
            self.last_pwm_l = 0
            self.last_pwm_r = 0
            # Send immediate stop packet
            self._raw_send(self.format_packet(0, 0))
            # Send tracking disabled status notification packet
            packet = self.format_status_packet("TRACKING_DISABLED")
            self._raw_send(packet)
            print("[SERIAL] >> TRACKING DISABLED: Sent stop & TRACKING_DISABLED via serial.")
            return False

    def toggle_tracking(self, enabled=None):
        """Toggles or sets the tracking enable state."""
        with self.lock:
            target = not self.tracking_enabled if enabled is None else bool(enabled)
        if target:
            return self.enable_tracking()
        else:
            return self.disable_tracking()

    def send_differential_drive(self, speed_l, speed_r):
        """
        Translates Left & Right wheel speeds into hardware PWM and transmits over serial.
        Called by vision engine on each processed frame.
        Guarantees 0 PWM if tracking is disabled or emergency stopped.
        """
        with self.lock:
            if self.emergency_stopped or not self.motors_enabled or not self.tracking_enabled:
                pwm_l, pwm_r = 0, 0
            else:
                pwm_l = self.speed_to_pwm(speed_l, is_left=True)
                pwm_r = self.speed_to_pwm(speed_r, is_left=False)

            self.last_pwm_l = pwm_l
            self.last_pwm_r = pwm_r

            packet = self.format_packet(pwm_l, pwm_r)
            # Only transmit differential drive packets if tracking is enabled or if stopping
            if self.tracking_enabled and self.motors_enabled and not self.emergency_stopped:
                self._raw_send(packet)
            return pwm_l, pwm_r, packet

    def emergency_stop(self):
        """Immediately halts all motor output and disables tracking."""
        with self.lock:
            self.emergency_stopped = True
            self.tracking_enabled = False
            self.motors_enabled = False
            self.last_pwm_l = 0
            self.last_pwm_r = 0
            self._raw_send(self.format_packet(0, 0))
            self._raw_send(self.format_status_packet("TRACKING_DISABLED"))
            print("[EMERGENCY STOP] All rover motors cut to 0 PWM & tracking disabled!")
            return True

    def reset_estop(self):
        """Clears the emergency stop latch (leaves tracking in standby until user presses Run)."""
        with self.lock:
            self.emergency_stopped = False
            print("[INFO] Emergency stop cleared. Motors in STANDBY (Press RUN to start tracking).")
            return True

    def toggle_motors(self, enabled=None):
        """Enables or disables motor power commands."""
        return self.toggle_tracking(enabled)

    def manual_test_drive(self, command):
        """
        Executes manual test motions for benchtop verification.
        Commands: 'forward', 'reverse', 'spin_left', 'spin_right', 'stop'
        """
        with self.lock:
            test_pwm = int(self.cfg.get("pwm_max", 255) * 0.55) # 55% power
            self.emergency_stopped = False
            self.motors_enabled = True

            if command == "forward":
                pwm_l, pwm_r = test_pwm, test_pwm
            elif command == "reverse":
                pwm_l, pwm_r = -test_pwm, -test_pwm
            elif command == "spin_left":
                pwm_l, pwm_r = -test_pwm, test_pwm
            elif command == "spin_right":
                pwm_l, pwm_r = test_pwm, -test_pwm
            else: # 'stop'
                pwm_l, pwm_r = 0, 0

            self.last_pwm_l = pwm_l
            self.last_pwm_r = pwm_r
            packet = self.format_packet(pwm_l, pwm_r)
            self._raw_send(packet)
            return pwm_l, pwm_r, packet

    def _watchdog_loop(self):
        """
        Safety watchdog: if no new drive command has been sent within
        watchdog_timeout seconds, automatically transmits a stop command.
        Prevents runaway rover if camera, OS, or computer vision hangs.
        """
        while self.running:
            time.sleep(0.1)
            timeout = self.cfg.get("watchdog_timeout", 0.5)
            with self.lock:
                if self.is_open and not self.emergency_stopped:
                    if (time.time() - self.last_tx_time) > timeout:
                        if self.last_pwm_l != 0 or self.last_pwm_r != 0:
                            self.last_pwm_l = 0
                            self.last_pwm_r = 0
                            self._raw_send(self.format_packet(0, 0))
                            # print("[WATCHDOG] Frame timeout detected - Stopped motors for safety.")

    def close(self):
        """Cleans up threads and closes serial link."""
        self.running = False
        self.disconnect()


# Singleton instance for simple global access
_global_controller = None

def get_motor_controller():
    global _global_controller
    if _global_controller is None:
        _global_controller = SerialMotorController()
    return _global_controller
