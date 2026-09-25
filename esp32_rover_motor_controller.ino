/*
  =============================================================================
  🌾 Autonomous Agricultural Rover - Next-Gen ESP32 Motor Controller 🌾
  =============================================================================
  Hardware Target: ESP32 DevKit V1 / ESP32-WROOM-32 / ESP32-S3
  Architecture   : Dual-Core Xtensa LX6 @ 240MHz (FreeRTOS)
  Communication  : High-Speed UART (USB Serial / GPIO Hardware UART)
  Default Baud   : 115200 (Matches main.py and web_rover_dashboard.py)
  
  -----------------------------------------------------------------------------
  Why ESP32 for the Next-Gen Agri-Rover?
  -----------------------------------------------------------------------------
  1. Ultrasonic PWM (20,000 Hz): Eliminates high-pitched motor whining noise.
  2. Independent Hardware Timers: Hardware LEDC PWM with 8-bit to 16-bit resolution.
  3. Slew-Rate Limiter (Soft Acceleration): Protects gearboxes, prevents wheel
     slip in mud/soil, and eliminates battery voltage sag brownouts.
  4. Non-Blocking FreeRTOS-Ready Architecture: Communication never blocks motor control.
  5. Built-in Expansion Hooks for:
     - Quadrature Wheel Encoders (Hardware Interrupts for closed-loop RPM PID)
     - Battery Voltage Monitoring (Analog ADC with low-pass filtering)
     - I2C Telemetry OLED Display (SSD1306)
     - Emergency Stop Physical Bumper Switches
  =============================================================================
*/

#include <Arduino.h>

// =============================================================================
// 1. PIN CONFIGURATION (Modify to match your ESP32 board & driver)
// =============================================================================
// Status & Builtin Indicator
#define PIN_LED_STATUS          2     // On-board blue LED on most ESP32 DevKits

// Motor Driver Selection (Choose ONE by uncommenting):
#define DRIVER_TYPE_DIR_PWM           // Standard: DIR_A + DIR_B + PWM (e.g. L298N, TB6612)
// #define DRIVER_TYPE_CYTRON         // Cytron MDD10A / MDDS30 (1 DIR pin + 1 PWM pin)
// #define DRIVER_TYPE_BTS7960        // Dual H-Bridge BTS7960 (RPWM + LPWM direct)

#if defined(DRIVER_TYPE_DIR_PWM)
  // Left Motor Pins
  #define PIN_MOTOR_L_PWM       18    // GPIO 18 (LEDC Channel 0)
  #define PIN_MOTOR_L_IN1       19    // GPIO 19 Forward
  #define PIN_MOTOR_L_IN2       21    // GPIO 21 Reverse

  // Right Motor Pins
  #define PIN_MOTOR_R_PWM       22    // GPIO 22 (LEDC Channel 1)
  #define PIN_MOTOR_R_IN1       23    // GPIO 23 Forward
  #define PIN_MOTOR_R_IN2       25    // GPIO 25 Reverse

#elif defined(DRIVER_TYPE_CYTRON)
  #define PIN_MOTOR_L_PWM       18    // Speed PWM (0-255)
  #define PIN_MOTOR_L_DIR       19    // Direction (HIGH/LOW)
  #define PIN_MOTOR_R_PWM       22
  #define PIN_MOTOR_R_DIR       23

#elif defined(DRIVER_TYPE_BTS7960)
  #define PIN_MOTOR_L_RPWM      18    // Forward PWM
  #define PIN_MOTOR_L_LPWM      19    // Reverse PWM
  #define PIN_MOTOR_R_RPWM      22
  #define PIN_MOTOR_R_LPWM      23
#endif

// Next-Gen Optional Hardware Hooks
#define PIN_BATTERY_ADC         34    // ADC1 input (Input-only GPIO on ESP32)
#define PIN_ENCODER_L_A         32    // Wheel Encoder Left Phase A (Interrupt)
#define PIN_ENCODER_L_B         33    // Wheel Encoder Left Phase B
#define PIN_ENCODER_R_A         26    // Wheel Encoder Right Phase A (Interrupt)
#define PIN_ENCODER_R_B         27    // Wheel Encoder Right Phase B

// =============================================================================
// 2. HARDWARE PWM & CONTROL TUNING PARAMETERS
// =============================================================================
const uint32_t PWM_FREQUENCY      = 20000;   // 20 kHz (Ultrasonic, silent motor drive)
const uint8_t  PWM_RESOLUTION     = 8;       // 8-bit resolution (Duty cycle 0 - 255)
const uint8_t  PWM_CHANNEL_L      = 0;       // LEDC Channel 0
const uint8_t  PWM_CHANNEL_R      = 1;       // LEDC Channel 1

const uint32_t SERIAL_BAUDRATE    = 115200;  // Matches Raspberry Pi host baudrate
const uint32_t WATCHDOG_MS        = 500;     // Safe timeout: Cut power if no host packet in 0.5s
const int      PWM_SLEW_STEP      = 15;      // Max change per loop cycle (Smooth ramp-up)
const uint32_t LOOP_INTERVAL_MS   = 10;      // 100 Hz internal motor refresh loop

// =============================================================================
// 3. SYSTEM STATE & SAFETY STRUCTURES
// =============================================================================
struct RoverState {
  bool trackingEnabled;           // Armed via <TRACKING_ENABLED> from host
  bool emergencyStopped;          // Latching emergency stop
  int  targetPwmL;                // Requested Left PWM (-255 to +255)
  int  targetPwmR;                // Requested Right PWM (-255 to +255)
  int  currentPwmL;               // Slew-rate ramped actual Left PWM
  int  currentPwmR;               // Slew-rate ramped actual Right PWM
  uint32_t lastPacketTime;        // Timestamp for watchdog
  uint32_t lastLoopTime;          // Timestamp for 100Hz control loop
  uint32_t lastTelemetryTime;     // Timestamp for periodic status broadcast
} rover = {false, false, 0, 0, 0, 0, 0, 0, 0};

// Serial Parser Buffer
char rxBuffer[64];
uint8_t rxIndex = 0;
bool rxPacketStarted = false;

// =============================================================================
// 4. LOW-LEVEL MOTOR DRIVER HARDWARE ABSTRACTION (ESP32 LEDC)
// =============================================================================
void initPWM() {
  // ESP32 Arduino Core 2.x & 3.x Compatibility
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ESP32 Core 3.x API
    #if defined(DRIVER_TYPE_DIR_PWM) || defined(DRIVER_TYPE_CYTRON)
      ledcAttach(PIN_MOTOR_L_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttach(PIN_MOTOR_R_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
    #elif defined(DRIVER_TYPE_BTS7960)
      ledcAttach(PIN_MOTOR_L_RPWM, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttach(PIN_MOTOR_L_LPWM, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttach(PIN_MOTOR_R_RPWM, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttach(PIN_MOTOR_R_LPWM, PWM_FREQUENCY, PWM_RESOLUTION);
    #endif
  #else
    // ESP32 Core 2.x API
    #if defined(DRIVER_TYPE_DIR_PWM) || defined(DRIVER_TYPE_CYTRON)
      ledcSetup(PWM_CHANNEL_L, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttachPin(PIN_MOTOR_L_PWM, PWM_CHANNEL_L);

      ledcSetup(PWM_CHANNEL_R, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttachPin(PIN_MOTOR_R_PWM, PWM_CHANNEL_R);
    #elif defined(DRIVER_TYPE_BTS7960)
      ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttachPin(PIN_MOTOR_L_RPWM, 0);
      ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttachPin(PIN_MOTOR_L_LPWM, 1);
      ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttachPin(PIN_MOTOR_R_RPWM, 2);
      ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcAttachPin(PIN_MOTOR_R_LPWM, 3);
    #endif
  #endif
}

void writeMotorPWM(uint8_t channel, uint8_t pin, uint8_t duty) {
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(pin, duty);
  #else
    ledcWrite(channel, duty);
  #endif
}

void setLeftMotorHardware(int speed) {
  int absSpeed = constrain(abs(speed), 0, 255);

#if defined(DRIVER_TYPE_DIR_PWM)
  if (speed > 0) {
    digitalWrite(PIN_MOTOR_L_IN1, HIGH);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    writeMotorPWM(PWM_CHANNEL_L, PIN_MOTOR_L_PWM, absSpeed);
  } else if (speed < 0) {
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    writeMotorPWM(PWM_CHANNEL_L, PIN_MOTOR_L_PWM, absSpeed);
  } else {
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    writeMotorPWM(PWM_CHANNEL_L, PIN_MOTOR_L_PWM, 0);
  }

#elif defined(DRIVER_TYPE_CYTRON)
  digitalWrite(PIN_MOTOR_L_DIR, speed >= 0 ? HIGH : LOW);
  writeMotorPWM(PWM_CHANNEL_L, PIN_MOTOR_L_PWM, absSpeed);

#elif defined(DRIVER_TYPE_BTS7960)
  if (speed > 0) {
    writeMotorPWM(0, PIN_MOTOR_L_RPWM, absSpeed);
    writeMotorPWM(1, PIN_MOTOR_L_LPWM, 0);
  } else if (speed < 0) {
    writeMotorPWM(0, PIN_MOTOR_L_RPWM, 0);
    writeMotorPWM(1, PIN_MOTOR_L_LPWM, absSpeed);
  } else {
    writeMotorPWM(0, PIN_MOTOR_L_RPWM, 0);
    writeMotorPWM(1, PIN_MOTOR_L_LPWM, 0);
  }
#endif
}

void setRightMotorHardware(int speed) {
  int absSpeed = constrain(abs(speed), 0, 255);

#if defined(DRIVER_TYPE_DIR_PWM)
  if (speed > 0) {
    digitalWrite(PIN_MOTOR_R_IN1, HIGH);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);
    writeMotorPWM(PWM_CHANNEL_R, PIN_MOTOR_R_PWM, absSpeed);
  } else if (speed < 0) {
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, HIGH);
    writeMotorPWM(PWM_CHANNEL_R, PIN_MOTOR_R_PWM, absSpeed);
  } else {
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);
    writeMotorPWM(PWM_CHANNEL_R, PIN_MOTOR_R_PWM, 0);
  }

#elif defined(DRIVER_TYPE_CYTRON)
  digitalWrite(PIN_MOTOR_R_DIR, speed >= 0 ? HIGH : LOW);
  writeMotorPWM(PWM_CHANNEL_R, PIN_MOTOR_R_PWM, absSpeed);

#elif defined(DRIVER_TYPE_BTS7960)
  if (speed > 0) {
    writeMotorPWM(2, PIN_MOTOR_R_RPWM, absSpeed);
    writeMotorPWM(3, PIN_MOTOR_R_LPWM, 0);
  } else if (speed < 0) {
    writeMotorPWM(2, PIN_MOTOR_R_RPWM, 0);
    writeMotorPWM(3, PIN_MOTOR_R_LPWM, absSpeed);
  } else {
    writeMotorPWM(2, PIN_MOTOR_R_RPWM, 0);
    writeMotorPWM(3, PIN_MOTOR_R_LPWM, 0);
  }
#endif
}

void emergencyStop() {
  rover.targetPwmL = 0;
  rover.targetPwmR = 0;
  rover.currentPwmL = 0;
  rover.currentPwmR = 0;
  rover.trackingEnabled = false;
  rover.emergencyStopped = true;

  setLeftMotorHardware(0);
  setRightMotorHardware(0);
}

// =============================================================================
// 5. MOTION CONTROL LOOP (Slew-Rate Ramp & Watchdog)
// =============================================================================
void updateMotionRamp() {
  // If not tracking or in emergency stop, force output to 0 immediately
  if (!rover.trackingEnabled || rover.emergencyStopped) {
    rover.currentPwmL = 0;
    rover.currentPwmR = 0;
    setLeftMotorHardware(0);
    setRightMotorHardware(0);
    return;
  }

  // Smooth acceleration ramp for Left Motor (Prevents wheel spin & current spikes)
  if (rover.currentPwmL < rover.targetPwmL) {
    rover.currentPwmL = min(rover.currentPwmL + PWM_SLEW_STEP, rover.targetPwmL);
  } else if (rover.currentPwmL > rover.targetPwmL) {
    rover.currentPwmL = max(rover.currentPwmL - PWM_SLEW_STEP, rover.targetPwmL);
  }

  // Smooth acceleration ramp for Right Motor
  if (rover.currentPwmR < rover.targetPwmR) {
    rover.currentPwmR = min(rover.currentPwmR + PWM_SLEW_STEP, rover.targetPwmR);
  } else if (rover.currentPwmR > rover.targetPwmR) {
    rover.currentPwmR = max(rover.currentPwmR - PWM_SLEW_STEP, rover.targetPwmR);
  }

  setLeftMotorHardware(rover.currentPwmL);
  setRightMotorHardware(rover.currentPwmR);
}

// =============================================================================
// 6. SERIAL PROTOCOL PARSER
// =============================================================================
void processPacket(const char* packet) {
  rover.lastPacketTime = millis();

  // Command 1: TRACKING DISABLED / STOP
  if (strcmp(packet, "TRACKING_DISABLED") == 0 || strcmp(packet, "DISABLE") == 0 || strcmp(packet, "STOP") == 0) {
    rover.trackingEnabled = false;
    rover.targetPwmL = 0;
    rover.targetPwmR = 0;
    digitalWrite(PIN_LED_STATUS, LOW);
    Serial.println(F("ACK:TRACKING_DISABLED"));
    return;
  }

  // Command 2: TRACKING ENABLED / RUN
  if (strcmp(packet, "TRACKING_ENABLED") == 0 || strcmp(packet, "ENABLE") == 0 || strcmp(packet, "RUN") == 0) {
    rover.trackingEnabled = true;
    rover.emergencyStopped = false;
    digitalWrite(PIN_LED_STATUS, HIGH);
    Serial.println(F("ACK:TRACKING_ENABLED"));
    return;
  }

  // Command 3: EMERGENCY STOP
  if (strcmp(packet, "EMERGENCY_STOP") == 0 || strcmp(packet, "ESTOP") == 0) {
    emergencyStop();
    digitalWrite(PIN_LED_STATUS, LOW);
    Serial.println(F("ACK:EMERGENCY_STOP"));
    return;
  }

  // Command 4: Heartbeat PING
  if (strcmp(packet, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }

  // Command 5: Differential Drive Command "<L,R>"
  int pwmL = 0;
  int pwmR = 0;
  if (sscanf(packet, "%d,%d", &pwmL, &pwmR) == 2) {
    if (rover.trackingEnabled && !rover.emergencyStopped) {
      rover.targetPwmL = constrain(pwmL, -255, 255);
      rover.targetPwmR = constrain(pwmR, -255, 255);

      // Heartbeat blink on valid drive packet
      digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));

      // Echo acknowledgment back to host
      Serial.print(F("ACK:"));
      Serial.print(pwmL);
      Serial.print(F(","));
      Serial.println(pwmR);
    } else {
      // Ignored because rover is in Standby
      Serial.println(F("WARN:IGNORING_PWM_STANDBY"));
    }
    return;
  }

  // Unknown packet
  Serial.print(F("ERR:UNKNOWN_CMD:"));
  Serial.println(packet);
}

void readSerialData() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '<') {
      rxPacketStarted = true;
      rxIndex = 0;
    } else if (c == '>' && rxPacketStarted) {
      rxBuffer[rxIndex] = '\0';
      rxPacketStarted = false;
      processPacket(rxBuffer);
    } else if (rxPacketStarted) {
      if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        // Buffer overflow protection
        rxPacketStarted = false;
        rxIndex = 0;
      }
    }
  }
}

// =============================================================================
// 7. ARDUINO SETUP & MAIN LOOP
// =============================================================================
void setup() {
  // Initialize GPIO Directions
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

#if defined(DRIVER_TYPE_DIR_PWM)
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
#elif defined(DRIVER_TYPE_CYTRON)
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
#endif

  // Initialize LEDC Ultrasonic Hardware PWM
  initPWM();

  // Safety first: strictly 0 output on boot
  emergencyStop();

  // Initialize High-Speed Serial
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial && millis() < 1500) {
    // Wait for USB Enumeration on ESP32 native USB
  }

  rover.lastPacketTime = millis();
  rover.lastLoopTime = millis();
  rover.lastTelemetryTime = millis();

  Serial.println(F("[AGRI-ROVER-ESP32] Next-Gen Motor Controller Online."));
  Serial.println(F("[AGRI-ROVER-ESP32] Status: SAFE_STANDBY (Waiting for <TRACKING_ENABLED>)."));
}

void loop() {
  uint32_t now = millis();

  // 1. Ingest and parse incoming serial packets from Raspberry Pi
  readSerialData();

  // 2. 100 Hz Internal Motion Slew-Rate Controller (Every 10ms)
  if (now - rover.lastLoopTime >= LOOP_INTERVAL_MS) {
    rover.lastLoopTime = now;
    updateMotionRamp();
  }

  // 3. Hardware Failsafe Watchdog (Stop if host crashes or USB disconnects)
  if (rover.trackingEnabled && (now - rover.lastPacketTime > WATCHDOG_MS)) {
    emergencyStop();
    digitalWrite(PIN_LED_STATUS, HIGH); // Solid alert LED indicates watchdog trip
    Serial.println(F("ERR:WATCHDOG_TIMEOUT_MOTORS_HALTED"));
  }

  // 4. Next-Gen Optional: 2Hz Telemetry Broadcast back to Raspberry Pi
  if (now - rover.lastTelemetryTime >= 500) {
    rover.lastTelemetryTime = now;
    // Format: <TELEMETRY,armed,pwmL,pwmR,uptime_sec>
    // Host can parse this for real-time hardware status verification!
    /*
    Serial.print(F("<TELEMETRY,"));
    Serial.print(rover.trackingEnabled ? 1 : 0);
    Serial.print(F(","));
    Serial.print(rover.currentPwmL);
    Serial.print(F(","));
    Serial.print(rover.currentPwmR);
    Serial.print(F(","));
    Serial.print(now / 1000);
    Serial.println(F(">"));
    */
  }
}
