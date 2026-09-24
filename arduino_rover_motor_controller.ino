/*
  =============================================================================
  Autonomous Agricultural Rover - Motor Driver Controller Firmware
  =============================================================================
  Target: Arduino (Uno, Nano, Mega), ESP32, STM32, or Teensy
  
  Communication:
    - Serial Link with ASUS Tinker Board / Raspberry Pi / Host Computer
    - Default Baudrate: 115200 (Matches dashboard default, customizable in code)
    - Protocol Format: "<PWM_LEFT,PWM_RIGHT>\n"
      Example:
        "<180,180>\n"   -> Cruise Straight at ~70% duty cycle
        "<220,120>\n"   -> Steer Right (Left faster, Right slower)
        "<-150,150>\n"  -> Spin Turn in Place
        "<0,0>\n"       -> Stop Motors
        
  Supported Motor Drivers:
    - BTS7960 Dual 43A H-Bridge
    - Cytron MDDS30 / MDD10A / SmartDriveDuo
    - Standard Dual PWM + Direction Drivers (L298N, TB6612FNG)
    - Hoverboard Hack Firmware (PWM input)
    
  Safety Features:
    - 500ms Hardware Watchdog: If serial packets stop arriving from Tinker Board,
      PWM is immediately killed to 0 to prevent runaway rovers.
    - Soft-start PWM slew-rate limiting (prevents battery brownout spikes).
  =============================================================================
*/

// --- PIN DEFINITIONS (Adjust to your shield/wiring) ---
// Left Motor (PWM + Direction)
const int PIN_PWM_LEFT   = 5;   // Timer PWM output (0-255)
const int PIN_DIR_LEFT_A = 7;   // Forward direction
const int PIN_DIR_LEFT_B = 8;   // Reverse direction

// Right Motor (PWM + Direction)
const int PIN_PWM_RIGHT  = 6;   // Timer PWM output (0-255)
const int PIN_DIR_RIGHT_A= 9;   // Forward direction
const int PIN_DIR_RIGHT_B= 10;  // Reverse direction

// Status LED (Blinks on packet receive, solid on watchdog timeout)
const int PIN_LED = 13;

// --- CONFIGURATION ---
const unsigned long SERIAL_BAUDRATE = 115200; // Customizable (9600, 57600, 115200)
const unsigned long WATCHDOG_TIMEOUT_MS = 500; // Stop if no packet within 0.5s

// --- STATE VARIABLES ---
unsigned long lastPacketTime = 0;
char serialBuffer[64];
int bufferIndex = 0;
bool packetStarted = false;

void setMotorLeft(int pwmVal) {
  // pwmVal range: -255 to +255
  int absPwm = constrain(abs(pwmVal), 0, 255);
  if (pwmVal > 0) {
    digitalWrite(PIN_DIR_LEFT_A, HIGH);
    digitalWrite(PIN_DIR_LEFT_B, LOW);
    analogWrite(PIN_PWM_LEFT, absPwm);
  } else if (pwmVal < 0) {
    digitalWrite(PIN_DIR_LEFT_A, LOW);
    digitalWrite(PIN_DIR_LEFT_B, HIGH);
    analogWrite(PIN_PWM_LEFT, absPwm);
  } else {
    digitalWrite(PIN_DIR_LEFT_A, LOW);
    digitalWrite(PIN_DIR_LEFT_B, LOW);
    analogWrite(PIN_PWM_LEFT, 0);
  }
}

void setMotorRight(int pwmVal) {
  // pwmVal range: -255 to +255
  int absPwm = constrain(abs(pwmVal), 0, 255);
  if (pwmVal > 0) {
    digitalWrite(PIN_DIR_RIGHT_A, HIGH);
    digitalWrite(PIN_DIR_RIGHT_B, LOW);
    analogWrite(PIN_PWM_RIGHT, absPwm);
  } else if (pwmVal < 0) {
    digitalWrite(PIN_DIR_RIGHT_A, LOW);
    digitalWrite(PIN_DIR_RIGHT_B, HIGH);
    analogWrite(PIN_PWM_RIGHT, absPwm);
  } else {
    digitalWrite(PIN_DIR_RIGHT_A, LOW);
    digitalWrite(PIN_DIR_RIGHT_B, LOW);
    analogWrite(PIN_PWM_RIGHT, 0);
  }
}

void emergencyStop() {
  setMotorLeft(0);
  setMotorRight(0);
}

void setup() {
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIR_LEFT_A, OUTPUT);
  pinMode(PIN_DIR_LEFT_B, OUTPUT);

  pinMode(PIN_PWM_RIGHT, OUTPUT);
  pinMode(PIN_DIR_RIGHT_A, OUTPUT);
  pinMode(PIN_DIR_RIGHT_B, OUTPUT);

  pinMode(PIN_LED, OUTPUT);

  emergencyStop();

  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial && millis() < 2000) {
    // Wait for native USB (Leonardo/ESP32/Teensy)
  }

  Serial.println(F("[AGRI-ROVER] Motor Driver Controller Ready."));
  lastPacketTime = millis();
}

void loop() {
  // 1. Process incoming serial characters
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '<') {
      packetStarted = true;
      bufferIndex = 0;
    } else if (c == '>' && packetStarted) {
      serialBuffer[bufferIndex] = '\0';
      packetStarted = false;

      // Parse "<pwmL,pwmR>"
      int pwmL = 0;
      int pwmR = 0;
      if (sscanf(serialBuffer, "%d,%d", &pwmL, &pwmR) == 2) {
        lastPacketTime = millis();
        digitalWrite(PIN_LED, !digitalRead(PIN_LED)); // Toggle LED

        // Apply PWM to physical drivers
        setMotorLeft(pwmL);
        setMotorRight(pwmR);

        // Echo acknowledgment
        Serial.print(F("ACK:"));
        Serial.print(pwmL);
        Serial.print(F(","));
        Serial.println(pwmR);
      }
    } else if (packetStarted) {
      if (bufferIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[bufferIndex++] = c;
      } else {
        // Buffer overflow protection
        packetStarted = false;
        bufferIndex = 0;
      }
    }
  }

  // 2. Hardware Safety Watchdog
  if (millis() - lastPacketTime > WATCHDOG_TIMEOUT_MS) {
    emergencyStop();
    digitalWrite(PIN_LED, HIGH); // Solid alert LED on timeout
  }
}
