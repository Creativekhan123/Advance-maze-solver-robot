#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

// =====================================================
// USER CONFIGURATION & CALIBRATION SETTINGS
// =====================================================

// If true, robot automatically starts autonomous navigation 5 seconds after power-on.
// If false, robot waits for 'A' command in Serial Monitor.
bool AutoStart_automonus = true;

// ===== MOVEMENT CALIBRATION (EXPERIMENTALLY MEASURED) =====
// 10 cm = 650 encoder counts on physical robot
const long COUNTS_FOR_10CM    = 650;
// 90 degree in-place turn = 520 encoder counts per wheel
const long COUNTS_FOR_90_DEG  = 520;
// 180 degree in-place U-turn = 1040 encoder counts per wheel
const long COUNTS_FOR_180_DEG = 1040;

// Advance forward before 90-degree turn when FRONT IS OPEN (e.g. cross or T-junction)
const float TURN_ALIGN_FORWARD_CM    = 10.0;
// Advance forward before 90-degree turn when FRONT IS BLOCKED / WALL (safe clearance to avoid hitting front wall)
const float TURN_ALIGN_FRONT_WALL_CM = 1.0;
// Advance forward after 90-degree turn to enter the new corridor before reading sensors
const float POST_TURN_FORWARD_CM     = 10.0;

// ===== MAZE & SENSOR SETTINGS =====
// Distance < 15.0 cm: WALL / BLOCKED
// Distance >= 15.0 cm: OPEN
const float WALL_THRESHOLD_CM = 15.0;

// ===== MOTOR SPEEDS (0-255) =====
const int LEFT_MOTOR_SPEED  = 100;   // Base speed for left motor
const int RIGHT_MOTOR_SPEED = 100;   // Base speed for right motor
const int TURN_SPEED        = 80;    // Rotation speed for turns

// Approach slowdown speeds near target count
const int SLOWDOWN_SPEED_1  = 70;    // 80-92% of target
const int SLOWDOWN_SPEED_2  = 50;    // 92-100% of target

// Motor direction invert flags (set true if wired reversed)
bool INVERT_LEFT_MOTOR  = false;
bool INVERT_RIGHT_MOTOR = true;   // Inverted so right motor spins forward in sync with left

// ===== ACTIVE MOTOR BRAKING =====
// Active brake duration (ms) on TB6612FNG (AIN1=HIGH, AIN2=HIGH)
const int BRAKE_TIME_MS = 80;

// ===== STRAIGHT-LINE ENCODER SYNC =====
float ENCODER_SYNC_KP = 0.5;

// ===== DEBUG FLAGS =====
const bool DEBUG_SENSORS   = true;
const bool DEBUG_DECISIONS = true;
const bool DEBUG_MOVEMENT  = true;
const bool DEBUG_MOTORS    = true;
const bool DEBUG_ENCODERS  = true;
const unsigned long ENCODER_PRINT_INTERVAL_MS = 250;

// =====================================================
// HARDWARE PIN DEFINITIONS
// =====================================================

// --- Left Motor (Motor A) ---
const int AIN1 = 14;
const int AIN2 = 27;
const int PWMA = 23;

// --- Right Motor (Motor B) ---
const int BIN1 = 18;
const int BIN2 = 19;
const int PWMB = 5;

// TB6612 STBY is physically wired to 3.3V (no pin needed)

// --- Encoders ---
const int LEFT_ENCODER_PIN  = 33;   // GPIO 33
const int RIGHT_ENCODER_PIN = 34;   // GPIO 34 (input-only, external pullup)

// --- VL53L0X Laser Distance Sensors ---
const int F_XSHUT = 13;             // Front sensor shutdown -> GPIO 13
const int L_XSHUT = 25;             // Left sensor shutdown  -> GPIO 25
const int R_XSHUT = 26;             // Right sensor shutdown -> GPIO 26

const int SDA_PIN = 21;             // I2C SDA
const int SCL_PIN = 22;             // I2C SCL

// Unique I2C addresses assigned at boot
const uint8_t ADDR_FRONT = 0x30;
const uint8_t ADDR_RIGHT = 0x31;
const uint8_t ADDR_LEFT  = 0x29;    // Default address

// PWM configuration
const int PWM_FREQUENCY  = 5000;
const int PWM_RESOLUTION = 8;
const int PWMA_CHANNEL   = 0;
const int PWMB_CHANNEL   = 1;

// =====================================================
// GLOBAL OBJECTS & STATE VARIABLES
// =====================================================

VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

bool frontOK = false;
bool leftOK  = false;
bool rightOK = false;

volatile long left_encoder_count  = 0;
volatile long right_encoder_count = 0;

bool isMoving = false;
bool autonomousActive = false;

// Decision types
enum DirectionDecision {
    DECISION_LEFT,
    DECISION_FORWARD,
    DECISION_RIGHT,
    DECISION_UTURN
};

// =====================================================
// INTERRUPT SERVICE ROUTINES (Single-channel counts)
// =====================================================

void IRAM_ATTR handleLeftEncoder()  { left_encoder_count++;  }
void IRAM_ATTR handleRightEncoder() { right_encoder_count++; }

// =====================================================
// THREAD-SAFE ENCODER HELPERS
// =====================================================

void readEncoders(long &lCount, long &rCount) {
    noInterrupts();
    lCount = left_encoder_count;
    rCount = right_encoder_count;
    interrupts();
}

void clearEncoderCounts() {
    noInterrupts();
    left_encoder_count  = 0;
    right_encoder_count = 0;
    interrupts();
    if (DEBUG_MOVEMENT) Serial.println("Encoder counts cleared.");
}

void printEncoderCounts() {
    long l, r;
    readEncoders(l, r);
    Serial.print("Left Encoder : "); Serial.println(l);
    Serial.print("Right Encoder: "); Serial.println(r);
    Serial.print("Difference   : "); Serial.println(l - r);
}

// Distance conversion
long distanceToCounts(float cm) {
    return (long)(cm * (float)COUNTS_FOR_10CM / 10.0f);
}

float countsToCm(long counts) {
    return (float)counts * 10.0f / (float)COUNTS_FOR_10CM;
}

// =====================================================
// MOTOR CONTROL & ACTIVE BRAKING (TB6612FNG)
// =====================================================

void setLeftMotor(int speed, bool forward) {
    if (INVERT_LEFT_MOTOR) forward = !forward;
    speed = constrain(speed, 0, 255);
    ledcWrite(PWMA_CHANNEL, speed);
    if (forward) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
    else         { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
}

void setRightMotor(int speed, bool forward) {
    if (INVERT_RIGHT_MOTOR) forward = !forward;
    speed = constrain(speed, 0, 255);
    ledcWrite(PWMB_CHANNEL, speed);
    if (forward) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
    else         { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
}

void stopMotors() {
    ledcWrite(PWMA_CHANNEL, 0);
    ledcWrite(PWMB_CHANNEL, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    isMoving = false;
}

void brakeBothMotors() {
    // TB6612FNG short-circuit brake: both input pins HIGH + full PWM
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    ledcWrite(PWMA_CHANNEL, 255);
    ledcWrite(PWMB_CHANNEL, 255);
    delay(BRAKE_TIME_MS);
    // Release to safe idle state
    ledcWrite(PWMA_CHANNEL, 0);
    ledcWrite(PWMB_CHANNEL, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    isMoving = false;
}

// =====================================================
// CALIBRATED ENCODER MOVEMENT WITH BRAKING & CENTERING
// =====================================================

// Forward declaration of sensor reader
float readSensorDistanceCm(VL53L0X &sensor, bool isOK, const char *label, bool &isValid);

// Move target distance in cm (positive = forward, negative = backward)
// Pure straight-line encoder synchronization (no side wall centering)
bool moveDistance(float targetCm) {
    long targetCounts = distanceToCounts(abs(targetCm));
    if (targetCounts <= 0) return false;

    bool forward = (targetCm >= 0);
    if (DEBUG_MOVEMENT) {
        Serial.print("ACTION: MOVE "); Serial.print(forward ? "FORWARD " : "BACKWARD ");
        Serial.print(abs(targetCm), 1); Serial.println(" CM");
        Serial.print("TARGET: "); Serial.print(targetCounts); Serial.println(" COUNTS");
    }

    clearEncoderCounts();
    delay(10);
    isMoving = true;
    unsigned long lastPrint = 0;

    while (true) {
        // Immediate stop check
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                brakeBothMotors();
                autonomousActive = false;
                Serial.println("EMERGENCY STOPPED BY USER.");
                return false;
            }
        }

        long lCount, rCount;
        readEncoders(lCount, rCount);
        long avgCount = (lCount + rCount) / 2;

        if (avgCount >= targetCounts) {
            break;
        }

        // Approach slowdown near target count
        int baseSpeed;
        if (avgCount >= (long)(targetCounts * 0.92f)) {
            baseSpeed = SLOWDOWN_SPEED_2;
        } else if (avgCount >= (long)(targetCounts * 0.80f)) {
            baseSpeed = SLOWDOWN_SPEED_1;
        } else {
            baseSpeed = LEFT_MOTOR_SPEED;
        }

        // Pure straight-line encoder synchronization (Left & Right wheel counts stay matched)
        long errorVal = lCount - rCount;
        int leftSpeed  = baseSpeed - (int)(ENCODER_SYNC_KP * errorVal);
        int rightSpeed = baseSpeed + (int)(ENCODER_SYNC_KP * errorVal);

        leftSpeed  = constrain(leftSpeed,  30, 255);
        rightSpeed = constrain(rightSpeed, 30, 255);

        setLeftMotor(leftSpeed,  forward);
        setRightMotor(rightSpeed, forward);

        if (DEBUG_ENCODERS && (millis() - lastPrint >= ENCODER_PRINT_INTERVAL_MS)) {
            lastPrint = millis();
            Serial.print("Left: "); Serial.print(lCount);
            Serial.print("  Right: "); Serial.print(rCount);
            Serial.print("  Target: "); Serial.println(targetCounts);
        }

        delay(1);
    }

    // Active brake when target count reached
    if (DEBUG_MOVEMENT) {
        long l, r; readEncoders(l, r);
        Serial.print("LEFT COUNT : "); Serial.println(l);
        Serial.print("RIGHT COUNT: "); Serial.println(r);
        Serial.println("ACTIVE BRAKE");
    }
    brakeBothMotors();

    if (DEBUG_MOVEMENT) Serial.println("MOVEMENT COMPLETE");
    return true;
}

// In-place rotation (rotateLeft = true for Left CCW, false for Right CW)
bool rotateCounts(long targetCounts, bool rotateLeft) {
    if (targetCounts <= 0) return false;

    if (DEBUG_MOVEMENT) {
        Serial.print("ACTION: TURN "); Serial.print(rotateLeft ? "LEFT " : "RIGHT ");
        Serial.print(rotateLeft ? "90 DEG" : (targetCounts == COUNTS_FOR_180_DEG ? "180 DEG" : "90 DEG"));
        Serial.println();
        Serial.print("TARGET: "); Serial.print(targetCounts); Serial.println(" COUNTS");
    }

    clearEncoderCounts();
    delay(10);
    isMoving = true;
    unsigned long lastPrint = 0;

    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                brakeBothMotors();
                autonomousActive = false;
                Serial.println("EMERGENCY STOPPED BY USER.");
                return false;
            }
        }

        long lCount, rCount;
        readEncoders(lCount, rCount);
        long avgCount = (lCount + rCount) / 2;

        if (avgCount >= targetCounts) {
            break;
        }

        // Approach slowdown for turn
        int turnSpd;
        if (avgCount >= (long)(targetCounts * 0.90f)) {
            turnSpd = SLOWDOWN_SPEED_2;
        } else if (avgCount >= (long)(targetCounts * 0.78f)) {
            turnSpd = SLOWDOWN_SPEED_1;
        } else {
            turnSpd = TURN_SPEED;
        }

        if (rotateLeft) {
            // Left backward, Right forward
            setLeftMotor(turnSpd, false);
            setRightMotor(turnSpd, true);
        } else {
            // Left forward, Right backward
            setLeftMotor(turnSpd, true);
            setRightMotor(turnSpd, false);
        }

        if (DEBUG_ENCODERS && (millis() - lastPrint >= ENCODER_PRINT_INTERVAL_MS)) {
            lastPrint = millis();
            Serial.print("Left: "); Serial.print(lCount);
            Serial.print("  Right: "); Serial.print(rCount);
            Serial.print("  Target: "); Serial.println(targetCounts);
        }
        delay(1);
    }

    if (DEBUG_MOVEMENT) {
        long l, r; readEncoders(l, r);
        Serial.print("LEFT COUNT : "); Serial.println(l);
        Serial.print("RIGHT COUNT: "); Serial.println(r);
        Serial.println("ACTIVE BRAKE");
    }
    brakeBothMotors();

    if (DEBUG_MOVEMENT) Serial.println("MOVEMENT COMPLETE");
    return true;
}

// Action helpers
bool moveForward10cm() { return moveDistance(10.0f); }
bool turnLeft90()      { return rotateCounts(COUNTS_FOR_90_DEG, true); }
bool turnRight90()     { return rotateCounts(COUNTS_FOR_90_DEG, false); }
bool turnUturn()       { return rotateCounts(COUNTS_FOR_180_DEG, true); }

// =====================================================
// CONTINUOUS MANUAL MOVEMENT (V03 compat)
// =====================================================

void continuousMove(bool forward) {
    Serial.println(forward ? "Moving FORWARD (send S to stop)" : "Moving BACKWARD (send S to stop)");
    isMoving = true;
    clearEncoderCounts();

    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                brakeBothMotors();
                Serial.println("STOPPED.");
                printEncoderCounts();
                return;
            }
        }

        long lCount, rCount;
        readEncoders(lCount, rCount);
        long errorVal = lCount - rCount;

        int leftSpeed  = LEFT_MOTOR_SPEED  - (int)(ENCODER_SYNC_KP * errorVal);
        int rightSpeed = RIGHT_MOTOR_SPEED + (int)(ENCODER_SYNC_KP * errorVal);
        leftSpeed  = constrain(leftSpeed,  30, 255);
        rightSpeed = constrain(rightSpeed, 30, 255);

        setLeftMotor(leftSpeed,  forward);
        setRightMotor(rightSpeed, forward);
        delay(1);
    }
}

void continuousRotate(bool rotateLeft) {
    Serial.println(rotateLeft ? "Rotating LEFT (send S to stop)" : "Rotating RIGHT (send S to stop)");
    isMoving = true;
    clearEncoderCounts();

    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                brakeBothMotors();
                Serial.println("STOPPED.");
                printEncoderCounts();
                return;
            }
        }

        if (rotateLeft) {
            setLeftMotor(TURN_SPEED, false);
            setRightMotor(TURN_SPEED, true);
        } else {
            setLeftMotor(TURN_SPEED, true);
            setRightMotor(TURN_SPEED, false);
        }
        delay(1);
    }
}

// =====================================================
// SENSOR INITIALIZATION & READING
// =====================================================

bool initSensor(VL53L0X &sensor, int xshutPin, uint8_t newAddress, const char *label) {
    digitalWrite(xshutPin, HIGH);
    delay(10);

    if (!sensor.init()) {
        Serial.print("  [FAILED] ");
        Serial.print(label);
        Serial.println(" sensor did not respond on 0x29.");
        return false;
    }

    if (newAddress != 0x29) {
        sensor.setAddress(newAddress);
    }

    sensor.startContinuous();
    Serial.print("  [OK]     ");
    Serial.print(label);
    Serial.print(" sensor  -> address 0x");
    Serial.println(newAddress, HEX);
    return true;
}

void setupSensors() {
    pinMode(F_XSHUT, OUTPUT);
    pinMode(L_XSHUT, OUTPUT);
    pinMode(R_XSHUT, OUTPUT);

    // Pull all XSHUT LOW to hold in reset
    digitalWrite(F_XSHUT, LOW);
    digitalWrite(L_XSHUT, LOW);
    digitalWrite(R_XSHUT, LOW);
    delay(10);

    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println("Initialising VL53L0X sensors...");
    // 1. Front (0x30)
    frontOK = initSensor(sensorFront, F_XSHUT, ADDR_FRONT, "Front");
    // 2. Right (0x31)
    rightOK = initSensor(sensorRight, R_XSHUT, ADDR_RIGHT, "Right");
    // 3. Left (0x29)
    leftOK  = initSensor(sensorLeft,  L_XSHUT, ADDR_LEFT,  "Left ");

    Serial.println("----------------------------------------");
    Serial.print("  Front Sensor : "); Serial.println(frontOK ? "OK" : "FAILED");
    Serial.print("  Right Sensor : "); Serial.println(rightOK ? "OK" : "FAILED");
    Serial.print("  Left  Sensor : "); Serial.println(leftOK  ? "OK" : "FAILED");
    Serial.println("----------------------------------------");

    if (!frontOK && !rightOK && !leftOK) {
        Serial.println("!!! CRITICAL ERROR: ALL SENSORS FAILED !!!");
        Serial.println("Motors locked. Fix sensor wiring and reset.");
        while (true) delay(1000);
    }
}

// Safe distance reader (returns distance in cm, or -1.0 if invalid)
float readSensorDistanceCm(VL53L0X &sensor, bool isOK, const char *label, bool &isValid) {
    if (!isOK) {
        isValid = false;
        return -1.0f;
    }
    uint16_t distMm = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
        isValid = false;
        return -1.0f;
    }
    // In Pololu VL53L0X: 8190 or 8191 (or values > 2000 mm) indicate OUT OF RANGE.
    // In a maze corridor, out-of-range means wide open corridor ahead (NO WALL), NOT an error!
    if (distMm >= 8190 || distMm > 2000) {
        isValid = true;
        return 999.0f; // Open pathway (>= WALL_THRESHOLD_CM)
    }
    if (distMm == 0) {
        // Zero mm typically indicates obstacle pressed against sensor cover
        isValid = true;
        return 0.0f;
    }
    isValid = true;
    return (float)distMm / 10.0f;
}

// Global toggle for continuous live sensor monitor
bool liveSensorMonitor = false;

// Prints Forward, Left, and Right distances in cm on a single clean line
void printLaserSensorsLineCm() {
    bool fValid = false, lValid = false, rValid = false;
    float distF = readSensorDistanceCm(sensorFront, frontOK, "Front", fValid);
    float distL = readSensorDistanceCm(sensorLeft,  leftOK,  "Left",  lValid);
    float distR = readSensorDistanceCm(sensorRight, rightOK, "Right", rValid);

    bool fOpen = fValid && (distF >= WALL_THRESHOLD_CM);
    bool lOpen = lValid && (distL >= WALL_THRESHOLD_CM);
    bool rOpen = rValid && (distR >= WALL_THRESHOLD_CM);

    Serial.print("FORWARD: ");
    if (distF >= 900.0f) Serial.print(">200.0 cm [OPEN]");
    else if (fValid)     { Serial.print(distF, 1); Serial.print(" cm ["); Serial.print(fOpen ? "OPEN" : "WALL"); Serial.print("]"); }
    else                 Serial.print("ERROR [WALL]");

    Serial.print("  |  LEFT: ");
    if (distL >= 900.0f) Serial.print(">200.0 cm [OPEN]");
    else if (lValid)     { Serial.print(distL, 1); Serial.print(" cm ["); Serial.print(lOpen ? "OPEN" : "WALL"); Serial.print("]"); }
    else                 Serial.print("ERROR [WALL]");

    Serial.print("  |  RIGHT: ");
    if (distR >= 900.0f) Serial.print(">200.0 cm [OPEN]");
    else if (rValid)     { Serial.print(distR, 1); Serial.print(" cm ["); Serial.print(rOpen ? "OPEN" : "WALL"); Serial.print("]"); }
    else                 Serial.print("ERROR [WALL]");

    Serial.println();
}

// Standalone detailed diagnostic print (useful to test sensors without robot moving)
void printSensorDiagnostic() {
    bool lValid = false, fValid = false, rValid = false;
    float distF = readSensorDistanceCm(sensorFront, frontOK, "Front", fValid);
    float distL = readSensorDistanceCm(sensorLeft,  leftOK,  "Left",  lValid);
    float distR = readSensorDistanceCm(sensorRight, rightOK, "Right", rValid);

    bool fOpen = fValid && (distF >= WALL_THRESHOLD_CM);
    bool lOpen = lValid && (distL >= WALL_THRESHOLD_CM);
    bool rOpen = rValid && (distR >= WALL_THRESHOLD_CM);

    Serial.println("==================================================");
    Serial.println("LASER DISTANCE READINGS (in cm)");
    Serial.println("==================================================");
    Serial.print("  FORWARD : ");
    if (distF >= 900.0f) Serial.print("> 200.0 cm (OPEN SPACE)");
    else if (fValid)     { Serial.print(distF, 1); Serial.print(" cm"); }
    else                 Serial.print("INVALID / TIMEOUT");
    Serial.print("  --> "); Serial.println(fOpen ? "[OPEN]" : "[WALL]");

    Serial.print("  LEFT    : ");
    if (distL >= 900.0f) Serial.print("> 200.0 cm (OPEN SPACE)");
    else if (lValid)     { Serial.print(distL, 1); Serial.print(" cm"); }
    else                 Serial.print("INVALID / TIMEOUT");
    Serial.print("  --> "); Serial.println(lOpen ? "[OPEN]" : "[WALL]");

    Serial.print("  RIGHT   : ");
    if (distR >= 900.0f) Serial.print("> 200.0 cm (OPEN SPACE)");
    else if (rValid)     { Serial.print(distR, 1); Serial.print(" cm"); }
    else                 Serial.print("INVALID / TIMEOUT");
    Serial.print("  --> "); Serial.println(rOpen ? "[OPEN]" : "[WALL]");

    // Show what decision the robot would make
    DirectionDecision decision = makeDecision(lOpen, fOpen, rOpen);
    Serial.print("  --> DECISION: ");
    if (decision == DECISION_LEFT)          Serial.println("LEFT (Turn Left 90)");
    else if (decision == DECISION_FORWARD)  Serial.println("FORWARD (Move 10 cm)");
    else if (decision == DECISION_RIGHT)    Serial.println("RIGHT (Turn Right 90)");
    else                                    Serial.println("U-TURN (Turn 180)");
    Serial.println("==================================================");
}

// =====================================================
// AUTONOMOUS MAZE DECISION & NAVIGATION LOGIC
// =====================================================

// Priority: LEFT -> FRONT -> RIGHT -> U-TURN
DirectionDecision makeDecision(bool leftOpen, bool frontOpen, bool rightOpen) {
    if (leftOpen) {
        return DECISION_LEFT;
    } else if (frontOpen) {
        return DECISION_FORWARD;
    } else if (rightOpen) {
        return DECISION_RIGHT;
    } else {
        return DECISION_UTURN;
    }
}

void performAutonomousStep() {
    Serial.println();
    Serial.println("========================================");
    Serial.println(" LASER DISTANCE SENSOR READINGS (cm)");
    Serial.println("========================================");

    bool leftValid, frontValid, rightValid;
    float distFront = readSensorDistanceCm(sensorFront, frontOK, "Front", frontValid);
    float distLeft  = readSensorDistanceCm(sensorLeft,  leftOK,  "Left",  leftValid);
    float distRight = readSensorDistanceCm(sensorRight, rightOK, "Right", rightValid);

    // Wall vs Open classification (threshold = 15.0 cm)
    bool frontOpen = frontValid && (distFront >= WALL_THRESHOLD_CM);
    bool leftOpen  = leftValid  && (distLeft  >= WALL_THRESHOLD_CM);
    bool rightOpen = rightValid && (distRight >= WALL_THRESHOLD_CM);

    // Print Forward
    Serial.print("  FORWARD : ");
    if (distFront >= 900.0f) Serial.print("> 200.0 cm");
    else if (frontValid)     { Serial.print(distFront, 1); Serial.print(" cm"); }
    else                     Serial.print("ERROR / TIMEOUT");
    Serial.print("  --> ["); Serial.print(frontOpen ? "OPEN" : "WALL"); Serial.println("]");

    // Print Left
    Serial.print("  LEFT    : ");
    if (distLeft >= 900.0f)  Serial.print("> 200.0 cm");
    else if (leftValid)      { Serial.print(distLeft, 1); Serial.print(" cm"); }
    else                     Serial.print("ERROR / TIMEOUT");
    Serial.print("  --> ["); Serial.print(leftOpen ? "OPEN" : "WALL"); Serial.println("]");

    // Print Right
    Serial.print("  RIGHT   : ");
    if (distRight >= 900.0f) Serial.print("> 200.0 cm");
    else if (rightValid)     { Serial.print(distRight, 1); Serial.print(" cm"); }
    else                     Serial.print("ERROR / TIMEOUT");
    Serial.print("  --> ["); Serial.print(rightOpen ? "OPEN" : "WALL"); Serial.println("]");

    Serial.println("----------------------------------------");

    // Decide
    DirectionDecision decision = makeDecision(leftOpen, frontOpen, rightOpen);
    Serial.print("  DECISION: ");
    if (decision == DECISION_LEFT)          Serial.println("LEFT -> Turn Left 90 (L90)");
    else if (decision == DECISION_FORWARD)  Serial.println("FORWARD -> Move 10 cm (M 10)");
    else if (decision == DECISION_RIGHT)    Serial.println("RIGHT -> Turn Right 90 (R90)");
    else                                    Serial.println("U-TURN -> Turn 180 (U)");
    Serial.println("========================================");
    Serial.println();

    switch (decision) {
        case DECISION_LEFT: {
            Serial.println("DECISION: LEFT");
            float preDist = frontOpen ? TURN_ALIGN_FORWARD_CM : TURN_ALIGN_FRONT_WALL_CM;
            if (frontOpen) {
                Serial.print(">> Front OPEN: Moving ");
                Serial.print(TURN_ALIGN_FORWARD_CM, 1);
                Serial.println(" cm forward to align wheel axis...");
            } else {
                Serial.print(">> Front BLOCKED: Moving ");
                Serial.print(TURN_ALIGN_FRONT_WALL_CM, 1);
                Serial.println(" cm forward (safe clearance)...");
            }
            if (moveDistance(preDist)) {
                delay(80);
                Serial.println(">> Turning Left 90 deg (L90)...");
                if (turnLeft90()) {
                    delay(80);
                    Serial.print(">> Moving ");
                    Serial.print(POST_TURN_FORWARD_CM, 1);
                    Serial.println(" cm forward into new corridor...");
                    moveDistance(POST_TURN_FORWARD_CM);
                }
            }
            break;
        }

        case DECISION_FORWARD:
            Serial.println("DECISION: FORWARD");
            Serial.println(">> Moving Forward 10 cm (M 10)...");
            moveForward10cm();
            break;

        case DECISION_RIGHT: {
            Serial.println("DECISION: RIGHT");
            // For DECISION_RIGHT, front is always blocked (otherwise FORWARD would have been chosen)
            Serial.print(">> Front BLOCKED: Moving ");
            Serial.print(TURN_ALIGN_FRONT_WALL_CM, 1);
            Serial.println(" cm forward (safe clearance)...");
            if (moveDistance(TURN_ALIGN_FRONT_WALL_CM)) {
                delay(80);
                Serial.println(">> Turning Right 90 deg (R90)...");
                if (turnRight90()) {
                    delay(80);
                    Serial.print(">> Moving ");
                    Serial.print(POST_TURN_FORWARD_CM, 1);
                    Serial.println(" cm forward into new corridor...");
                    moveDistance(POST_TURN_FORWARD_CM);
                }
            }
            break;
        }

        case DECISION_UTURN:
            Serial.println("DECISION: U-TURN (Dead-End: Rotating 180 deg in place without moving forward)");
            Serial.println(">> Rotating 180 deg U-turn (U)...");
            turnUturn();
            break;
    }

    Serial.println("================================");

    // Short stabilization pause before reading sensors again
    delay(100);
}

// =====================================================
// HELP MENU
// =====================================================

void printHelp() {
    Serial.println();
    Serial.println("========================================");
    Serial.println(" FASTEST MAZE SOLVER - V04 COMMANDS");
    Serial.println("========================================");
    Serial.println("  A      = Start Autonomous Maze Navigation");
    Serial.println("  S      = STOP immediately & Exit Autonomous");
    Serial.println();
    Serial.println("  M 10   = Move forward 10 cm (calibrated)");
    Serial.println("  M -10  = Move backward 10 cm (calibrated)");
    Serial.println("  L90    = Rotate 90 deg left (calibrated)");
    Serial.println("  R90    = Rotate 90 deg right (calibrated)");
    Serial.println("  U      = Rotate 180 deg U-turn (calibrated)");
    Serial.println();
    Serial.println("  F      = Forward continuous (send S to stop)");
    Serial.println("  B      = Backward continuous (send S to stop)");
    Serial.println("  L      = Rotate left continuous (send S to stop)");
    Serial.println("  R      = Rotate right continuous (send S to stop)");
    Serial.println();
    Serial.println("  C      = Clear encoder counts");
    Serial.println("  P      = Print encoder counts");
    Serial.println("  V      = Live sensor readings monitor (toggle ON/OFF)");
    Serial.println("  H      = Help menu");
    Serial.println("========================================");
    Serial.println();
}

// =====================================================
// SERIAL COMMAND HANDLER
// =====================================================

void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    String upper = cmd;
    upper.toUpperCase();

    // Autonomous Mode
    if (upper == "A") {
        Serial.println(">>> STARTING AUTONOMOUS MAZE NAVIGATION <<<");
        Serial.println("Priority: LEFT -> FRONT -> RIGHT -> U-TURN");
        Serial.println("Send S at any time to STOP.");
        autonomousActive = true;
        return;
    }

    if (upper == "S") {
        autonomousActive = false;
        brakeBothMotors();
        Serial.println("ROBOT STOPPED. Autonomous mode disabled.");
        return;
    }

    // Direct movement commands
    if (upper == "L90") { turnLeft90(); return; }
    if (upper == "R90") { turnRight90(); return; }
    if (upper == "U")   { turnUturn(); return; }

    if (upper.startsWith("M ") || upper.startsWith("M\t")) {
        String valStr = cmd.substring(2);
        valStr.trim();
        float dist = valStr.toFloat();
        if (dist == 0.0 && valStr != "0") {
            Serial.println("ERROR: Invalid distance. Example: M 10");
            return;
        }
        moveDistance(dist);
        return;
    }

    // Continuous motion
    if (upper == "F") { continuousMove(true); return; }
    if (upper == "B") { continuousMove(false); return; }
    if (upper == "L") { continuousRotate(true); return; }
    if (upper == "R") { continuousRotate(false); return; }

    // Encoders & Help
    if (upper == "C") { clearEncoderCounts(); return; }
    if (upper == "P") { printEncoderCounts(); return; }
    if (upper == "H") { printHelp(); return; }

    // Live Sensor Monitor toggle
    if (upper == "V") {
        liveSensorMonitor = !liveSensorMonitor;
        if (liveSensorMonitor) {
            Serial.println(">>> LIVE SENSOR MONITOR ENABLED (Send 'V' or 'S' to stop) <<<");
        } else {
            Serial.println(">>> LIVE SENSOR MONITOR DISABLED <<<");
        }
        return;
    }

    Serial.print("Unknown command: "); Serial.println(cmd);
    Serial.println("Type H for help.");
}

// =====================================================
// setup()
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    // Motor direction pins
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

    // PWM configuration
    ledcSetup(PWMA_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWMB_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWMA, PWMA_CHANNEL);
    ledcAttachPin(PWMB, PWMB_CHANNEL);

    // SAFETY FIRST: Motors must be stopped at boot!
    stopMotors();

    // Encoder interrupts
    pinMode(LEFT_ENCODER_PIN,  INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT); // GPIO 34 input-only
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN),  handleLeftEncoder,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), handleRightEncoder, RISING);

    // Startup banner
    Serial.println();
    Serial.println("========================================");
    Serial.println(" FASTEST MAZE SOLVER");
    Serial.println(" VERSION 04");
    Serial.println(" BASIC MAZE NAVIGATION");
    Serial.println("========================================");
    Serial.println();
    Serial.print("  Auto-Start Autonomous: "); Serial.println(AutoStart_automonus ? "ENABLED" : "DISABLED");
    Serial.print("  Wall/Open Threshold  : "); Serial.print(WALL_THRESHOLD_CM, 1); Serial.println(" cm");
    Serial.println("Movement Calibration:");
    Serial.print("  Counts for 10 cm     : "); Serial.println(COUNTS_FOR_10CM);
    Serial.print("  Counts for 90 deg    : "); Serial.println(COUNTS_FOR_90_DEG);
    Serial.print("  Counts for 180 deg   : "); Serial.println(COUNTS_FOR_180_DEG);
    Serial.print("  Pre-turn Open (cm)   : "); Serial.println(TURN_ALIGN_FORWARD_CM, 1);
    Serial.print("  Pre-turn Wall (cm)   : "); Serial.println(TURN_ALIGN_FRONT_WALL_CM, 1);
    Serial.print("  Post-turn (cm)       : "); Serial.println(POST_TURN_FORWARD_CM, 1);
    Serial.print("  Active Brake Time    : "); Serial.print(BRAKE_TIME_MS); Serial.println(" ms");
    Serial.println();

    // Initialize laser distance sensors
    setupSensors();

    if (AutoStart_automonus) {
        Serial.println();
        Serial.println("SYSTEM READY");
        Serial.println("AUTO START IN 5...");
        for (int i = 4; i >= 1; i--) {
            unsigned long t0 = millis();
            while (millis() - t0 < 1000) {
                if (Serial.available() > 0) {
                    char c = Serial.read();
                    if (c == 'S' || c == 's') {
                        Serial.println("AUTO START CANCELLED. Robot STOPPED.");
                        AutoStart_automonus = false;
                        autonomousActive = false;
                        break;
                    }
                }
                delay(10);
            }
            if (!AutoStart_automonus) break;
            Serial.print(i);
            Serial.println("...");
        }

        if (AutoStart_automonus) {
            unsigned long t0 = millis();
            while (millis() - t0 < 1000) {
                if (Serial.available() > 0) {
                    char c = Serial.read();
                    if (c == 'S' || c == 's') {
                        Serial.println("AUTO START CANCELLED. Robot STOPPED.");
                        AutoStart_automonus = false;
                        autonomousActive = false;
                        break;
                    }
                }
                delay(10);
            }
        }

        if (AutoStart_automonus) {
            Serial.println("AUTONOMOUS MODE STARTED");
            autonomousActive = true;
        }
    } else {
        Serial.println();
        Serial.println("  Robot is STOPPED. Waiting for command.");
        Serial.println("  Send 'A' to start autonomous maze navigation.");
        printHelp();
    }
}

// =====================================================
// loop()
// =====================================================

void loop() {
    // Process Serial commands
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        handleCommand(line);
    }

    // Live continuous laser sensor printing (when robot is idle/stopped)
    // Prints: FORWARD: XX.X cm [OPEN/WALL]  |  LEFT: XX.X cm [OPEN/WALL]  |  RIGHT: XX.X cm [OPEN/WALL]
    static unsigned long lastSensorDebugPrint = 0;
    if (!autonomousActive && !isMoving && (millis() - lastSensorDebugPrint >= 250)) {
        lastSensorDebugPrint = millis();
        printLaserSensorsLineCm();
    }

    // If autonomous navigation is active, run step-by-step
    // Architecture: READ -> CLASSIFY -> DECIDE -> MOVE -> BRAKE -> READ AGAIN
    if (autonomousActive) {
        performAutonomousStep();
    }
}
