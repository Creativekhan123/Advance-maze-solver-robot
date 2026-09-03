#include <Arduino.h>
#include <math.h>

// =====================================================
// USER CALIBRATION SETTINGS
// Adjust these values based on physical testing.
// =====================================================

// --- Motor Base Speeds (0-255) ---
// Keep low for calibration. Increase later for speed.
const int LEFT_MOTOR_SPEED  = 100;   // Forward/backward speed for left motor
const int RIGHT_MOTOR_SPEED = 100;   // Forward/backward speed for right motor
const int TURN_SPEED        = 80;    // Speed used during rotation (lower = more precise)

// --- Motor Direction Inversion ---
// Set true if the physical motor spins in the wrong direction.
// Change ONLY this; do not swap wires.
bool INVERT_LEFT_MOTOR  = false;
bool INVERT_RIGHT_MOTOR = false;

// --- PWM Configuration ---
const int PWM_FREQUENCY  = 5000;   // PWM frequency in Hz
const int PWM_RESOLUTION = 8;      // 8-bit: 0..255

// --- Robot Physical Dimensions ---
// Measure these carefully from your physical robot.
float WHEEL_DIAMETER_CM = 3.3;    // Diameter of each drive wheel in cm
float WHEEL_BASE_CM     = 10.2;   // Distance between left and right wheel contact points in cm
float ROBOT_WIDTH_CM    = 11.2;   // Total width of robot in cm

// --- Encoder Calibration Values (EXPERIMENTALLY MEASURED) ---
// These values were measured on the physical robot.
// Update them if the robot is modified or wheels are changed.

// Encoder counts required to travel exactly 10 cm.
// Measured: 10 cm = 650 counts on both wheels.
// To recalibrate: use command D and adjust until M 10 = 10 cm.
const long COUNTS_FOR_10CM = 650;

// Encoder counts (per wheel) for a 90-degree in-place turn.
// Measured: 90 deg = 520 counts on both wheels.
// To recalibrate: use command T and adjust until L90 = exactly 90 deg.
const long COUNTS_FOR_90_DEG  = 520;

// Encoder counts (per wheel) for a 180-degree in-place turn.
// Initial value = 2 x COUNTS_FOR_90_DEG. Physically verify with U command.
const long COUNTS_FOR_180_DEG = 1040;

// --- Straight-Line Sync Correction ---
// Simple proportional correction to keep left/right wheels equal.
// If robot curves left: increase this slightly.
// If robot curves right: decrease this slightly.
// 0 = no correction. Typical starting range: 0.3 to 1.5
float ENCODER_SYNC_KP = 0.5;

// --- Encoder Debug Output ---
// How often (ms) encoder counts are printed when DEBUG_ENCODERS = true.
const unsigned long ENCODER_PRINT_INTERVAL_MS = 250;
const bool DEBUG_ENCODERS = true;   // Set false to silence periodic prints
const bool DEBUG_MOTORS   = true;   // Print motor command feedback

// --- Active Motor Braking ---
// Active brake duration after reaching encoder target.
// The TB6612FNG is driven into its BRAKE state (AIN1=HIGH, AIN2=HIGH)
// for this duration to stop motor momentum quickly.
// Increase if robot still overshoots after stopping.
// Decrease if braking causes jerk or overshoots in reverse.
const int BRAKE_TIME_MS = 80;

// --- Target Approach Slowdown Speeds ---
// When approaching the encoder target, the robot slows down in two stages
// to reduce overshoot before active braking takes effect.
// Stage 1: from 80% of target onward — medium speed.
// Stage 2: from 92% of target onward — very slow.
// Set both equal to LEFT/RIGHT_MOTOR_SPEED to disable slowdown.
const int SLOWDOWN_SPEED_1 = 70;   // Speed at 80-92% of target counts
const int SLOWDOWN_SPEED_2 = 50;   // Speed at 92-100% of target counts

// =====================================================
// END USER CALIBRATION SETTINGS
// =====================================================


// --- Motor A (LEFT wheel) pins ---
const int AIN1 = 14;
const int AIN2 = 27;
const int PWMA = 23;

// --- Motor B (RIGHT wheel) pins ---
const int BIN1 = 19;
const int BIN2 = 18;
const int PWMB = 5;

// --- Encoder pins ---
const int LEFT_ENCODER_PIN  = 33;   // GPIO 33
const int RIGHT_ENCODER_PIN = 34;   // GPIO 34 (input-only, no pullup)

// --- PWM ledc channels ---
const int PWMA_CHANNEL = 0;
const int PWMB_CHANNEL = 1;

// --- Encoder counters (modified in ISR) ---
volatile long left_encoder_count  = 0;
volatile long right_encoder_count = 0;

// --- Current movement state ---
bool isMoving = false;


// =====================================================
// Interrupt Service Routines
// =====================================================

void IRAM_ATTR handleLeftEncoder()  { left_encoder_count++;  }
void IRAM_ATTR handleRightEncoder() { right_encoder_count++; }


// =====================================================
// Motor Control
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

// =====================================================
// Active Motor Braking (TB6612FNG)
// =====================================================
// The TB6612FNG enters BRAKE mode when both direction pins
// are HIGH at the same time. This creates a short-circuit
// across the motor windings, generating back-EMF that
// slows the motor much faster than simply cutting PWM.
//
// Brake sequence:
//   1. Set AIN1=HIGH, AIN2=HIGH (brake state)
//   2. Apply full PWM to maximise braking force
//   3. Hold for BRAKE_TIME_MS
//   4. Cut PWM and release pins to idle (coast-safe state)

void brakeLeftMotor() {
    // TB6612FNG BRAKE: both direction pins HIGH
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA_CHANNEL, 255);   // Full PWM for maximum braking force
    delay(BRAKE_TIME_MS);
    // Release to safe idle
    ledcWrite(PWMA_CHANNEL, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
}

void brakeRightMotor() {
    // TB6612FNG BRAKE: both direction pins HIGH
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB_CHANNEL, 255);   // Full PWM for maximum braking force
    delay(BRAKE_TIME_MS);
    // Release to safe idle
    ledcWrite(PWMB_CHANNEL, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

void brakeBothMotors() {
    // Apply brake to both motors simultaneously
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    ledcWrite(PWMA_CHANNEL, 255);
    ledcWrite(PWMB_CHANNEL, 255);
    delay(BRAKE_TIME_MS);
    // Release both to safe idle
    ledcWrite(PWMA_CHANNEL, 0);
    ledcWrite(PWMB_CHANNEL, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    isMoving = false;
}

// Thread-safe read of both encoder counts
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
    Serial.println("Encoder counts cleared.");
}

void printEncoderCounts() {
    long l, r;
    readEncoders(l, r);
    Serial.print("Left Encoder : "); Serial.println(l);
    Serial.print("Right Encoder: "); Serial.println(r);
    Serial.print("Difference   : "); Serial.println(l - r);
}


// =====================================================
// Distance Helpers
// =====================================================
// Based on experimentally measured values:
//   COUNTS_FOR_10CM = 650  (10 cm = 650 encoder counts)

// Convert a distance in cm to the required encoder count.
long distanceToCounts(float cm) {
    return (long)(cm * (float)COUNTS_FOR_10CM / 10.0f);
}

// Convert an encoder count back to cm (for display only).
float countsToCm(long counts) {
    return (float)counts * 10.0f / (float)COUNTS_FOR_10CM;
}


// =====================================================
// Encoder-Based Move: Distance
// Blocks until target reached or S is pressed.
// Uses simple proportional sync to stay straight.
// =====================================================

void moveDistance(float targetCm) {
    if (COUNTS_FOR_10CM <= 0) {
        Serial.println("ERROR: COUNTS_FOR_10CM is not set.");
        Serial.println("Set COUNTS_FOR_10CM in the USER CALIBRATION SETTINGS section.");
        return;
    }

    long targetCounts = distanceToCounts(abs(targetCm));
    if (targetCounts <= 0) {
        Serial.println("ERROR: Invalid distance.");
        return;
    }

    bool forward = (targetCm >= 0);
    Serial.print("Moving "); Serial.print(forward ? "FORWARD" : "BACKWARD");
    Serial.print(" "); Serial.print(abs(targetCm)); Serial.println(" cm");
    Serial.print("Target counts: "); Serial.println(targetCounts);

    clearEncoderCounts();
    delay(10);

    isMoving = true;
    unsigned long lastPrint = 0;

    while (true) {
        // Check for manual stop
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                stopMotors();
                Serial.println("STOPPED by user.");
                printEncoderCounts();
                return;
            }
        }

        long lCount, rCount;
        readEncoders(lCount, rCount);
        long avgCount = (lCount + rCount) / 2;

        // Check if target reached
        if (avgCount >= targetCounts) {
            break;
        }

        // --- Approach slowdown ---
        // Reduce speed near the target to minimise overshoot
        // before active braking engages.
        int baseSpeed;
        if (avgCount >= (long)(targetCounts * 0.92f)) {
            baseSpeed = SLOWDOWN_SPEED_2;   // very slow: 92-100%
        } else if (avgCount >= (long)(targetCounts * 0.80f)) {
            baseSpeed = SLOWDOWN_SPEED_1;   // medium: 80-92%
        } else {
            baseSpeed = LEFT_MOTOR_SPEED;   // normal speed: 0-80%
        }

        // Proportional sync correction (applied on top of slowdown)
        long errorVal = lCount - rCount;
        int leftSpeed  = baseSpeed - (int)(ENCODER_SYNC_KP * errorVal);
        int rightSpeed = baseSpeed + (int)(ENCODER_SYNC_KP * errorVal);
        leftSpeed  = constrain(leftSpeed,  30, 255);
        rightSpeed = constrain(rightSpeed, 30, 255);

        setLeftMotor(leftSpeed,  forward);
        setRightMotor(rightSpeed, forward);

        // Debug print
        if (millis() - lastPrint >= ENCODER_PRINT_INTERVAL_MS) {
            lastPrint = millis();
            Serial.print("Left: "); Serial.print(lCount);
            Serial.print("  Right: "); Serial.print(rCount);
            Serial.print("  Target: "); Serial.print(targetCounts);
            Serial.print("  Spd: "); Serial.println(baseSpeed);
        }

        delay(1);
    }

    // --- Active brake ---
    if (DEBUG_MOTORS) {
        long l, r; readEncoders(l, r);
        Serial.print("TARGET REACHED  Left:"); Serial.print(l);
        Serial.print("  Right:"); Serial.println(r);
        Serial.println("ACTIVE BRAKE");
        Serial.print("Brake time: "); Serial.print(BRAKE_TIME_MS); Serial.println(" ms");
    }
    brakeBothMotors();

    Serial.println("Movement complete.");
    printEncoderCounts();
    long l2, r2;
    readEncoders(l2, r2);
    Serial.print("Estimated distance: ");
    Serial.print(countsToCm((l2 + r2) / 2));
    Serial.println(" cm");
}


// =====================================================
// Encoder-Based Rotate
// direction: true = left (CCW), false = right (CW)
// =====================================================

void rotateCounts(long targetCounts, bool rotateLeft) {
    if (targetCounts <= 0) {
        Serial.println("ERROR: Invalid turn count (calibrate first).");
        return;
    }

    Serial.print("Rotating "); Serial.println(rotateLeft ? "LEFT" : "RIGHT");
    Serial.print("Target counts: "); Serial.println(targetCounts);

    clearEncoderCounts();
    delay(10);

    isMoving = true;
    unsigned long lastPrint = 0;

    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                stopMotors();
                Serial.println("STOPPED by user.");
                printEncoderCounts();
                return;
            }
        }

        long lCount, rCount;
        readEncoders(lCount, rCount);
        long avgCount = (lCount + rCount) / 2;

        if (avgCount >= targetCounts) break;

        // --- Approach slowdown for turns ---
        int turnSpd;
        if (avgCount >= (long)(targetCounts * 0.90f)) {
            turnSpd = SLOWDOWN_SPEED_2;   // very slow: 90-100%
        } else if (avgCount >= (long)(targetCounts * 0.78f)) {
            turnSpd = SLOWDOWN_SPEED_1;   // medium: 78-90%
        } else {
            turnSpd = TURN_SPEED;         // normal: 0-78%
        }

        if (rotateLeft) {
            // Left wheel backward, right wheel forward
            setLeftMotor(turnSpd, false);
            setRightMotor(turnSpd, true);
        } else {
            // Left wheel forward, right wheel backward
            setLeftMotor(turnSpd, true);
            setRightMotor(turnSpd, false);
        }

        if (millis() - lastPrint >= ENCODER_PRINT_INTERVAL_MS) {
            lastPrint = millis();
            Serial.print("Left: "); Serial.print(lCount);
            Serial.print("  Right: "); Serial.print(rCount);
            Serial.print("  Target: "); Serial.print(targetCounts);
            Serial.print("  Spd: "); Serial.println(turnSpd);
        }

        delay(1);
    }

    // --- Active brake ---
    if (DEBUG_MOTORS) {
        long l, r; readEncoders(l, r);
        Serial.print("TARGET REACHED  Left:"); Serial.print(l);
        Serial.print("  Right:"); Serial.println(r);
        Serial.println("ACTIVE BRAKE");
        Serial.print("Brake time: "); Serial.print(BRAKE_TIME_MS); Serial.println(" ms");
    }
    brakeBothMotors();

    Serial.println("Rotation complete.");
    printEncoderCounts();
}


// =====================================================
// Continuous Movement (manual stop with S)
// =====================================================

void continuousMove(bool forward) {
    if (DEBUG_MOTORS) Serial.println(forward ? "Moving FORWARD (send S to stop)" : "Moving BACKWARD (send S to stop)");
    isMoving = true;
    clearEncoderCounts();

    unsigned long lastPrint = 0;

    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                stopMotors();
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
        leftSpeed  = constrain(leftSpeed,  40, 255);
        rightSpeed = constrain(rightSpeed, 40, 255);

        setLeftMotor(leftSpeed,  forward);
        setRightMotor(rightSpeed, forward);

        if (DEBUG_ENCODERS && millis() - lastPrint >= ENCODER_PRINT_INTERVAL_MS) {
            lastPrint = millis();
            Serial.print("Left: "); Serial.print(lCount);
            Serial.print("  Right: "); Serial.print(rCount);
            Serial.print("  Diff: "); Serial.println(errorVal);
        }
        delay(1);
    }
}

void continuousRotate(bool rotateLeft) {
    if (DEBUG_MOTORS) Serial.println(rotateLeft ? "Rotating LEFT (send S to stop)" : "Rotating RIGHT (send S to stop)");
    isMoving = true;
    clearEncoderCounts();

    unsigned long lastPrint = 0;

    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'S' || c == 's') {
                stopMotors();
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

        if (DEBUG_ENCODERS && millis() - lastPrint >= ENCODER_PRINT_INTERVAL_MS) {
            lastPrint = millis();
            long l, r;
            readEncoders(l, r);
            Serial.print("Left: "); Serial.print(l);
            Serial.print("  Right: "); Serial.println(r);
        }
        delay(1);
    }
}


// =====================================================
// Calibration Help Printers
// =====================================================

void printWheelCalibrationHelp() {
    Serial.println();
    Serial.println("=== DISTANCE CALIBRATION STATUS (W) ===");
    Serial.println();
    Serial.print("  COUNTS_FOR_10CM   = "); Serial.println(COUNTS_FOR_10CM);
    Serial.print("  COUNTS_FOR_90_DEG = "); Serial.println(COUNTS_FOR_90_DEG);
    Serial.print("  COUNTS_FOR_180_DEG= "); Serial.println(COUNTS_FOR_180_DEG);
    Serial.println();
    Serial.println("  To recalibrate distance:");
    Serial.println("    1. Place robot on floor. Mark start position.");
    Serial.println("    2. Send: M 10");
    Serial.println("    3. Measure actual distance with a ruler.");
    Serial.println("    4. Adjust COUNTS_FOR_10CM in code:");
    Serial.println("       - Moved less than 10cm: decrease COUNTS_FOR_10CM");
    Serial.println("       - Moved more than 10cm: increase COUNTS_FOR_10CM");
    Serial.println("========================================");
    Serial.println();
}

void printDistanceCalibrationHelp() {
    Serial.println();
    Serial.println("=== DISTANCE CALIBRATION (D) ===");
    Serial.println();
    Serial.println("Goal: verify robot travels accurate distance.");
    Serial.println();
    Serial.println("Current calibration:");
    Serial.print("  COUNTS_FOR_10CM = "); Serial.println(COUNTS_FOR_10CM);
    Serial.print("  10 cm target counts = "); Serial.println(distanceToCounts(10.0));
    Serial.print("  1 cm = "); Serial.print((float)COUNTS_FOR_10CM / 10.0f, 1); Serial.println(" counts");
    Serial.println();
    Serial.println("Procedure:");
    Serial.println("  1. Place robot on floor. Mark start position.");
    Serial.println("  2. Send: M 10   (move forward 10 cm)");
    Serial.println("  3. Measure actual distance with a ruler.");
    Serial.println("  4. If actual != 10 cm, adjust COUNTS_FOR_10CM:");
    Serial.println("     - Robot moved less: decrease COUNTS_FOR_10CM");
    Serial.println("     - Robot moved more: increase COUNTS_FOR_10CM");
    Serial.println("  5. Upload and repeat until M 10 = 10 cm.");
    Serial.println("=================================");
    Serial.println();
}

void printTurnCalibrationHelp() {
    Serial.println();
    Serial.println("=== TURN CALIBRATION (T) ===");
    Serial.println();
    Serial.println("Current turn calibration:");
    Serial.print("  COUNTS_FOR_90_DEG  = "); Serial.println(COUNTS_FOR_90_DEG);
    Serial.print("  COUNTS_FOR_180_DEG = "); Serial.println(COUNTS_FOR_180_DEG);
    Serial.println();
    Serial.println("To re-measure COUNTS_FOR_90_DEG:");
    Serial.println("  1. Place robot on floor. Mark starting orientation.");
    Serial.println("  2. Send C to clear encoder counts.");
    Serial.println("  3. Send L (rotate left continuously).");
    Serial.println("  4. When robot has rotated exactly 90 degrees, send S.");
    Serial.println("  5. Send P. Average of Left and Right = COUNTS_FOR_90_DEG.");
    Serial.println("  6. Repeat 3 times. Update code and upload.");
    Serial.println("  7. Set COUNTS_FOR_180_DEG = 2 x COUNTS_FOR_90_DEG.");
    Serial.println("============================");
    Serial.println();
}


// =====================================================
// Help Menu
// =====================================================

void printHelp() {
    Serial.println();
    Serial.println("========================================");
    Serial.println(" V03 COMMANDS");
    Serial.println("========================================");
    Serial.println("  F      = Forward (sync, send S to stop)");
    Serial.println("  B      = Backward (sync, send S to stop)");
    Serial.println("  L      = Rotate left (send S to stop)");
    Serial.println("  R      = Rotate right (send S to stop)");
    Serial.println("  S      = STOP immediately");
    Serial.println();
    Serial.println("  M 10   = Move forward 10 cm (calibrated)");
    Serial.println("  M -10  = Move backward 10 cm (calibrated)");
    Serial.println();
    Serial.println("  L90    = Rotate 90 deg left (calibrated)");
    Serial.println("  R90    = Rotate 90 deg right (calibrated)");
    Serial.println("  U      = Rotate 180 deg (calibrated)");
    Serial.println();
    Serial.println("  C      = Clear encoder counts");
    Serial.println("  P      = Print encoder counts");
    Serial.println();
    Serial.println("  W      = Wheel calibration guide");
    Serial.println("  D      = Distance calibration guide");
    Serial.println("  T      = Turn calibration guide");
    Serial.println();
    Serial.println("  H      = Help");
    Serial.println("========================================");
    Serial.println();
}


// =====================================================
// Serial Command Parser
// Reads a full line then processes it.
// =====================================================

void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    String upper = cmd;
    upper.toUpperCase();

    // --- Single character commands ---
    if (upper == "F") { continuousMove(true);   return; }
    if (upper == "B") { continuousMove(false);  return; }
    if (upper == "L") { continuousRotate(true); return; }
    if (upper == "R") { continuousRotate(false);return; }

    if (upper == "S") {
        stopMotors();
        Serial.println("STOP");
        return;
    }

    if (upper == "C") { clearEncoderCounts(); return; }
    if (upper == "P") { printEncoderCounts(); return; }

    if (upper == "H") { printHelp(); return; }
    if (upper == "W") { printWheelCalibrationHelp(); return; }
    if (upper == "D") { printDistanceCalibrationHelp(); return; }
    if (upper == "T") { printTurnCalibrationHelp(); return; }

    // --- L90 / R90 ---
    if (upper == "L90") {
        if (COUNTS_FOR_90_DEG <= 0) {
            Serial.println("ERROR: COUNTS_FOR_90_DEG not calibrated.");
            Serial.println("Perform turn calibration (command T) first.");
            return;
        }
        rotateCounts(COUNTS_FOR_90_DEG, true);
        return;
    }
    if (upper == "R90") {
        if (COUNTS_FOR_90_DEG <= 0) {
            Serial.println("ERROR: COUNTS_FOR_90_DEG not calibrated.");
            Serial.println("Perform turn calibration (command T) first.");
            return;
        }
        rotateCounts(COUNTS_FOR_90_DEG, false);
        return;
    }

    // --- U (180 deg) ---
    if (upper == "U") {
        long target = COUNTS_FOR_180_DEG;
        if (target <= 0 && COUNTS_FOR_90_DEG > 0) {
            target = COUNTS_FOR_90_DEG * 2;
            Serial.println("Note: COUNTS_FOR_180_DEG not set, using 2 x COUNTS_FOR_90_DEG.");
        }
        if (target <= 0) {
            Serial.println("ERROR: Turn calibration not done. Use command T.");
            return;
        }
        rotateCounts(target, true);
        return;
    }

    // --- M <distance> ---
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

    // --- Unknown ---
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

    // PWM
    ledcSetup(PWMA_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWMB_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWMA, PWMA_CHANNEL);
    ledcAttachPin(PWMB, PWMB_CHANNEL);

    // SAFETY: stop motors at boot
    stopMotors();

    // Encoder interrupts
    pinMode(LEFT_ENCODER_PIN,  INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN),  handleLeftEncoder,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), handleRightEncoder, RISING);

    // Startup banner
    Serial.println();
    Serial.println("========================================");
    Serial.println(" FASTEST MAZE SOLVER");
    Serial.println(" VERSION 03");
    Serial.println(" MOVEMENT CALIBRATION");
    Serial.println("========================================");
    Serial.println();
    Serial.print("  Wheel Diameter   = "); Serial.print(WHEEL_DIAMETER_CM); Serial.println(" cm");
    Serial.print("  Wheel Base       = "); Serial.print(WHEEL_BASE_CM);     Serial.println(" cm");
    Serial.println();
    Serial.println("  --- Calibration Values (MEASURED) ---");
    Serial.print("  COUNTS_FOR_10CM    = "); Serial.println(COUNTS_FOR_10CM);
    Serial.print("  COUNTS_FOR_90_DEG  = "); Serial.println(COUNTS_FOR_90_DEG);
    Serial.print("  COUNTS_FOR_180_DEG = "); Serial.println(COUNTS_FOR_180_DEG);
    Serial.print("  ENCODER_SYNC_KP    = "); Serial.println(ENCODER_SYNC_KP);
    Serial.println();
    Serial.print("  1 cm = "); Serial.print((float)COUNTS_FOR_10CM / 10.0f, 1); Serial.println(" counts");
    Serial.print("  M 10 will target "); Serial.print(distanceToCounts(10.0)); Serial.println(" counts");

    Serial.println();
    Serial.println("  Robot is STOPPED. Waiting for command.");
    printHelp();
}


// =====================================================
// loop()
// =====================================================

void loop() {
    // Read a full line from Serial
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        handleCommand(line);
    }

    // Periodic encoder debug (only when not in a blocking move)
    static unsigned long lastEncoderPrint = 0;
    if (!isMoving && DEBUG_ENCODERS &&
        (millis() - lastEncoderPrint >= ENCODER_PRINT_INTERVAL_MS))
    {
        lastEncoderPrint = millis();
        long l, r;
        readEncoders(l, r);
        Serial.print("Left: "); Serial.print(l);
        Serial.print("  Right: "); Serial.print(r);
        Serial.print("  Diff: "); Serial.println(l - r);
    }
}
