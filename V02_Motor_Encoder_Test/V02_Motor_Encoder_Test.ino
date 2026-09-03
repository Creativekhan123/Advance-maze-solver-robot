#include <Arduino.h>

// =====================================================
// USER CONFIGURATION
// =====================================================

// Left motor PWM speed: 0-255. Keep low (~100) for initial testing.
const int LEFT_MOTOR_SPEED = 100;
// Right motor PWM speed: 0-255.
const int RIGHT_MOTOR_SPEED = 100;
// Speed used during left/right rotation.
const int TURN_SPEED = 100;

// Set true if left motor physically spins in the wrong direction.
// Change this before touching motor wires.
const bool INVERT_LEFT_MOTOR = false;
// Set true if right motor physically spins in the wrong direction.
const bool INVERT_RIGHT_MOTOR = false;

// PWM carrier frequency in Hz.
const int PWM_FREQUENCY = 5000;
// PWM bit resolution (8-bit means range 0..255).
const int PWM_RESOLUTION = 8;

// ESP32 ledc channels assigned to each motor PWM pin.
const int PWMA_CHANNEL = 0; // Motor A left wheel
const int PWMB_CHANNEL = 1; // Motor B right wheel

// Print motor command feedback to Serial Monitor.
const bool DEBUG_MOTORS = true;
// Periodically auto-print encoder counts.
const bool DEBUG_ENCODERS = true;

// How often (ms) encoder counts are auto-printed when DEBUG_ENCODERS=true.
const unsigned long ENCODER_PRINT_INTERVAL_MS = 250;

// =====================================================
// END USER CONFIGURATION
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
const int LEFT_ENCODER_PIN = 33;  // GPIO 33
const int RIGHT_ENCODER_PIN = 34; // GPIO 34 -- input-only, no internal pullup

// Encoder pulse counters.
// volatile required because they are modified inside ISR.
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// =====================================================
// Interrupt Service Routines
// =====================================================
// Single-channel encoder: we count RISING pulses.
// Direction cannot be determined without a second channel.
// V02 goal: verify pulses are being counted correctly.

void IRAM_ATTR handleLeftEncoder() { left_encoder_count++; }

void IRAM_ATTR handleRightEncoder() { right_encoder_count++; }

// =====================================================
// Motor Control Functions
// =====================================================

// Set left motor speed and direction.
void setLeftMotor(int speed, bool forward) {
  if (INVERT_LEFT_MOTOR)
    forward = !forward;
  speed = constrain(speed, 0, 255);
  ledcWrite(PWMA_CHANNEL, speed);
  if (forward) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
}

// Set right motor speed and direction.
void setRightMotor(int speed, bool forward) {
  if (INVERT_RIGHT_MOTOR)
    forward = !forward;
  speed = constrain(speed, 0, 255);
  ledcWrite(PWMB_CHANNEL, speed);
  if (forward) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
}

// Stop both motors immediately.
void stopMotors() {
  ledcWrite(PWMA_CHANNEL, 0);
  ledcWrite(PWMB_CHANNEL, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void moveForward() {
  setLeftMotor(LEFT_MOTOR_SPEED, true);
  setRightMotor(RIGHT_MOTOR_SPEED, true);
  if (DEBUG_MOTORS)
    Serial.println("Moving FORWARD");
}

void moveBackward() {
  setLeftMotor(LEFT_MOTOR_SPEED, false);
  setRightMotor(RIGHT_MOTOR_SPEED, false);
  if (DEBUG_MOTORS)
    Serial.println("Moving BACKWARD");
}

void rotateLeft() {
  // Left wheel backward, right wheel forward
  setLeftMotor(TURN_SPEED, false);
  setRightMotor(TURN_SPEED, true);
  if (DEBUG_MOTORS)
    Serial.println("Rotating LEFT");
}

void rotateRight() {
  // Left wheel forward, right wheel backward
  setLeftMotor(TURN_SPEED, true);
  setRightMotor(TURN_SPEED, false);
  if (DEBUG_MOTORS)
    Serial.println("Rotating RIGHT");
}

void leftMotorOnly() {
  // Stop right motor, run left only
  ledcWrite(PWMB_CHANNEL, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  setLeftMotor(LEFT_MOTOR_SPEED, true);
  if (DEBUG_MOTORS)
    Serial.println("Left motor ONLY");
}

void rightMotorOnly() {
  // Stop left motor, run right only
  ledcWrite(PWMA_CHANNEL, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  setRightMotor(RIGHT_MOTOR_SPEED, true);
  if (DEBUG_MOTORS)
    Serial.println("Right motor ONLY");
}

// =====================================================
// Encoder Utilities
// =====================================================

// Thread-safe read and print of both encoder counts.
void printEncoderCounts() {
  noInterrupts();
  long lCount = left_encoder_count;
  long rCount = right_encoder_count;
  interrupts();
  Serial.print("Left Encoder : ");
  Serial.println(lCount);
  Serial.print("Right Encoder: ");
  Serial.println(rCount);
}

// Reset both encoder counters to zero.
void clearEncoderCounts() {
  noInterrupts();
  left_encoder_count = 0;
  right_encoder_count = 0;
  interrupts();
  Serial.println("Encoder counts cleared.");
}

// =====================================================
// Help Menu
// =====================================================

void printHelp() {
  Serial.println();
  Serial.println("========================================");
  Serial.println(" COMMANDS");
  Serial.println("========================================");
  Serial.println("  F = Forward");
  Serial.println("  B = Backward");
  Serial.println("  L = Rotate Left");
  Serial.println("  R = Rotate Right");
  Serial.println("  S = Stop");
  Serial.println("  1 = Left motor only");
  Serial.println("  2 = Right motor only");
  Serial.println("  C = Clear encoder counts");
  Serial.println("  P = Print encoder counts");
  Serial.println("  H = Help");
  Serial.println("========================================");
  Serial.println();
}

// =====================================================
// Serial Command Handler
// =====================================================

void handleSerialCommand(char cmd) {
  switch (cmd) {
  case 'f':
  case 'F':
    moveForward();
    break;
  case 'b':
  case 'B':
    moveBackward();
    break;
  case 'l':
  case 'L':
    rotateLeft();
    break;
  case 'r':
  case 'R':
    rotateRight();
    break;
  case 's':
  case 'S':
    stopMotors();
    if (DEBUG_MOTORS)
      Serial.println("STOP");
    break;
  case '1':
    leftMotorOnly();
    break;
  case '2':
    rightMotorOnly();
    break;
  case 'c':
  case 'C':
    clearEncoderCounts();
    break;
  case 'p':
  case 'P':
    printEncoderCounts();
    break;
  case 'h':
  case 'H':
    printHelp();
    break;
  case '\r':
  case '\n':
    break; // ignore line endings silently
  default:
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    Serial.println("Type H for help.");
    break;
  }
}

// =====================================================
// setup()
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor direction pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // PWM setup using ESP32 ledc API
  ledcSetup(PWMA_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWMB_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWMA_CHANNEL);
  ledcAttachPin(PWMB, PWMB_CHANNEL);

  // SAFETY: ensure both motors are stopped at boot
  stopMotors();

  // Encoder pin setup
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT); // GPIO34 has no internal pullup
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), handleLeftEncoder,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), handleRightEncoder,
                  RISING);

  // Startup banner
  Serial.println();
  Serial.println("========================================");
  Serial.println(" FASTEST MAZE SOLVER");
  Serial.println(" VERSION 02");
  Serial.println(" MOTOR + ENCODER TEST");
  Serial.println("========================================");
  Serial.println();
  Serial.println("  Motor A = LEFT  wheel  (AIN1=14 AIN2=27 PWM=23)");
  Serial.println("  Motor B = RIGHT wheel  (BIN1=19 BIN2=18 PWM=5)");
  Serial.println();
  Serial.println("  Left  Encoder = GPIO 33");
  Serial.println("  Right Encoder = GPIO 34");
  Serial.println();
  Serial.print("  PWM Frequency  = ");
  Serial.print(PWM_FREQUENCY);
  Serial.println(" Hz");
  Serial.print("  PWM Resolution = ");
  Serial.print(PWM_RESOLUTION);
  Serial.println("-bit");
  Serial.println();
  Serial.print("  Left  Motor Speed = ");
  Serial.println(LEFT_MOTOR_SPEED);
  Serial.print("  Right Motor Speed = ");
  Serial.println(RIGHT_MOTOR_SPEED);
  Serial.print("  Turn  Speed       = ");
  Serial.println(TURN_SPEED);
  Serial.println();
  Serial.print("  Invert Left Motor  : ");
  Serial.println(INVERT_LEFT_MOTOR ? "YES" : "NO");
  Serial.print("  Invert Right Motor : ");
  Serial.println(INVERT_RIGHT_MOTOR ? "YES" : "NO");
  Serial.println();
  Serial.println("  Robot is STOPPED. Waiting for command.");
  printHelp();
}

// =====================================================
// loop()
// =====================================================
void loop() {
  // Process Serial commands
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    handleSerialCommand(cmd);
  }

  // Auto-print encoder counts at configured interval
  static unsigned long lastEncoderPrint = 0;
  if (DEBUG_ENCODERS &&
      (millis() - lastEncoderPrint >= ENCODER_PRINT_INTERVAL_MS)) {
    lastEncoderPrint = millis();
    printEncoderCounts();
  }
}
