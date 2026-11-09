/**
 * ============================================================
 *  HeadingLock_IMU.ino
 *  ESP32 + MPU6050 + TB6612FNG  --  Heading-Lock Experiment
 * ============================================================
 *
 *  PURPOSE
 *  -------
 *  Keep the robot facing whatever direction it was facing when
 *  the program started / calibration finished.
 *  Uses only the MPU6050 Z-axis gyroscope for relative yaw
 *  estimation.  No absolute compass, no encoders, no ToF,
 *  no Bluetooth, no maze-solving.
 *
 *  HARDWARE
 *  --------
 *  MCU         : ESP32 DOIT V1 / DevKit V1
 *  IMU         : MPU6050   SDA=GPIO21  SCL=GPIO22
 *  Motor driver: TB6612FNG (STBY tied to 3.3 V)
 *    Left  motor : AIN1=14  AIN2=27  PWMA=23
 *    Right motor : BIN1=19  BIN2=18  PWMB=5
 *
 *  HOW IT WORKS
 *  ------------
 *  1. Calibrate gyro bias while robot is stationary.
 *  2. Set targetHeading = 0 deg (whatever direction that is).
 *  3. Continuously integrate gyro Z to track currentHeading.
 *  4. headingError = targetHeading - currentHeading  (normalised)
 *  5. PD correction drives motors differentially to spin back.
 *  6. Inside HEADING_DEADBAND -> motors stop.
 *
 *  KNOWN LIMITATION
 *  ----------------
 *  MPU6050 gyro drifts slowly over time.  This system is a
 *  short-duration, relative heading lock -- not a permanent
 *  compass replacement.  A magnetometer would be needed for
 *  long-duration absolute heading.
 *
 * ============================================================
 */

#include <Wire.h>

// ============================================================
//  TUNING PARAMETERS  -- edit these to match your robot
// ============================================================

// --- Gyro calibration ----------------------------------------
#define GYRO_CALIBRATION_SAMPLES  1000   // samples at startup (robot must be still)

// --- PD controller -------------------------------------------
#define KP   4.5f   // proportional gain  (increase if sluggish, decrease if oscillates)
#define KD   0.08f  // derivative gain    (dampens overshoot; set 0 to use pure P)

// --- Dead-band -----------------------------------------------
#define HEADING_DEADBAND   2.0f  // degrees -- motors stay off inside this zone

// --- Motor PWM limits (0-255) --------------------------------
#define MIN_CORRECTION_PWM   55   // minimum PWM that actually moves the motors
#define MAX_CORRECTION_PWM  200   // cap to avoid full-throttle spins

// --- Direction sense -----------------------------------------
// If the robot corrects in the WRONG direction, set this to true.
#define INVERT_CORRECTION_DIRECTION  false

// --- Serial debug --------------------------------------------
#define DEBUG_ENABLED      true
#define DEBUG_INTERVAL_MS  100   // print to Serial every N milliseconds

// ============================================================
//  PIN DEFINITIONS  -- TB6612FNG
// ============================================================

// Left motor  (Channel A on TB6612FNG)
#define PIN_AIN1   14
#define PIN_AIN2   27
#define PIN_PWMA   23

// Right motor  (Channel B on TB6612FNG)
#define PIN_BIN1   19
#define PIN_BIN2   18
#define PIN_PWMB    5

// ============================================================
//  LEDC (ESP32 PWM) — pin-based API (Core 3.x compatible)
// ============================================================
#define LEDC_FREQ_HZ 5000
#define LEDC_RES_BITS   8   // 0-255  (duty range 0-255)

// ============================================================
//  MPU6050 I2C
// ============================================================
#define MPU6050_ADDR   0x68
#define GYRO_Z_H_REG   0x47   // GYRO_ZOUT_H register
#define PWR_MGMT_1_REG 0x6B

// Gyro full-scale +-250 deg/s -> sensitivity = 131.0 LSB/(deg/s)
#define GYRO_SENSITIVITY  131.0f

// ============================================================
//  GLOBAL STATE
// ============================================================
float gyroBiasZ      = 0.0f;   // calibrated bias (raw LSB/s)
float currentHeading = 0.0f;   // integrated yaw (degrees)
float targetHeading  = 0.0f;   // locked target  (degrees)

unsigned long prevTimeMicros  = 0;
unsigned long lastDebugMillis = 0;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void    setupMPU();
void    calibrateGyro();
float   readRawGyroZ();
void    updateHeading();
void    setTargetHeading();
float   calculateHeadingError();
float   normalizeAngle(float angle);

void    setupMotors();
void    setMotor(int dirA, int pwmA_val, int dirB, int pwmB_val);
void    rotateLeft(int pwm);
void    rotateRight(int pwm);
void    stopMotors();

void    printDebug(float gyroZ_dps, float error, int correction);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);  // let serial port settle

  Serial.println("============================================");
  Serial.println("  HeadingLock_IMU  --  ESP32 + MPU6050");
  Serial.println("============================================");

  // 1. I2C
  Wire.begin(21, 22);   // SDA=21, SCL=22
  Wire.setClock(400000);

  // 2. MPU6050
  setupMPU();

  // 3. Motors (initialise but keep stopped)
  setupMotors();
  stopMotors();

  // 4. Gyro calibration -- keep robot STILL
  Serial.println("\n[CALIBRATION] Keep robot COMPLETELY STILL ...");
  calibrateGyro();
  Serial.print("[CALIBRATION] Done. Bias = ");
  Serial.print(gyroBiasZ, 4);
  Serial.println(" raw LSB/s");

  // 5. Reset heading and lock target
  currentHeading  = 0.0f;
  targetHeading   = 0.0f;
  prevTimeMicros  = micros();

  Serial.println("[HEADING LOCK] Target = 0.0 deg");
  Serial.println("[READY] Rotate the robot to test correction.");
  Serial.println("============================================\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // --- 1. Update heading from gyro --------------------------
  updateHeading();

  // --- 2. Compute error & correction ------------------------
  float error = calculateHeadingError();

  int correctionPWM = 0;

  if (fabsf(error) >= HEADING_DEADBAND) {
    // Proportional term
    float rawCorrection = KP * error;

    // Derivative term (damp oscillation using current gyro Z)
    float gyroZ_dps  = (readRawGyroZ() - gyroBiasZ) / GYRO_SENSITIVITY;
    rawCorrection   -= KD * gyroZ_dps;

    // Clamp to PWM limits
    rawCorrection = constrain(rawCorrection,
                              -(float)MAX_CORRECTION_PWM,
                               (float)MAX_CORRECTION_PWM);

    // Apply minimum useful PWM so motors actually move
    if (rawCorrection > 0.0f && rawCorrection < MIN_CORRECTION_PWM)
      rawCorrection = (float)MIN_CORRECTION_PWM;
    else if (rawCorrection < 0.0f && rawCorrection > -MIN_CORRECTION_PWM)
      rawCorrection = -(float)MIN_CORRECTION_PWM;

    correctionPWM = (int)rawCorrection;

    // Optional direction inversion (flip if wiring is reversed)
    if (INVERT_CORRECTION_DIRECTION)
      correctionPWM = -correctionPWM;

    //
    //  Motor correction logic
    //  ----------------------
    //  Positive headingError means currentHeading < targetHeading,
    //  i.e. the robot has rotated CW (negative yaw change) relative
    //  to target.  We need to spin CCW to return -> rotateLeft.
    //
    //  Negative headingError -> robot rotated CCW -> spin CW -> rotateRight.
    //
    if (correctionPWM > 0)
      rotateLeft(correctionPWM);
    else
      rotateRight(-correctionPWM);

  } else {
    // Within deadband -- robot is aligned
    stopMotors();
  }

  // --- 3. Debug output --------------------------------------
#if DEBUG_ENABLED
  unsigned long now = millis();
  if (now - lastDebugMillis >= DEBUG_INTERVAL_MS) {
    lastDebugMillis = now;
    float gyroZ_dps = (readRawGyroZ() - gyroBiasZ) / GYRO_SENSITIVITY;
    printDebug(gyroZ_dps, error, correctionPWM);
  }
#endif
}

// ============================================================
//  MPU6050 FUNCTIONS
// ============================================================

/** Write one byte to an MPU6050 register */
static void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

/** Wake up MPU6050, set gyro to +-250 deg/s full-scale */
void setupMPU() {
  // Wake from sleep
  mpuWrite(PWR_MGMT_1_REG, 0x00);
  delay(100);

  // GYRO_CONFIG register 0x1B -> FS_SEL=0 (+-250 deg/s)
  mpuWrite(0x1B, 0x00);
  delay(10);

  // ACCEL_CONFIG register 0x1C -> AFS_SEL=0 (+-2g, not used but set cleanly)
  mpuWrite(0x1C, 0x00);
  delay(10);

  // DLPF_CONFIG 0x1A -> bandwidth ~42 Hz (smooths gyro noise)
  mpuWrite(0x1A, 0x03);
  delay(10);

  Serial.println("[MPU6050] Initialised. FS=+-250 deg/s, DLPF=42Hz");
}

/** Read raw 16-bit signed gyro Z from the sensor */
float readRawGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_Z_H_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (uint8_t)true);

  if (Wire.available() < 2) return 0.0f;
  int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
  return (float)raw;
}

/**
 *  Collect GYRO_CALIBRATION_SAMPLES readings while stationary
 *  and compute average bias on Z axis.
 */
void calibrateGyro() {
  double sum = 0.0;

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    sum += readRawGyroZ();
    delay(2);  // ~2 ms per sample -> ~2 s total for 1000 samples
  }

  gyroBiasZ = (float)(sum / GYRO_CALIBRATION_SAMPLES);
}

/**
 *  Integrate gyro Z to accumulate yaw.
 *  Uses real delta-time from micros() -- no fixed-dt assumption.
 */
void updateHeading() {
  unsigned long nowMicros = micros();
  float dt = (float)(nowMicros - prevTimeMicros) * 1e-6f;  // seconds
  prevTimeMicros = nowMicros;

  // Guard against implausibly large dt (e.g. first call after a delay)
  if (dt <= 0.0f || dt > 0.5f) return;

  float rawZ  = readRawGyroZ();
  float gyroZ = (rawZ - gyroBiasZ) / GYRO_SENSITIVITY;  // deg/s

  currentHeading += gyroZ * dt;
  currentHeading  = normalizeAngle(currentHeading);
}

/**
 *  Set the target heading equal to the current heading.
 *  Called once after calibration (in setup).
 */
void setTargetHeading() {
  targetHeading = currentHeading;
}

/**
 *  Compute heading error with correct shortest-path wrap-around.
 *  Result is always in (-180, +180].
 *
 *  Example:
 *    target =  179 deg,  current = -179 deg  ->  error = -2 deg  (NOT 358 deg)
 */
float calculateHeadingError() {
  return normalizeAngle(targetHeading - currentHeading);
}

/**
 *  Normalise an angle to (-180, +180] degrees.
 *
 *  Examples:
 *    normalizeAngle( 370)  ->   10
 *    normalizeAngle(-190)  ->  170
 *    normalizeAngle(  -2)  ->   -2
 */
float normalizeAngle(float angle) {
  while (angle >  180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================

/** Configure motor pins and LEDC PWM (pin-based API, Core 3.x) */
void setupMotors() {
  // Direction pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // Attach PWM pins — ledcAttach(pin, freq, resolution_bits)
  ledcAttach(PIN_PWMA, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(PIN_PWMB, LEDC_FREQ_HZ, LEDC_RES_BITS);

  stopMotors();
  Serial.println("[MOTORS] Initialised (TB6612FNG, LEDC PWM 5 kHz 8-bit).");
}

/**
 *  Low-level motor driver.
 *  dirA / dirB : +1=forward, -1=backward, 0=brake
 *  pwmA_val / pwmB_val : 0-255
 */
void setMotor(int dirA, int pwmA_val, int dirB, int pwmB_val) {
  // --- Left motor (Channel A) ---
  pwmA_val = constrain(pwmA_val, 0, 255);
  if (dirA > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (dirA < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    pwmA_val = 0;
  }
  ledcWrite(PIN_PWMA, pwmA_val);  // pin-based write (Core 3.x)

  // --- Right motor (Channel B) ---
  pwmB_val = constrain(pwmB_val, 0, 255);
  if (dirB > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else if (dirB < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    pwmB_val = 0;
  }
  ledcWrite(PIN_PWMB, pwmB_val);  // pin-based write (Core 3.x)
}

/** Brake both motors to ground (AIN1=AIN2=LOW) */
void stopMotors() {
  setMotor(0, 0, 0, 0);
}

/**
 *  Spin robot CCW in place (viewed from above).
 *  Left motor BACKWARD, Right motor FORWARD.
 *
 *  Positive headingError -> robot drifted CW -> correct CCW.
 */
void rotateLeft(int pwm) {
  pwm = constrain(pwm, 0, 255);
  setMotor(-1, pwm, +1, pwm);
}

/**
 *  Spin robot CW in place (viewed from above).
 *  Left motor FORWARD, Right motor BACKWARD.
 *
 *  Negative headingError -> robot drifted CCW -> correct CW.
 */
void rotateRight(int pwm) {
  pwm = constrain(pwm, 0, 255);
  setMotor(+1, pwm, -1, pwm);
}

// ============================================================
//  DEBUG OUTPUT
// ============================================================
void printDebug(float gyroZ_dps, float error, int correction) {
  Serial.print("GyroZ: ");        Serial.print(gyroZ_dps, 2);
  Serial.print(" deg/s  |  Heading: "); Serial.print(currentHeading, 2);
  Serial.print(" deg  |  Target: ");    Serial.print(targetHeading, 2);
  Serial.print(" deg  |  Error: ");     Serial.print(error, 2);
  Serial.print(" deg  |  PWM: ");       Serial.println(correction);
}
