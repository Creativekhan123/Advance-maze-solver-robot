/**
 * ============================================================
 *  GyroRotationZ_Test.ino
 *  ESP32 + MPU6050  --  Gyro Z Rotation Accumulator
 * ============================================================
 *
 *  PURPOSE
 *  -------
 *  Measure and accumulate the total relative rotation around
 *  the Z axis (yaw) using the MPU6050 gyroscope only.
 *
 *  No motors. No heading correction. No PID. No maze solving.
 *
 *  PUBLIC API
 *  ----------
 *    updateGyroRotation()   -- call every loop iteration
 *    getGyroRotationZ()     -- returns accumulated degrees
 *    resetGyroRotationZ()   -- zeroes the accumulator
 *
 *  LIBRARY REQUIREMENT
 *  -------------------
 *  None beyond the built-in <Wire.h>.
 *  All MPU6050 communication is via raw I2C register reads.
 *  No third-party library installation needed.
 *
 *  HARDWARE
 *  --------
 *  MCU : ESP32 DOIT V1 / DevKit V1
 *  IMU : MPU6050   SDA=GPIO21   SCL=GPIO22
 *
 * ============================================================
 */

#include <Wire.h>

// ============================================================
//  TUNING PARAMETERS  -- adjust these as needed
// ============================================================

// Number of stationary samples used to calculate gyro bias.
// Robot MUST be completely still during calibration (~2 s at 1000).
#define GYRO_CALIBRATION_SAMPLES   1000

// Set true if clockwise rotation produces negative values on your
// physical setup and you want clockwise to be positive.
#define GYRO_Z_INVERT              false

// Angular velocity below this threshold (deg/s) is treated as zero.
// Helps reduce slow accumulation of noise while stationary.
// Keep small so slow intentional rotations are still detected.
#define GYRO_DEADZONE              0.5f   // deg/s

// Serial print interval
#define DEBUG_INTERVAL_MS          100    // milliseconds

// ============================================================
//  MPU6050 REGISTER MAP
// ============================================================
#define MPU6050_ADDR     0x68   // I2C address (AD0 = LOW)
#define REG_PWR_MGMT_1   0x6B   // power management
#define REG_GYRO_CONFIG  0x1B   // gyro full-scale select
#define REG_ACCEL_CONFIG 0x1C   // accel full-scale (unused but initialised)
#define REG_CONFIG       0x1A   // DLPF config
#define REG_GYRO_ZOUT_H  0x47   // gyro Z high byte (followed by low byte)
#define REG_WHO_AM_I     0x75   // WHO_AM_I register (default 0x68)

// Gyro sensitivity for +-250 deg/s full-scale
#define GYRO_SENSITIVITY  131.0f   // LSB / (deg/s)

// ============================================================
//  PRIVATE GLOBAL STATE
// ============================================================
static float         gyroBiasZ     = 0.0f;  // stationary bias (raw LSB/s)
static float         rotationZ     = 0.0f;  // accumulated rotation (degrees)
static unsigned long prevTimeMicros = 0;     // timestamp of last update

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
bool  setupMPU6050();
void  calibrateGyro();
float readRawGyroZ();

// --- Public API ---
void  updateGyroRotation();
float getGyroRotationZ();
void  resetGyroRotationZ();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("============================================");
  Serial.println("  GyroRotationZ_Test  --  ESP32 + MPU6050");
  Serial.println("============================================");

  // Initialise I2C
  Wire.begin(21, 22);      // SDA=21, SCL=22
  Wire.setClock(400000);   // 400 kHz fast mode

  // Check and initialise MPU6050; do not proceed until detected
  Serial.println("[MPU6050] Detecting sensor...");
  while (!setupMPU6050()) {
    Serial.println("[ERROR] MPU6050 NOT found! Check wiring (SDA=21, SCL=22, VCC, GND). Retrying in 1s...");
    delay(1000);
  }
  Serial.println("[MPU6050] Sensor detected and initialized successfully!");

  // Calibrate gyro bias -- keep robot STILL
  Serial.println("\n[CALIBRATION] Keep robot COMPLETELY STILL ...");
  calibrateGyro();
  Serial.print("[CALIBRATION] Complete. gyroBiasZ = ");
  Serial.print(gyroBiasZ, 4);
  Serial.println(" raw LSB/s");

  // Reset accumulator and timer
  resetGyroRotationZ();

  Serial.println("[READY] Rotate the robot to accumulate rotation.");
  Serial.println("        Call resetGyroRotationZ() to zero the counter.");
  Serial.println("============================================\n");
  Serial.println("  Gyro Z (deg/s)   |   Rotation Z (deg)");
  Serial.println("------------------+--------------------");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // Always update the gyro accumulator every iteration.
  // The faster this is called, the more accurate the integration.
  updateGyroRotation();

  // Print at a fixed interval (not every microsecond)
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= DEBUG_INTERVAL_MS) {
    lastPrint = now;

    // Read instantaneous gyro Z for display (already corrected and inverted)
    float rawZ     = readRawGyroZ();
    float corrZ    = (rawZ - gyroBiasZ) / GYRO_SENSITIVITY;  // deg/s
    if (GYRO_Z_INVERT) corrZ = -corrZ;

    Serial.print("Gyro Z: ");
    Serial.print(corrZ, 2);
    Serial.print(" deg/s   |   Rotation Z: ");
    Serial.print(getGyroRotationZ(), 2);
    Serial.println(" deg");
  }
}

// ============================================================
//  MPU6050 SETUP
// ============================================================

/** Write one byte to an MPU6050 register over I2C */
static bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission(true) == 0);
}

/** Read one byte from an MPU6050 register over I2C */
static uint8_t mpuRead(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1, (uint8_t)true);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

/**
 *  Check connection and initialise the MPU6050:
 *    - Verify device responds on I2C address
 *    - Check WHO_AM_I register (0x68 or 0x72 for MPU6050/MPU6500)
 *    - Wake from sleep
 *    - Gyro full-scale = +-250 deg/s  (sensitivity = 131 LSB/deg/s)
 *    - DLPF bandwidth  = 42 Hz        (reduces high-freq noise)
 *  Returns true if detected and configured, false otherwise.
 */
bool setupMPU6050() {
  // Test I2C communication
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Read WHO_AM_I register (should be 0x68 on MPU6050, 0x70/0x72 on compatible variants)
  uint8_t whoAmI = mpuRead(REG_WHO_AM_I);
  if (whoAmI == 0x00 || whoAmI == 0xFF) {
    return false;
  }

  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) return false;   // wake up (clear sleep bit)
  delay(100);
  if (!mpuWrite(REG_GYRO_CONFIG, 0x00)) return false;  // FS_SEL=0 -> +-250 deg/s
  delay(10);
  if (!mpuWrite(REG_ACCEL_CONFIG, 0x00)) return false; // AFS_SEL=0 -> +-2g
  delay(10);
  if (!mpuWrite(REG_CONFIG, 0x03)) return false;       // DLPF_CFG=3 -> 42 Hz bandwidth
  delay(10);

  Serial.print("[MPU6050] Online. WHO_AM_I=0x");
  Serial.print(whoAmI, HEX);
  Serial.println(" | FS=+-250 deg/s | DLPF=42 Hz");
  return true;
}

// ============================================================
//  GYRO READ
// ============================================================

/**
 *  Read the raw 16-bit signed gyro Z value from the MPU6050.
 *  Returns value in sensor LSB units (NOT converted to deg/s yet).
 */
float readRawGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (uint8_t)true);

  if (Wire.available() < 2) return 0.0f;
  int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
  return (float)raw;
}

// ============================================================
//  GYRO CALIBRATION
// ============================================================

/**
 *  Collect GYRO_CALIBRATION_SAMPLES gyro Z readings while the
 *  robot is completely stationary, then store the average as
 *  gyroBiasZ.
 *
 *  This average is subtracted from every future reading so that
 *  a perfectly still robot shows 0 deg/s.
 */
void calibrateGyro() {
  double sum = 0.0;

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    sum += readRawGyroZ();
    delay(2);   // ~2 ms per sample --> ~2 seconds total
  }

  gyroBiasZ = (float)(sum / (double)GYRO_CALIBRATION_SAMPLES);
}

// ============================================================
//  PUBLIC API
// ============================================================

/**
 *  updateGyroRotation()
 *  --------------------
 *  Read the gyro Z, subtract bias, apply deadzone, convert to
 *  deg/s, multiply by real elapsed time (micros), accumulate
 *  into rotationZ.
 *
 *  Call this as fast as possible in loop() for best accuracy.
 *  The integration uses real dt from micros() -- no fixed-delay
 *  assumption.
 */
void updateGyroRotation() {
  unsigned long nowMicros = micros();
  float dt = (float)(nowMicros - prevTimeMicros) * 1e-6f;  // convert to seconds
  prevTimeMicros = nowMicros;

  // Guard: skip implausibly large dt (e.g. very first call after a delay)
  if (dt <= 0.0f || dt > 0.5f) return;

  // Read and correct gyro Z
  float rawZ     = readRawGyroZ();
  float corrZ    = (rawZ - gyroBiasZ) / GYRO_SENSITIVITY;  // deg/s

  // Apply optional sign inversion
  if (GYRO_Z_INVERT) corrZ = -corrZ;

  // Apply deadzone -- noise below this threshold is treated as zero
  if (fabsf(corrZ) < GYRO_DEADZONE) corrZ = 0.0f;

  // Integrate angular velocity into accumulated rotation
  rotationZ += corrZ * dt;
}

/**
 *  getGyroRotationZ()
 *  ------------------
 *  Returns the total accumulated Z-axis rotation in degrees
 *  since the last resetGyroRotationZ() call.
 *
 *  Positive = clockwise  (if GYRO_Z_INVERT = false)
 *  Negative = counter-clockwise
 *
 *  (Or the opposite if GYRO_Z_INVERT = true)
 */
float getGyroRotationZ() {
  return rotationZ;
}

/**
 *  resetGyroRotationZ()
 *  --------------------
 *  Zeroes the rotation accumulator.
 *  The NEXT call to updateGyroRotation() starts from 0.
 *
 *  Use this before any movement you want to measure,
 *  e.g. before turning 90 degrees.
 */
void resetGyroRotationZ() {
  rotationZ      = 0.0f;
  prevTimeMicros = micros();  // prevent a large stale dt spike after reset
}

// ============================================================
//  END OF FILE
// ============================================================
