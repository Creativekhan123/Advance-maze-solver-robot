/*
 * ============================================================
 *  FASTEST MAZE SOLVER — V01 Sensor Test
 *  Target board : ESP32 DOIT DevKit V1
 *  Purpose      : Verify all three VL53L0X laser sensors
 *                 initialize correctly and return valid
 *                 distance readings (mm) independently.
 *
 *  No motors, encoders, MPU6050, or maze logic in this file.
 * ============================================================
 *
 *  Required library (install via Arduino Library Manager
 *  or PlatformIO lib_deps):
 *    Pololu VL53L0X  ->  "VL53L0X by Pololu"
 *    (pololu/VL53L0X @ ^1.3.1)
 *
 *  Baud rate: 115200
 * ============================================================
 */

#include <VL53L0X.h>
#include <Wire.h>

// =====================================================
// USER CONFIGURATION
// =====================================================

// --- I2C Bus ---
// SDA and SCL pins for the ESP32 (default hardware I2C)
const int SDA_PIN = 21; // I2C data line
const int SCL_PIN = 22; // I2C clock line

// --- XSHUT (shutdown / enable) Pins ---
// Each sensor's XSHUT pin lets us boot sensors one at a time
// so we can assign each a unique I2C address.
const int F_XSHUT = 13; // Front sensor XSHUT -> GPIO 13
const int R_XSHUT = 26; // Right sensor XSHUT -> GPIO 26
const int L_XSHUT = 25; // Left  sensor XSHUT -> GPIO 25

// --- Unique I2C Addresses assigned at boot ---
// Default VL53L0X address is 0x29.
// We reassign two sensors to avoid conflicts.
const uint8_t ADDR_FRONT = 0x30; // Address for Front sensor
const uint8_t ADDR_RIGHT = 0x31; // Address for Right sensor
const uint8_t ADDR_LEFT = 0x29;  // Left sensor keeps default

// --- Sensor Timing ---
// How often (in milliseconds) the Serial Monitor is updated.
// Lowering this value gives faster updates; raising it reduces
// output clutter.
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 200;

// --- Debug Flags ---
// Set to true  -> print detailed sensor init messages.
// Set to false -> suppress init detail (still shows pass/fail).
const bool DEBUG_SENSOR_STATUS = true;

// Set to true  -> continuously print distance readings.
// Set to false -> suppress the distance output (useful when
//                testing a single sensor at a time).
const bool DEBUG_SENSORS = true;

// --- Distance Threshold (reference only in V01) ---
// Not used for logic in this version, but kept here so it is
// easy to find and set for later versions.
const int DISTANCE_THRESHOLD_MM = 160; // mm - maze wall trigger

// =====================================================
// END USER CONFIGURATION
// =====================================================

// --- Sensor objects ---
VL53L0X sensorFront;
VL53L0X sensorRight;
VL53L0X sensorLeft;

// --- Init status flags ---
bool frontOK = false;
bool rightOK = false;
bool leftOK = false;

// =====================================================
// Helper: initialise one VL53L0X sensor
// =====================================================
bool initSensor(VL53L0X &sensor, int xshutPin, uint8_t newAddress,
                const char *label) {
  // Pull XSHUT HIGH to enable this sensor
  digitalWrite(xshutPin, HIGH);
  delay(10); // Allow sensor to boot (datasheet: >= 1.2 ms)

  // Attempt to initialise at the default address 0x29
  if (!sensor.init()) {
    if (DEBUG_SENSOR_STATUS) {
      Serial.print("  [FAILED] ");
      Serial.print(label);
      Serial.println(" sensor did not respond on 0x29.");
      Serial.println("         Check wiring and XSHUT connection.");
    }
    return false;
  }

  // Reassign I2C address (skip if keeping the default 0x29)
  if (newAddress != 0x29) {
    sensor.setAddress(newAddress);
  }

  // Switch to continuous measurement mode.
  sensor.startContinuous();

  if (DEBUG_SENSOR_STATUS) {
    Serial.print("  [OK]     ");
    Serial.print(label);
    Serial.print(" sensor  ->  address 0x");
    Serial.println(newAddress, HEX);
  }

  return true;
}

// =====================================================
// setup()
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500); // Give Serial Monitor time to connect

  // --- Print banner ---
  Serial.println();
  Serial.println("========================================");
  Serial.println(" FASTEST MAZE SOLVER - V01");
  Serial.println(" SENSOR TEST");
  Serial.println("========================================");
  Serial.println();

  // --- Configure XSHUT pins ---
  // Pull ALL sensors LOW (shutdown) so only one boots at a time.
  pinMode(F_XSHUT, OUTPUT);
  pinMode(R_XSHUT, OUTPUT);
  pinMode(L_XSHUT, OUTPUT);

  digitalWrite(F_XSHUT, LOW);
  digitalWrite(R_XSHUT, LOW);
  digitalWrite(L_XSHUT, LOW);
  delay(10); // Ensure all sensors are fully shut down

  // --- Start I2C bus ---
  Wire.begin(SDA_PIN, SCL_PIN);

  // -----------------------------------------------
  // Initialise sensors ONE AT A TIME.
  // While one sensor is being assigned its address,
  // the others remain in shutdown (XSHUT LOW) so
  // there is no I2C address collision.
  // -----------------------------------------------

  if (DEBUG_SENSOR_STATUS) {
    Serial.println("Initialising sensors...");
    Serial.println();
  }

  // 1. Front sensor  (ADDR_FRONT = 0x30)
  frontOK = initSensor(sensorFront, F_XSHUT, ADDR_FRONT, "Front");

  // 2. Right sensor  (ADDR_RIGHT = 0x31)
  rightOK = initSensor(sensorRight, R_XSHUT, ADDR_RIGHT, "Right");

  // 3. Left sensor   (ADDR_LEFT  = 0x29  -- keeps default)
  leftOK = initSensor(sensorLeft, L_XSHUT, ADDR_LEFT, "Left ");

  // --- Summary ---
  Serial.println();
  Serial.println("----------------------------------------");
  Serial.println(" Sensor Initialisation Summary");
  Serial.println("----------------------------------------");
  Serial.print("  Front Sensor : ");
  Serial.println(frontOK ? "OK" : "FAILED");
  Serial.print("  Right Sensor : ");
  Serial.println(rightOK ? "OK" : "FAILED");
  Serial.print("  Left  Sensor : ");
  Serial.println(leftOK ? "OK" : "FAILED");
  Serial.println("----------------------------------------");
  Serial.println();

  if (!frontOK && !rightOK && !leftOK) {
    Serial.println("!!! ALL SENSORS FAILED !!!");
    Serial.println("Check I2C wiring (SDA=21, SCL=22) and");
    Serial.println("XSHUT connections (F=13, R=26, L=25).");
    Serial.println("Halting. Reset the board after fixing.");
    while (true) {
      delay(1000);
    }
  }

  if (DEBUG_SENSORS) {
    Serial.println("Starting continuous readings...");
    Serial.println("(press Reset to restart)");
    Serial.println();
  }
}

// =====================================================
// loop()
// =====================================================
void loop() {
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= SENSOR_UPDATE_INTERVAL_MS) {
    lastPrint = millis();

    if (DEBUG_SENSORS) {

      // ---- Front ----
      Serial.print("Front : ");
      if (!frontOK) {
        Serial.println("NOT INITIALIZED");
      } else {
        uint16_t d = sensorFront.readRangeContinuousMillimeters();
        if (sensorFront.timeoutOccurred()) {
          Serial.println("ERROR (timeout)");
        } else {
          Serial.print(d);
          Serial.println(" mm");
        }
      }

      // ---- Left ----
      Serial.print("Left  : ");
      if (!leftOK) {
        Serial.println("NOT INITIALIZED");
      } else {
        uint16_t d = sensorLeft.readRangeContinuousMillimeters();
        if (sensorLeft.timeoutOccurred()) {
          Serial.println("ERROR (timeout)");
        } else {
          Serial.print(d);
          Serial.println(" mm");
        }
      }

      // ---- Right ----
      Serial.print("Right : ");
      if (!rightOK) {
        Serial.println("NOT INITIALIZED");
      } else {
        uint16_t d = sensorRight.readRangeContinuousMillimeters();
        if (sensorRight.timeoutOccurred()) {
          Serial.println("ERROR (timeout)");
        } else {
          Serial.print(d);
          Serial.println(" mm");
        }
      }

      Serial.println("---");
    }
  }
}
