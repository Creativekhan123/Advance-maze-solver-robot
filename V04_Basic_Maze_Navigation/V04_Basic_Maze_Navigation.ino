#include <Adafruit_VL53L0X.h>
#include <Wire.h>

Adafruit_VL53L0X F_lox = Adafruit_VL53L0X();
Adafruit_VL53L0X L_lox = Adafruit_VL53L0X();
Adafruit_VL53L0X R_lox = Adafruit_VL53L0X();

// Variables

volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// All connections
// encoder pins
const int left_encoder_pin = 33;
const int right_encoder_pin = 34;
const int turn_encoder_value = 540; // Example value for a 90-degree turn
const int each_step_encoder_value =
    25; // Example value for moving forward a certain distance
const bool autoMode = true; // Example value for automatic mode

// Motor A
const int motorTurnSpeed = 100; // Example speed value for turning
const int extraSteps =
    200; // Example extra steps to move forward before turning

const int AIN1 = 14;
const int AIN2 = 27;
const int PWMA = 23;

// Motor B
const int BIN1 = 19;
const int BIN2 = 18;
const int PWMB = 5;

// sensor pins
const int F_Sensor = 13;
const int R_Sensor = 26;
const int L_Sensor = 25;

// sensor distance values
int f = 0;
int r = 0;
int l = 0;
const int distance_threshold = 180; // Example threshold value in millimeters
const int too_close_distance = 5;

void Off_Sensor() {
  digitalWrite(F_Sensor, LOW);
  digitalWrite(R_Sensor, LOW);
  digitalWrite(L_Sensor, LOW);
}

void IRAM_ATTR leftEncoderISR() { left_encoder_count++; }
void IRAM_ATTR rightEncoderISR() { right_encoder_count++; }

void pinModeSetup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(F_Sensor, OUTPUT);
  pinMode(R_Sensor, OUTPUT);
  pinMode(L_Sensor, OUTPUT);

  pinMode(left_encoder_pin, INPUT_PULLUP);
  pinMode(right_encoder_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left_encoder_pin), leftEncoderISR,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), rightEncoderISR,
                  RISING);
}
void motorA(int speed, int target_count) {
  if (speed > 0) {
    analogWrite(PWMA, abs(speed));
    left_encoder_count = 0;
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    left_encoder_count = 0;
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(speed));
  }

  while (left_encoder_count < target_count) {
    // Wait until the left encoder count reaches the target count
  }

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
}

void motorB(int speed, int target_count) {
  if (speed > 0) {
    right_encoder_count = 0;
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(speed));
  } else {
    right_encoder_count = 0;
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(speed));
  }
  while (right_encoder_count < target_count) {
    // Wait until the right encoder count reaches the target count
  }
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);
}

void moveBothMotors(int speedA, int targetA, int speedB, int targetB) {

  left_encoder_count = 0;
  right_encoder_count = 0;

  // Start Motor A
  if (speedA > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  analogWrite(PWMA, abs(speedA));

  // Start Motor B
  if (speedB > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMB, abs(speedB));

  // Both motors running
  while (left_encoder_count < targetA || right_encoder_count < targetB) {

    // Stop Motor A when it reaches target
    if (left_encoder_count >= targetA) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, 255);
    }

    // Stop Motor B when it reaches target
    if (right_encoder_count >= targetB) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMB, 255);
    }
  }

  // Make absolutely sure both are stopped
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);
}
void keep_center() {
  if (r < too_close_distance) {
    moveBothMotors(150, each_step_encoder_value, -150, each_step_encoder_value);
  } else if (l < too_close_distance) {
    moveBothMotors(-150, each_step_encoder_value, 150, 150);
  } else if (f < too_close_distance) {
    moveBothMotors(-150, each_step_encoder_value, -150,
                   each_step_encoder_value);
  }

  if (r < distance_threshold / 2.8 || l < distance_threshold / 2.8) {

    // this function will keep the robot in the center of the path by adjusting
    // the speed of the motors based on the distance readings from the left and
    // right sensors.
    if (r > l) {
      // if the left distance is greater than the right distance, it means the
      // robot is closer to the right wall, so we need to turn left
      motorA(255, each_step_encoder_value);  // Exa  mple target count
      motorB(-100, each_step_encoder_value); // Example target count
    } else if (l > r) {
      // if the right distance is greater than the left distance, it means the
      // robot is closer to the left wall, so we need to turn right
      motorA(-100, each_step_encoder_value); // Example target count
      motorB(255, each_step_encoder_value);  // Example target count
    }

  } else {
    // if both distances are equal, it means the robot is in the center of the
    // path, so we can move forward
    moveBothMotors(255, each_step_encoder_value * 2, 255,
                   each_step_encoder_value * 2); // Move forward a bit
  }
}
void autonomus() {
  if (autoMode) {

    // when all three side blocked, rotate 180 degrees
    if (f < distance_threshold / 1.5 && r < distance_threshold &&
        l < distance_threshold) {
      // move forward a bit, then Rotate 180 degrees

      moveBothMotors(255, 20, -255, 20);

      moveBothMotors(255, turn_encoder_value * 1.8, -255,
                     turn_encoder_value * 1.8); // Rotate 180 degrees

      delay(1000); // Optional delay to allow the robot to complete the turn

      // GO backward
      moveBothMotors(-255, 600, -255, 600); // Move backward a bit

      delay(1000); // Optional delay to allow the robot to complete the turn

    }

    // when forward and left is blocked, only right way found
    else if (r > distance_threshold / 1.5 && f < distance_threshold &&
             l < distance_threshold) {

      // Move forward a bit, then Rotate 90 degrees to the right

      moveBothMotors(255, extraSteps, 255, extraSteps); // Move forward a bit

      motorA(255, turn_encoder_value * 2);

      delay(1000); // Optional delay to allow the robot to complete the turn
    }

    // When forwrad and Right is blocked, only left way found
    else if (l > distance_threshold && f < distance_threshold / 1.5 &&
             r < distance_threshold) {
      // Move forward a bit, then Rotate 90 degrees to the left

      moveBothMotors(255, extraSteps, 255, extraSteps);

      // Rotate 90 degrees to the left
      motorB(255, turn_encoder_value * 2);

      delay(1000); // Optional delay to allow the robot to complete the turn
    }

    // three way found, chose right side
    else if (r > distance_threshold && f > distance_threshold / 1.5 &&
             l > distance_threshold) {

      // move forward a bit, then Rotate 90 degrees to the right
      motorA(100, extraSteps * 3); // Move forward a bit
      motorB(100, extraSteps * 3); // Move forward a bit

      motorA(255, turn_encoder_value * 2);
      delay(1000); // Optional delay to allow the robot to complete the turn

      moveBothMotors(255, extraSteps, 255, extraSteps);

    }

    // when only forward is blocked chose right side

    else if (f < distance_threshold / 1.5 && r > distance_threshold &&
             l > distance_threshold) {
      // Rotate 90 degrees to the right

      motorA(255, turn_encoder_value);
      delay(1000); // Optional delay to allow the robot to complete the turn
    }

    else if (f > distance_threshold / 1.5 && r < distance_threshold &&
             l > distance_threshold) {
      // keep moving forward

      moveBothMotors(255, each_step_encoder_value * 2, 255,
                     each_step_encoder_value * 2);
    } else if (f > distance_threshold / 1.5 && r > distance_threshold * 2 &&
               l < distance_threshold) {
      // keep moving forward
      moveBothMotors(255, each_step_encoder_value * 2, 255,
                     each_step_encoder_value * 2);

      // turn right
      motorA(255, turn_encoder_value);
      moveBothMotors(255, each_step_encoder_value, 255,
                     each_step_encoder_value);
      delay(1000);
    }

    else {
      keep_center();
    }
  }
}
void setup() {
  pinModeSetup();

  Serial.begin(115200);
  Wire.begin(21, 22);
  Off_Sensor();

  digitalWrite(F_Sensor, HIGH);
  delay(10);
  if (!F_lox.begin(0x30, false, &Wire)) {
    Serial.println("Forward Sensor fail.");
    while (1)
      ;
  }

  digitalWrite(R_Sensor, HIGH);
  delay(10);
  if (!R_lox.begin(0x31, false, &Wire)) {
    Serial.println("RIGHT Sensor fail");
    while (1)
      ;
  }

  digitalWrite(L_Sensor, HIGH);
  delay(10);
  if (!L_lox.begin(0x32, false, &Wire)) {
    Serial.println("LEFT Sensor fail.");
    while (1)
      ;
  }

  Serial.println("VL53L0X Ready!");
}
void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  L_lox.rangingTest(&measure, false);
  Serial.print("LEFT: ");
  if (measure.RangeStatus != 4) {
    l = measure.RangeMilliMeter;
    Serial.print(l);
    Serial.print(" mm  ");
  } else {
    Serial.print(" out of range ");
  }

  F_lox.rangingTest(&measure, false);
  Serial.print("FORWARD: ");
  if (measure.RangeStatus != 4) {
    f = measure.RangeMilliMeter;
    Serial.print(f);
    Serial.print(" mm   ");
  } else {
    Serial.print(" out of range ");
  }
  R_lox.rangingTest(&measure, false);
  Serial.print("RIGHT: ");
  if (measure.RangeStatus != 4) {
    r = measure.RangeMilliMeter;
    Serial.print(r);
    Serial.println(" mm   ");
  } else {
    Serial.println(" out of range ");
  }

  autonomus();

  if (Serial.available()) {
    char val = Serial.read();
    if (val == '\r' || val == '\n') {
      return;
    }
    Serial.print("Received: ");
    Serial.println(val);

    if (val == 'w') {
      motorA(255, each_step_encoder_value); // Example target count
      motorB(255, each_step_encoder_value); // Example target count
    } else if (val == ' ') {
      motorA(0, 0);
      motorB(0, 0);
    } else if (val == 's') {
      motorA(-255, each_step_encoder_value); // Example target count
      motorB(-255, each_step_encoder_value); // Example target count

    } else if (val == 'a') {

      motorB(255, turn_encoder_value);  // Example target count
      motorA(-255, turn_encoder_value); // Example target count

    } else if (val == 'd') {
      motorA(255, turn_encoder_value);  // Example target count
      motorB(-255, turn_encoder_value); // Example target count

    } else if (val == 'c') {
      motorA(255, turn_encoder_value * 2);  // Example target count
      motorB(-255, turn_encoder_value * 2); // Example target count
    } else {
      Serial.println("Invalid input");
    }
  }
}
