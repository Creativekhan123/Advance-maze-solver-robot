# FASTEST MAZE SOLVER ROBOT — VERSION 04

## Basic Maze Navigation (Sensors + Decision Logic + Calibrated Motion)

---

## 1. Purpose of V04

Version 04 integrates the verified capabilities from previous versions:
* **V01**: 3 × VL53L0X laser distance sensors on custom I2C addresses
* **V02**: Dual DC motor control & interrupt-based encoder pulse counting
* **V03**: Experimentally measured movement calibration (10 cm, 90°, 180°) with **active motor braking**

The primary goal of V04 is **basic autonomous maze navigation** using a step-by-step decision architecture:
```text
READ SENSORS -> CLASSIFY WALLS -> DECIDE DIRECTION -> MOVE -> ACTIVE BRAKE -> STOP -> READ AGAIN
```

> **IMPORTANT:** V04 is **NOT** the final maze solver. It does **NOT** include path recording, path memory, EEPROM storage, loop optimization, MPU6050 heading control, or wall-centering PID. It is the first end-to-end integration test of sensor classification and calibrated movement.

---

## 2. Hardware Overview

* **Microcontroller**: ESP32 DOIT DevKit V1
* **Motor Driver**: TB6612FNG Dual H-Bridge
* **Motors**: 2 × N20 Micro DC Gearmotors with Hall Encoders
* **Sensors**: 3 × VL53L0X Time-of-Flight Laser Distance Sensors
* **STBY Pin**: Physically wired to 3.3V (Always Enabled)

---

## 3. Hardware Pin Table

| Peripheral | Signal / Pin | ESP32 GPIO | Notes |
|---|---|---|---|
| **Left Motor (Motor A)** | AIN1 | GPIO 14 | Direction Pin 1 |
| | AIN2 | GPIO 27 | Direction Pin 2 |
| | PWMA | GPIO 23 | Hardware PWM (Channel 0) |
| **Right Motor (Motor B)** | BIN1 | GPIO 18 | Direction Pin 1 |
| | BIN2 | GPIO 19 | Direction Pin 2 |
| | PWMB | GPIO 5 | Hardware PWM (Channel 1) |
| **TB6612 STBY** | STBY | Hardware 3.3V | No ESP32 control pin |
| **Left Encoder** | Signal | GPIO 33 | Interrupt RISING |
| **Right Encoder** | Signal | GPIO 34 | Interrupt RISING (Input-only pin) |
| **Front Sensor** | XSHUT | GPIO 13 | Reassigned to I2C Address `0x30` |
| **Left Sensor** | XSHUT | GPIO 25 | Default I2C Address `0x29` |
| **Right Sensor** | XSHUT | GPIO 26 | Reassigned to I2C Address `0x31` |
| **I2C Bus** | SDA | GPIO 21 | Shared I2C Data |
| | SCL | GPIO 22 | Shared I2C Clock |

---

## 4. Autonomous Auto-Start & Straight-Line Motion Control

Near the top of `V04_Basic_Maze_Navigation.ino`:
```cpp
bool AutoStart_automonus = true;  // Starts autonomous navigation 5s after power-on
float ENCODER_SYNC_KP   = 0.5;   // Pure straight-line encoder synchronization gain
```

* **5-Second Countdown**: If `AutoStart_automonus = true`, the robot counts down 5 seconds at boot (`5... 4... 3... 2... 1...`) and begins autonomous maze navigation automatically. Sending `S` over Serial cancels the countdown and stops the robot.
* **Straight-Line Motion**: Wall centering has been completely removed to avoid side-sensor noise, I2C bus delays, and twitching. The robot relies purely on **high-speed encoder synchronization** (`error = lCount - rCount`), ensuring both wheels turn identically in straight lines.

---

## 5. Calibrated Movement & Active Braking

Values are experimentally calibrated and fixed:
```cpp
const long COUNTS_FOR_10CM    = 650;   // 10 cm = 650 encoder counts
const long COUNTS_FOR_90_DEG  = 520;   // 90 deg = 520 encoder counts per wheel
const long COUNTS_FOR_180_DEG = 1040;  // 180 deg U-turn = 1040 encoder counts per wheel
const int  BRAKE_TIME_MS      = 80;    // Active braking hold time
```

### Active Braking Sequence (TB6612FNG)
1. Both direction pins driven `HIGH` (`AIN1=HIGH, AIN2=HIGH` / `BIN1=HIGH, BIN2=HIGH`).
2. PWM driven to `255` (short-circuit electrical braking to cancel motor momentum).
3. Held for `BRAKE_TIME_MS` (80 ms).
4. Released to safe idle (`LOW, LOW`, PWM `0`).

---

## 6. Maze Decision Priority & Wheel-Axis Turn Alignment

Because sensors are positioned at the **front** of the robot while drive wheels are at the **rear**, rotating immediately upon detecting an opening would cause the robot to clip the corridor corner.

Therefore:
* **90° Turns (`LEFT` / `RIGHT`)**: 3-step sequence:
  1. **Move Forward**:
     * If **Front is OPEN** (T-junction or cross): moves **10.0 cm** (`TURN_ALIGN_FORWARD_CM`).
     * If **Front is BLOCKED / WALL** (corner or dead T): moves only **4.0 cm** (`TURN_ALIGN_FRONT_WALL_CM`) to safely clear the corner without ramming into the front wall.
  2. **Rotate 90°** (pivots into new corridor)
  3. **Move Forward 10.0 cm** (`POST_TURN_FORWARD_CM`) (fully enters new corridor before reading sensors again)
* **180° U-Turn (`U`)**: In a dead-end, the robot **does NOT move forward** (avoids front wall) and rotates 180° directly in place.

```text
      LEFT OPEN?
       ├── YES ──> [1] Move Forward (10cm if front open / 4cm if front wall) ──> [2] Turn LEFT 90° ──> [3] Move Forward 10cm
       └── NO
            ↓
     FRONT OPEN?
       ├── YES ──> Move FORWARD 10 cm (M 10)
       └── NO
            ↓
     RIGHT OPEN?
       ├── YES ──> [1] Move Forward 4cm (Front is Wall) ──> [2] Turn RIGHT 90° ──> [3] Move Forward 10cm
       └── NO
            ↓
        DEAD END (All Blocked)
            └───> Turn 180° U-TURN in place (U) [NO forward step]
```

---

## 7. Serial Commands (115200 Baud)

| Command | Description |
|---|---|
| `A` | **Start Autonomous Maze Navigation** |
| `S` | **Emergency STOP & Exit Autonomous Mode** |
| `M 10` | Move forward 10 cm (calibrated) |
| `M -10` | Move backward 10 cm (calibrated) |
| `L90` | Rotate 90° Left (calibrated) |
| `R90` | Rotate 90° Right (calibrated) |
| `U` | Rotate 180° U-turn (calibrated) |
| `F` | Continuous Forward (send `S` to stop) |
| `B` | Continuous Backward (send `S` to stop) |
| `L` | Continuous Left rotation (send `S` to stop) |
| `R` | Continuous Right rotation (send `S` to stop) |
| `C` | Clear encoder counters |
| `P` | Print current encoder counts and difference |
| `V` | **Live Sensor Monitor (Toggle ON/OFF)** |
| `H` | Display Help Menu |

---

## 8. Upload Instructions

1. Ensure `platformio.ini` points to `src_dir = V04_Basic_Maze_Navigation` and includes `pololu/VL53L0X @ ^1.3.1`:
   ```ini
   [platformio]
   src_dir = V04_Basic_Maze_Navigation

   [env:esp32doit-devkit-v1]
   platform = espressif32
   board = esp32doit-devkit-v1
   framework = arduino
   monitor_speed = 115200
   lib_deps =
       pololu/VL53L0X @ ^1.3.1
   ```
2. In PlatformIO / Antigravity IDE, run:
   ```bash
   pio run --target upload
   ```
3. Open the Serial Monitor:
   ```bash
   pio device monitor -b 115200
   ```

---

## 9. Physical Testing Procedure

Perform testing in this exact order:

### Test 1: Boot & Sensor Verification
* Power up the robot.
* Verify startup banner prints and all 3 sensors show `[OK]`.
* Motors **must remain completely stopped**.

### Test 2: Encoder Check
* Send `P` -> Ensure `Left Encoder: 0`, `Right Encoder: 0`.

### Test 3: Manual 90° Left Turn
* Send `L90`. Robot must rotate 90° left, active brake, and stop cleanly.

### Test 4: Manual 90° Right Turn
* Send `R90`. Robot must rotate 90° right, active brake, and stop cleanly.

### Test 5: Manual 10 cm Move
* Send `M 10`. Robot must travel forward ~10 cm, active brake, and stop.

### Test 6: Autonomous Mode Launch
* Place the robot inside the maze corridor.
* Send `A`. Observe decision cycles printing to Serial.

### Test 7: Left-Priority Verification
* Place an obstacle in Front and Right, leave Left open.
* Expected: `DECISION: LEFT` -> Executes `L90`.

### Test 8: Front-Priority Verification
* Block Left and Right, leave Front open.
* Expected: `DECISION: FORWARD` -> Executes `M 10`.

### Test 9: Right-Priority Verification
* Block Left and Front, leave Right open.
* Expected: `DECISION: RIGHT` -> Executes `R90`.

### Test 10: Dead-End / U-Turn Verification
* Block all 3 directions (Left, Front, Right).
* Expected: `DECISION: U-TURN` -> Executes `U` (180° turn).

### Test 11: Emergency Stop
* Send `S` at any time during autonomous navigation. Motors must stop and autonomous mode exits.

---

## 10. Expected Serial Monitor Output

```text
================================
MAZE SENSOR READING
================================
LEFT  : 8.2 cm  [WALL]
FRONT : 25.4 cm [OPEN]
RIGHT : 7.8 cm  [WALL]

DECISION: FORWARD

ACTION: MOVE FORWARD 10.0 CM
TARGET: 650 COUNTS
LEFT COUNT : 650
RIGHT COUNT: 649
ACTIVE BRAKE
MOVEMENT COMPLETE
================================
```

---

## 11. Safety Notes

* **Fail-Safe Sensor Policy**: If any sensor times out or disconnects, it is automatically treated as a **WALL / BLOCKED** to prevent collisions.
* **No Autonomous Movement at Boot**: Autonomous mode will never engage without the explicit `A` serial command.
* **Instant Brake on Stop**: The `S` command instantly applies active braking and terminates the autonomous loop.

---

## 12. Troubleshooting

* **Robot turns in opposite direction**: Toggle `INVERT_LEFT_MOTOR` or `INVERT_RIGHT_MOTOR` in `USER CONFIGURATION`.
* **Sensor init fails at boot**: Verify XSHUT pin wiring (Front: 13, Left: 25, Right: 26) and I2C pullups.
* **Robot overshoots target**: Increase `BRAKE_TIME_MS` (e.g. from 80 ms to 100 ms).
* **Robot stops too abruptly / skids**: Decrease `BRAKE_TIME_MS` (e.g. to 60 ms).

---

## 13. V04 Acceptance Criteria

- [ ] Code compiles without error.
- [ ] ESP32 boots without movement; motors remain stopped.
- [ ] All 3 VL53L0X sensors initialize and report distances.
- [ ] Wall threshold (< 15 cm = WALL, >= 15 cm = OPEN) works accurately.
- [ ] Left decision priority executes when Left is open.
- [ ] Front decision executes when Left is blocked and Front is open.
- [ ] Right decision executes when Left & Front are blocked and Right is open.
- [ ] 180° U-turn executes in dead-ends (all 3 sides blocked).
- [ ] Calibrated movements (650 counts forward, 520 counts 90°, 1040 counts 180°) execute accurately.
- [ ] Active braking engages at the end of each movement step.
- [ ] `S` command halts motion and exits autonomous mode immediately.
- [ ] No maze solving, path memory, or MPU6050 logic included.
