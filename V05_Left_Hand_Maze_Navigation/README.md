# FASTEST MAZE SOLVER ROBOT — LEFT-HAND MAZE NAVIGATION

## Autonomous Left-Wall Follower Navigation Engine

---

## 1. Overview & Algorithm Logic

This version adapts the calibrated motor kinematics, smooth forward cruising (`moveBothMotorsWithoutBreak`), and precision junction braking of **V05** into a dedicated **Left-Hand Rule (Left-Wall Follower)** maze solver.

### Decision Priority Order:
$$\mathbf{LEFT \;\;>\;\; STRAIGHT \;(FORWARD) \;\;>\;\; RIGHT \;\;>\;\; BACK \;(180^\circ \;U\text{-}TURN)}$$

At any junction or dead end, the robot strictly evaluates this priority:

1. **Turn Left** whenever the left path is open (highest priority).
2. **Go Straight (Forward)** if the left path is blocked but the forward path is open.
3. **Turn Right** if both the left and forward paths are blocked, but the right path is open.
4. **Turn Around (180° U-Turn)** if Left, Forward, and Right are all blocked (Dead End).

---

## 2. Left-Hand vs Right-Hand Decision Comparison

| Maze Situation | Sensor State | Right-Hand Action | **Left-Hand Action (This Version)** |
|---|---|---|---|
| **3-Way / 4-Way Crossroad** | Left, Front, Right all open | Turn Right | **Turn Left** |
| **T-Junction** | Front blocked, Left & Right open | Turn Right | **Turn Left** |
| **Front & Left Open** | Left & Front open, Right blocked | Go Straight | **Turn Left** |
| **Front & Right Open** | Right & Front open, Left blocked | Turn Right | **Go Straight** |
| **Only Left Open** | Front & Right blocked | Turn Left | **Turn Left** |
| **Only Right Open** | Front & Left blocked | Turn Right | **Turn Right** |
| **Corridor** | Walls on Left & Right, Front open | Go Straight | **Go Straight** |
| **Dead End** | Walls on all 3 sides | 180° U-Turn | **180° U-Turn** |

---

## 3. Motion & Braking System

* **Smooth Forward Cruising (`moveBothMotorsWithoutBreak`)**:
  When travelling straight down a corridor, the robot moves in continuous steps without applying the active H-bridge short-circuit brake. This keeps the robot rolling smoothly at speed without jerky stops.
* **Junction & Turn Braking (`brakeBothMotors`)**:
  Before any 90° or 180° turn, the robot automatically applies active electronic brakes (`brakeBothMotors()`) and pauses for `rest` (150 ms) to eliminate forward momentum, ensuring exact rotational accuracy.
* **Wheelbase Offset Compensation (`extraSteps = 800`)**:
  When turning into a side branch, the robot moves forward `extraSteps` to align its wheel axle with the center of the junction before executing the 90° turn.

---

## 4. Hardware Configuration & Pinout

* **ESP32 DevKit V1**
* **Sensors**: 3× VL53L0X Time-of-Flight Laser Ranging Sensors (I2C: `SDA=21`, `SCL=22`)
  * Front Sensor XSHUT: `GPIO 13` (Address `0x30`)
  * Right Sensor XSHUT: `GPIO 26` (Address `0x31`)
  * Left Sensor XSHUT: `GPIO 25` (Address `0x32`)
* **Motors**: Dual TB6612FNG H-Bridge Driver
  * Left Motor (Motor A): `AIN1=14`, `AIN2=27`, `PWMA=23`
  * Right Motor (Motor B): `BIN1=19`, `BIN2=18`, `PWMB=5`
* **Encoders**:
  * Left Wheel Encoder: `GPIO 33`
  * Right Wheel Encoder: `GPIO 34`

