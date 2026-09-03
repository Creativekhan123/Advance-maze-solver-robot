# V03 — Movement Calibration

## Purpose

V03 establishes the relationship between encoder counts and real-world movement.

By the end of V03 you will know:
- How many encoder counts = 1 full wheel revolution
- How many encoder counts = 1 cm of travel
- How many encoder counts = a 90-degree in-place turn
- Whether the robot travels straight and how to correct drift

No sensors, MPU6050, or maze logic are used in this version.

---

## Hardware Used

| Component | Pins |
|---|---|
| Motor A — LEFT wheel | AIN1=14, AIN2=27, PWMA=23 |
| Motor B — RIGHT wheel | BIN1=19, BIN2=18, PWMB=5 |
| Left Encoder | GPIO 33 |
| Right Encoder | GPIO 34 (input-only) |
| TB6612FNG STBY | Connected to 3.3V (always enabled) |

### Robot Dimensions
| Parameter | Value |
|---|---|
| Wheel Diameter | 3.3 cm |
| Wheel Base (axle-to-axle) | 10.2 cm |
| Robot Width | 11.2 cm |

---

## Key Calibration Variables (in the USER CALIBRATION SETTINGS section)

| Variable | Default | Description |
|---|---|---|
| `LEFT_MOTOR_SPEED` | 100 | Forward/backward speed (0-255) |
| `RIGHT_MOTOR_SPEED` | 100 | Right wheel speed |
| `TURN_SPEED` | 80 | Speed during rotation |
| `INVERT_LEFT_MOTOR` | false | Flip left motor direction |
| `INVERT_RIGHT_MOTOR` | false | Flip right motor direction |
| `WHEEL_DIAMETER_CM` | 3.3 | Physical wheel diameter |
| `WHEEL_BASE_CM` | 10.2 | Distance between wheel contact points |
| `COUNTS_PER_WHEEL_REV` | **0.0** | ⚠ Must be calibrated |
| `COUNTS_FOR_90_DEG` | **0** | ⚠ Must be calibrated |
| `COUNTS_FOR_180_DEG` | **0** | ⚠ Must be calibrated (or auto = 2x 90) |
| `ENCODER_SYNC_KP` | 0.5 | Proportional gain for straight-line sync |

---

## platformio.ini

Update `src_dir` to point to V03:

```ini
[platformio]
src_dir = V03_Movement_Calibration

[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
```

---

## Serial Monitor

**Baud rate = 115200**

Use `pio device monitor -b 115200` or the Antigravity IDE serial monitor.

---

## Serial Commands

| Command | Action |
|---|---|
| `F` | Forward continuous (sync enabled, send S to stop) |
| `B` | Backward continuous |
| `L` | Rotate left continuously (send S to stop) |
| `R` | Rotate right continuously |
| `S` | **STOP immediately** |
| `M 10` | Move forward 10 cm (needs calibration) |
| `M -10` | Move backward 10 cm |
| `L90` | Rotate 90° left (needs calibration) |
| `R90` | Rotate 90° right |
| `U` | Rotate 180° |
| `C` | Clear encoder counts |
| `P` | Print encoder counts + difference |
| `W` | Print wheel revolution calibration guide |
| `D` | Print distance calibration guide |
| `T` | Print turn calibration guide |
| `H` | Show all commands |

---

## Calibration Procedure

Follow these steps **in order**.

---

### Step 1 — Wheel Revolution Calibration

**Goal**: Find how many encoder counts = 1 full wheel revolution.

1. Lift robot off the ground.
2. Send `W` to read the calibration guide.
3. Send `C` to clear encoder counts.
4. Put a mark on the wheel edge.
5. Rotate the wheel by hand exactly **one full revolution**.
6. Send `P` and record the `Left Encoder` value.
7. Send `C` and repeat **3 times**.
8. Average the 3 results.
9. Open `V03_Movement_Calibration.ino` and set:
   ```cpp
   float COUNTS_PER_WHEEL_REV = <your average>;
   ```
10. Upload and verify the startup banner shows the new value.

---

### Step 2 — Distance Calibration (10 cm test)

**Goal**: Verify that `M 10` moves the robot approximately 10 cm.

1. Place robot on a flat surface.
2. Mark the robot's starting position.
3. Send `M 10`.
4. Measure the actual distance with a ruler.
5. If actual distance ≠ 10 cm:
   - Robot moved **less**: decrease `COUNTS_PER_WHEEL_REV` slightly.
   - Robot moved **more**: increase `COUNTS_PER_WHEEL_REV` slightly.
6. Repeat until `M 10` reliably travels 10 cm.

---

### Step 3 — Straight-Line Test

**Goal**: Verify the robot travels straight using encoder sync.

1. Send `C` to clear counts.
2. Send `F` to move forward.
3. Observe the Serial Monitor — watch `Left`, `Right`, and `Diff` values.
4. Stop with `S` after ~50 cm.
5. If robot curves:
   - **Left**: reduce `ENCODER_SYNC_KP` slightly.
   - **Right**: increase `ENCODER_SYNC_KP` slightly.
6. Repeat until the robot travels straight.

---

### Step 4 — 90-Degree Turn Calibration

**Goal**: Find how many encoder counts = exactly 90°.

1. Send `T` to read the guide.
2. Place robot on the floor. Mark starting orientation with tape.
3. Send `C` to clear counts.
4. Send `L` to start slow rotation.
5. When the robot has rotated **exactly 90°**, send `S`.
6. Send `P` and record both encoder counts.
7. Average Left and Right values. That is your `COUNTS_FOR_90_DEG`.
8. Repeat **3 times**, calculate the average.
9. Set in code:
   ```cpp
   long COUNTS_FOR_90_DEG    = <your average>;
   long COUNTS_FOR_180_DEG   = <your average> * 2;
   ```
10. Upload. Test with `L90` and `R90`. Verify the robot turns approximately 90°.

---

### Step 5 — 180-Degree Turn Test

1. Send `U` and verify the robot turns approximately 180°.
2. If under/over: adjust `COUNTS_FOR_180_DEG` independently.

---

## Safety Rules

- Motors are **STOPPED at boot** — the robot will not move automatically.
- If `COUNTS_PER_WHEEL_REV` is 0, `M`, `L90`, `R90`, `U` will **not execute** and will print an error.
- Send `S` at any time to immediately stop the robot during a movement.
- Start testing with the robot lifted off the ground.

---

## Acceptance Criteria (PASS / FAIL)

V03 is **PASS** when ALL of the following are confirmed physically:

**Encoder:**
- [ ] `COUNTS_PER_WHEEL_REV` measured and confirmed stable
- [ ] Left encoder responds to left wheel independently
- [ ] Right encoder responds to right wheel independently

**Distance:**
- [ ] `M 10` moves the robot approximately 10 cm
- [ ] `M -10` moves the robot backward approximately 10 cm
- [ ] Forward and backward movement work correctly

**Straight Movement:**
- [ ] Robot travels reasonably straight using `F`
- [ ] Encoder difference is visible in Serial output
- [ ] `ENCODER_SYNC_KP` is tuned so drift is minimal

**Rotation:**
- [ ] `COUNTS_FOR_90_DEG` measured and confirmed
- [ ] `L90` rotates approximately 90° left
- [ ] `R90` rotates approximately 90° right
- [ ] `U` rotates approximately 180°

**Safety:**
- [ ] Robot starts stopped
- [ ] `S` always stops movement
- [ ] Uncalibrated commands print an error and do NOT move

---

## What to Report After Testing

1. What is your measured `COUNTS_PER_WHEEL_REV`?
2. Does `M 10` travel accurately?
3. Does the robot travel straight?
4. What is your measured `COUNTS_FOR_90_DEG`?
5. Do `L90` and `R90` work?
6. Any unexpected behavior?

Do NOT proceed to V04 until V03 is confirmed PASS.
