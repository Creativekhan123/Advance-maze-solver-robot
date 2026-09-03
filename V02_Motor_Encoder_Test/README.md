# V02 — Motor + Encoder Test

## Purpose

V02 verifies that:

- Both motors can be individually and jointly controlled.
- Forward, backward, left rotation, and right rotation work correctly.
- Motor direction can be corrected by flipping a single `bool` variable.
- Both encoders generate and count pulses during movement.
- The robot is fully **manually controlled** via Serial Monitor commands.

No sensors, MPU6050, or autonomous logic are used in this version.

---

## Hardware Used

| Component | Pins |
|---|---|
| Motor A — LEFT wheel | AIN1=14, AIN2=27, PWMA=23 |
| Motor B — RIGHT wheel | BIN1=19, BIN2=18, PWMB=5 |
| Left Encoder | GPIO 33 |
| Right Encoder | GPIO 34 (input-only) |
| TB6612FNG STBY | Connected to 3.3V (always enabled) |

---

## Key Configurable Variables

Open `V02_Motor_Encoder_Test.ino` and find the **USER CONFIGURATION** section at the top.

| Variable | Default | Purpose |
|---|---|---|
| `LEFT_MOTOR_SPEED` | 100 | Forward/backward speed of left motor (0-255) |
| `RIGHT_MOTOR_SPEED` | 100 | Forward/backward speed of right motor (0-255) |
| `TURN_SPEED` | 100 | Speed during left/right rotation |
| `INVERT_LEFT_MOTOR` | false | Set `true` if left motor spins backwards |
| `INVERT_RIGHT_MOTOR` | false | Set `true` if right motor spins backwards |
| `DEBUG_MOTORS` | true | Print motor command feedback to Serial |
| `DEBUG_ENCODERS` | true | Auto-print encoder counts periodically |
| `ENCODER_PRINT_INTERVAL_MS` | 250 | How often encoder counts are auto-printed |

### Fixing wrong motor direction

If a motor physically spins the wrong way, do NOT swap the motor wires.
Instead change the inversion variable:

```cpp
const bool INVERT_LEFT_MOTOR  = true;   // flip left motor
const bool INVERT_RIGHT_MOTOR = true;   // flip right motor
```

---

## platformio.ini

Make sure your `platformio.ini` points to V02 as the source:

```ini
[platformio]
src_dir = V02_Motor_Encoder_Test

[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
```

> No extra `lib_deps` are needed — V02 only uses the Arduino ESP32 core.

---

## Upload Instructions

1. Update `platformio.ini`: change `src_dir = V02_Motor_Encoder_Test`
2. Click **PlatformIO: Upload** (→ button on the bottom toolbar in VS Code)
3. After upload, click **PlatformIO: Serial Monitor** (plug icon)
4. Set baud rate: **115200**

---

## Serial Monitor

**Baud rate = 115200**

Press **EN/RST** on the ESP32 if no output appears after upload.

---

## Serial Commands

| Key | Action |
|---|---|
| `F` | Move forward (both motors) |
| `B` | Move backward (both motors) |
| `L` | Rotate left (left back, right forward) |
| `R` | Rotate right (left forward, right back) |
| `S` | **Stop** both motors |
| `1` | Run **left motor only** |
| `2` | Run **right motor only** |
| `C` | Clear encoder counts to 0 |
| `P` | Print current encoder counts |
| `H` | Show command help |

Commands are case-insensitive.

---

## Expected Startup Output

```
========================================
 FASTEST MAZE SOLVER
 VERSION 02
 MOTOR + ENCODER TEST
========================================

  Motor A = LEFT  wheel  (AIN1=14 AIN2=27 PWM=23)
  Motor B = RIGHT wheel  (BIN1=19 BIN2=18 PWM=5)

  Left  Encoder = GPIO 33
  Right Encoder = GPIO 34

  PWM Frequency  = 5000 Hz
  PWM Resolution = 8-bit

  Left  Motor Speed = 100
  Right Motor Speed = 100
  Turn  Speed       = 100

  Invert Left Motor  : NO
  Invert Right Motor : NO

  Robot is STOPPED. Waiting for command.

========================================
 COMMANDS
========================================
  F = Forward
  B = Backward
  L = Rotate Left
  R = Rotate Right
  S = Stop
  1 = Left motor only
  2 = Right motor only
  C = Clear encoder counts
  P = Print encoder counts
  H = Help
========================================
```

---

## Physical Test Procedure

> **IMPORTANT**: Lift the robot so the wheels are off the ground before running any motor command.

---

### Test 1 — Startup Safety

1. Upload V02 and open Serial Monitor at **115200 baud**.
2. Press **EN/RST** on the ESP32.
3. Verify the startup banner prints and `Robot is STOPPED.` appears.
4. **Confirm both wheels are stationary.**

---

### Test 2 — Left Motor Only

1. With the robot lifted, send: `1`
2. Verify **only the left wheel** spins.
3. The right wheel must remain still.
4. Send: `S` — verify left wheel stops.

---

### Test 3 — Right Motor Only

1. Send: `2`
2. Verify **only the right wheel** spins.
3. Send: `S`

---

### Test 4 — Forward

1. Send: `F`
2. Both wheels should spin in the **same direction** (forward).
3. Send: `S`

> If one wheel spins backward, set its inversion variable to `true` and re-upload.

---

### Test 5 — Backward

1. Send: `B`
2. Both wheels should spin **opposite to forward**.
3. Send: `S`

---

### Test 6 — Rotate Left

1. Send: `L`
2. Expected:
   - Left wheel → **backward**
   - Right wheel → **forward**
3. Send: `S`

---

### Test 7 — Rotate Right

1. Send: `R`
2. Expected:
   - Left wheel → **forward**
   - Right wheel → **backward**
3. Send: `S`

---

### Test 8 — Encoder Manual Test

1. Send: `C` (clear counts)
2. Manually rotate the **left wheel** by hand several times.
3. Send: `P`
4. Verify `Left Encoder` count has increased. `Right Encoder` should remain near 0.
5. Send: `C`
6. Manually rotate the **right wheel** by hand several times.
7. Send: `P`
8. Verify `Right Encoder` count has increased. `Left Encoder` should remain near 0.

---

### Test 9 — Encoder with Motor

1. Send: `C`
2. Send: `F`
3. Let wheels spin for 2–3 seconds.
4. Send: `S`
5. Send: `P`
6. Both encoder counts should have **increased significantly**.

---

## Acceptance Criteria (PASS / FAIL)

V02 is **PASS** only when ALL of the following are true:

**Motors:**
- [ ] Left motor runs independently (command `1`)
- [ ] Right motor runs independently (command `2`)
- [ ] Both motors move forward on `F`
- [ ] Both motors move backward on `B`
- [ ] Robot rotates left on `L` (left back, right forward)
- [ ] Robot rotates right on `R` (left forward, right back)
- [ ] `S` stops both motors reliably
- [ ] Motor direction can be corrected with `INVERT_LEFT/RIGHT_MOTOR`

**Encoders:**
- [ ] Left encoder count increases when left wheel moves
- [ ] Right encoder count increases when right wheel moves
- [ ] Counts are independent (left wheel does not affect right encoder and vice versa)
- [ ] `C` clears both counts to 0
- [ ] `P` prints both counts correctly

**Safety:**
- [ ] Robot does NOT move automatically after boot
- [ ] Unknown commands print an error, do NOT cause movement
- [ ] `S` always stops the robot

---

## What to Report Back

After physical testing, tell me:

1. Did the startup banner print correctly?
2. Did any motor spin in the wrong direction? Which one?
3. Did both encoders count pulses during motor movement?
4. Were the encoder counts independent?
5. Did the `S` stop command work reliably?
6. Any unexpected behavior or Serial output?

Do NOT proceed to V03 until V02 has passed all acceptance criteria.
