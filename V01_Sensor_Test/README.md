# V01 — Sensor Test

## Purpose

V01 confirms that all **three VL53L0X laser distance sensors** are correctly wired, initialise without I2C address conflicts, and return independent, valid distance readings in millimetres.

No motors, encoders, or MPU6050 are used in this version.

---

## Hardware

| Sensor  | XSHUT GPIO | I2C Address (after init) |
|---------|-----------|--------------------------|
| Front   | GPIO 13   | 0x30                     |
| Right   | GPIO 26   | 0x31                     |
| Left    | GPIO 25   | 0x29 (default)           |

**I2C Bus:**

| Signal | GPIO |
|--------|------|
| SDA    | 21   |
| SCL    | 22   |

> All three sensors share the same SDA/SCL lines.
> XSHUT pins are used to boot each sensor one at a time
> so a unique I2C address can be assigned before the next
> sensor is powered on. This prevents address collisions.

---

## Required Library

Install **"VL53L0X by Pololu"** before compiling.

### Arduino IDE
1. Open **Sketch → Include Library → Manage Libraries**
2. Search for `VL53L0X`
3. Select **"VL53L0X by Pololu"** and click **Install**

### PlatformIO
Add to `platformio.ini`:

```ini
lib_deps =
    pololu/VL53L0X @ ^1.3.1
```

---

## Upload Instructions

### Arduino IDE
1. Open `V01_Sensor_Test.ino` in Arduino IDE.
2. Go to **Tools → Board** → select **"ESP32 Dev Module"** or
   **"DOIT ESP32 DEVKIT V1"**.
3. Go to **Tools → Port** → select the correct COM port.
4. Press the **Upload** button (→).
5. If upload fails, hold the **BOOT** button on the ESP32
   while the upload starts, then release it.

### PlatformIO
```
pio run -t upload
```

---

## Serial Monitor

**Baud rate = 115200**

Open the Serial Monitor **after** uploading.  
Restart the ESP32 (press the **EN/RST** button) if you do
not see any output.

---

## Expected Output

### Normal startup (all sensors OK):

```
========================================
 FASTEST MAZE SOLVER - V01
 SENSOR TEST
========================================

Initialising sensors...

  [OK]     Front sensor  ->  address 0x30
  [OK]     Right sensor  ->  address 0x31
  [OK]     Left  sensor  ->  address 0x29

----------------------------------------
 Sensor Initialisation Summary
----------------------------------------
  Front Sensor : OK
  Right Sensor : OK
  Left  Sensor : OK
----------------------------------------

Starting continuous readings...
(press Reset to restart)

Front : 312 mm
Left  : 98 mm
Right : 205 mm
---
Front : 315 mm
Left  : 97 mm
Right : 206 mm
---
```

### If a sensor fails to init:

```
  [FAILED] Right sensor did not respond on 0x29.
           Check wiring and XSHUT connection.

----------------------------------------
 Sensor Initialisation Summary
----------------------------------------
  Front Sensor : OK
  Right Sensor : FAILED
  Left  Sensor : OK
```

### If a reading times out during operation:

```
Front : ERROR (timeout)
Left  : 210 mm
Right : 330 mm
---
```

---

## Physical Test Procedure

1. Upload the code and open the Serial Monitor at **115200 baud**.
2. Confirm the initialisation summary shows **OK** for all three sensors.
3. **Test Front sensor:**
   - Hold an object (a book, your hand) directly in front of the robot.
   - Verify the `Front` reading decreases as you move the object closer.
   - Remove the object and verify the reading increases.
4. **Test Left sensor:**
   - Hold an object to the left side of the robot.
   - Verify the `Left` reading changes.
   - Remove the object and verify the reading returns to a larger value.
5. **Test Right sensor:**
   - Hold an object to the right side of the robot.
   - Verify the `Right` reading changes.
   - Remove the object and verify it returns.
6. **Independence test:**
   - Move objects around all three sides simultaneously.
   - Verify that each reading changes only in response to the object on its own side.
   - Confirm that `Left`, `Right`, and `Front` readings are independent.
7. Confirm **no "ERROR (timeout)"** messages appear during normal operation.

---

## Acceptance Criteria (PASS / FAIL)

V01 is **PASS** only when ALL of the following are true:

- [ ] Serial Monitor output appears (board communicates)
- [ ] All three sensors show **OK** in the init summary
- [ ] `Front` produces valid millimetre readings
- [ ] `Left` produces valid millimetre readings
- [ ] `Right` produces valid millimetre readings
- [ ] `Front` reacts **only** to objects in front
- [ ] `Left` reacts **only** to objects on the left
- [ ] `Right` reacts **only** to objects on the right
- [ ] All three sensors work **simultaneously** without interference
- [ ] No I2C address conflict (no garbled readings on one sensor when another is near an object)
- [ ] Serial output is stable (no repeated crashes or resets)
- [ ] **No motors activate** during the entire test

If any criterion fails, report the Serial Monitor output and the
failure symptom so V01 can be debugged before moving to V02.
