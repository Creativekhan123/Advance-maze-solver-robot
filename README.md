# Advance Maze Solver Robot

An autonomous high-speed maze-solving robot developed on the **ESP32 DOIT DevKit V1** with the **TB6612FNG** dual motor driver, optical wheel encoders, and three **VL53L0X Time-of-Flight (ToF)** laser distance sensors.

---

## Hardware Architecture

| Component | Hardware Model | Connection / Pins | Function |
|---|---|---|---|
| **MCU** | ESP32 DOIT DevKit V1 | Dual-core 240 MHz, 3.3V Logic | Master Controller |
| **Motor Driver** | TB6612FNG | Motor A & Motor B (STBY to 3.3V) | Active short-circuit braking & dual H-bridge drive |
| **Left Motor (A)** | N20 DC Gearmotor | `AIN1: GPIO 14`, `AIN2: GPIO 27`, `PWMA: GPIO 23` | Left wheel drive |
| **Right Motor (B)** | N20 DC Gearmotor | `BIN1: GPIO 18`, `BIN2: GPIO 19`, `PWMB: GPIO 5` | Right wheel drive |
| **Left Encoder** | Optical single-channel | `GPIO 33` (Interrupt, RISING) | Left wheel pulse counter |
| **Right Encoder** | Optical single-channel | `GPIO 34` (Interrupt, RISING) | Right wheel pulse counter |
| **Front Laser Sensor** | VL53L0X ToF Laser | `XSHUT: GPIO 13`, `Addr: 0x30` | Forward obstacle & wall detection |
| **Left Laser Sensor** | VL53L0X ToF Laser | `XSHUT: GPIO 25`, `Addr: 0x29` | Left corridor detection |
| **Right Laser Sensor** | VL53L0X ToF Laser | `XSHUT: GPIO 26`, `Addr: 0x31` | Right corridor detection |
| **I2C Bus** | Shared I2C | `SDA: GPIO 21`, `SCL: GPIO 22` | Sensor communication bus |

---

## Project Progression & Versions

The project follows an incremental engineering methodology where each version tests and proves a specific subsystem:

```text
Maze_solver_all_versios/
├── V01_Sensor_Test/             # Pass: 3x VL53L0X I2C XSHUT sequencing & live ranging
├── V02_Motor_Encoder_Test/      # Pass: TB6612FNG motor drive, PWM & encoder interrupts
├── V03_Movement_Calibration/    # Pass: 10 cm, 90°, 180° calibrated motion & active braking
└── V04_Basic_Maze_Navigation/   # Pass: Autonomous maze solving, turn alignment & live telemetry
```

### [V01 — Sensor Test](V01_Sensor_Test/)
- Sequential XSHUT reset and initialization of 3 VL53L0X sensors on addresses `0x30`, `0x29`, and `0x31`.
- Verified accurate distance measurement across all three channels.

### [V02 — Motor & Encoder Test](V02_Motor_Encoder_Test/)
- Verified directional motor control (Forward, Backward, Left rotation, Right rotation).
- Real-time encoder interrupt pulse counting on GPIO 33 & 34.
- Serial command interface for manual diagnostic control.

### [V03 — Movement Calibration & Active Braking](V03_Movement_Calibration/)
- Experimentally measured motion constants:
  - **10 cm Forward**: `650 counts`
  - **90° In-Place Rotation**: `520 counts`
  - **180° In-Place U-Turn**: `1040 counts`
- Implemented **Active Short-Circuit Motor Braking** (`AIN1=HIGH, AIN2=HIGH` / `BIN1=HIGH, BIN2=HIGH` with PWM `255` for 80 ms) on the TB6612FNG to eliminate battery-powered motor inertia and coasting.

### [V04 — Basic Maze Navigation](V04_Basic_Maze_Navigation/)
- **Autonomous Navigation Engine**: Classic Left-Hand Priority (`LEFT` → `FRONT` → `RIGHT` → `U-TURN`).
- **Wheel-Axis Turn Alignment**:
  - **Front OPEN**: Advances 10 cm forward before 90° turn to align the rear wheel axis with the intersection center.
  - **Front BLOCKED / WALL**: Advances 4 cm forward (adaptive safe clearance) before 90° turn to avoid ramming into the front wall.
  - **Post-Turn Advance**: Advances 10 cm forward into the new corridor before taking sensor readings.
- **Dead-End U-Turn**: Rotates 180° directly on the spot without moving forward.
- **Auto-Start Countdown**: 5-second power-on countdown with `S` abort.
- **Live Laser Telemetry**: Streams real-time `FORWARD`, `LEFT`, and `RIGHT` distances in centimeters (cm) with wall classification (`[WALL]` vs `[OPEN]`).

---

## Build & Upload with PlatformIO

1. Clone repository:
   ```bash
   git clone https://github.com/Creativekhan123/Advance-maze-solver-robot.git
   cd Advance-maze-solver-robot
   ```

2. Select the version to build in `platformio.ini`:
   ```ini
   [platformio]
   src_dir = V04_Basic_Maze_Navigation
   ```

3. Build and upload:
   ```bash
   pio run --target upload
   ```

4. Open Serial Monitor (115200 baud):
   ```bash
   pio device monitor -b 115200
   ```
