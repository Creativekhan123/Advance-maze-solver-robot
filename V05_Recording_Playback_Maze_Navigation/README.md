# FASTEST MAZE SOLVER ROBOT — VERSION 05

## Autonomous Navigation + Decision Recording + Recursive Path Simplification + Flash Playback

---

## 1. Overview & Purpose

Version 05 builds directly upon the validated motion, turning, and centering systems of **Version 04**, adding:
1. **Decision Recording**: Observes the robot's real-time autonomous navigation choices ($R, F, L, B$) at true maze junctions.
2. **Corridor Motion Filtering**: Suppresses repetitive forward steps so only meaningful branch decisions are recorded.
3. **Recursive Path Simplification**: Automatically detects dead ends ($B$) and simplifies previous turn sequences using relative-turn reduction rules.
4. **Persistent Flash Memory (ESP32 NVS via `<Preferences.h>`)**: Stores the learned optimal path in Flash storage so it survives power-cycles and reboots.
5. **Playback Mode (Fast Run)**: Re-runs the maze following the stored solved route without re-exploring dead-end branches.

> **IMPORTANT**: Version 04 navigation and motor parameters remain 100% untouched. Version 05 adopts the identical hardware pinouts, motor speeds, turn encoder targets, and sensor thresholds.

---

## 2. Interactive Serial Interface (115200 Baud)

At boot, the robot initializes all hardware and loads any existing solved path from Flash, then waits in Standby:

```text
================================
     MAZE SOLVER VERSION 05     
================================
Commands:
  'R' - Recording / Exploration Mode
  'P' - Playback Mode (Fast Run)
  'C' - Clear Stored Flash Solution
  'S' - Emergency Stop
Waiting for command...
```

### Supported Commands

| Key | Mode / Action | Description |
|:---:|---|---|
| `R` / `r` | **Recording / Exploration** | Starts 10-second countdown, then begins exploration with $R > F > L > B$ priority. Records junctions, simplifies dead-ends, and saves progress to Flash. |
| `P` / `p` | **Playback Mode** | Checks for valid saved path in Flash. If found, starts 10-second countdown and runs the optimal shortest path to the goal. |
| `C` / `c` | **Clear Solution** | Clears the NVS partition in Flash and resets path buffers for a new maze. |
| `S` / `s` | **Emergency Stop** | Halts all motors immediately and puts robot in Standby. |
| `E` / `e` | **Manual Maze End** | Manually triggers the maze finish sequence, saves the current path, and marks it valid. |

---

## 3. Configurable Start Delay

Configured near the top of `V05_Recording_Playback_Maze_Navigation.ino`:

```cpp
const unsigned long START_DELAY_SECONDS = 10;
const unsigned long START_DELAY_MS = START_DELAY_SECONDS * 1000;
```

When either `R` or `P` is selected, the robot displays a 10-second countdown before activating the motors:

```text
RECORDING / EXPLORATION mode selected.
Navigation will start in 10 seconds...
10... 9... 8... 7... 6... 5... 4... 3... 2... 1... GO!
```

---

## 4. Path Simplification Rules

Whenever a dead-end ($B$) is recorded, the path simplifier evaluates the trailing 3 decisions and collapses them into a single equivalent relative turn:

| Exploration Sequence | Simplified Decision | Geometric Rationale |
|:---:|:---:|---|
| `R R B` | **`F`** | Right + Right + 180° = Straight Forward |
| `R F B` | **`L`** | Right + Straight + 180° = Turn Left |
| `R L B` | **`B`** | Right + Left + 180° = Back (sub-branch dead end) |
| `F R B` | **`L`** | Straight + Right + 180° = Turn Left |
| `F F B` | **`B`** | Straight + Straight + 180° = Back (sub-branch dead end) |
| `F L B` | **`R`** | Straight + Left + 180° = Turn Right |
| `L R B` | **`B`** | Left + Right + 180° = Back (sub-branch dead end) |
| `L F B` | **`R`** | Left + Straight + 180° = Turn Right |
| `L L B` | **`F`** | Left + Left + 180° = Straight Forward |

### Recursive Simplification Example

Exploration path: `R -> F -> F -> F -> B`
1. Last 3: `F F B` $\rightarrow$ `B` (Path becomes: `R F B`)
2. Recursion triggers on new trailing `B`:
   `R F B` $\rightarrow$ `L` (Path becomes: `L`)
3. Final learned path: `L`

---

## 5. Hardware Parameters (Identical to V04)

* **Left Motor**: `AIN1: 14`, `AIN2: 27`, `PWMA: 23`
* **Right Motor**: `BIN1: 19`, `BIN2: 18`, `PWMB: 5`
* **Encoders**: `Left: 33`, `Right: 34`
* **Sensors**: `Front: 13 (0x30)`, `Right: 26 (0x31)`, `Left: 25 (0x32)`
* **I2C Bus**: `SDA: 21`, `SCL: 22`
* **Calibrated Constants**:
  * `motorForwardSpeed = 200`
  * `motorTurnSpeed = 150`
  * `turn_encoder_value = 520`
  * `extraSteps = 800`
  * `distance_threshold = 180`
  * `too_close_distance = 7`
