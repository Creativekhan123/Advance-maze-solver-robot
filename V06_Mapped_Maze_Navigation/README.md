# V06 — Bluetooth learning, estimated map, shortest recorded route

This is a separate PlatformIO project based on **V05_Left_Hand_Maze_Navigation**.
The original versions and root `platformio.ini` are unchanged. The root still
selects V04; use the commands below to build/upload V06 explicitly.

## Use

Pair the Android phone with **MazeSolver-V6**, then connect in Serial Bluetooth
Terminal using Bluetooth Classic/SPP. Send ASCII characters (not binary bytes).
USB Serial at 115200 supports the same commands. Newline/CR endings are ignored.

| Command | Action |
|---|---|
| `1` | Start a new exploration from the robot's current position/direction. |
| `2` | During learning: brake, mark the stopped location as the goal, optimize and save. During replay: stop without changing the saved route. |
| `3` | Replay the last successfully saved route. |
| `0` or `S` | Abort without saving an unfinished exploration. |
| `?` | Show instructions and saved-route availability while idle. |
| `M` | Print the current RAM map's nodes and edges while idle. |
| `C` | Clear the saved solution while idle. |

The robot stays stopped after boot. There is no countdown. Send `1` only when
positioned at the start. At the finish send `2`, wait for **Saved ...**, lift the
robot and replace it at the **same start position AND heading**, then send `3`.
The saved route survives power-off. A new exploration does not overwrite it until
`2` successfully saves a completed route. Faults/aborts retain the earlier route.
Sending `1`/`3` while moving does not restart the robot.

## Build and upload

From the repository root:

```powershell
pio run -d V06_Mapped_Maze_Navigation
pio run -d V06_Mapped_Maze_Navigation -t upload
pio device monitor -b 115200
```

Alternatively open this folder itself as the PlatformIO project. The source is
`main.cpp`, not an Arduino IDE `.ino` sketch. Dependencies are pinned to the
versions used for verification.

The supplied **4 MB flash partition table** allocates 128 KB to NVS for the route
blob and a 3 MB application partition for Bluetooth firmware. Upload the complete
PlatformIO project, including its partition table; do not upload just the app
binary against V05's old partition table. This changes the flash layout and is
not an OTA-compatible layout. No firmware was uploaded during development.

## What remains from V05

The original `autonomus()` decisions and maneuver calls are retained, with
interruptible waits replacing `delay(rest)`:

- Forward PWM **200**, turning PWM **150**, rest setting **150 ms**.
- **520** encoder counts for 90 degrees, **1040** for 180 degrees.
- **800** extra alignment/advance counts; **800 / 3** on the original blocked-front branches.
- The T-junction's **5-count reverse**, and the dead-end's **600-count reverse after turning**.
- Original left/straight/right/back priority and distance thresholds.
- Same motor, sensor and encoder pins; sensor addresses **0x30 / 0x31 / 0x32**.
- Original sequential steering corrections, with a forward fallback if both
  side distances are equal.

The motor implementation now accounts for encoder pulses continuously, has stall
and timeout checks, services stop commands, and explicitly brakes the unused
wheel during single-wheel corrections. Stops include a 60 ms settling interval
for measuring braking pulses. Mapping checkpoints add stops for stable readings.
Thus the **maneuver counts and sequence are preserved, but exact timing and
coasting behavior are not identical to V05**. Replay uses the same speeds; the
intended time saving comes from removing excursions, not increasing PWM.

## Mapping and optimization

1. Signed encoder distances estimate wheel-axis position and heading. Encoder
   polarity is inferred from the motor command because these are single-channel
   encoders. Coordinates are in **encoder counts**, so no guessed centimeter
   conversion or assumption of an 18 cm square grid is needed.
2. Record junction checkpoints, turn centres, and the movement segments between
   them. Corridor motion is stored as forward distance; replay applies fresh
   V05 wall-centering corrections instead of repeating old steering twitches.
3. Match revisited poses conservatively. Ordinary checkpoints require compatible
   position, heading and relative openings. Turning centres can connect different
   headings only when their world-frame opening signatures agree and the heading
   difference fits a calibrated 90/180-degree turn. These connectors allow a
   return from a dead end to be replaced by a turn at the earlier junction.
4. On `2`, append the actual stopped position as a separate goal and run Dijkstra
   over the resulting directed graph. The cost is average absolute wheel travel,
   including turn effort. This is **shortest estimated wheel effort through the
   recognized recorded graph**, not a guarantee of shortest geometric distance
   or fastest elapsed time. Reverse corridors are not invented.
5. Save the selected movement program and sensor checkpoints in NVS with version,
   length, command validation and checksum. The full exploration map remains in
   RAM for `M`; only the optimized replay program survives power-off.

The planner can remove an excursion only if its return is recognized. A junction
passed straight through without a known turning centre has more conservative
matching than one where the axle's turning position was recorded. Unexplored
passages and unrecognized connections cannot create shortcuts. Left-hand
exploration itself is retained: it can circle in some looped mazes and is not a
complete exploration strategy for every possible maze.

## Practical limits and tuning

This firmware has **not been validated on the physical robot**. With unknown
geometry, single-channel encoders and no active gyro, an error-free map/replay
cannot be guaranteed. Wheel slip, unequal motors and accumulated heading errors
can miss a true revisit or mistake similar nearby junctions for the same place.
Ambiguous candidates stay separate, but odometry is still an estimate.

`RouteMap.h` keeps the tunable matching values together:

- `MATCH_DISTANCE = 180` counts (roughly 1.6–2.3 cm from your 800-count estimate).
- `MATCH_ANGLE = 12` degrees.
- `MIN_REVISIT_TRAVEL = 2400` counts to avoid treating the immediate same event
  as a new visit.

Do not widen matching thresholds just to force more shortcuts: that can join
different junctions. Inspect `M` output and test a known small loop first.

Replay checks stable sensor signatures at recorded junction checkpoints and stops
on a mismatch rather than choosing an alternative turn. Travel between checkpoints
uses encoder distances, not absolute positioning; it does not automatically
correct every accumulated distance error. Final stopping has encoder resolution
and braking error. A `2` received during a turn records a shortened final movement;
the finish can therefore have a partially turned heading. Bluetooth delivery and
braking introduce finite latency; the finish is the stopped position, not the
instant you touched the phone screen.

Invalid/out-of-range readings are not treated as openings. The robot stops if a
sensor has no valid reading for 300 ms. Large open areas that produce sustained
out-of-range returns may therefore require a different sensing strategy. A front
clearance below 35 mm during forward drive stops the robot. These checks can stop
a V05 maneuver that previously continued close to a wall; they do not change its
requested encoder target. Turning still requires adequate physical clearance.

Limits: **256 poses, 512 edges, 2048 recorded commands**. Capacity exhaustion,
encoder stalls (700 ms without progress), and movement timeouts abort learning
without replacing the saved solution. Bluetooth disconnect is not an automatic
stop; the robot continues unless a stop command or fault occurs.

## Verification and first robot test

The ESP32 firmware builds successfully. Compiler-executed assertions in
`tests/compile_time_routes.cpp` verify shortest-route selection, heading shortcuts,
unreachable routes, ambiguous landmark rejection, and capacity/command checks.
`tests/test_route_map.cpp` additionally contains runtime odometry/checksum tests;
it compiles, but execution on this development PC was blocked by Windows Device
Guard. Those runtime tests are **not claimed as passed**.

`python V06_Mapped_Maze_Navigation/tests/check_v5_preserved.py` checks the copied
V05 maneuver function against the original source.

For hardware validation, first verify `0` and `2` interrupt straight motion and a
turn. Then learn/replay a short corridor, one corner, and a small dead-end route.
Finally test a loop with a known shortcut and inspect `M`. Confirm the saved
finish location and repeat after power-off. Use the same starting placement for
every replay. Keep the current speeds until those tests are reliable.
