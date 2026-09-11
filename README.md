# Ray v2 — MicroMouse Firmware

Firmware for our MicroMouse robot: ESP32 (Arduino framework), 3x VL53L0X
ToF wall sensors, quadrature wheel encoders. No IMU.

This is a from-scratch control/motion architecture modeled on
[ukmarsbots' mazerunner-core](https://github.com/ukmars/mazerunner-core) —
a proven, competition-run reference design — adapted from its single-core
AVR + analog-IR + 500Hz-ISR shape to our dual-core ESP32 + ToF + FreeRTOS
shape. See "What changed from v1" below for what specifically was ported
and what was deliberately left out of this pass.

## Architecture

Three FreeRTOS tasks (`tasks.cpp`), each owning a different concern:

- **control task (core 1, highest priority)** — the fast, deterministic
  loop (`control_loop.h`), fixed at `CONTROL_LOOP_HZ` (500Hz):
  `odometry_update() → motion_update() → drive_controller_update()`.
  Never touches I2C or Serial, so nothing can stall it.
- **sensor task (core 0)** — the slower, I2C-bound loop
  (`sensor_loop.h`), at `SENSOR_LOOP_HZ` (100Hz): polls the three ToF
  sensors, recomputes the continuous steering correction, and
  periodically refreshes the cached battery voltage.
- **mouse task (core 0)** — `mouse.cpp`'s `mouse_run()`: a plain
  blocking sequence (explore to center → return to start → plan →
  speed run) that waits on state the other two tasks maintain in the
  background, the same way ukmarsbots' single AVR core just calls
  blocking `Motion` methods while its 500Hz ISR does the real work.

```
odometry.h        encoder ticks -> mm/deg per control tick
profile.h         trapezoidal (accel/cruise/brake) motion profile - pure math, unit tested
pd.h              generic PD controller (no integral) - pure math, unit tested
motion.h          two profile_t instances (forward, rotation) + move()/turn()/spin_turn() - cross-task-safe
drive_controller.h  PD(target vs actual) + per-wheel feedforward -> battery-compensated motor voltage
steering.h        continuous cross-track correction from the side ToF sensors -> feeds drive_controller's rotation target
sensor.h          ToF bring-up (continuous ranging mode) + per-sensor calibration + wall booleans
maze.h            flood-fill (exploration) + Dijkstra turn-minimizing planner (speed run) - unchanged logic from v1
mouse.h           high-level search/turn/centering behavior built on all of the above
control_loop.h / sensor_loop.h / tasks.h   the two fixed-rate ticks + the FreeRTOS task wiring
```

Every tuning constant lives in **`include/config/`** — see below.

**See [`TUNING.md`](TUNING.md) for the full step-by-step tuning procedure**
(bring-up → geometry → feedforward → PD gains → turn geometry → speed) —
every bench `RUN_MODE_*` mentioned there is in `src/main.cpp`.

## `include/config/` — every tuning constant, in one place

This is the file set to open for calibration; nothing outside it should
hardcode a physical constant, gain, or threshold.

| File | Contents |
|---|---|
| `pins.h` | every GPIO assignment |
| `robot_physical.h` | wheel diameter, ticks/rev (per side), turn radius, encoder/motor polarity, mouse footprint |
| `maze_layout.h` | maze width/height, cell size, back-wall-to-center, sensing position |
| `sensor_calibration.h` | per-ToF-sensor linear calibration equation (`scale`, `offset_mm`), mount offsets, wall thresholds, front-reliability limit |
| `motion_tuning.h` | loop rates, motor voltage limits, **feedforward** (`FF_LEFT/RIGHT_KV/BIAS/KA`), **PD gains** for every controller (`PD_FORWARD_*`, `PD_ROTATION_*`, `PD_STEERING_*`), search/turn/speed-run speeds and accelerations |
| `turn_params.h` | per-turn-type geometry (entry/exit offset, angle, omega, alpha, trigger distance) + spin-turn trim |
| `config.h` | includes all of the above — most code just does `#include "config/config.h"` |

Every value marked `PLACEHOLDER` or `TODO` is an untuned starting guess,
not measured data — tune in this order (each stage's comments in
`motion_tuning.h` document the procedure):

1. **Feedforward** (`FF_*`) — open-loop, `drive_controller_disable()`.
2. **Forward PD** (`PD_FORWARD_*`) — closed-loop straight-line tracking.
3. **Rotation PD** (`PD_ROTATION_*`) — closed-loop turning.
4. **Steering PD** (`PD_STEERING_*`) — closed-loop wall-centering.
5. **Turn geometry** (`turn_params.h`) — entry/exit offsets, same
   bench procedure as ukmarsbots' `test_SS90E()`.

Runtime (no-reflash) gain tuning is also available:
`drive_controller_set_forward_gains()` /
`drive_controller_set_rotation_gains()` / `steering_set_gains()` update
the live PD instances directly — wire these to a serial command if you
want to iterate without reflashing each time.

## Hardware

- ESP32 dev board
- 3x VL53L0X ToF sensors on one I2C bus (SDA=GPIO21, SCL=GPIO22),
  separate XSHUT pins (front=18, right=19, left=5)
- TB6612FNG-style dual motor driver (left: PWM=26/IN1=33/IN2=25; right:
  PWM=27/IN1=14/IN2=32); STBY tied high in hardware
- Battery voltage divider on GPIO34 (R1=14.1kΩ, R2=9.4kΩ)
- Quadrature wheel encoders (left: A=4/B=16; right: A=17/B=23)
- Optional single push-button on a GPIO of your choice (set
  `PIN_BUTTON` in `config/pins.h`) for aborting a run — the firmware
  runs the same without one, just with no way to stop a run early
  short of power-cycling

All of the above are `config/pins.h` — change wiring there only.

## Building

```
pio run                 # build the maze-run firmware
pio run -t upload       # flash it
pio device monitor       # serial logs, 115200 baud
pio test -e native       # host-run unit tests (maze, pd, profile, filter)
```

`src/main.cpp` has a `RUN_MODE` switch at the top:
- `RUN_MODE_MAZE` (default) — the real thing.
- `RUN_MODE_BRINGUP` — motors stay off; streams ToF readings, encoder
  ticks, and battery voltage every 200ms, for sanity-checking sensors
  before trusting the mouse to drive on them.

## Testing

- **`test/`** (`pio test -e native`) — host-native Unity tests for the
  zero-hardware-dependency modules: `maze.cpp` (grid/flood-fill/Dijkstra),
  `pd.cpp` (generic PD math), `profile.cpp` (trapezoidal motion math),
  `filter.cpp` (despike+EMA). Everything else touches Arduino/FreeRTOS/I2C
  somewhere in its call chain and is covered by `RUN_MODE_BRINGUP` /
  on-robot testing instead.
- **`tools/maze_cli`** — host CLI running the actual `maze.cpp` algorithm
  against a maze file, for validating/visualizing search behavior without
  hardware.

## What changed from v1

The previous version of this firmware had the maze-solving code
(`solver.cpp`/`tasks.cpp`) drive the robot **open-loop** (fixed PWM for a
fixed number of milliseconds), while a separately-built PID/encoder
control stack (`control.cpp`, `turns.cpp`) sat unused. This rewrite
replaces both with the single pipeline described above:

- **Every move goes through the same closed loop** — encoder odometry,
  a trapezoidal speed profile, and a PD+feedforward controller — instead
  of three disconnected implementations.
- **Continuous wall-centering** while driving straight (`steering.h`),
  not just wall-mapping after the fact.
- **Feedforward + battery-voltage-compensated PWM** (`drive_controller.cpp`,
  `motor.cpp`), so gains are far easier to tune against motor stiction
  and speed doesn't drift as the battery discharges.
- **PID → PD** everywhere (`pid.h/.cpp` → `pd.h/.cpp`): every controller
  here tracks a continuously-updated profile target rather than a fixed
  setpoint, so there's no steady-state error for an integral term to
  correct, and one less thing to tune/wind up.
- **The unused MPU9250 IMU code was removed.** ukmarsbots' own default
  configuration is encoder-only for both position and heading (a gyro is
  an optional enhancement it explicitly supports, not a requirement), so
  going IMU-less isn't a gap relative to the reference design — this
  firmware leans fully into that approach.
- **Continuous ToF ranging mode** (`sensor.cpp`) instead of blocking
  single-shot reads, so the sensor loop isn't spending 20-30ms per
  sensor per poll.
- **Dual-core-correct locking** (`sync.h`): shared state crossing a task
  boundary uses a `portMUX_TYPE` spinlock, not `noInterrupts()`/
  `interrupts()` — the latter only blocks the *current* ESP32 core and
  would not have been safe once cross-core shared state was introduced.

Deliberately **out of scope for this pass** (noted in `mouse.h`/`turn_params.h`):
hand-start detection (start-by-covering-a-sensor), wall-following/wander
test modes, and the bench calibration routines ukmarsbots ships
(front-sensor-vs-distance table, sensor-spin calibration, edge
detection) — `RUN_MODE_BRINGUP` covers basic sensor/encoder sanity
checking in the meantime. Turn geometry (`turn_params.h`'s entry/exit
offsets and triggers) is ported as untuned starting guesses and needs
the same on-bench tuning pass ukmarsbots documents for its own turns.

## Tools

- **`tools/maze_cli`** — a host-buildable CLI that runs the actual
  `maze.cpp` algorithm (flood fill + Dijkstra) against a maze, so it can
  be validated/visualized without hardware. See its README.
