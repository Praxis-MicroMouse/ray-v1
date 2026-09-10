# Ray v1 — MicroMouse Firmware

Firmware for our MicroMouse robot, built with [PlatformIO](https://platformio.org/) on an ESP32 (Arduino framework).

## Purpose

The robot senses walls on three sides (front, left, right) with VL53L0X
time-of-flight (ToF) sensors, drives two motors, reads wheel encoders and
a 9-DoF IMU, and monitors its own battery. This firmware brings up every
one of those, plus:

- **PID tuning + telemetry dashboard** (the current default build) — a
  serial protocol (`comms.h`) plus a companion local web app
  (`tools/dashboard`) for tuning the robot's control loops and watching
  every sensor live, on the bench, without reflashing between tweaks.
- **Maze solving** — flood fill for exploration and a turn-minimizing
  Dijkstra planner for the speed run, ported from the
  [`mms-c`](../mms-c/Main.c) simulator reference algorithm, runnable
  single-task (`solver.h`) or split across both ESP32 cores as FreeRTOS
  tasks (`tasks.h`). Validated against the actual algorithm code, not a
  reimplementation, via `tools/maze_cli`.

The codebase is written to be modular: each piece of hardware/functionality
gets its own header + implementation pair, and `main` just wires the
modules together.

## Hardware

- ESP32 dev board
- 3x VL53L0X ToF sensors on one I2C bus (SDA/SCL lines tied together)
- Each sensor's XSHUT pin wired separately so they can be brought up one at a
  time and assigned unique I2C addresses:
  - Front: GPIO 18
  - Right: GPIO 19
  - Left: GPIO 5
- I2C bus: SDA = GPIO 21, SCL = GPIO 22
- TB6612FNG-style dual motor driver:
  - Motor A: PWM = GPIO 26, IN1 = GPIO 25, IN2 = GPIO 33
  - Motor B: PWM = GPIO 27, IN1 = GPIO 14, IN2 = GPIO 32
  - STBY is assumed tied high in hardware (not software controlled)

  Note: GPIO34/35 are input-only on the ESP32 and can't drive PWM or a
  direction pin — if you see those referenced anywhere for motor B, they're
  wrong; the pins above are what's actually wired.
- Battery voltage divider on GPIO 34 (ADC1, input-only — a natural fit for
  an analog input): R1 = 14.1kΩ (battery+ to ADC pin), R2 = 9.4kΩ (ADC pin
  to GND), giving a divider ratio of 0.4 — a 3.7V nominal cell (up to 4.2V
  charged) reads back as ~1.2-1.7V at the ADC pin, comfortably inside the
  ESP32's 0-3.3V range.
- Quadrature wheel encoders (GA12-N20 motors, magnetic Hall encoder on
  the motor shaft — before the gearbox — 7 pulses/revolution per
  channel):
  - Left: A = GPIO 4, B = GPIO 16
  - Right: A = GPIO 17, B = GPIO 23
  - Gearbox ratio is still a placeholder (`ENCODER_GEARBOX_RATIO` in
    `encoder.h`, currently 1.0) — set it to the real GA12-N20 ratio
    (e.g. 30/50/100) once known, or output-shaft RPM and `control.h`'s
    distance-per-tick will both be wrong by that factor.
- IMU — either an MPU-9250/9255 (accel+gyro+magnetometer) or an MPU-6500
  (same accel/gyro core, no magnetometer); `mpu9250_init()` detects which
  one via `WHO_AM_I` and works with either. Shares the ToF sensors' I2C bus
  (SDA = GPIO 21, SCL = GPIO 22), address 0x68 (AD0 tied low). Suggested
  INT pin: GPIO 13 (not currently used — the driver polls instead of using
  the interrupt).

## Code structure

```
include/
  sensor.h       # public C-style API for the ToF sensor module
  filter.h       # public C-style API for the despike+smoothing filter sensor.cpp applies
  telemetry.h    # public C-style API for the serial telemetry module
  motor.h        # public C-style API for the motor driver module
  drive.h        # public C-style API for the simple movement module
  battery.h      # public C-style API for the battery voltage module
  encoder.h      # public C-style API for the wheel encoder module
  mpu9250.h      # public C-style API for the IMU module
  pid.h          # public C-style API for the generic PID controller
  control.h      # public C-style API for the concrete PID-driven control loops
  comms.h        # public C-style API for the serial telemetry/tuning protocol
  maze.h         # public C-style API for the maze grid + flood-fill/Dijkstra search
  solver.h       # public C-style API for the physical maze-solving run (single task)
  tasks.h        # public C-style API for the dual-core RTOS version of the solver
src/
  sensor.cpp     # ToF sensor implementation (I2C/XSHUT bring-up, reads, logging)
  filter.cpp     # despike + exponential-smoothing filter, one instance per ToF channel
  telemetry.cpp  # streams sensor readings over serial as DATA,... lines
  motor.cpp      # motor driver implementation (direction pins + LEDC PWM)
  drive.cpp      # simple forward/turn movement built on the motor module
  battery.cpp    # battery voltage divider reading over ADC + charge estimate
  encoder.cpp    # quadrature encoder tick counting via pin-change interrupts
  mpu9250.cpp    # MPU9250/6500 accel/gyro/mag driver over raw I2C register access
  pid.cpp        # generic PID controller
  control.cpp    # straight-line/turn/wall-centering PID loops built on pid.h
  comms.cpp      # serial line protocol: telemetry out, PID gains + RUN commands in
  maze.cpp       # maze grid state + flood-fill/Dijkstra search (no hardware calls)
  solver.cpp     # drives the real robot through a maze.cpp search using sensor.h/drive.h
  tasks.cpp      # same solve, split into a planning task (core 0) + control task (core 1)
  main.cpp       # setup()/loop() — initializes and calls the modules
platformio.ini   # board/framework config + library dependencies
tools/
  maze_cli/      # host CLI that runs maze.cpp's actual algorithm against a maze - see its README
  dashboard/     # local web app: PID tuning + live telemetry + maze validator - see its README
```

- **`sensor.h`** declares the sensor module's public interface: pin/address
  constants, the `sensor_reading_t` struct, and `sensor_init()` /
  `sensor_read_all()`. Any other module only needs to include this header.
- **`sensor.cpp`** contains the implementation: it resets all three sensors via
  XSHUT, brings them up one at a time so each can be assigned a unique I2C
  address (they'd otherwise collide on the shared bus), enables the ESP32's
  internal pull-ups on SDA/SCL before `Wire.begin()` (no external pull-ups
  on the breakouts, and `Wire.begin()` doesn't reliably enable them itself),
  and reads distances via the Adafruit VL53L0X library in its
  `HIGH_ACCURACY` profile (longer timing budget, lower noise — matters most
  at the short 0-10cm ranges used for tuning). Before calling
  `Adafruit_VL53L0X::begin()` for each sensor, it does a cheap, bounded I2C
  presence probe at the VL53L0X's default address and skips that sensor
  (logging why) if nothing ACKs — `begin()`'s own internal init/calibration
  polling loop has **no timeout** and has been observed to hang `setup()`
  forever on a sensor that never responds, taking the whole board (and
  every telemetry line with it) down; the probe avoids ever entering that
  path for a sensor that clearly isn't there. Every raw reading is run
  through `filter.h`'s despike+smoothing filter (one instance per channel)
  before being returned, since VL53L0X readings are prone to occasional
  wild single-sample spikes from stray reflections. Every step logs to
  serial (`[SENSOR] ...`) for debugging.
- **`filter.h`/`filter.cpp`** — a small stateful filter for noisy,
  spike-prone distance readings: a new reading more than
  `FILTER_SPIKE_THRESHOLD_MM` away from the current estimate is held back
  unless the *previous* raw reading already agreed with it (so a real fast
  change, like a wall appearing, still gets through after one confirming
  sample, while a lone bad reading doesn't move the estimate at all);
  accepted readings are blended in via exponential moving average
  (`FILTER_EMA_ALPHA`). Generic (`filter_t` + `filter_update()`), so it's
  not tied to ToF sensors specifically, but `sensor.cpp` is its only
  current user.
- **`telemetry.h`/`telemetry.cpp`** print one sensor reading per call as a
  machine-parseable serial line (`DATA,<millis>,<front_mm>,<right_mm>,<left_mm>`),
  kept separate from the `[SENSOR]` debug logs so a host tool can filter for
  `DATA,` lines and ignore the rest. Not used by the current default build
  (see `comms.h` for the richer protocol that superseded it for tuning) —
  kept for a lighter-weight sensor-only stream if that's ever useful again.
- **`motor.h`** declares the motor module's public interface:
  `motor_id_t` (`MOTOR_A`/`MOTOR_B`), pin constants, `motor_init()`,
  `motor_set_speed(motor, speed)` (-255..255, negative = reverse), and
  `motor_stop_all()`.
- **`motor.cpp`** sets up each motor's direction pins as digital outputs and
  its PWM (speed) pin via the ESP32's LEDC peripheral (20kHz, 8-bit duty),
  and logs every init/speed change (`[MOTOR] ...`).
- **`drive.h`/`drive.cpp`** implement simple two-wheel differential drive on
  top of `motor`: `drive_forward(speed)`, `drive_turn_left(speed)` /
  `drive_turn_right(speed)` (pivot turns — wheels spin opposite directions
  in place), and `drive_stop()`. `drive.h` also exposes the
  wheel-to-motor/polarity mapping (`DRIVE_LEFT_MOTOR`/`DRIVE_RIGHT_MOTOR`/
  `DRIVE_LEFT_SIGN`/`DRIVE_RIGHT_SIGN`) so other modules needing independent
  per-wheel control — `control.cpp`'s PID loops, notably — reuse the same
  mapping instead of guessing it again. Left/right-to-Motor-A/B and each
  wheel's polarity are guesses; if a wheel spins the wrong way, flip its
  sign in `drive.h` rather than rewiring.
- **`battery.h`/`battery.cpp`** read the ADC (8-sample average, `ADC_11db`
  attenuation for full 0-3.3V range), convert through the known divider
  ratio, and log the battery voltage (`[BATTERY] ...`) every call, with a
  warning line if it drops below `BATTERY_LOW_VOLTAGE` (3.3V).
  `battery_get_percent(voltage)` maps that to a rough 0-100% charge
  estimate via a piecewise-linear 1S LiPo discharge curve — good enough for
  a dashboard gauge, not a calibrated fuel gauge (it'll read a bit low
  under load, from voltage sag).
- **`encoder.h`/`encoder.cpp`** decode each wheel's quadrature encoder via a
  `CHANGE` interrupt on its channel-A pin (reading channel B at that instant
  to get direction), and expose a running signed tick count per wheel via
  `encoder_get_ticks()` / `encoder_reset()` — 2 ticks per encoder pulse,
  since `CHANGE` fires on both edges (`ENCODER_TICKS_PER_MOTOR_REV` = 2 x
  the datasheet's 7 PPR = 14). `encoder_init()` logs and skips any encoder
  whose pins are left at `-1` rather than touching undefined hardware
  (not currently the case — both are wired, see Hardware above).
  `encoder_get_motor_rpm()`/`encoder_get_output_rpm()` turn the tick rate
  into motor-shaft/output-shaft RPM, averaged over whatever interval
  they're actually called at (comms.cpp calls them once per telemetry
  tick); output RPM divides by `ENCODER_GEARBOX_RATIO`, still a 1.0
  placeholder pending the real GA12-N20 ratio.
- **`mpu9250.h`/`mpu9250.cpp`** talk to the IMU directly over I2C register
  access (no external library) — accel + gyro from the MPU9250/9255/6500
  core (whichever is actually on the board; `mpu9250_init()` checks
  `WHO_AM_I` and works with any of the three), plus magnetometer from the
  embedded AK8963 chip when present (MPU9250/9255 only — reached via I2C
  bypass mode; an MPU6500 has none, and `mpu9250_read()` just leaves the
  mag fields at zero in that case rather than failing). `mpu9250_init()`
  sets accel/gyro full-scale range and low-pass filtering and starts the
  magnetometer in continuous mode when available. `mpu9250_read()` returns
  accel (g), gyro (deg/s), mag (µT), and temperature (°C), logging every
  call (`[MPU9250] ...`). It shares the ToF sensors' I2C bus; suggested
  pins (INT, address) are documented in `mpu9250.h`.
- **`pid.h`/`pid.cpp`** — a minimal generic PID controller (`pid_ctrl_t` +
  `pid_update()`), with anti-windup clamping on the integral term. One
  instance per control loop; gains are set/read independently at runtime
  (see `control.h`/`comms.h`) rather than baked in at compile time. (Named
  `pid_ctrl_t`, not `pid_t` — the latter collides with POSIX's process-ID
  typedef, which Arduino.h pulls in transitively.)
- **`control.h`/`control.cpp`** — three concrete PID-driven maneuvers, each
  tunable independently:
  - `CONTROL_LOOP_STRAIGHT` — dual-wheel encoder speed sync while driving
    forward (`control_run_straight(target_mm, ...)`).
  - `CONTROL_LOOP_TURN` — gyro-integrated heading hold while pivoting
    (`control_run_turn(target_deg, ...)`).
  - `CONTROL_LOOP_WALLCENTER` — ToF left/right centering while driving
    forward (`control_run_wallcenter(duration_ms, ...)`).

  Plus one open-loop maneuver not tied to a PID loop:
  `control_run_spin(motor, pwm, ...)` just holds one motor at a constant
  PWM (no target, no closed-loop correction) for up to
  `CONTROL_SPIN_MAX_RUN_MS` (60s) — meant for benchtop tests like timing
  **output-shaft** rotations by hand (mark the wheel, use a stopwatch) to
  check whether two candidate motors have matching gearbox ratios.
  `encoder_get_motor_rpm()` can't answer that on its own: the encoder is
  on the motor shaft, *before* the gearbox, so it only sees the bare
  motor's free speed — two motors can report identical motor RPM and
  still have different gearbox ratios (a known issue with cheap N20
  gearmotor batches).

  Each `control_run_*()` blocks until its maneuver finishes, hits its
  hard safety timeout (`CONTROL_MAX_RUN_MS`, 5s, for the three PID
  maneuvers above), or `control_request_abort()` is called — and calls a
  `tick_cb` every ~10ms so the caller (`comms.cpp`) can stream telemetry
  and poll for that abort request while the maneuver runs.
  `WHEEL_DIAMETER_MM` is a measured-by-hand guess (tune it); its
  `ENCODER_TICKS_PER_REV` is derived from `encoder.h`'s
  `ENCODER_TICKS_PER_MOTOR_REV` and `ENCODER_GEARBOX_RATIO`, so it's only
  as accurate as that ratio is.
- **`comms.h`/`comms.cpp`** implement the serial protocol the dashboard
  (`tools/dashboard`) speaks: plain-text commands in (`PID <loop> <kp> <ki>
  <kd>`, `GETPID <loop>`, `RUN <maneuver> <arg>`, `RUN stop`), one JSON
  telemetry object out per line (sensor/battery/encoder/RPM/PWM/IMU/
  PID-debug readings) — both roughly every 50ms when idle, and once per control-loop
  iteration during a `RUN`. See the comment block at the top of `comms.h`
  for the exact schema.
- **`maze.h`/`maze.cpp`** hold the maze grid (walls-per-cell bitmask) and two
  search algorithms over it, both pure grid logic — no sensor/motor calls —
  so neither depends on real hardware:
  - `maze_flood_fill()` — a multi-source BFS that computes each cell's
    shortest distance (in cell-steps) to a set of goal cells, plus
    `maze_choose_next_direction()` to pick the best open neighbor. This is
    ported from the [`mms-c`](../mms-c/Main.c) simulator reference
    algorithm and is what exploration uses: it only needs to know about
    walls sensed so far, and adapts as more are discovered.
  - `maze_plan_min_turn_path()` — Dijkstra over an expanded (cell, heading)
    state graph, where driving forward one cell and pivoting 90 degrees in
    place are separately-weighted edges (`MAZE_MOVE_COST`, `MAZE_TURN_COST`
    — turning costs more, so the cheapest path trades a few extra cells for
    fewer turns when that's available). This assumes the maze is already
    fully explored — unlike flood fill it won't self-correct from a wrong
    guess — and is meant for planning the fast "speed run" once the map is
    known, since turns cost real time a plain cell-count metric ignores.
  Maze size defaults to a full 16x16 grid (`MAZE_WIDTH`/`MAZE_HEIGHT`);
  change those for a different contest maze size. `MAZE_CELL_SIZE_MM`
  (180 - measured: cells are 18cm x 18cm) is the physical size of one
  cell; `solver.h`'s wall-detection threshold is derived from it. Both
  search algorithms are validated against a host CLI in `tools/maze_cli`
  — see its README.
- **`solver.h`/`solver.cpp`** are the single-task hardware glue: `solver_run()`
  explores using `maze_flood_fill()` (senses walls with `sensor_read_all()` —
  a wall is "there" if a ToF reading is under `SOLVER_WALL_THRESHOLD_MM` —
  and moves with `drive_forward()`/`drive_turn_left()`/`drive_turn_right()`
  in place of the simulator's `API_*` calls), the same
  search-to-center-then-return-to-start loop as `mms-c/Main.c`'s `main()`.
  Movement is timed/open-loop by default (`SOLVER_CELL_MOVE_TIME_MS`,
  `SOLVER_TURN_90_TIME_MS` — both untuned placeholders); once `encoder.h`'s
  pins are wired up and `SOLVER_CELL_TICKS` is set to a measured
  ticks-per-cell value, it switches to counting encoder ticks instead. Not
  wired into `main.cpp` by default — see the commented-out block near the
  top of `main.cpp` to enable it (in place of the PID dashboard build).
- **`tasks.h`/`tasks.cpp`** run the same overall solve as `solver.cpp` but
  split across the ESP32's two cores as separate FreeRTOS tasks, talking
  over two queues:
  - **Planning task (core 0)** owns the maze grid and both search
    algorithms — flood fill while exploring, then `maze_plan_min_turn_path()`
    once back at the start — and never touches a sensor or motor directly;
    it only sends action requests (sense/move/turn) and reads back sensed
    walls.
  - **Control task (core 1)** owns all hardware I/O: it calls `sensor_init()`
    itself on startup, then executes each requested action with
    `drive_forward()`/`drive_turn_left()`/`drive_turn_right()` and reports
    sensed walls back after every one.
  During exploration the two necessarily hand off in lockstep (the next
  decision depends on what the last move sensed), but once the map is known
  the planning task computes the whole speed-run path up front — see the
  note in `tasks.cpp` on queueing it further ahead for real overlap between
  the two cores. `tasks_start()` spawns both tasks and returns immediately;
  call it once from `setup()` and leave `loop()` idle. This is an
  alternative to `solver.h`, not a complement to it — see the note in
  `main.cpp` where both are wired in as opt-in blocks.
- **`main.cpp`** — **current default build: the PID-tuning dashboard.**
  Brings up motors, encoders, the IMU, and the ToF sensors in `setup()`,
  then in `loop()` dispatches incoming serial commands (`comms_poll()`) and
  sends one telemetry line roughly every 50ms (`comms_tick()`) — see
  `comms.h` above and `tools/dashboard`. The maze-solving builds
  (`solver.h`/`tasks.h`) and the earlier bare motion-test wiring are left
  as commented-out alternatives at the top of the file — mutually
  exclusive with this build and each other; swap one in once PID is dialed
  in and you're ready to run the actual maze.

Note: implementation files are `.cpp` rather than `.c` because the Arduino/ESP32
core and the VL53L0X sensor library are C++ (classes, `Wire`, etc.) — a plain C
compiler can't build against them. The public APIs (e.g. `sensor.h`) are kept
in plain C style regardless, so modules stay simple to call from `main`.

## Building

```
pio run          # build
pio run -t upload   # flash to the board
pio device monitor  # view serial logs (115200 baud) - or use tools/dashboard instead
```

## Tools

- **`tools/maze_cli`** — a host-buildable CLI that runs the actual
  `maze.cpp` algorithm (flood fill + Dijkstra) against a maze, so it can
  be validated/visualized without hardware or the `mms` simulator's GUI.
  See its README for the build step and I/O protocol.
- **`tools/dashboard`** — the local web app for PID tuning, live
  telemetry, and maze-algorithm validation described above. See its
  README for setup and usage. It calls `tools/maze_cli` for the maze
  validator half, and can launch the real `mms` simulator AppImage
  (`../mms-x86_64.AppImage` relative to this repo, as configured in
  `tools/dashboard/server.py`) as a separate window for visual/manual
  reference — scripting that GUI itself to auto-run and report back a
  path was out of scope given the timeline, so the dashboard's own maze
  validator (driving the real algorithm code directly) is the primary way
  to check the algorithm, not the AppImage.
