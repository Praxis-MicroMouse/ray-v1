# Ray v1 — MicroMouse Firmware

Firmware for our MicroMouse robot, built with [PlatformIO](https://platformio.org/) on an ESP32 (Arduino framework).

## Purpose

The robot senses walls on three sides (front, left, right) with VL53L0X
time-of-flight (ToF) sensors, drives two motors, reads wheel encoders and
a 9-DoF IMU, and monitors its own battery. This firmware brings up every
one of those, plus:

- **PID-driven control loops** (`control.h`) — dual-wheel encoder speed
  sync while driving straight, gyro-integrated heading hold while
  turning, and ToF-based wall centering, each independently tunable.
- **Maze solving** — flood fill for exploration and a turn-minimizing
  Dijkstra planner for the speed run, ported from the
  [`mms-c`](../mms-c/Main.c) simulator reference algorithm, runnable
  single-task (`solver.h`) or split across both ESP32 cores as FreeRTOS
  tasks (`tasks.h`). Validated against the actual algorithm code, not a
  reimplementation, via `tools/maze_cli`.

The codebase is written to be modular: each piece of hardware/functionality
gets its own header + implementation pair. `main.cpp` wires the modules
together into a set of build modes (motors-only, drive test, sensor
telemetry, obstacle avoidance, battery/IMU bring-up, motor PID sync,
encoder calibration, a one-cell straight-line test, and the two maze
solver variants) — see the top of `main.cpp` for the `MODE_*` switches
that select which one gets compiled in. Only one build mode is active at
a time; flip it there and reflash rather than maintaining separate sketches.

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
- Quadrature wheel encoders:
  - Left: A = GPIO 4, B = GPIO 16
  - Right: A = GPIO 17, B = GPIO 23
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
  maze.h         # public C-style API for the maze grid + flood-fill/Dijkstra search
  solver.h       # public C-style API for the physical maze-solving run (single task)
  tasks.h        # public C-style API for the dual-core RTOS version of the solver
  ota.h          # public C-style API for the WiFi AP + OTA flashing module
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
  maze.cpp       # maze grid state + flood-fill/Dijkstra search (no hardware calls)
  solver.cpp     # drives the real robot through a maze.cpp search using sensor.h/drive.h
  tasks.cpp      # same solve, split into a planning task (core 0) + control task (core 1)
  ota.cpp        # WiFi access point + ArduinoOTA bring-up for wireless testing
  main.cpp       # setup()/loop() — MODE_* build-select switches between test/solve builds
platformio.ini   # board/framework config + library dependencies
tools/
  maze_cli/      # host CLI that runs maze.cpp's actual algorithm against a maze - see its README
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
  machine-parseable line (`DATA,<millis>,<front_mm>,<right_mm>,<left_mm>`)
  over Serial, and — once `telemetry_init()` has run and a laptop is
  connected to `ota.h`'s access point — the same line as a UDP broadcast
  (`TELEMETRY_UDP_PORT`) so it's readable wirelessly too. Kept separate
  from the `[SENSOR]` debug logs so a host tool (or just your own eyes on
  the serial monitor) can filter for `DATA,` lines and ignore the rest.
  Used by `main.cpp`'s `MODE_SENSOR_TELEMETRY` build (the current default)
  — motors stay off, only the ToF sensors are brought up, for bench-tuning
  sensor placement/thresholds in isolation.
- **`ota.h`/`ota.cpp`** bring up the ESP32 as its own WiFi access point
  (`OTA_AP_SSID`/`OTA_AP_PASSWORD`) and start `ArduinoOTA` on it, so the
  testing phase doesn't need a USB cable or a shared router network — the
  laptop connects straight to the board's AP. See "Wireless testing (OTA)"
  below for the full workflow. Currently wired into `MODE_SENSOR_TELEMETRY`
  only; add `ota_init()`/`ota_handle()` calls to another `MODE_*` block the
  same way if you need OTA there too (careful with motor-driving modes —
  `ota_handle()` still needs calling regularly, so don't let a maneuver
  block for too long without it).
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
  a rough "how much is left" readout, not a calibrated fuel gauge (it'll
  read a bit low under load, from voltage sag).
- **`encoder.h`/`encoder.cpp`** decode each wheel's quadrature encoder via a
  `CHANGE` interrupt on its channel-A pin (reading channel B at that instant
  to get direction), and expose a running signed tick count per wheel via
  `encoder_get_ticks()` / `encoder_reset()`. `encoder_init()` logs and skips
  any encoder whose pins (`ENCODER_LEFT_A_PIN` etc.) are set to `-1` rather
  than touching undefined hardware — not currently the case, both are wired
  per the pins listed under Hardware above.
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
  (see `control.h`) rather than baked in at compile time. (Named
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
  check whether two candidate motors have matching gearbox ratios (their
  motor-shaft speed alone won't reveal that — two motors can spin their
  bare shafts at the same rate and still have different gearbox ratios, a
  known issue with cheap N20 gearmotor batches).

  Each `control_run_*()` blocks until its maneuver finishes, hits its
  hard safety timeout (`CONTROL_MAX_RUN_MS`, 5s, for the three PID
  maneuvers above), or `control_request_abort()` is called — and calls an
  optional `tick_cb` every ~10ms so a caller can observe/abort mid-maneuver
  (pass `nullptr` if that's not needed, as `main.cpp`'s `MODE_STRAIGHT_18CM`
  does). `WHEEL_DIAMETER_MM`/`ENCODER_TICKS_PER_REV` are measured (32mm,
  715 ticks/rev) via `main.cpp`'s `MODE_ENCODER_CALIBRATION` build - see
  the comment above them in `control.h` if the wheel or encoder changes
  and they need re-measuring.
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
  `SOLVER_TURN_90_TIME_MS` — both untuned placeholders); once
  `SOLVER_CELL_TICKS` is set to a measured ticks-per-cell value (derive it
  from `control.h`'s now-measured `ENCODER_TICKS_PER_REV`/
  `WHEEL_DIAMETER_MM`), it switches to counting encoder ticks instead. Not
  active by default — select `main.cpp`'s `MODE_MAZE_SOLVER` to enable it.
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
  alternative to `solver.h`, not a complement to it — select `main.cpp`'s
  `MODE_MAZE_SOLVER_RTOS` instead of `MODE_MAZE_SOLVER` to use it.
- **`main.cpp`** — every `setup()`/`loop()` build this firmware has needed
  lives here at once, each wrapped in `#if defined(MODE_...)`. A block of
  `#define MODE_*` lines near the top of the file selects which one
  compiles in; uncomment exactly one (a `#error` check refuses to build
  otherwise) and reflash. Current default: `MODE_SENSOR_TELEMETRY`
  (motors off, ToF sensors only, streamed as `DATA,` lines — for bench
  sensor tuning). Other modes: `MODE_MOTORS_ONLY`, `MODE_DRIVE_TEST`,
  `MODE_OBSTACLE_AVOID`, `MODE_BATTERY_IMU_BRINGUP`, `MODE_MOTOR_PID_SYNC`,
  `MODE_ENCODER_CALIBRATION`, `MODE_STRAIGHT_18CM`, `MODE_FULL_SEND_1M`
  (open-loop, full PWM, 1m straight-line stress test — see below),
  `MODE_MAZE_SOLVER`, and `MODE_MAZE_SOLVER_RTOS` — see each block's
  comment in `main.cpp` for what it does.

Note: implementation files are `.cpp` rather than `.c` because the Arduino/ESP32
core and the VL53L0X sensor library are C++ (classes, `Wire`, etc.) — a plain C
compiler can't build against them. The public APIs (e.g. `sensor.h`) are kept
in plain C style regardless, so modules stay simple to call from `main`.

## Building

```
pio run          # build
pio run -t upload   # flash to the board
pio device monitor  # view serial logs (115200 baud)
```

## Wireless testing (OTA)

`MODE_SENSOR_TELEMETRY` (the current default build) brings up its own WiFi
access point via `ota.h` instead of requiring the laptop and board to
share a network — the board *is* the network:

1. Flash once over USB as usual (`pio run -t upload`) so `ota_init()` is
   on the board.
2. Connect your laptop's WiFi to the access point it starts:
   SSID `MicroMouse`, password `mmouse2026` (see `include/ota.h` to
   change either). The board is always reachable at `192.168.4.1`.
3. **Receiving values:** `telemetry_send()` broadcasts the same `DATA,...`
   lines it prints over Serial as UDP packets to port `4210`. Listen for
   them from the laptop with e.g.:
   ```
   nc -ul 4210
   ```
   or point a small Python/MATLAB UDP socket at `0.0.0.0:4210`.
4. **Sending code flashes:** build and flash new firmware over the same
   link instead of USB:
   ```
   pio run -e esp32dev_ota -t upload
   ```
   `esp32dev_ota` (in `platformio.ini`) is identical to `esp32dev` except
   `upload_protocol = espota` targeting the board's fixed AP address
   (`192.168.4.1`) — no mDNS discovery needed since that address never
   changes on this dedicated AP.

This is scoped to the testing phase and to `MODE_SENSOR_TELEMETRY` only —
there's no auth on the OTA endpoint beyond the AP's WiFi password, which is
fine for a private point-to-point link but shouldn't be treated as
hardened. If you need OTA in another `MODE_*` block, wire in `ota_init()`/
`ota_handle()`/`telemetry_init()` the same way that block does.

## Testing

Two different kinds of test, because most of this codebase touches real
hardware and most of it doesn't:

- **`test/`** — host-native unit tests (PlatformIO + Unity) for the
  modules with zero Arduino/hardware dependency: `maze.cpp` (grid logic,
  flood fill, Dijkstra planner), `pid.cpp` (generic controller math), and
  `filter.cpp` (despike + EMA smoothing). Run them with:
  ```
  pio test -e native
  ```
  No board needed — these compile and run directly on your machine (see
  `[env:native]` in `platformio.ini`, which restricts that environment's
  `src/` build to just those three files via `build_src_filter`). Every
  other module (`motor`, `sensor`, `drive`, `battery`, `encoder`,
  `mpu9250`, `control`, `solver`, `tasks`, ...) calls into `Arduino.h`/real
  peripherals somewhere in its call chain, so it can't be exercised this
  way without a hardware mock — not worth building for this project.

- **`main.cpp`'s `MODE_*` builds** — the on-device equivalent for
  everything hardware-bound: each `MODE_*` block (see the top of
  `main.cpp`) brings up exactly the module(s) it's testing and prints
  what it reads/does over serial, so you flash it, watch the serial
  monitor (or move the robot), and eyeball whether it's behaving —
  `MODE_MOTORS_ONLY` for per-motor direction checks, `MODE_SENSOR_TELEMETRY`
  for ToF readings, `MODE_ENCODER_CALIBRATION` for tick counting, and so
  on. To add a new one for another hardware component, follow the same
  shape as the existing blocks:
  ```cpp
  // 1. Add a #define near the top of main.cpp, in the MODE_* list and
  //    the MODE_COUNT macro (both already list every existing mode):
  // #define MODE_MY_COMPONENT_TEST   // one-line description of what it checks

  // 2. Add the block itself (anywhere among the other #if defined(MODE_...)
  //    blocks - order doesn't matter, only one is ever compiled in):
  #if defined(MODE_MY_COMPONENT_TEST)
  // What this checks and how to read the output - e.g. "logs raw ADC
  // counts each second; confirm they track battery voltage as expected."

  void setup()
  {
      Serial.begin(115200);
      delay(1000);

      Serial.println("[MAIN] booting (my component test)...");
      my_component_init();   // whichever module(s) this is exercising
  }

  void loop()
  {
      // Read/exercise the component and print something you can verify
      // by eye - a raw reading, a derived value, a pass/fail check.
      Serial.printf("[MAIN] reading = %d\n", my_component_read());
      delay(200);
  }
  #endif // MODE_MY_COMPONENT_TEST
  ```
  Then flip the `#define` at the top of the file to your new mode
  (commenting out whichever was active) and reflash — the `MODE_COUNT`
  `#error` check will refuse to build if that edit leaves zero or more
  than one mode active, so a stray uncomment can't silently ship the
  wrong build.

## Tools

- **`tools/maze_cli`** — a host-buildable CLI that runs the actual
  `maze.cpp` algorithm (flood fill + Dijkstra) against a maze, so it can
  be validated/visualized without hardware or the `mms` simulator's GUI.
  See its README for the build step and I/O protocol.
