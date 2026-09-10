# Ray v1 — MicroMouse Firmware

Firmware for our MicroMouse robot, built with [PlatformIO](https://platformio.org/) on an ESP32 (Arduino framework).

## Purpose

The robot needs to sense walls on three sides (front, left, right) as it navigates
the maze. This code brings up three VL53L0X time-of-flight (ToF) distance sensors
sharing a single I2C bus and reads distance measurements from each, with debug
logging over serial so behavior can be verified during bring-up and testing.

The codebase is written to be modular: each piece of hardware/functionality gets
its own header + implementation pair, and `main` just wires the modules together.
This keeps `main` small and makes it straightforward to add new modules (motors,
encoders, IMU, maze-solving logic, etc.) without touching existing ones.

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
- Quadrature wheel encoders — **not wired up yet**. `encoder.h` leaves all
  four pins unset (`-1`) until they're mounted/tuned; suggested pins once
  they are:
  - Left: A = GPIO 4, B = GPIO 16
  - Right: A = GPIO 17, B = GPIO 23
- MPU-9250 9-DoF IMU (accel/gyro/magnetometer), on the same I2C bus as the
  ToF sensors (SDA = GPIO 21, SCL = GPIO 22), address 0x68 (AD0 tied low).
  Suggested INT pin: GPIO 13 (not currently used — the driver polls
  instead of using the interrupt).

## Code structure

```
include/
  sensor.h       # public C-style API for the ToF sensor module
  telemetry.h    # public C-style API for the serial telemetry module
  motor.h        # public C-style API for the motor driver module
  drive.h        # public C-style API for the simple movement module
  battery.h      # public C-style API for the battery voltage module
  encoder.h      # public C-style API for the wheel encoder module
  mpu9250.h      # public C-style API for the IMU module
  maze.h         # public C-style API for the maze grid + flood-fill search
  solver.h       # public C-style API for the physical maze-solving run
src/
  sensor.cpp     # ToF sensor implementation (I2C/XSHUT bring-up, reads, logging)
  telemetry.cpp  # streams sensor readings over serial for host tools (e.g. MATLAB)
  motor.cpp      # motor driver implementation (direction pins + LEDC PWM)
  drive.cpp      # simple forward/turn movement built on the motor module
  battery.cpp    # battery voltage divider reading over ADC
  encoder.cpp    # quadrature encoder tick counting via pin-change interrupts
  mpu9250.cpp    # MPU9250 accel/gyro/mag driver over raw I2C register access
  maze.cpp       # maze grid state + flood-fill search (no hardware calls)
  solver.cpp     # drives the real robot through a maze.cpp search using sensor.h/drive.h
  main.cpp       # setup()/loop() — initializes and calls the modules
platformio.ini   # board/framework config + library dependencies
matlab/
  live_tof_plot.m  # live-plots the streamed sensor readings, for tuning
```

- **`sensor.h`** declares the sensor module's public interface: pin/address
  constants, the `sensor_reading_t` struct, and `sensor_init()` /
  `sensor_read_all()`. Any other module only needs to include this header.
- **`sensor.cpp`** contains the implementation: it resets all three sensors via
  XSHUT, brings them up one at a time so each can be assigned a unique I2C
  address (they'd otherwise collide on the shared bus), and reads distances via
  the Adafruit VL53L0X library in its `HIGH_ACCURACY` profile (longer timing
  budget, lower noise — matters most at the short 0-10cm ranges used for
  tuning). Every step logs to serial (`[SENSOR] ...`) for debugging.
- **`telemetry.h`/`telemetry.cpp`** print one sensor reading per call as a
  machine-parseable serial line (`DATA,<millis>,<front_mm>,<right_mm>,<left_mm>`),
  kept separate from the `[SENSOR]` debug logs so a host tool can filter for
  `DATA,` lines and ignore the rest. Used by `matlab/live_tof_plot.m`.
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
  in place), and `drive_stop()`. Left/right-to-Motor-A/B and each wheel's
  polarity are guesses; if a wheel spins the wrong way, flip its sign in
  `drive.cpp` rather than rewiring.
- **`battery.h`/`battery.cpp`** read the ADC (8-sample average, `ADC_11db`
  attenuation for full 0-3.3V range), convert through the known divider
  ratio, and log the battery voltage (`[BATTERY] ...`) every call, with a
  warning line if it drops below `BATTERY_LOW_VOLTAGE` (3.3V).
- **`encoder.h`/`encoder.cpp`** decode each wheel's quadrature encoder via a
  `CHANGE` interrupt on its channel-A pin (reading channel B at that instant
  to get direction), and expose a running signed tick count per wheel via
  `encoder_get_ticks()` / `encoder_reset()`. The four pins
  (`ENCODER_LEFT_A_PIN` etc.) are left at `-1` in `encoder.h` since the
  encoders aren't wired up yet — `encoder_init()` logs and skips any encoder
  whose pins are unset rather than touching undefined hardware. Suggested
  pins are documented in `encoder.h`; fill them in once mounted, then re-flash.
- **`mpu9250.h`/`mpu9250.cpp`** talk to the MPU9250 IMU directly over I2C
  register access (no external library) — accel + gyro from the MPU9250
  core, plus magnetometer from the embedded AK8963 chip (reached via I2C
  bypass mode once the MPU9250 is configured). `mpu9250_init()` verifies
  `WHO_AM_I` on both chips, sets accel/gyro full-scale range and low-pass
  filtering, and starts the magnetometer in continuous mode.
  `mpu9250_read()` returns accel (g), gyro (deg/s), mag (µT), and
  temperature (°C), logging every call (`[MPU9250] ...`). It shares the ToF
  sensors' I2C bus; suggested pins (INT, address) are documented in
  `mpu9250.h`.
- **`maze.h`/`maze.cpp`** hold the maze grid (walls-per-cell bitmask) and a
  multi-source BFS flood fill that computes each cell's shortest distance
  (in cell-steps) to a set of goal cells, plus `maze_choose_next_direction()`
  to pick the best open neighbor. This is pure grid logic ported from the
  [`mms-c`](../mms-c/Main.c) simulator reference algorithm — no sensor/motor
  calls — so the search itself doesn't depend on real hardware. Maze size
  defaults to a full 16x16 grid (`MAZE_WIDTH`/`MAZE_HEIGHT`); change those
  for a different contest maze size.
- **`solver.h`/`solver.cpp`** are the hardware glue: `solver_run()` runs the
  same search-to-center-then-return-to-start loop as `mms-c/Main.c`'s
  `main()`, but senses walls with `sensor_read_all()` (a wall is "there" if
  a ToF reading is under `SOLVER_WALL_THRESHOLD_MM`) and moves the real
  robot with `drive_forward()`/`drive_turn_left()`/`drive_turn_right()`
  instead of the simulator's `API_*` calls. Movement is timed/open-loop by
  default (`SOLVER_CELL_MOVE_TIME_MS`, `SOLVER_TURN_90_TIME_MS` — both
  untuned placeholders); once `encoder.h`'s pins are wired up and
  `SOLVER_CELL_TICKS` is set to a measured ticks-per-cell value, it switches
  to counting encoder ticks instead. Not wired into `main.cpp` by default —
  see the commented-out block at the top of `loop()` to enable it.
- **`main.cpp`** initializes serial, the motor, encoder, and IMU modules in
  `setup()`, then runs a **one-shot motion test on every boot**: forward,
  pivot left, pivot right, then stop — logged via `[MAIN]`/`[DRIVE]`, with
  encoder ticks printed after each move and one IMU reading per loop
  iteration (skipped if the MPU9250 didn't initialize). Speeds and
  durations are untuned placeholders. **The robot moves as soon as it's
  powered on** — place it in a clear area before flashing/resetting it.
  (The ToF sensor/battery/telemetry modules exist in the tree but aren't
  called from this build — see the git history for a version that wires
  them in.)

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

## Live-plotting sensor data in MATLAB

For fine-tuning sensor placement/mounting, `matlab/live_tof_plot.m` opens the
board's serial port, reads the `DATA,...` telemetry lines, and live-plots
front/right/left distance over time. The plot is zoomed to a 0-10cm axis with
0.5cm gridlines and per-sample markers, since that's the near-field range
that matters for placement tuning.

1. Flash and connect the board (`pio run -t upload`), then **close** any open
   `pio device monitor`/serial terminal — only one program can hold the serial
   port at a time.
2. Open `matlab/live_tof_plot.m` in MATLAB and set `PORT_NAME` at the top
   (run `serialportlist("available")` in MATLAB if you're not sure which port
   it is — on Linux it's usually `/dev/ttyUSB0` or `/dev/ttyACM0`).
3. Run the script. A plot window opens and updates live; close the window to
   stop.

Requires MATLAB R2019b or newer (uses the built-in `serialport` object — no
Instrument Control Toolbox needed).
