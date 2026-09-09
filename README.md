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

## Code structure

```
include/
  sensor.h       # public C-style API for the ToF sensor module
  telemetry.h    # public C-style API for the serial telemetry module
src/
  sensor.cpp     # ToF sensor implementation (I2C/XSHUT bring-up, reads, logging)
  telemetry.cpp  # streams sensor readings over serial for host tools (e.g. MATLAB)
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
  the Adafruit VL53L0X library. Every step logs to serial (`[SENSOR] ...`) for
  debugging.
- **`telemetry.h`/`telemetry.cpp`** print one sensor reading per call as a
  machine-parseable serial line (`DATA,<millis>,<front_mm>,<right_mm>,<left_mm>`),
  kept separate from the `[SENSOR]` debug logs so a host tool can filter for
  `DATA,` lines and ignore the rest. Used by `matlab/live_tof_plot.m`.
- **`main.cpp`** is intentionally minimal: it initializes serial + the sensor
  module in `setup()`, then each loop reads all three sensors and sends the
  reading out over telemetry.

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
front/right/left distance over time.

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
