# MicroMouse Dashboard

Local web app for PID tuning and maze-algorithm validation while the
robot is on the bench. Two independent halves that happen to share one
page:

1. **Live telemetry + PID tuning** — connects to the robot over serial
   (using the line protocol in `../../include/comms.h`), graphs ToF/
   battery/encoder/RPM/motor-PWM/IMU data live, and lets you read/set PID
   gains and trigger test maneuvers (`RUN straight/turn/wallcenter`) one
   control loop at a time, watching setpoint-vs-measured live while it
   runs.
2. **Maze algorithm validator** — runs the *actual* robot algorithm
   (`../../src/maze.cpp`, via `../maze_cli`) against a maze on your
   machine, so you can watch the exploration + speed-run path without
   touching hardware or scripting the `mms` simulator's GUI. A separate
   button just launches the real `mms` AppImage for visual/manual
   reference alongside it.

## Setup

Needs Flask + pyserial. If `python3` on your `PATH` is a PlatformIO
virtualenv without them (common on a dev machine that also has
PlatformIO installed), use the system interpreter explicitly:

```
/usr/bin/python3 -m pip install -r requirements.txt   # if not already present
```

Build the maze algorithm CLI once (needed for the maze validator half):

```
../maze_cli/build.sh
```

## Run

```
/usr/bin/python3 server.py
```

Then open **http://127.0.0.1:5055**.

## Using it

- **Connect**: pick the robot's serial port (usually `/dev/ttyUSB0`) and
  hit Connect. The firmware needs to be flashed with the PID-tuning
  dashboard build — that's `src/main.cpp`'s default build as of this
  writing (bringing up `comms.h`/`control.h`). Telemetry starts flowing
  immediately; the sensor/battery/encoder/RPM/PWM/IMU panels update live.
  The RPM panel shows motor-shaft speed straight from the encoder pulse
  rate, and output-shaft speed divided by `ENCODER_GEARBOX_RATIO` in
  `include/encoder.h` (a 1.0 placeholder until the real GA12-N20 ratio is
  set there — set it, then output RPM is trustworthy). The Motor PWM
  panel shows each wheel's actual last-commanded signed speed (-255..255,
  from `motor_get_speed()`) — useful for sanity-checking that a PID
  loop's output is doing something sane before trusting its
  setpoint-vs-measured numbers.
- **Motor Spin Test**: holds one motor at a fixed PWM, open-loop, with no
  PID and no target (up to 60s, or until Stop) — for comparing candidate
  motors on the bench. Since the RPM panel measures the motor shaft
  *before* the gearbox, two motors can show identical motor RPM there and
  still have different gearbox ratios; to actually confirm two motors'
  **output**-shaft speed matches (and so their gearbox ratios match, for
  same-model motors), mark the wheel/output shaft on each candidate, spin
  it here at a fixed PWM, and time output rotations with a stopwatch —
  compare that timing across candidates directly, independent of what the
  RPM panel or `ENCODER_GEARBOX_RATIO` say.
- **PID tuning**: pick a loop tab (straight/turn/wallcenter), read the
  firmware's current gains ("Read from firmware"), adjust Kp/Ki/Kd and
  "Set gains", then "Run" a test maneuver with an argument (straight =
  target mm, turn = target degrees, wallcenter = duration ms). The chart
  plots setpoint/measured/output live while it runs; "Stop" aborts
  immediately if something looks wrong (there's also a firmware-side
  5-second hard timeout per maneuver regardless).
  - **`straight` and its distance/speed telemetry need the wheel
    encoders physically wired** — `include/encoder.h`'s pins are unset
    until then, so ticks stay at 0. `turn` (gyro) and `wallcenter` (ToF)
    don't depend on encoders.
  - `WHEEL_DIAMETER_MM`/`ENCODER_TICKS_PER_REV` in `include/control.h`
    are geometry guesses — measure and correct them once encoders are on,
    or the `straight` loop's mm numbers (and hence its tuning) won't mean
    what they say.
- **Maze validator**: "Run Search" generates a random solvable maze and
  replays the robot's real exploration algorithm against it — the
  discovered wall map is kept in the page's memory (that's the "stored in
  memory" the maze-solving competition flow needs). "Run Speed Run" then
  plans the fewest-turns path (Dijkstra, `maze_plan_min_turn_path()`) over
  that discovered map and animates it separately. Both routes call the
  same compiled `maze.cpp` the robot runs — this validates the actual
  algorithm, not a reimplementation of it. "Launch mms Simulator" just
  opens the real AppImage as a separate window for visual/manual
  reference; it isn't wired to the browser's maze view (automating the
  Qt GUI itself was out of scope given the timeline — see the note in the
  top-level README).

## Files

- `server.py` — Flask backend: serial bridge (background reader thread +
  SSE fan-out to the page), `/api/maze/run` (shells out to `../maze_cli`),
  `/api/mms/launch` (spawns the AppImage).
- `static/index.html` — the entire frontend (HTML/CSS/JS, no external
  libraries or CDN — deliberately offline-capable, including the line
  charts, which are hand-rolled `<canvas>` drawing).
- `requirements.txt` — `flask`, `pyserial`.
