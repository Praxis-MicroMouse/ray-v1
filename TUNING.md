# Tuning Guide

This is the step-by-step procedure for taking the mouse from "compiles
and boots" to "reliably solves a maze." Do the steps in order — each one
assumes the previous ones are already reasonably good, and skipping
ahead just means chasing a symptom whose real cause is three steps back.

Every constant mentioned here lives in `include/config/` — see
`README.md`'s config table if you need to find one. Every bench routine
mentioned here is a build-time `RUN_MODE_*` in `src/main.cpp`.

**Before you start:** put the mouse somewhere it can move freely —
several open floor cells at minimum, more for the later steps — and know
where the power switch is. Nothing here is destructive to the hardware,
but a badly-tuned controller can genuinely make the mouse dart or spin
unexpectedly the first time a new gain is loaded.

---

## How to run a mode

1. Open `src/main.cpp` and find this line near the top:
   ```cpp
   #define RUN_MODE RUN_MODE_MAZE
   ```
2. Change `RUN_MODE_MAZE` to whichever mode the step you're on calls for
   (see the table below), save.
3. Build and flash over USB, then open the serial monitor (115200 baud):
   ```
   pio run -e esp32dev -t upload
   pio device monitor
   ```
   (or one combined step: `pio run -e esp32dev -t upload -t monitor`).
   Over the OTA link instead of USB (see the "Wireless testing" section
   in `README.md` if that's not set up yet):
   ```
   pio run -e esp32dev_ota -t upload
   ```
4. Read the output. Power-cycle or hit reset to re-run the same mode
   from the start (most bench modes run once in `setup()` and idle in
   `loop()` — re-running means resetting the board, not waiting).
5. **When you're done tuning for the day, set `RUN_MODE` back to
   `RUN_MODE_MAZE` and reflash** before leaving the mouse alone or
   committing — every mode except `RUN_MODE_MAZE` and `RUN_MODE_BRINGUP`
   drives the motors as soon as `setup()` runs, with no confirmation
   prompt.

| `RUN_MODE_*` | Drives motors? | Used in | What it does |
|---|---|---|---|
| `MAZE` | Yes | (the real thing) | Full search + speed run |
| `BRINGUP` | No | Step 1 | Streams ToF/encoder/battery readings, motors idle |
| `FEEDFORWARD_LEFT` | Yes (left wheel only) | Step 4 | Open-loop volts-vs-speed sweep, left wheel |
| `FEEDFORWARD_RIGHT` | Yes (right wheel only) | Step 4 | Open-loop volts-vs-speed sweep, right wheel |
| `STRAIGHT_TEST` | Yes | Steps 3 & 5 | 4x (720mm forward/back), reports commanded vs. measured distance |
| `TURN_TEST` | Yes | Steps 3 & 6 | 4x 360° in-place spin, reports commanded vs. measured angle |
| `WALLCENTER_TEST` | Yes | Step 7 | Drives a corridor with steering engaged, reports cross-track error |

`FEEDFORWARD_LEFT`/`_RIGHT` are the two exceptions to step 3 above: they
run **standalone**, without `tasks_start()` (see the comment in
`main.cpp`) — nothing else is fighting for the motor output, which is
the point, but it also means `button_pressed()`-based abort doesn't work
in this mode. Know where the power switch is before running either one.

---

## Step 1 — Bring-up check

**Mode:** `RUN_MODE_BRINGUP`

Motors stay off. This just confirms every sensor is alive before you
trust it to drive on their output.

1. Flash and open the serial monitor. You should see a `[SENSOR] ... init
   OK` line for FRONT, RIGHT, and LEFT. If any says "not responding" or
   "init FAILED", fix the wiring/address conflict before going further —
   nothing downstream will work with a sensor that isn't there.
2. Watch the `[BRINGUP]` lines. Wave a hand in front of each ToF sensor
   in turn and confirm the matching `front=`/`right=`/`left=` number
   drops as your hand gets closer, and the `(1)`/`(0)` in-range flag
   goes to 0 past ~2m (`TOF_MAX_RANGE_MM`).
3. Spin each wheel by hand and confirm `encL=`/`encR=` counts up when
   spun forward. If a wheel counts *down* while turning forward, flip
   the matching sign in `config/robot_physical.h`
   (`ENCODER_LEFT_POLARITY`/`ENCODER_RIGHT_POLARITY`) rather than
   re-wiring.
4. Confirm `batt=` reads a sane voltage for whatever's plugged in (a 1S
   LiPo should read 3.3–4.2V).

Don't move on until all of this looks right — every later step builds a
number on top of these readings.

---

## Step 2 — Wheel/encoder geometry

Measure, don't guess — these two constants scale every distance the
firmware ever computes.

1. Mark a wheel, hand-turn it through exactly 10 full revolutions, read
   the tick delta off the bring-up mode's `encL`/`encR` (reset it first
   by power-cycling, or just note the start and end values). Divide by
   10. Do this for **each wheel separately** — set
   `ENCODER_TICKS_PER_REV_LEFT`/`_RIGHT` in `config/robot_physical.h`
   independently; it's normal for them to differ slightly.
2. Measure the wheel diameter with calipers (over the tire, not the
   hub) → `WHEEL_DIAMETER_MM`.

`MM_PER_TICK_LEFT`/`_RIGHT` are derived from these automatically — don't
edit them directly.

---

## Step 3 — Turn radius

**Mode:** `RUN_MODE_TURN_TEST`

This calibrates `TURN_RADIUS_MM` (`config/robot_physical.h`), the single
constant odometry uses to convert wheel-speed difference into a heading
change estimate. Get this right before Step 6 (rotation PD) — it's the
"how far is the target" input that controller is tracking.

1. Flash `RUN_MODE_TURN_TEST`. It's a top-level function run through
   `tasks_start()`, so the full control/sensor pipeline is live — the
   motors WILL spin.
2. It commands four back-to-back 360° in-place spins and prints one line
   per spin: `commanded_deg,measured_deg,error_deg` (measured comes from
   odometry, not a stopwatch/protractor).
3. If `error_deg` is consistently **negative** (measured < commanded —
   the mouse is under-rotating for the ticks it counted), **decrease**
   `TURN_RADIUS_MM`. If consistently **positive**, **increase** it.
4. Adjust and re-flash until `error_deg` is small and doesn't keep
   growing across the four repeats (a small *constant* per-spin error is
   fine and gets corrected by Step 6; a per-spin error that keeps
   compounding means the radius is still off).

This bench routine is more accurate than eyeballing a hand-rotated 360°
— trust the numbers here over an earlier hand-tuned guess.

---

## Step 4 — Feedforward characterization

**Mode:** `RUN_MODE_FEEDFORWARD_LEFT`, then `RUN_MODE_FEEDFORWARD_RIGHT`

**Do this with the wheels able to spin freely and the mouse able to
drive in a straight line for a couple of meters — it runs open-loop at
increasing speed.** This is the step every later PD gain assumes is
already done reasonably well: feedforward is what gets the motor most of
the way to the right voltage before the PD term has to correct anything,
which is what makes the PD gains in Steps 5–7 tunable at all against
motor stiction.

1. Flash `RUN_MODE_FEEDFORWARD_LEFT`. This is **standalone** — it does
   NOT go through `tasks_start()`, so nothing is fighting it for the
   motor output.
2. It steps the left motor through 0.5V, 1.0V, 1.5V, ... up to
   `MOTOR_MAX_VOLTS`, holding each for a moment then measuring
   steady-state speed from the encoder. Output is `volts,speed_mm_s`.
3. Copy the numbers into a spreadsheet (or plot them) — speed should be
   roughly linear in voltage once past the point where the motor
   actually starts turning. Fit a line:
   - The **slope** is `1/FF_LEFT_KV` (units: mm/s per Volt) →
     `FF_LEFT_KV = 1 / slope`.
   - The **x-intercept** (voltage where the line would cross speed=0 —
     i.e. the minimum voltage before the wheel overcomes stiction and
     starts moving at all) is `FF_LEFT_BIAS`.
4. Repeat with `RUN_MODE_FEEDFORWARD_RIGHT` for `FF_RIGHT_KV`/`_BIAS`.
   It's normal and expected for left/right to come out a bit different.
5. Set all four in `config/motion_tuning.h`. Leave `FF_*_KA` (the
   acceleration feedforward term) at 0 for now — Steps 5–6 will tell you
   if you need it.

**Sanity check before moving on:** with feedforward alone (PD gains can
still be placeholders), the mouse should already drive *approximately*
straight and turn *approximately* the right amount — the PD terms exist
to correct the remaining error, not to do all the work.

---

## Step 5 — Forward PD

**Mode:** `RUN_MODE_STRAIGHT_TEST`

Tunes `PD_FORWARD_KP`/`PD_FORWARD_KD` — how tightly odometry-measured
distance tracks the forward motion profile's target.

1. Flash `RUN_MODE_STRAIGHT_TEST`. It drives 720mm forward, then 720mm
   back, four times, printing `commanded_mm,measured_mm,error_mm` after
   each leg.
2. Starting point: `PD_FORWARD_KP` small, `PD_FORWARD_KD = 0`. Increase
   `PD_FORWARD_KP` until you see the mouse hunt/oscillate audibly or
   visibly (a faint buzzing/jerkiness while driving, not just motor
   whine) — then back off to about 60–70% of that value.
3. Add `PD_FORWARD_KD` a little at a time to damp out any residual
   overshoot at the start/end of each leg (watch/listen for a
   "lurch-then-settle" at the start of a move — KD smooths that out).
   Too much KD shows up as sluggish response to the initial speed
   command.
4. You're done when `error_mm` is small (single-digit mm) and doesn't
   grow between the four repeats.

If `error_mm` has a consistent bias (e.g. always +5mm regardless of
direction) rather than noise, suspect Step 2's tick calibration before
chasing it with more KP.

---

## Step 6 — Rotation PD

**Mode:** `RUN_MODE_TURN_TEST`

Same mode as Step 3, now tuning `PD_ROTATION_KP`/`PD_ROTATION_KD` instead
of `TURN_RADIUS_MM` — do this only after Step 3's radius is settled.

1. Same procedure as Step 5: raise `PD_ROTATION_KP` until you see/hear
   oscillation at the end of each spin (the mouse wobbling around the
   target heading instead of settling cleanly), back off ~30%, then add
   `PD_ROTATION_KD` to damp the settle.
2. `error_deg` per spin should be small and non-growing across the four
   repeats, same acceptance criterion as Step 3 — you're now tightening
   the same measurement, not re-deriving it.

---

## Step 7 — Steering PD (wall centering)

**Mode:** `RUN_MODE_WALLCENTER_TEST`

**Needs a straight corridor with walls on both sides, at least ~900mm
long** (five maze cells) — build one out of spare maze walls if you
don't have a straight run in an actual maze yet.

1. Flash and place the mouse centered in the corridor, facing down its
   length.
2. It drives the length with continuous wall-centering engaged, logging
   `pos_mm,left_mm,right_mm,steer_adjust_deg_s` a few times a second.
3. Watch `left_mm`/`right_mm` converge toward each other and stay close
   as it drives. If the mouse visibly drifts toward one wall and the
   numbers diverge, increase `PD_STEERING_KP`. If it oscillates
   side-to-side (you'll see `steer_adjust_deg_s` swinging between large
   positive and negative values), back `PD_STEERING_KP` off and/or add
   `PD_STEERING_KD`.
4. `STEERING_ADJUST_LIMIT_DEG_S` caps how hard steering can pull —
   raise it only if you've confirmed the mouse needs a harder correction
   than the limit allows (visibly clipping at the limit rather than
   settling), not as a first response to drift.

If the mouse tracks one wall fine but not the other, that's usually a
`TOF_*_MOUNT_OFFSET_MM`/`TOF_*_SCALE`/`TOF_*_OFFSET_MM` mismatch between
the left and right sensors (`config/sensor_calibration.h`) rather than a
steering-gain problem — check those first.

---

## Step 8 — Sensor calibration (if needed)

Most VL53L0X units are accurate out of the box and this step can be
skipped — only come back to it if Step 7 showed a persistent left/right
mismatch, or if wall detection in Step 9/10 seems to trigger at visibly
different distances than expected.

1. Place the mouse with a given sensor exactly `NEAR_MM` from a flat
   wall (a ruler or spacer block), read the raw mm off
   `RUN_MODE_BRINGUP`. Repeat at `FAR_MM`.
2. `SCALE = (FAR_MM - NEAR_MM) / (raw_far - raw_near)`,
   `OFFSET_MM = NEAR_MM - SCALE * raw_near` — set both in
   `config/sensor_calibration.h` for that sensor.
3. `TOF_FRONT_REFERENCE_MM` (used by `mouse.cpp`'s `adjust_position()`):
   center the mouse by eye against a front wall and read the corrected
   front distance — that's the value.

---

## Step 9 — Turn geometry (search turns)

`config/turn_params.h`'s `entry_offset_mm`/`exit_offset_mm`/`trigger_mm`
per turn type are ported as untuned starting guesses. Tune with a
two-cell bench setup (straight, turn, straight) once Steps 1–7 are
solid:

1. Set `RUN_MODE` to drive a manual left/right turn sequence (or extend
   `bench.cpp` with a `bench_turn_smooth_test()` calling `mouse.cpp`'s
   internal turn logic directly, if you want a dedicated mode — it isn't
   one yet) and watch how the mouse tracks the wall on exit.
2. If it clips the inside wall or swings wide on the outside wall on
   exit, adjust `entry_offset_mm` (moves where the pivot starts relative
   to the cell boundary) and `exit_offset_mm` (how far it drives out
   after pivoting before considering itself aligned).
3. If a wall appears closer than the turn's geometry assumes (a short
   cell, or the mouse entered off-center), `trigger_mm` is what starts
   the turn early instead of running into it — raise it if turns are
   still clipping a close wall, lower it if turns are starting
   noticeably before the wall a normal-distance cell would have.

`TURN_TRIM_DEG_LEFT`/`_RIGHT` (same file) is a separate small-angle trim
specifically for in-place spin turns (`turn_to_face()`,
`mouse_turn_back()`) — use it if Step 6 already tracks 360° spins well
but a sequence of quarter-turns still drifts off the original heading
over a run.

---

## Step 10 — First full run

Set `RUN_MODE_MAZE`, place the mouse at the start of an actual maze, and
run it. Watch the `[MOUSE]` serial log — it prints every action taken
and any `PANIC`/`ERR` line. A few things to expect the first time:

- Early runs will be conservative and a bit rough at cell transitions —
  that's Step 9's turn geometry still being approximate. Don't chase
  turn geometry further until a full run at least *completes*.
- If it panics with "no route to target," that's the maze wall-mapping
  (`sensor_get_walls()`'s `TOF_WALL_THRESHOLD_*_MM`) misreading a wall
  that isn't there, or missing one that is — revisit Step 8.
- If `[TASKS]` never prints `sensors_ok=1`, go back to Step 1.

## Step 11 — Speed run

Once a full run completes reliably at `SEARCH_SPEED_MM_S`/
`SEARCH_ACCEL_MM_S2`, raise `FAST_RUN_SPEED_MM_S`/`FAST_RUN_ACCEL_MM_S2`
(used only for the post-exploration speed run) incrementally — a small
step at a time, re-running the maze between each — rather than jumping
straight to a target number. Every PD/FF stage above was tuned at search
speed; a large speed increase can expose gain limits that didn't show up
at the slower pace, especially in the rotation controller during turns.

---

## Quick troubleshooting

| Symptom | Likely cause | Where to look |
|---|---|---|
| Wheel spins the wrong way for a commanded forward move | Motor polarity | `MOTOR_LEFT_POLARITY`/`_RIGHT_POLARITY`, `robot_physical.h` |
| Encoder counts down while wheel spins forward | Encoder polarity | `ENCODER_LEFT_POLARITY`/`_RIGHT_POLARITY`, `robot_physical.h` |
| Mouse turns the wrong physical direction for a "left" command | Sign convention mismatch | Re-check Step 3's `error_deg` sign against `robot_physical.h`'s polarity, not `mouse.cpp`'s angle signs |
| Buzzing/audible oscillation while driving straight or turning | A PD gain (usually KP) too high | Back off KP ~30%, re-add KD gradually |
| Mouse drifts to one wall in a corridor | Steering gain too low, or a sensor calibration mismatch between sides | Step 7, then Step 8 |
| Speed/behavior changes noticeably as the battery discharges | Should not happen — `motor.cpp`'s battery-voltage compensation exists specifically to prevent this | Confirm `battery_update()` is actually running (check `[BATTERY]` log lines appear) |
| A single ToF reading occasionally jumps wildly | Normal — `filter.cpp` despikes this already | Only worth chasing if it happens on *every* read, not occasionally |
