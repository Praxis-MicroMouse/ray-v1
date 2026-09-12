#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#include "motor.h"

// Concrete PID-driven control loops built on pid.h + drive.h/motor.h +
// encoder.h/sensor.h. Each loop's gains are tunable at runtime via
// control_set_gains() so they can be iterated on without reflashing.
//
// Wheel/encoder geometry.
// Both measured directly: ENCODER_TICKS_PER_REV via encoder calibration
// mode method 1 (hand-turn a wheel N full revs, read the tick delta) -
// averaged 715 ticks/rev across trials. WHEEL_DIAMETER_MM via calipers
// over the tire - 3.2cm.
#define WHEEL_DIAMETER_MM     32.0f
#define ENCODER_TICKS_PER_REV 715.0f

// Hard safety ceiling on any single RUN maneuver, regardless of whether
// it reaches its target - guards against a bad gain set driving forever.
#define CONTROL_MAX_RUN_MS 5000

// Longer ceiling for control_run_straight_centered() below, which is
// meant to cover multi-meter runs (a straight-line PID's short 5s
// ceiling would cut a long run off before it reaches target_mm).
#define CONTROL_LONG_RUN_MS 15000

// Longer ceiling for control_run_spin() below, which is meant to be held
// steady long enough to time output-shaft rotations by hand.
#define CONTROL_SPIN_MAX_RUN_MS 60000

typedef enum {
    CONTROL_LOOP_STRAIGHT = 0,  // dual-wheel encoder speed sync while driving forward
    CONTROL_LOOP_TURN,          // dual-wheel encoder arc-length sync while pivoting
    CONTROL_LOOP_WALLCENTER,    // ToF left/right centering while driving forward
    CONTROL_LOOP_COUNT
} control_loop_id_t;

typedef struct {
    control_loop_id_t loop;
    float setpoint;
    float measurement;
    float output;
    bool  active;
} control_debug_t;

// Called once per control-loop iteration (roughly every 10ms) during a
// maneuver, so a caller can observe progress (e.g. stream telemetry) and
// decide whether to call control_request_abort() - pass nullptr if
// nothing needs to happen mid-maneuver.
typedef void (*control_tick_cb_t)(void);

void control_init(void);

void control_set_gains(control_loop_id_t loop, float kp, float ki, float kd);
void control_get_gains(control_loop_id_t loop, float *kp, float *ki, float *kd);

// Converts a raw encoder tick count to millimeters using the geometry
// constants above.
float control_ticks_to_mm(int32_t ticks);

// Runs a maneuver to completion, to CONTROL_MAX_RUN_MS, or until
// control_request_abort() is called (from within tick_cb, typically).
// Blocking, by design - the point is to drive the robot through exactly
// one maneuver at a time.
void control_run_straight(float target_mm, int16_t base_speed, control_tick_cb_t tick_cb);

// Pivots by an arbitrary signed angle (degrees) - positive turns right/
// clockwise, negative turns left/counterclockwise - using the same
// dual-wheel encoder sync idea as control_run_straight(): both wheels
// are driven at base_speed in opposite directions and the PID trims the
// difference between their traveled arc lengths so they stay matched
// until the target arc length (derived from TURNS_WHEEL_TRACK_MM) is
// reached.
void control_run_turn(float target_deg, int16_t base_speed, control_tick_cb_t tick_cb);
void control_run_wallcenter(uint32_t duration_ms, int16_t base_speed, control_tick_cb_t tick_cb);

// Drives forward target_mm, using the CONTROL_LOOP_WALLCENTER PID to
// steer off the ToF left/right wall distances (same centering error as
// control_run_wallcenter()) instead of control_run_straight()'s dual-
// wheel encoder sync - so it tracks the corridor's centerline rather
// than just holding the two wheels at matched speeds. Encoder ticks
// still measure progress toward target_mm (ToF sensing alone can't tell
// distance traveled). Bounded by CONTROL_LONG_RUN_MS rather than the
// shorter CONTROL_MAX_RUN_MS, for multi-meter runs.
void control_run_straight_centered(float target_mm, int16_t base_speed, control_tick_cb_t tick_cb);

// Open-loop "just spin this motor and hold" test - no PID, no target,
// not tied to a CONTROL_LOOP_* id. Meant for benchtop comparisons: e.g.
// timing output-shaft rotations by hand (mark the wheel, stopwatch) to
// check that two candidate motors have matching gearbox ratios - two
// motors can spin their bare motor shaft at the same rate and still gear
// down differently, so that alone can't answer it. Runs at a constant
// `pwm` (signed, -255..255) until control_request_abort() or
// CONTROL_SPIN_MAX_RUN_MS, whichever comes first.
void control_run_spin(motor_id_t motor, int16_t pwm, control_tick_cb_t tick_cb);

// Most recent debug snapshot (setpoint/measurement/output) of whichever
// loop last ran, for telemetry - valid even after the maneuver ends
// (active becomes false, the rest holds the last values).
control_debug_t control_get_debug(void);

// Signals an in-progress control_run_*() to stop on its next iteration.
void control_request_abort(void);

#endif // CONTROL_H
