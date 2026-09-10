#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#include "encoder.h"

// Concrete PID-driven control loops built on pid.h + drive.h/motor.h +
// encoder.h/sensor.h/mpu9250.h. Each loop's gains are tunable at runtime
// (see comms.h for the serial commands that call into this module) so
// they can be iterated on without reflashing - that's the whole point of
// the tools/dashboard companion app.
//
// Wheel diameter - TUNE by measuring the actual wheel.
#define WHEEL_DIAMETER_MM 32.0f

// Ticks per one full wheel/output-shaft revolution, derived from the
// encoder's motor-shaft spec and the gearbox ratio (see encoder.h) -
// only as accurate as ENCODER_GEARBOX_RATIO is set there.
#define ENCODER_TICKS_PER_REV (ENCODER_TICKS_PER_MOTOR_REV * ENCODER_GEARBOX_RATIO)

// Hard safety ceiling on any single RUN maneuver, regardless of whether
// it reaches its target - guards against a bad gain set driving forever.
#define CONTROL_MAX_RUN_MS 5000

typedef enum {
    CONTROL_LOOP_STRAIGHT = 0,  // dual-wheel encoder speed sync while driving forward
    CONTROL_LOOP_TURN,          // gyro-integrated heading hold while pivoting
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
// RUN maneuver, so the caller (comms.cpp) can send a telemetry line and
// check for an abort command without control.cpp knowing anything about
// serial/comms itself.
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
// one maneuver at a time while the dashboard watches it happen live.
void control_run_straight(float target_mm, int16_t base_speed, control_tick_cb_t tick_cb);
void control_run_turn(float target_deg, int16_t base_speed, control_tick_cb_t tick_cb);
void control_run_wallcenter(uint32_t duration_ms, int16_t base_speed, control_tick_cb_t tick_cb);

// Most recent debug snapshot (setpoint/measurement/output) of whichever
// loop last ran, for telemetry - valid even after the maneuver ends
// (active becomes false, the rest holds the last values).
control_debug_t control_get_debug(void);

// Signals an in-progress control_run_*() to stop on its next iteration.
void control_request_abort(void);

#endif // CONTROL_H
