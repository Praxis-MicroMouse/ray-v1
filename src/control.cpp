#include "control.h"

#include <Arduino.h>
#include <math.h>

#include "pid.h"
#include "motor.h"
#include "drive.h"
#include "encoder.h"
#include "sensor.h"
#include "turns.h" // TURNS_WHEEL_TRACK_MM - shared pivot-turn geometry

#define CONTROL_LOOP_HZ 100
#define CONTROL_DT_MS   (1000 / CONTROL_LOOP_HZ)

static pid_ctrl_t s_pid[CONTROL_LOOP_COUNT];
static control_debug_t s_debug;
static volatile bool s_abort = false;

void control_init(void) {
    // Starting gains are guesses, not measurements - tune all three via
    // control_set_gains(). Integral limits keep a stuck integrator from
    // railing the output while gains are still being found.
    pid_init(&s_pid[CONTROL_LOOP_STRAIGHT],   1.0f, 0.0f, 0.0f, 200.0f);
    pid_init(&s_pid[CONTROL_LOOP_TURN],       2.0f, 0.0f, 0.1f, 90.0f);
    pid_init(&s_pid[CONTROL_LOOP_WALLCENTER], 0.8f, 0.0f, 0.05f, 100.0f);

    s_debug.loop = CONTROL_LOOP_STRAIGHT;
    s_debug.setpoint = 0.0f;
    s_debug.measurement = 0.0f;
    s_debug.output = 0.0f;
    s_debug.active = false;
}

void control_set_gains(control_loop_id_t loop, float kp, float ki, float kd) {
    if (loop < 0 || loop >= CONTROL_LOOP_COUNT) return;
    pid_set_gains(&s_pid[loop], kp, ki, kd);
    pid_reset(&s_pid[loop]);
}

void control_get_gains(control_loop_id_t loop, float *kp, float *ki, float *kd) {
    if (loop < 0 || loop >= CONTROL_LOOP_COUNT) return;
    *kp = s_pid[loop].kp;
    *ki = s_pid[loop].ki;
    *kd = s_pid[loop].kd;
}

float control_ticks_to_mm(int32_t ticks) {
    return ticks * (PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV;
}

void control_request_abort(void) {
    s_abort = true;
}

control_debug_t control_get_debug(void) {
    return s_debug;
}

void control_run_straight(float target_mm, int16_t base_speed, control_tick_cb_t tick_cb) {
    s_abort = false;
    pid_reset(&s_pid[CONTROL_LOOP_STRAIGHT]);
    encoder_reset(ENCODER_LEFT);
    encoder_reset(ENCODER_RIGHT);

    uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    while (!s_abort) {
        uint32_t now = millis();
        if (now - start_ms > CONTROL_MAX_RUN_MS) break;
        if (now - last_ms < CONTROL_DT_MS) { delay(1); continue; }
        float dt_s = (now - last_ms) / 1000.0f;
        last_ms = now;

        int32_t left_ticks = encoder_get_ticks(ENCODER_LEFT);
        int32_t right_ticks = encoder_get_ticks(ENCODER_RIGHT);
        float left_mm = control_ticks_to_mm(left_ticks);
        float right_mm = control_ticks_to_mm(right_ticks);
        float avg_mm = (left_mm + right_mm) / 2.0f;

        // error = how far left has drifted ahead of right; trim each
        // side's speed in opposite directions to close the gap.
        float error = left_mm - right_mm;
        float correction = pid_update(&s_pid[CONTROL_LOOP_STRAIGHT], 0.0f, error, dt_s);

        int16_t left_out  = (int16_t) constrain(DRIVE_LEFT_SIGN  * (base_speed - correction), -255, 255);
        int16_t right_out = (int16_t) constrain(DRIVE_RIGHT_SIGN * (base_speed + correction), -255, 255);
        motor_set_speed(DRIVE_LEFT_MOTOR, left_out);
        motor_set_speed(DRIVE_RIGHT_MOTOR, right_out);

        s_debug.loop = CONTROL_LOOP_STRAIGHT;
        s_debug.setpoint = target_mm;
        s_debug.measurement = avg_mm;
        s_debug.output = correction;
        s_debug.active = true;

        if (tick_cb) tick_cb();

        if (avg_mm >= target_mm) break;
    }

    drive_stop();
    s_debug.active = false;
}

// Positive degrees = rightward/clockwise (viewed from above); if the
// mouse turns the wrong way, flip DRIVE_LEFT_SIGN/DRIVE_RIGHT_SIGN's
// roles below rather than negating every call site - same pragmatic
// approach as drive.h's DRIVE_*_SIGN.
void control_run_turn(float target_deg, int16_t base_speed, control_tick_cb_t tick_cb) {
    s_abort = false;
    pid_reset(&s_pid[CONTROL_LOOP_TURN]);
    encoder_reset(ENCODER_LEFT);
    encoder_reset(ENCODER_RIGHT);

    float target_arc_mm = fabsf(target_deg) * (float)M_PI / 180.0f * (TURNS_WHEEL_TRACK_MM / 2.0f);
    int16_t left_dir = (target_deg >= 0.0f) ? 1 : -1;
    int16_t right_dir = -left_dir;

    uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    while (!s_abort) {
        uint32_t now = millis();
        if (now - start_ms > CONTROL_MAX_RUN_MS) break;
        if (now - last_ms < CONTROL_DT_MS) { delay(1); continue; }
        float dt_s = (now - last_ms) / 1000.0f;
        last_ms = now;

        float left_mm = fabsf(control_ticks_to_mm(encoder_get_ticks(ENCODER_LEFT)));
        float right_mm = fabsf(control_ticks_to_mm(encoder_get_ticks(ENCODER_RIGHT)));
        float measured_arc_mm = (left_mm + right_mm) / 2.0f;

        // error = how far left has out-paced right in traveled arc length
        // (both wheels should cover equal arcs, just in opposite spin
        // directions, during a clean pivot); trim each side's speed in
        // opposite directions to close the gap - same dual-wheel sync
        // idea as control_run_straight(), applied to a pivot instead of
        // a straight line.
        float error = left_mm - right_mm;
        float correction = pid_update(&s_pid[CONTROL_LOOP_TURN], 0.0f, error, dt_s);

        int16_t left_out  = (int16_t) constrain(DRIVE_LEFT_SIGN  * left_dir  * (base_speed - correction), -255, 255);
        int16_t right_out = (int16_t) constrain(DRIVE_RIGHT_SIGN * right_dir * (base_speed + correction), -255, 255);
        motor_set_speed(DRIVE_LEFT_MOTOR, left_out);
        motor_set_speed(DRIVE_RIGHT_MOTOR, right_out);

        s_debug.loop = CONTROL_LOOP_TURN;
        s_debug.setpoint = target_arc_mm;
        s_debug.measurement = measured_arc_mm;
        s_debug.output = correction;
        s_debug.active = true;

        if (tick_cb) tick_cb();

        if (measured_arc_mm >= target_arc_mm) break;
    }

    drive_stop();
    s_debug.active = false;
}

void control_run_wallcenter(uint32_t duration_ms, int16_t base_speed, control_tick_cb_t tick_cb) {
    s_abort = false;
    pid_reset(&s_pid[CONTROL_LOOP_WALLCENTER]);

    uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    while (!s_abort) {
        uint32_t now = millis();
        if (now - start_ms >= duration_ms) break;
        if (now - start_ms > CONTROL_MAX_RUN_MS) break;
        if (now - last_ms < CONTROL_DT_MS) { delay(1); continue; }
        float dt_s = (now - last_ms) / 1000.0f;
        last_ms = now;

        sensor_reading_t reading;
        sensor_read_all(&reading);
        // positive = closer to the left wall than the right one -> steer right to re-center
        float error = (float) reading.right_mm - (float) reading.left_mm;
        float correction = pid_update(&s_pid[CONTROL_LOOP_WALLCENTER], 0.0f, error, dt_s);

        int16_t left_out  = (int16_t) constrain(DRIVE_LEFT_SIGN  * (base_speed + correction), -255, 255);
        int16_t right_out = (int16_t) constrain(DRIVE_RIGHT_SIGN * (base_speed - correction), -255, 255);
        motor_set_speed(DRIVE_LEFT_MOTOR, left_out);
        motor_set_speed(DRIVE_RIGHT_MOTOR, right_out);

        s_debug.loop = CONTROL_LOOP_WALLCENTER;
        s_debug.setpoint = 0.0f;
        s_debug.measurement = error;
        s_debug.output = correction;
        s_debug.active = true;

        if (tick_cb) tick_cb();
    }

    drive_stop();
    s_debug.active = false;
}

// Same ToF centering error/correction as control_run_wallcenter(), but
// gated on encoder-measured distance instead of a fixed duration - see
// control.h for why (multi-meter runs need the longer CONTROL_LONG_RUN_MS
// ceiling, not CONTROL_MAX_RUN_MS).
void control_run_straight_centered(float target_mm, int16_t base_speed, control_tick_cb_t tick_cb) {
    s_abort = false;
    pid_reset(&s_pid[CONTROL_LOOP_WALLCENTER]);
    encoder_reset(ENCODER_LEFT);
    encoder_reset(ENCODER_RIGHT);

    uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    while (!s_abort) {
        uint32_t now = millis();
        if (now - start_ms > CONTROL_LONG_RUN_MS) break;
        if (now - last_ms < CONTROL_DT_MS) { delay(1); continue; }
        float dt_s = (now - last_ms) / 1000.0f;
        last_ms = now;

        float left_mm = control_ticks_to_mm(encoder_get_ticks(ENCODER_LEFT));
        float right_mm = control_ticks_to_mm(encoder_get_ticks(ENCODER_RIGHT));
        float avg_mm = (left_mm + right_mm) / 2.0f;

        sensor_reading_t reading;
        sensor_read_all(&reading);
        // positive = closer to the left wall than the right one -> steer right to re-center
        float error = (float) reading.right_mm - (float) reading.left_mm;
        float correction = pid_update(&s_pid[CONTROL_LOOP_WALLCENTER], 0.0f, error, dt_s);

        int16_t left_out  = (int16_t) constrain(DRIVE_LEFT_SIGN  * (base_speed + correction), -255, 255);
        int16_t right_out = (int16_t) constrain(DRIVE_RIGHT_SIGN * (base_speed - correction), -255, 255);
        motor_set_speed(DRIVE_LEFT_MOTOR, left_out);
        motor_set_speed(DRIVE_RIGHT_MOTOR, right_out);

        s_debug.loop = CONTROL_LOOP_WALLCENTER;
        s_debug.setpoint = target_mm;
        s_debug.measurement = avg_mm;
        s_debug.output = correction;
        s_debug.active = true;

        if (tick_cb) tick_cb();

        if (avg_mm >= target_mm) break;
    }

    drive_stop();
    s_debug.active = false;
}

void control_run_spin(motor_id_t motor, int16_t pwm, control_tick_cb_t tick_cb) {
    s_abort = false;
    motor_set_speed(motor, pwm);

    uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    while (!s_abort) {
        uint32_t now = millis();
        if (now - start_ms > CONTROL_SPIN_MAX_RUN_MS) break;
        if (now - last_ms < CONTROL_DT_MS) { delay(1); continue; }
        last_ms = now;

        if (tick_cb) tick_cb();
    }

    motor_set_speed(motor, 0);
}
