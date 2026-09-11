#include "steering.h"

#include <Arduino.h>

#include "pd.h"
#include "sensor.h"
#include "sync.h"
#include "config/motion_tuning.h"
#include "config/sensor_calibration.h"

static pd_ctrl_t s_pd;
static steering_mode_t s_mode = STEERING_OFF;

// steering_update() is called once per sensor_loop_tick(), but
// sensor.cpp's round-robin sensor_poll() no longer runs at a fixed
// SENSOR_LOOP_HZ cadence - each call blocks for roughly one full
// sensor power-cycle, several times longer than 1/SENSOR_LOOP_HZ. The
// derivative term below needs the REAL elapsed time between updates,
// not an assumed fixed one, so it's measured directly via micros().
static uint32_t s_last_update_us = 0;
static bool s_have_last_update = false;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_adjustment = 0.0f;

void steering_init(void) {
    pd_init(&s_pd, PD_STEERING_KP, PD_STEERING_KD);
    s_mode = STEERING_OFF;
    s_have_last_update = false;
    SYNC(s_mux) { s_adjustment = 0.0f; }
}

void steering_set_mode(steering_mode_t mode) {
    pd_reset(&s_pd); // fresh derivative history - don't react to a mode-old error jump
    s_have_last_update = false; // don't compute a dt spanning however long steering was off/on a different mode
    s_mode = mode;
    if (mode == STEERING_OFF) {
        SYNC(s_mux) { s_adjustment = 0.0f; }
    }
}

steering_mode_t steering_get_mode(void) {
    return s_mode;
}

void steering_set_gains(float kp, float kd) {
    pd_set_gains(&s_pd, kp, kd);
}

void steering_get_gains(float *kp, float *kd) {
    pd_get_gains(&s_pd, kp, kd);
}

static float constrain_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void steering_update(void) {
    if (s_mode == STEERING_OFF) {
        SYNC(s_mux) { s_adjustment = 0.0f; }
        return;
    }

    sensor_reading_t r;
    sensor_get_latest(&r);

    // At very close range to a wall ahead, a ToF's narrow-FoV beam can
    // still pick up cross-talk off it, corrupting the side channels -
    // hold the last correction rather than react to a bogus error.
    if (r.front_mm <= TOF_FRONT_RELIABILITY_LIMIT_MM) {
        return;
    }

    sensor_walls_t walls;
    sensor_get_walls(&walls);

    // Positive error = too far right (needs a leftward/negative heading
    // rate to correct), matching maze.h's rightward-positive turn
    // convention being applied in the opposite sense here - see
    // drive_controller.cpp for how this folds into the rotation target.
    float error = 0.0f;
    switch (s_mode) {
        case STEERING_NORMAL:
            if (walls.left && walls.right) {
                error = (float)r.left_mm - (float)r.right_mm;
            } else if (walls.left) {
                error = (float)r.left_mm - TOF_WALL_THRESHOLD_SIDE_MM;
            } else if (walls.right) {
                error = TOF_WALL_THRESHOLD_SIDE_MM - (float)r.right_mm;
            }
            break;
        case STEERING_LEFT_WALL:
            error = (float)r.left_mm - TOF_WALL_THRESHOLD_SIDE_MM;
            break;
        case STEERING_RIGHT_WALL:
            error = TOF_WALL_THRESHOLD_SIDE_MM - (float)r.right_mm;
            break;
        default:
            break;
    }

    uint32_t now_us = micros();
    // Fall back to the nominal period for the first update after a mode
    // change (has_prev is false then anyway, so this dt doesn't feed a
    // real derivative) rather than measuring against a stale timestamp.
    float dt_s = s_have_last_update ? (now_us - s_last_update_us) / 1000000.0f : (1.0f / SENSOR_LOOP_HZ);
    s_last_update_us = now_us;
    s_have_last_update = true;

    float adjustment = pd_update_error(&s_pd, error, dt_s);
    adjustment = constrain_f(adjustment, -STEERING_ADJUST_LIMIT_DEG_S, STEERING_ADJUST_LIMIT_DEG_S);

    SYNC(s_mux) { s_adjustment = adjustment; }
}

float steering_get_adjustment(void) {
    float v;
    SYNC(s_mux) { v = s_adjustment; }
    return v;
}
