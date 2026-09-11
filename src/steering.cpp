#include "steering.h"

#include "pd.h"
#include "sensor.h"
#include "sync.h"
#include "config/motion_tuning.h"
#include "config/sensor_calibration.h"

static pd_ctrl_t s_pd;
static steering_mode_t s_mode = STEERING_OFF;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_adjustment = 0.0f;

void steering_init(void) {
    pd_init(&s_pd, PD_STEERING_KP, PD_STEERING_KD);
    s_mode = STEERING_OFF;
    SYNC(s_mux) { s_adjustment = 0.0f; }
}

void steering_set_mode(steering_mode_t mode) {
    pd_reset(&s_pd); // fresh derivative history - don't react to a mode-old error jump
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

    float dt_s = 1.0f / SENSOR_LOOP_HZ;
    float adjustment = pd_update_error(&s_pd, error, dt_s);
    adjustment = constrain_f(adjustment, -STEERING_ADJUST_LIMIT_DEG_S, STEERING_ADJUST_LIMIT_DEG_S);

    SYNC(s_mux) { s_adjustment = adjustment; }
}

float steering_get_adjustment(void) {
    float v;
    SYNC(s_mux) { v = s_adjustment; }
    return v;
}
