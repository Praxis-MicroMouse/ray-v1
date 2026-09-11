#include "pd.h"

void pd_init(pd_ctrl_t *pd, float kp, float kd) {
    pd->kp = kp;
    pd->kd = kd;
    pd->prev_error = 0.0f;
    pd->has_prev = false;
}

void pd_set_gains(pd_ctrl_t *pd, float kp, float kd) {
    pd->kp = kp;
    pd->kd = kd;
}

void pd_get_gains(const pd_ctrl_t *pd, float *kp, float *kd) {
    *kp = pd->kp;
    *kd = pd->kd;
}

void pd_reset(pd_ctrl_t *pd) {
    pd->prev_error = 0.0f;
    pd->has_prev = false;
}

float pd_update_error(pd_ctrl_t *pd, float error, float dt_s) {
    float derivative = 0.0f;
    if (pd->has_prev && dt_s > 0.0f) {
        derivative = (error - pd->prev_error) / dt_s;
    }
    pd->prev_error = error;
    pd->has_prev = true;

    return pd->kp * error + pd->kd * derivative;
}

float pd_update(pd_ctrl_t *pd, float setpoint, float measurement, float dt_s) {
    return pd_update_error(pd, setpoint - measurement, dt_s);
}
