#include "pid.h"

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->has_prev = false;
}

void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_reset(pid_ctrl_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->has_prev = false;
}

float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float dt_s) {
    float error = setpoint - measurement;

    pid->integral += error * dt_s;
    if (pid->integral_limit > 0.0f) {
        if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }

    float derivative = 0.0f;
    if (pid->has_prev && dt_s > 0.0f) {
        derivative = (error - pid->prev_error) / dt_s;
    }
    pid->prev_error = error;
    pid->has_prev = true;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
