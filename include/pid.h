#ifndef PID_H
#define PID_H

#include <stdbool.h>

// Minimal, generic PID controller. One instance per control loop (see
// control.h) so gains can be tuned independently, live over serial (see
// comms.h), without recompiling/reflashing.

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    float integral_limit; // anti-windup clamp on the integral term; 0 = unlimited
    bool  has_prev;       // false until the first update(), so the first
                           // derivative term doesn't spike from an undefined prev_error
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float integral_limit);
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

// Clears integral/derivative history (call when starting a new maneuver
// or after changing gains) - keeps the gains themselves.
void pid_reset(pid_ctrl_t *pid);

// Runs one PID step: error = setpoint - measurement, dt in seconds.
// Returns the raw controller output (kp*e + ki*integral + kd*derivative) -
// the caller clamps/scales it to a motor PWM range.
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float dt_s);

#endif // PID_H
