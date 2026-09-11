#ifndef PD_H
#define PD_H

#include <stdbool.h>

// Minimal, generic PD controller (proportional + derivative only - no
// integral term). This is the ONE controller implementation used
// everywhere a closed loop is needed: forward position tracking,
// rotation/heading tracking, and cross-track steering all instantiate
// their own pd_ctrl_t and pull gains from config/motion_tuning.h.
//
// Why PD, not PID: every use here tracks a continuously-updated target
// fed by a motion profile (see profile.h) rather than a fixed setpoint,
// so the accumulated tracking error naturally stays near zero without
// needing an integral term to eliminate steady-state offset - and
// integral windup is one less failure mode to guard against while
// tuning. This mirrors ukmarsbots' position/rotation/steering
// controllers, which are all PD as well.

typedef struct {
    float kp;
    float kd;

    float prev_error;
    bool  has_prev; // false until the first update(), so the first
                     // derivative term doesn't spike from an undefined prev_error
} pd_ctrl_t;

void pd_init(pd_ctrl_t *pd, float kp, float kd);
void pd_set_gains(pd_ctrl_t *pd, float kp, float kd);
void pd_get_gains(const pd_ctrl_t *pd, float *kp, float *kd);

// Clears derivative history (call when starting a new maneuver or after
// changing gains) - keeps the gains themselves.
void pd_reset(pd_ctrl_t *pd);

// Runs one PD step on a precomputed error (e.g. an accumulated tracking
// error that isn't a simple setpoint-minus-measurement, such as
// drive_controller.cpp's rotation loop, which folds in a continuous
// steering-rate term). dt in seconds.
float pd_update_error(pd_ctrl_t *pd, float error, float dt_s);

// Runs one PD step as error = setpoint - measurement, then pd_update_error().
// This is the common case: setpoint/measurement are two independently
// accumulated running totals (e.g. profile position vs. odometry
// distance) rather than a target that resets each call.
float pd_update(pd_ctrl_t *pd, float setpoint, float measurement, float dt_s);

#endif // PD_H
