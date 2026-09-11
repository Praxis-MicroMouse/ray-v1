#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

// Per-tick PD position/rotation control + per-wheel feedforward, combined
// into battery-voltage-compensated motor voltages - mirrors ukmarsbots'
// Motors::update_controllers(). This is the module that actually decides
// what voltage to apply; motion.h only decides WHERE the robot should be
// (the profile targets), and odometry.h reports where it actually is.
//
// Gains/feedforward constants all come from config/motion_tuning.h -
// nothing here hardcodes a number.
//
// CONTROL-TASK ONLY: drive_controller_update() (and everything else here)
// must only be called from the fast control loop (control_loop.h), after
// odometry_update() and motion_update() have both run for this tick.
// Nothing here is exposed to other tasks, so no locking is needed.

void drive_controller_init(void);

void drive_controller_enable(void);
void drive_controller_disable(); // motors coast every tick while disabled

// Clears PD derivative history and feedforward's previous-speed memory -
// call when starting a fresh maneuver sequence (motion_reset_drive_system()
// does this for you).
void drive_controller_reset(void);

// One control-tick's worth of PD + feedforward -> motor voltage output.
// No-op (beyond coasting the motors) while disabled.
void drive_controller_update(void);

// Runtime gain tuning (no reflash needed) - mirrors the old control.cpp's
// control_set_gains()/control_get_gains() convenience.
void drive_controller_set_forward_gains(float kp, float kd);
void drive_controller_get_forward_gains(float *kp, float *kd);
void drive_controller_set_rotation_gains(float kp, float kd);
void drive_controller_get_rotation_gains(float *kp, float *kd);

#endif // DRIVE_CONTROLLER_H
