#ifndef STEERING_H
#define STEERING_H

// Continuous cross-track correction from the left/right ToF sensors,
// mirroring ukmarsbots' sensors.h steering logic. While driving straight
// down a corridor, this nudges the rotation controller (drive_controller.cpp)
// by a small heading RATE (deg/s) toward the corridor's center, instead of
// only correcting drift after the fact via wall-mapping.
//
// Threading model: steering_update() runs on the sensor task (right after
// sensor_poll() - see sensor_loop.h); steering_get_adjustment() is
// cross-core-safe and is read every control tick by drive_controller.cpp
// on the control task.

typedef enum {
    STEERING_OFF = 0,   // never steer from zero speed, or during a turn
    STEERING_NORMAL,    // favor whichever side wall is seen; both -> centered
    STEERING_LEFT_WALL, // follow the left wall only
    STEERING_RIGHT_WALL // follow the right wall only
} steering_mode_t;

void steering_init(void);

void steering_set_mode(steering_mode_t mode);
steering_mode_t steering_get_mode(void);

void steering_set_gains(float kp, float kd);
void steering_get_gains(float *kp, float *kd);

// Reads sensor_get_latest(), computes cross-track error for the current
// mode, runs the steering PD, clamps to STEERING_ADJUST_LIMIT_DEG_S, and
// stores the result. Call from the sensor task, once per sensor_poll().
void steering_update(void);

// Cross-core-safe read of the last steering_update() output (deg/s).
// 0 whenever the mode is STEERING_OFF.
float steering_get_adjustment(void);

#endif // STEERING_H
