#ifndef SENSOR_LOOP_H
#define SENSOR_LOOP_H

#include <stdbool.h>

// The slower, I2C-bound tick: poll the ToF sensors, recompute the
// steering correction, and refresh the cached battery voltage - all the
// work that can't run inside control_loop.h's fast, deterministic loop
// without blowing its timing budget. Runs at SENSOR_LOOP_HZ
// (config/motion_tuning.h) on its own task (tasks.cpp).

// Brings up the ToF sensors (sensor_init()) and steering. Returns false
// if any sensor failed to initialize (the loop still runs - sensor.cpp
// reports out-of-range for a missing sensor rather than touching it).
bool sensor_loop_init(void);

// One sensor-loop tick: sensor_poll() + steering_update() + a periodic
// battery_update(). Call at a fixed 1/SENSOR_LOOP_HZ cadence.
void sensor_loop_tick(void);

#endif // SENSOR_LOOP_H
