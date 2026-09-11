#ifndef TASKS_H
#define TASKS_H

// Splits the robot across the ESP32's two cores as three FreeRTOS tasks.
// This is the ONLY module that calls xTaskCreatePinnedToCore - every
// other module here exposes a plain init()/tick() (or a single blocking
// run()) and stays agnostic of how/when it gets called.
//
//   - "control" task (core 1, highest priority): control_loop.h's fast,
//     deterministic tick - odometry -> motion profiles -> PD+FF -> motor
//     voltage - at a fixed CONTROL_LOOP_HZ via vTaskDelayUntil(). Never
//     touches I2C or Serial; nothing may block it.
//   - "sensor" task (core 0, medium priority): sensor_loop.h's ToF
//     poll + steering update + periodic battery read, at SENSOR_LOOP_HZ.
//     Deliberately on the OTHER core from "control" so its I2C
//     transactions can never stall the motor loop's timing.
//   - "top" task (core 0, normal priority): calls whatever function is
//     passed to tasks_start() - mouse_run() (mouse.h) for the real maze
//     run, or one of bench.h's routines for a bench-test build. Either
//     way it's a long blocking sequence of motion_move()/motion_turn()
//     calls that just waits on state the other two tasks maintain in the
//     background, then idles once the function returns.
//
// Call tasks_start() once from setup(), after motor_init()/encoder_init()/
// button_init(); leave loop() idle afterward.

typedef void (*tasks_top_level_fn_t)(void);

void tasks_start(tasks_top_level_fn_t top_level_fn);

#endif // TASKS_H
