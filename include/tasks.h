#ifndef TASKS_H
#define TASKS_H

// Splits the maze run across the ESP32's two cores as two FreeRTOS
// tasks, communicating over queues (see tasks.cpp):
//
//   - Core 0 (PRO_CPU) - "planning" task: owns the maze grid, runs
//     flood-fill during exploration and the Dijkstra min-turn planner
//     (maze.h) for the speed run afterward. Pure computation - it never
//     touches a sensor or motor directly, only sends action requests
//     ("move forward", "turn left/right", "sense") and reads back wall
//     sensing results.
//   - Core 1 (APP_CPU) - "control" task: owns all hardware I/O. It reads
//     the ToF sensors (sensor.h) and drives the motors (drive.h) one
//     requested action at a time, reporting sensed walls back after
//     every action.
//
// Arduino's own setup()/loop() already run pinned to core 1 by default;
// tasks_start() spawns two *additional* tasks rather than replacing
// that, so call it once near the end of setup() (after motor_init()) and
// leave loop() idle, or doing something unrelated (e.g. battery
// logging) - the maze run itself happens entirely inside these two
// tasks. The control task calls sensor_init() itself on startup.
void tasks_start(void);

#endif // TASKS_H
