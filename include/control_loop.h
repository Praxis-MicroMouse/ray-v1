#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

// The fast, deterministic tick: odometry -> motion profiles -> PD+FF ->
// motor voltage, every CONTROL_LOOP_INTERVAL_S. This is the ESP32
// equivalent of ukmarsbots' 500Hz systick ISR, except run as a
// dedicated high-priority FreeRTOS task (tasks.cpp) rather than a
// hardware-timer ISR - Arduino/ESP-IDF discourage doing real work
// (floating point control math, motor driver calls) inside an ISR, and
// vTaskDelayUntil() gives plenty of timing precision for a robot moving
// at these speeds. Keeping this loop pinned to its own core and free of
// anything that blocks (no I2C, no Serial) is what actually matters for
// jitter-free control - see sensor_loop.h for the slower, I2C-bound work
// this loop deliberately does NOT do.

void control_loop_init(void);

// One control tick. Call at a fixed CONTROL_LOOP_INTERVAL_S cadence from
// a dedicated task - see tasks.cpp.
void control_loop_tick(void);

#endif // CONTROL_LOOP_H
