#ifndef BENCH_H
#define BENCH_H

#include "motor.h"

// Bench/characterization routines used by TUNING.md. All output is
// plain Serial.printf lines meant to be read off `pio device monitor` or
// piped through a plotter - no telemetry/CLI dependency.
//
// bench_feedforward() drives a motor directly, open-loop - it must be
// run STANDALONE (call it from setup()/loop(), NOT via tasks_start()),
// since the control task's drive_controller would otherwise fight it
// for the motor output every tick. Requires motor_init()+encoder_init()+
// battery_init() to have run first, and nothing else touching the motors.
//
// The other three are ordinary tasks_start() top-level functions - they
// run through the full three-task control/sensor pipeline, exactly like
// mouse_run() does, and report on how well it's tracking.

void bench_feedforward(motor_id_t motor);

void bench_straight_test(void);
void bench_turn_test(void);
void bench_wallcenter_test(void);

#endif // BENCH_H
