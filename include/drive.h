#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

#include "motor.h"

// Simple two-wheel differential drive built on top of the motor module.
// Left/right wheel <-> Motor A/B assignment is a guess (motor.h doesn't
// know physical mounting) - if "forward" actually spins one wheel
// backward, or a "turn" spins the wrong way, flip that motor's sign
// below rather than rewiring. Exposed here (not just inside drive.cpp)
// so other modules that need independent per-wheel control - control.cpp's
// PID loops, for instance - reuse the same wheel/polarity mapping instead
// of guessing it again.
#define DRIVE_LEFT_MOTOR  MOTOR_A
#define DRIVE_RIGHT_MOTOR MOTOR_B
#define DRIVE_LEFT_SIGN   1
#define DRIVE_RIGHT_SIGN  1

#define DRIVE_DEFAULT_SPEED 150  // out of 255, tune by testing

void drive_forward(int16_t speed);
void drive_turn_left(int16_t speed);
void drive_turn_right(int16_t speed);
void drive_stop(void);

#endif // DRIVE_H
