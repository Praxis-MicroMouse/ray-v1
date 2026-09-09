#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// Simple two-wheel differential drive built on top of the motor module.
// Left/right wheel <-> Motor A/B assignment is a guess (motor.h doesn't
// know physical mounting) - if "forward" actually spins one wheel
// backward, or a "turn" spins the wrong way, flip that motor's sign in
// drive.cpp rather than rewiring.

#define DRIVE_DEFAULT_SPEED 150  // out of 255, tune by testing

void drive_forward(int16_t speed);
void drive_turn_left(int16_t speed);
void drive_turn_right(int16_t speed);
void drive_stop(void);

#endif // DRIVE_H
