#ifndef TURNS_H
#define TURNS_H

#include <stdint.h>
#include <stdbool.h>

// Pivot-turn maneuvers, measured via the wheel encoders (encoder.h) - no
// MPU9250 fitted, so heading can't be gyro-integrated the way
// control.h's CONTROL_LOOP_TURN was originally meant to. During a pivot
// (wheels spinning in opposite directions, same speed) each wheel travels
// an arc of length theta * (TURNS_WHEEL_TRACK_MM / 2) - target_ticks is
// derived from that via control.h's control_ticks_to_mm(), so it reuses
// the already-measured WHEEL_DIAMETER_MM/ENCODER_TICKS_PER_REV instead of
// needing its own calibration constant for that part.
//
// TURNS_WHEEL_TRACK_MM (distance between the two wheels' contact
// centers) is an unverified starting guess below - measure it with a
// ruler/calipers for a better first try, then tune by testing: run
// turn_right_90(), measure the actual angle turned (protractor, or mark
// a reference line on the floor and the chassis), and rescale:
//   TURNS_WHEEL_TRACK_MM_NEW = TURNS_WHEEL_TRACK_MM_OLD * (90.0 / measured_degrees)
// (overturned => shrink it; underturned => grow it) and reflash.
//
// Positive degrees = rightward/clockwise (viewed from above); if the
// mouse turns the wrong way, flip DRIVE_LEFT_SIGN/DRIVE_RIGHT_SIGN's
// roles in turn_degrees() (turns.cpp) rather than negating every call
// site here - same pragmatic approach as drive.h's DRIVE_*_SIGN.

#define TURNS_WHEEL_TRACK_MM 120.0f // TODO: measure the real value and tune per the note above
#define TURNS_DEFAULT_SPEED 150     // out of 255 - pivot turns don't need full PWM like straight runs
#define TURNS_MAX_RUN_MS 3000       // safety ceiling if encoders don't confirm the turn completed

// No MPU9250 to bring up - kept as a function so call sites don't need to
// change if a gyro is added back later. Currently just a no-op.
bool turns_init(void);

// Pivots by an arbitrary signed angle (degrees) at the given base PWM
// speed - positive turns right/clockwise, negative turns left/
// counterclockwise. Blocking, to TURNS_MAX_RUN_MS at most.
void turn_degrees(float degrees, int16_t speed);

// Convenience wrappers for the common 90-degree pivot.
void turn_right_90(int16_t speed);
void turn_left_90(int16_t speed);

#endif // TURNS_H
