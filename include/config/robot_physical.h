#ifndef CONFIG_ROBOT_PHYSICAL_H
#define CONFIG_ROBOT_PHYSICAL_H

#include <Arduino.h> // PI

// ===========================================================================
// MEASURED PHYSICAL CONSTANTS - the numbers odometry.cpp turns encoder
// ticks into real-world mm/degrees. Get these right before touching any
// PD gain; a wrong geometry constant just makes every controller chase a
// target that's silently the wrong size.
// ===========================================================================

// ---- Wheel + encoder geometry ----
// Method: hand-turn each wheel through N full revolutions (N=10 is plenty),
// read the tick delta from odometry (or encoder_get_ticks() directly),
// divide by N. Average a few trials. Do this per side - it is normal for
// left/right to differ slightly (encoder disc runout, wheel tolerance).
#define WHEEL_DIAMETER_MM          32.0f
#define ENCODER_TICKS_PER_REV_LEFT  715.0f
#define ENCODER_TICKS_PER_REV_RIGHT 715.0f // TODO: measure independently of LEFT

// mm of travel per encoder tick - derived, do not edit directly.
#define MM_PER_TICK_LEFT  ((PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV_LEFT)
#define MM_PER_TICK_RIGHT ((PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV_RIGHT)

// ---- Turning geometry ----
// "Turn radius": half the effective track width as seen by odometry's
// rotation estimate. Because of wheel scrub during a pivot this is
// usually a little different from the physically-measured wheel-center
// spacing, so tune it directly rather than trusting a ruler.
//
// Calibration procedure (mirrors ukmarsbots' MOUSE_RADIUS tuning):
//   1. Place the mouse on a flat surface, mark its heading with a line.
//   2. Rotate it BY HAND through as close to 360 degrees as you can judge
//      by eye, back to the marked line, while logging odometry_robot_angle_deg().
//   3. If the reported angle is LESS than 360, decrease TURN_RADIUS_MM.
//      If MORE than 360, increase it. Repeat until it reads ~360.
//
// Starting value below reuses the old turns.h TURNS_WHEEL_TRACK_MM_LEFT
// (120mm, "tuned + confirmed accurate by testing") halved, since that
// constant played the same role (track width = 2 * turn radius).
#define TURN_RADIUS_MM 60.0f

// Degrees of robot rotation per mm of (right-wheel-travel - left-wheel-travel)
// - derived from TURN_RADIUS_MM, do not edit directly.
#define DEG_PER_MM_DIFFERENCE (180.0f / (2.0f * TURN_RADIUS_MM * PI))

#define RADIANS_PER_DEGREE (2.0f * PI / 360.0f)
#define DEGREES_PER_RADIAN (360.0f / (2.0f * PI))

// ---- Polarity ----
// Encoder counts backwards while driving forward? Flip the corresponding
// sign below rather than rewiring/reflashing a pin swap.
#define ENCODER_LEFT_POLARITY  (+1)
#define ENCODER_RIGHT_POLARITY (+1)

// A positive commanded voltage/PWM spins a wheel backwards? Flip here.
#define MOTOR_LEFT_POLARITY  (+1)
#define MOTOR_RIGHT_POLARITY (-1)

// ---- Robot footprint (mm) ----
// Used for cell-relative offsets (e.g. how far the mouse's wheel axle
// sits from a wall it's backed up against). TODO: measure with calipers.
#define MOUSE_LENGTH_MM 90.0f  // front bumper to back edge
#define MOUSE_WIDTH_MM  70.0f  // outer edge to outer edge, across the track

#endif // CONFIG_ROBOT_PHYSICAL_H
