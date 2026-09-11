#ifndef CONFIG_TURN_PARAMS_H
#define CONFIG_TURN_PARAMS_H

#include "config/motion_tuning.h"
#include "config/sensor_calibration.h"

// ===========================================================================
// SEARCH-TURN GEOMETRY (mirrors ukmarsbots' TurnParameters / turn_params[]).
//
// A search turn is executed WHILE MOVING (mouse.cpp's turn_smooth()): the
// mouse keeps driving forward at `speed_mm_s`, and starts the pivot either
// at a fixed `entry_offset_mm` past the cell boundary, or earlier if the
// front sensor crosses `trigger_mm` first (i.e. there's a wall closer
// than expected - start the turn a bit sooner so it still finishes before
// hitting it). After the pivot, it drives `exit_offset_mm` before it's
// considered aligned with the new heading.
//
// ALL entry/exit offsets below are UNTUNED STARTING GUESSES, ported
// as-is from ukmarsbots' own defaults since we have no measured data yet.
// Tune with a bench test: run two cells through a single turn (see
// mouse.cpp's future test hook) and adjust entry/exit offset by eye until
// the mouse tracks the wall centrally on exit, same procedure as
// ukmarsbots' test_SS90E().
// ===========================================================================

typedef struct {
    int   speed_mm_s;
    int   entry_offset_mm;
    int   exit_offset_mm;
    float angle_deg;
    float omega_deg_s;
    float alpha_deg_s2;
    int   trigger_mm;
} turn_params_t;

typedef enum {
    TURN_SS90_LEFT = 0,
    TURN_SS90_RIGHT,
    TURN_COUNT
} turn_type_t;

// Small trim (deg) added to an in-place spin-turn's commanded angle, to
// correct for left/right asymmetry (motor/wheel mismatch) without having
// to maintain two different TURN_RADIUS_MM values. Tune by spinning
// 4x90 degrees in one direction and checking the mouse ends up facing
// the way it started; positive = turn a little further.
#define TURN_TRIM_DEG_LEFT  0.0f
#define TURN_TRIM_DEG_RIGHT 0.0f

// clang-format off
static const turn_params_t TURN_PARAMS[TURN_COUNT] = {
    // speed,               entry, exit, angle, omega, alpha,               trigger
    { SEARCH_TURN_SPEED_MM_S,  70,   80,  90.0f, 280.0f, 2000.0f, TOF_WALL_THRESHOLD_FRONT_MM + 20 }, // SS90_LEFT
    { SEARCH_TURN_SPEED_MM_S,  70,   80, -90.0f, 280.0f, 2000.0f, TOF_WALL_THRESHOLD_FRONT_MM + 20 }, // SS90_RIGHT
};
// clang-format on

#endif // CONFIG_TURN_PARAMS_H
