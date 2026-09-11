#ifndef PROFILE_H
#define PROFILE_H

#include <stdint.h>
#include <stdbool.h>

// Trapezoidal motion profile - the same shape ukmarsbots' Profile class
// generates (accelerate / cruise / brake), ported to a plain struct +
// free-function style to match this codebase's convention (see pd.h,
// filter.h). Unit-agnostic: interpret distance/speed/acceleration as mm,
// degrees, or anything else, as long as you're consistent within one
// profile_t instance.
//
// Pure math, no Arduino/FreeRTOS dependency - runs on the host under
// `pio test -e native` (see test/test_profile). Cross-task-safe access
// (this instance is written by the control loop and read by another
// task) is NOT this module's job - see motion.h, which owns that locking
// around its two profile_t instances.
//
// Call profile_update() once per fixed-size control tick (dt_s should be
// the same value every call - CONTROL_LOOP_INTERVAL_S in
// config/motion_tuning.h).

typedef enum {
    PROFILE_IDLE = 0,
    PROFILE_ACCELERATING = 1,
    PROFILE_BRAKING = 2,
    PROFILE_FINISHED = 3,
} profile_state_t;

typedef struct {
    profile_state_t state;
    float position;
    float speed;
    float target_speed;
    float final_speed;
    float final_position;
    float acceleration;
    float one_over_acc;
    int8_t sign;
} profile_t;

// Zeroes position/speed/target, sets state to PROFILE_IDLE.
void profile_reset(profile_t *p);

bool profile_is_finished(const profile_t *p);

// Begins a profile; a subsequent call before completion supersedes all
// parameters (the profile "retargets" mid-motion rather than queuing).
//   distance      always positive - direction comes from top_speed's sign
//   top_speed     mm/s (or deg/s, etc) - negative moves in reverse
//   final_speed   speed to be at when `distance` is reached
//   acceleration  always positive
void profile_start(profile_t *p, float distance, float top_speed, float final_speed, float acceleration);

// Immediately targets zero speed and marks the profile finished (the
// profile will still ramp actual speed toward zero on subsequent
// profile_update() calls - "finished" means "no distance target left to
// honor", not "already at rest").
void profile_stop(profile_t *p);

// Forces speed to the current target and marks the profile finished.
void profile_finish(profile_t *p);

void profile_set_speed(profile_t *p, float speed);
void profile_set_target_speed(profile_t *p, float speed);

// Only used to correct position for e.g. forward error correction
// against a wall reference.
void profile_adjust_position(profile_t *p, float delta);
void profile_set_position(profile_t *p, float position);

float profile_position(const profile_t *p);
float profile_speed(const profile_t *p);
float profile_acceleration(const profile_t *p);

// Distance needed to reach final_speed from the current speed at the
// current acceleration.
float profile_get_braking_distance(const profile_t *p);

// Advances the profile by one control tick. Call unconditionally, every
// tick, regardless of state (a PROFILE_IDLE profile is a cheap no-op).
void profile_update(profile_t *p, float dt_s);

#endif // PROFILE_H
