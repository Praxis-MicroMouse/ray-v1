#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>

// Despike + exponential-smoothing filter for noisy, spike-prone
// distance readings (VL53L0X ToF sensors, primarily - stray reflections
// occasionally produce a single wildly-wrong sample). One instance per
// channel; stateful, so don't share one filter_t across sensors.
//
// Two stages, applied on every update:
//  1. Spike rejection: a new reading that jumps more than
//     FILTER_SPIKE_THRESHOLD_MM from the current smoothed estimate is
//     held back (the estimate doesn't move) UNLESS the *previous* raw
//     reading already agreed with it - so a genuinely fast real change
//     (a wall appearing/disappearing as the robot moves) still gets
//     through after one confirming sample, rather than being
//     permanently stuck.
//  2. Exponential moving average: accepted readings are blended into
//     the running estimate with weight FILTER_EMA_ALPHA (0-1; higher =
//     less smoothing, more responsive).

#define FILTER_SPIKE_THRESHOLD_MM 150.0f
#define FILTER_EMA_ALPHA 0.35f

typedef struct
{
    float estimate;
    float last_raw;
    bool initialized;
} filter_t;

// Resets a filter to a fresh, un-primed state; the first update() call
// afterward snaps straight to that reading instead of smoothing from 0.
void filter_init(filter_t *f);

// Feeds one raw reading through the filter and returns the smoothed value.
float filter_update(filter_t *f, float raw);

#endif // FILTER_H
