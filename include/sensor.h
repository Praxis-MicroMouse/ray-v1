#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// Three ToF (VL53L0X) sensors share one I2C bus (config/pins.h), but only
// ONE is ever powered on at a time: sensor_poll() cycles through them
// round-robin, fully powering a sensor on (XSHUT high), taking one
// measurement, then powering it back off (XSHUT low) before moving to
// the next. Every sensor uses the same default I2C address (there's
// nothing to reassign, since nothing else is ever live on the bus at the
// same time) - deliberately traded off against continuous, concurrent
// operation for the simplicity/robustness of never having more than one
// device driving the bus.
//
// Threading model: sensor_init() + sensor_poll() (called from
// sensor_loop_tick() - see sensor_loop.h) run on the sensor task;
// sensor_get_latest()/sensor_get_walls() are cross-core-safe and may be
// called from any task (mouse.cpp, steering.cpp).

typedef enum {
    SENSOR_FRONT = 0,
    SENSOR_RIGHT,
    SENSOR_LEFT,
    SENSOR_COUNT
} sensor_id_t;

typedef struct {
    uint16_t front_mm;
    uint16_t right_mm;
    uint16_t left_mm;
    bool     front_in_range;
    bool     right_in_range;
    bool     left_in_range;
} sensor_reading_t;

typedef struct {
    bool front;
    bool left;
    bool right;
} sensor_walls_t;

// Brings up I2C, resets all sensors via XSHUT (holding all three off),
// then powers each on in turn just long enough to confirm it responds
// and take one reading, logging OK/FAILED per sensor. Returns true only
// if all three sensors were found.
bool sensor_init(void);

// Services exactly ONE physical sensor per call (round-robin: front,
// right, left, front, ...) - powers it on, takes a measurement, powers
// it back off, runs the result through the per-sensor calibration
// equation (config/sensor_calibration.h) and the despike/EMA filter
// (filter.h), and updates the shared "latest reading" for that channel
// only. Each call blocks for roughly one full power-cycle (tens of ms) -
// call it from sensor_loop_tick(), NOT from anything timing-sensitive.
void sensor_poll(void);

// Cross-core-safe snapshot of the most recent reading sensor_poll() has
// assembled. Safe to call from any task, any time after sensor_init().
void sensor_get_latest(sensor_reading_t *out);

// Thresholds sensor_get_latest() against config/sensor_calibration.h's
// TOF_WALL_THRESHOLD_*_MM.
void sensor_get_walls(sensor_walls_t *out);

#endif // SENSOR_H
