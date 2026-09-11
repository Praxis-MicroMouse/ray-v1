#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// Three ToF (VL53L0X) sensors share one I2C bus (config/pins.h). Each
// sensor's XSHUT pin is used at boot to bring them up one at a time so
// each can be assigned its own I2C address, then each is switched into
// continuous-ranging mode so sensor_loop_tick() can poll for fresh
// samples cheaply instead of blocking ~20-30ms per sensor per call.
//
// Threading model: sensor_init() + sensor_loop_tick() (see sensor_loop.h)
// run on the sensor task; sensor_get_latest()/sensor_get_walls() are
// cross-core-safe and may be called from any task (mouse.cpp, steering.cpp).

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

// Brings up I2C, resets all sensors via XSHUT, assigns each a unique I2C
// address, and starts each in continuous-ranging mode. Returns true only
// if all three sensors were found.
bool sensor_init(void);

// Polls each channel for a fresh sample (non-blocking - VL53L0X's
// isRangeComplete()/readRangeResult()), runs it through the per-sensor
// calibration equation (config/sensor_calibration.h) and the despike/EMA
// filter (filter.h), and updates the shared "latest reading". Call
// frequently (SENSOR_LOOP_HZ, from sensor_loop_tick()) - each call is
// cheap (an I2C status read per channel, only occasionally a full
// distance read).
void sensor_poll(void);

// Cross-core-safe snapshot of the most recent reading sensor_poll() has
// assembled. Safe to call from any task, any time after sensor_init().
void sensor_get_latest(sensor_reading_t *out);

// Thresholds sensor_get_latest() against config/sensor_calibration.h's
// TOF_WALL_THRESHOLD_*_MM.
void sensor_get_walls(sensor_walls_t *out);

#endif // SENSOR_H
