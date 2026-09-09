#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// Three ToF (VL53L0X) sensors share one I2C bus (SDA/SCL tied together).
// Each sensor's XSHUT pin is used at boot to bring them up one at a time
// so each can be assigned its own I2C address.

#define SENSOR_XSHUT_FRONT 18
#define SENSOR_XSHUT_RIGHT 19
#define SENSOR_XSHUT_LEFT  5

#define SENSOR_I2C_SDA 21
#define SENSOR_I2C_SCL 22

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
} sensor_reading_t;

// Brings up I2C, resets all sensors via XSHUT, and assigns each a unique
// I2C address. Returns true only if all three sensors were found.
bool sensor_init(void);

// Reads all three sensors. Returns false (with a log line) if any sensor
// reading is invalid/out of range; readings still get filled in.
bool sensor_read_all(sensor_reading_t *out);

#endif // SENSOR_H
