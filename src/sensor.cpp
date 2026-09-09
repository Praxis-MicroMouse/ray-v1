#include "sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Base address used for the first sensor we bring up; the rest get
// base+1, base+2 assigned before their XSHUT is released.
#define SENSOR_I2C_ADDR_BASE 0x30
#define SENSOR_MAX_RANGE_MM  2000

static Adafruit_VL53L0X s_tof[SENSOR_COUNT];
static const uint8_t s_xshut_pin[SENSOR_COUNT] = {
    SENSOR_XSHUT_FRONT,
    SENSOR_XSHUT_RIGHT,
    SENSOR_XSHUT_LEFT
};
static const char *s_name[SENSOR_COUNT] = { "FRONT", "RIGHT", "LEFT" };

bool sensor_init(void) {
    Wire.begin(SENSOR_I2C_SDA, SENSOR_I2C_SCL);

    // Hold every sensor in reset (XSHUT low) first.
    for (int i = 0; i < SENSOR_COUNT; i++) {
        pinMode(s_xshut_pin[i], OUTPUT);
        digitalWrite(s_xshut_pin[i], LOW);
    }
    delay(10);

    bool all_ok = true;

    // Bring sensors up one at a time so each can be moved off the
    // default 0x29 address before the next one appears on the bus.
    for (int i = 0; i < SENSOR_COUNT; i++) {
        digitalWrite(s_xshut_pin[i], HIGH);
        delay(10);

        if (!s_tof[i].begin(SENSOR_I2C_ADDR_BASE + i)) {
            Serial.printf("[SENSOR] %s init FAILED (xshut=%d)\n",
                          s_name[i], s_xshut_pin[i]);
            all_ok = false;
            continue;
        }

        Serial.printf("[SENSOR] %s init OK (addr=0x%02X, xshut=%d)\n",
                      s_name[i], SENSOR_I2C_ADDR_BASE + i, s_xshut_pin[i]);
    }

    return all_ok;
}

static uint16_t read_one(sensor_id_t id) {
    VL53L0X_RangingMeasurementData_t m;
    s_tof[id].rangingTest(&m, false);

    if (m.RangeStatus == 4) {
        Serial.printf("[SENSOR] %s out of range\n", s_name[id]);
        return SENSOR_MAX_RANGE_MM;
    }

    return m.RangeMilliMeter;
}

bool sensor_read_all(sensor_reading_t *out) {
    out->front_mm = read_one(SENSOR_FRONT);
    out->right_mm = read_one(SENSOR_RIGHT);
    out->left_mm  = read_one(SENSOR_LEFT);

    Serial.printf("[SENSOR] front=%u mm right=%u mm left=%u mm\n",
                  out->front_mm, out->right_mm, out->left_mm);

    bool valid = out->front_mm < SENSOR_MAX_RANGE_MM
              || out->right_mm < SENSOR_MAX_RANGE_MM
              || out->left_mm  < SENSOR_MAX_RANGE_MM;
    return valid;
}
