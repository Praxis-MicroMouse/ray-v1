#include "sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "filter.h"

// Base address used for the first sensor we bring up; the rest get
// base+1, base+2 assigned before their XSHUT is released.
#define SENSOR_I2C_ADDR_BASE 0x30
#define SENSOR_MAX_RANGE_MM  2000

static Adafruit_VL53L0X s_tof[SENSOR_COUNT];
static filter_t s_filter[SENSOR_COUNT]; // despike + smooth each channel's raw readings - see filter.h
static const uint8_t s_xshut_pin[SENSOR_COUNT] = {
    SENSOR_XSHUT_FRONT,
    SENSOR_XSHUT_RIGHT,
    SENSOR_XSHUT_LEFT
};
static const char *s_name[SENSOR_COUNT] = { "FRONT", "RIGHT", "LEFT" };

bool sensor_init(void) {
    // Enable the ESP32's internal weak (~45k) pull-ups on SDA/SCL so the
    // bus has a defined idle-high level even without external pull-up
    // resistors on the sensor breakouts - cuts down on glitchy/garbled
    // I2C transactions (and the bogus readings that come with them) on
    // this DevKitC-1 board. Wire.begin() doesn't reliably do this itself
    // across core versions, so it's set explicitly here. Note these are
    // weak compared to a proper external 2.2k-4.7k pull-up per line - if
    // readings are still noisy with three devices on one bus, add real
    // resistors rather than relying on this alone.
    pinMode(SENSOR_I2C_SDA, INPUT_PULLUP);
    pinMode(SENSOR_I2C_SCL, INPUT_PULLUP);
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
        filter_init(&s_filter[i]);
        digitalWrite(s_xshut_pin[i], HIGH);
        delay(10);

        // High-accuracy profile: longer timing budget, tighter VCSEL
        // periods -> lower noise, which matters most at short (0-10cm)
        // range where we're fine-tuning sensor placement.
        if (!s_tof[i].begin(SENSOR_I2C_ADDR_BASE + i, false, &Wire,
                             Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY)) {
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

// Returns the despiked/smoothed reading (see filter.h) and, via
// *in_range, whether the *raw* reading this call was actually in range -
// tracked separately from the smoothed value so a sensor that's
// genuinely out of range doesn't get reported as "valid" just because
// its smoothed estimate hasn't finished climbing toward
// SENSOR_MAX_RANGE_MM yet.
static uint16_t read_one(sensor_id_t id, bool *in_range) {
    VL53L0X_RangingMeasurementData_t m;
    s_tof[id].rangingTest(&m, false);

    float raw_mm;
    if (m.RangeStatus == 4) {
        Serial.printf("[SENSOR] %s out of range\n", s_name[id]);
        raw_mm = SENSOR_MAX_RANGE_MM;
        *in_range = false;
    } else {
        raw_mm = m.RangeMilliMeter;
        *in_range = true;
    }

    return (uint16_t) filter_update(&s_filter[id], raw_mm);
}

bool sensor_read_all(sensor_reading_t *out) {
    bool front_in_range, right_in_range, left_in_range;
    out->front_mm = read_one(SENSOR_FRONT, &front_in_range);
    out->right_mm = read_one(SENSOR_RIGHT, &right_in_range);
    out->left_mm  = read_one(SENSOR_LEFT, &left_in_range);

    Serial.printf("[SENSOR] front=%u mm right=%u mm left=%u mm\n",
                  out->front_mm, out->right_mm, out->left_mm);

    return front_in_range || right_in_range || left_in_range;
}
