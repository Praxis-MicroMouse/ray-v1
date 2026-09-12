#include "sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Base address used for the first sensor we bring up; the rest get
// base+1, base+2 assigned before their XSHUT is released.
#define SENSOR_I2C_ADDR_BASE 0x30
#define SENSOR_MAX_RANGE_MM  2000

static Adafruit_VL53L0X s_tof[SENSOR_COUNT];
static bool s_sensor_ok[SENSOR_COUNT] = { false, false, false };
static const uint8_t s_xshut_pin[SENSOR_COUNT] = {
    SENSOR_XSHUT_FRONT,
    SENSOR_XSHUT_RIGHT,
    SENSOR_XSHUT_LEFT
};
static const char *s_name[SENSOR_COUNT] = { "FRONT", "RIGHT", "LEFT" };

// VL53L0X default I2C address before Adafruit_VL53L0X::begin() reassigns
// it - every sensor answers here right after its XSHUT is released.
#define SENSOR_DEFAULT_ADDR 0x29

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
        s_sensor_ok[i] = false;
        digitalWrite(s_xshut_pin[i], HIGH);
        delay(10);

        // Cheap, bounded presence check before calling begin(): if nothing
        // ACKs at the default address, the sensor isn't there/isn't
        // powered/isn't wired right - skip it rather than calling
        // Adafruit_VL53L0X::begin(), whose internal init/calibration
        // polling loop has no timeout and can block forever (has been
        // observed to hang setup() indefinitely on an unresponsive
        // sensor, taking the whole board down with it).
        Wire.beginTransmission(SENSOR_DEFAULT_ADDR);
        uint8_t probe_err = Wire.endTransmission();
        if (probe_err != 0) {
            Serial.printf("[SENSOR] %s not responding at default address (xshut=%d, i2c_err=%d) - skipping\n",
                          s_name[i], s_xshut_pin[i], probe_err);
            all_ok = false;
            continue;
        }

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

        s_sensor_ok[i] = true;
        Serial.printf("[SENSOR] %s init OK (addr=0x%02X, xshut=%d)\n",
                      s_name[i], SENSOR_I2C_ADDR_BASE + i, s_xshut_pin[i]);
    }

    return all_ok;
}

// Returns the raw reading and, via *in_range, whether it was actually in
// range.
static uint16_t read_one(sensor_id_t id, bool *in_range) {
    if (!s_sensor_ok[id]) {
        // Never initialized (see sensor_init()'s presence probe) - don't
        // touch the VL53L0X instance at all, just report "out of range".
        *in_range = false;
        return SENSOR_MAX_RANGE_MM;
    }

    VL53L0X_RangingMeasurementData_t m;
    s_tof[id].rangingTest(&m, false);

    if (m.RangeStatus == 4) {
        Serial.printf("[SENSOR] %s out of range\n", s_name[id]);
        *in_range = false;
        return SENSOR_MAX_RANGE_MM;
    }

    *in_range = true;
    return (uint16_t) m.RangeMilliMeter;
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
