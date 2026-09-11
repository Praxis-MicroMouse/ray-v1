#include "sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "filter.h"
#include "sync.h"
#include "config/pins.h"
#include "config/sensor_calibration.h"

// Base address used for the first sensor we bring up; the rest get
// base+1, base+2 assigned before their XSHUT is released.
#define SENSOR_I2C_ADDR_BASE 0x30

// VL53L0X default I2C address before Adafruit_VL53L0X::begin() reassigns
// it - every sensor answers here right after its XSHUT is released.
#define SENSOR_DEFAULT_ADDR 0x29

static Adafruit_VL53L0X s_tof[SENSOR_COUNT];
static filter_t s_filter[SENSOR_COUNT]; // despike + smooth each channel's raw readings - see filter.h
static bool s_sensor_ok[SENSOR_COUNT] = { false, false, false };
static bool s_last_in_range[SENSOR_COUNT] = { false, false, false };
static const uint8_t s_xshut_pin[SENSOR_COUNT] = {
    PIN_TOF_XSHUT_FRONT,
    PIN_TOF_XSHUT_RIGHT,
    PIN_TOF_XSHUT_LEFT
};
static const char *s_name[SENSOR_COUNT] = { "FRONT", "RIGHT", "LEFT" };
static const float s_scale[SENSOR_COUNT]  = { TOF_FRONT_SCALE, TOF_RIGHT_SCALE, TOF_LEFT_SCALE };
static const float s_offset[SENSOR_COUNT] = { TOF_FRONT_OFFSET_MM, TOF_RIGHT_OFFSET_MM, TOF_LEFT_OFFSET_MM };

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static sensor_reading_t s_latest = { TOF_MAX_RANGE_MM, TOF_MAX_RANGE_MM, TOF_MAX_RANGE_MM, false, false, false };

bool sensor_init(void) {
    // Enable the ESP32's internal weak (~45k) pull-ups on SDA/SCL so the
    // bus has a defined idle-high level even without external pull-up
    // resistors on the sensor breakouts. Weak compared to a proper
    // external 2.2k-4.7k pull-up per line - if readings are still noisy
    // with three devices on one bus, add real resistors.
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

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
        s_sensor_ok[i] = false;
        digitalWrite(s_xshut_pin[i], HIGH);
        delay(10);

        // Cheap, bounded presence check before calling begin(): if
        // nothing ACKs at the default address, the sensor isn't
        // there/isn't powered/isn't wired right - skip it rather than
        // calling Adafruit_VL53L0X::begin(), whose internal init/
        // calibration polling loop has no timeout and can block forever.
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

        s_tof[i].startRangeContinuous(TOF_CONTINUOUS_PERIOD_MS);
        s_sensor_ok[i] = true;
        Serial.printf("[SENSOR] %s init OK (addr=0x%02X, xshut=%d, continuous=%dms)\n",
                      s_name[i], SENSOR_I2C_ADDR_BASE + i, s_xshut_pin[i], TOF_CONTINUOUS_PERIOD_MS);
    }

    return all_ok;
}

// Applies the per-sensor linear calibration equation, then the despike/
// EMA filter. Returns the most recently accepted (filtered) estimate
// even when no new sample is ready this call - *fresh tells the caller
// whether a new raw sample actually arrived.
static float poll_one(sensor_id_t id, bool *fresh, bool *in_range) {
    if (!s_sensor_ok[id]) {
        *fresh = false;
        *in_range = false;
        return s_filter[id].initialized ? s_filter[id].estimate : (float)TOF_MAX_RANGE_MM;
    }

    if (!s_tof[id].isRangeComplete()) {
        *fresh = false;
        *in_range = s_last_in_range[id];
        return s_filter[id].estimate;
    }

    uint16_t raw = s_tof[id].readRangeResult();
    float corrected = raw * s_scale[id] + s_offset[id];

    bool ok = raw < (uint16_t)TOF_MAX_RANGE_MM;
    if (!ok) {
        corrected = (float)TOF_MAX_RANGE_MM;
    }

    s_last_in_range[id] = ok;
    *fresh = true;
    *in_range = ok;
    return filter_update(&s_filter[id], corrected);
}

void sensor_poll(void) {
    bool fresh, front_ok, right_ok, left_ok;

    float front_mm = poll_one(SENSOR_FRONT, &fresh, &front_ok);
    float right_mm = poll_one(SENSOR_RIGHT, &fresh, &right_ok);
    float left_mm  = poll_one(SENSOR_LEFT,  &fresh, &left_ok);

    SYNC(s_mux) {
        s_latest.front_mm = (uint16_t)front_mm;
        s_latest.right_mm = (uint16_t)right_mm;
        s_latest.left_mm  = (uint16_t)left_mm;
        s_latest.front_in_range = front_ok;
        s_latest.right_in_range = right_ok;
        s_latest.left_in_range  = left_ok;
    }
}

void sensor_get_latest(sensor_reading_t *out) {
    SYNC(s_mux) { *out = s_latest; }
}

void sensor_get_walls(sensor_walls_t *out) {
    // Plain thresholding - no front-reliability masking here. That mask
    // (TOF_FRONT_RELIABILITY_LIMIT_MM) exists for steering.cpp's
    // continuous cross-track *correction*, where a falsely-large error
    // near a front wall would fight the upcoming turn. Wall-mapping needs
    // the opposite: a wall that's really there right next to us must
    // still get recorded, even at close range - masking it here would
    // teach the maze map a false opening.
    sensor_reading_t r;
    sensor_get_latest(&r);

    out->front = r.front_mm < TOF_WALL_THRESHOLD_FRONT_MM;
    out->left  = r.left_mm  < TOF_WALL_THRESHOLD_SIDE_MM;
    out->right = r.right_mm < TOF_WALL_THRESHOLD_SIDE_MM;
}
