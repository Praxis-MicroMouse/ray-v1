#include "sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "filter.h"
#include "sync.h"
#include "config/pins.h"
#include "config/sensor_calibration.h"

// Every sensor uses the default address - only one is ever powered on at
// a time (see service_one_sensor()), so there's nothing to reassign and
// nothing that could collide.
#define SENSOR_ADDR 0x29

static Adafruit_VL53L0X s_tof; // ONE driver instance, re-begin()'d against whichever physical unit is currently powered
static filter_t s_filter[SENSOR_COUNT]; // despike + smooth each channel's readings - see filter.h
static bool s_sensor_ok[SENSOR_COUNT] = { false, false, false };
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

static int s_next_index = 0; // round-robin cursor for sensor_poll()

// Powers on exactly one physical sensor, re-initializes the shared
// VL53L0X driver against it, takes one measurement, then powers it back
// off before returning - every other sensor's XSHUT must already be LOW
// when this is called. begin() has to run every single time, not just
// once at boot: pulling XSHUT low is a hardware reset, so the sensor's
// configuration (VCSEL periods, timing budget, ...) is wiped every time
// it's powered off and has to be redone on every power-up.
static bool service_one_sensor(sensor_id_t id, float *out_mm, bool *out_in_range) {
    digitalWrite(s_xshut_pin[id], HIGH);
    delay(TOF_XSHUT_BOOT_DELAY_MS);

    // Cheap, bounded presence check before calling begin(): if nothing
    // ACKs at the default address, the sensor isn't there/isn't
    // powered/isn't wired right - bail out rather than calling
    // Adafruit_VL53L0X::begin(), whose internal init/calibration polling
    // loop has no timeout and can block forever on an unresponsive sensor.
    Wire.beginTransmission(SENSOR_ADDR);
    uint8_t probe_err = Wire.endTransmission();
    if (probe_err != 0) {
        digitalWrite(s_xshut_pin[id], LOW);
        return false;
    }

    // High-accuracy profile: longer timing budget, tighter VCSEL periods
    // -> lower noise, which matters most at short (0-10cm) range.
    if (!s_tof.begin(SENSOR_ADDR, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY)) {
        digitalWrite(s_xshut_pin[id], LOW);
        return false;
    }

    VL53L0X_RangingMeasurementData_t m;
    s_tof.rangingTest(&m, false);
    digitalWrite(s_xshut_pin[id], LOW); // power off before returning, success or not - never two sensors live at once

    if (m.RangeStatus == 4) {
        *out_mm = (float) TOF_MAX_RANGE_MM;
        *out_in_range = false;
    } else {
        *out_mm = (float) m.RangeMilliMeter;
        *out_in_range = true;
    }
    return true;
}

bool sensor_init(void) {
    // Enable the ESP32's internal weak (~45k) pull-ups on SDA/SCL so the
    // bus has a defined idle-high level even without external pull-up
    // resistors on the sensor breakouts. Weak compared to a proper
    // external 2.2k-4.7k pull-up per line - if a sensor is intermittently
    // failing to respond, add real resistors before suspecting anything else.
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // Hold every sensor in reset (XSHUT low) first - exactly one is ever
    // brought out of reset at a time, by service_one_sensor().
    for (int i = 0; i < SENSOR_COUNT; i++) {
        pinMode(s_xshut_pin[i], OUTPUT);
        digitalWrite(s_xshut_pin[i], LOW);
        filter_init(&s_filter[i]);
    }
    delay(10);

    bool all_ok = true;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        float mm;
        bool in_range;
        bool ok = service_one_sensor((sensor_id_t)i, &mm, &in_range);
        s_sensor_ok[i] = ok;

        if (ok) {
            float corrected = mm * s_scale[i] + s_offset[i];
            filter_update(&s_filter[i], corrected); // prime it so sensor_get_latest() is sane immediately
            Serial.printf("[SENSOR] %s init OK (xshut=%d)\n", s_name[i], s_xshut_pin[i]);
        } else {
            all_ok = false;
            Serial.printf("[SENSOR] %s init FAILED (xshut=%d)\n", s_name[i], s_xshut_pin[i]);
        }
    }

    s_next_index = 0;
    return all_ok;
}

// Services exactly ONE physical sensor per call, round-robin - NOT all
// three. Each call blocks for roughly one full power-on/measure/power-off
// cycle (boot delay + begin() + a ranging measurement), so a given
// sensor's reading is only refreshed once every SENSOR_COUNT calls to
// this function - noticeably staler than continuous-ranging concurrent
// operation would give, in exchange for only ever having one device live
// on the bus at a time. steering.cpp measures real elapsed time between
// its own updates rather than assuming a fixed call rate, precisely
// because of this.
void sensor_poll(void) {
    sensor_id_t id = (sensor_id_t) s_next_index;
    s_next_index = (s_next_index + 1) % SENSOR_COUNT;

    if (!s_sensor_ok[id]) {
        return; // never came up at init - don't retry a dead unit forever
    }

    float mm;
    bool in_range;
    if (!service_one_sensor(id, &mm, &in_range)) {
        return; // a previously-good sensor stopped responding - keep its last estimate rather than snapping to "out of range"
    }

    float corrected = mm * s_scale[id] + s_offset[id];
    float filtered = filter_update(&s_filter[id], corrected);

    SYNC(s_mux) {
        switch (id) {
            case SENSOR_FRONT: s_latest.front_mm = (uint16_t)filtered; s_latest.front_in_range = in_range; break;
            case SENSOR_RIGHT: s_latest.right_mm = (uint16_t)filtered; s_latest.right_in_range = in_range; break;
            case SENSOR_LEFT:  s_latest.left_mm  = (uint16_t)filtered; s_latest.left_in_range  = in_range; break;
            default: break;
        }
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
