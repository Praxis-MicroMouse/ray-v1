#include "battery.h"

#include <Arduino.h>

#include "sync.h"
#include "config/pins.h"
#include "config/motion_tuning.h"

#define BATTERY_DIVIDER_RATIO \
    (BATTERY_DIVIDER_R2_OHM / (BATTERY_DIVIDER_R1_OHM + BATTERY_DIVIDER_R2_OHM))
#define BATTERY_SAMPLE_COUNT 8

// Only log on a meaningful change (or periodically) - battery_update()
// runs at SENSOR_LOOP_HZ, far too often to printf every call.
#define BATTERY_LOG_INTERVAL_MS 2000

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_cached_voltage = MOTOR_NOMINAL_BATTERY_VOLTS;
static uint32_t s_last_log_ms = 0;

void battery_init(void) {
    // Max range (~0-3.3V), needed since the divided battery voltage
    // (~1.2-1.7V for a 3.0-4.2V single cell) sits well inside it.
    analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);
    SYNC(s_mux) { s_cached_voltage = MOTOR_NOMINAL_BATTERY_VOLTS; }
    Serial.printf("[BATTERY] init OK (adc_pin=%d, divider_ratio=%.3f)\n",
                  PIN_BATTERY_ADC, BATTERY_DIVIDER_RATIO);
}

void battery_update(void) {
    uint32_t sum_mv = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        sum_mv += analogReadMilliVolts(PIN_BATTERY_ADC);
    }
    float adc_v = (sum_mv / (float)BATTERY_SAMPLE_COUNT) / 1000.0f;
    float battery_v = adc_v / BATTERY_DIVIDER_RATIO;

    SYNC(s_mux) { s_cached_voltage = battery_v; }

    uint32_t now = millis();
    if (now - s_last_log_ms >= BATTERY_LOG_INTERVAL_MS) {
        s_last_log_ms = now;
        Serial.printf("[BATTERY] adc=%.3fV battery=%.2fV\n", adc_v, battery_v);
        if (battery_v < BATTERY_LOW_VOLTAGE) {
            Serial.printf("[BATTERY] WARNING: low battery (%.2fV < %.2fV)\n",
                          battery_v, BATTERY_LOW_VOLTAGE);
        }
    }
}

float battery_voltage(void) {
    float v;
    SYNC(s_mux) { v = s_cached_voltage; }
    return v;
}

float battery_get_percent(float voltage) {
    // (voltage, percent) breakpoints, highest first - a typical 1S LiPo
    // rest-voltage discharge curve, coarsely approximated.
    static const float v_pts[] = { 4.20f, 4.00f, 3.85f, 3.70f, 3.50f, 3.30f };
    static const float p_pts[] = { 100.0f, 80.0f, 60.0f, 40.0f, 15.0f, 0.0f };
    const int n = sizeof(v_pts) / sizeof(v_pts[0]);

    if (voltage >= v_pts[0]) return 100.0f;
    if (voltage <= v_pts[n - 1]) return 0.0f;

    for (int i = 0; i < n - 1; i++) {
        if (voltage <= v_pts[i] && voltage >= v_pts[i + 1]) {
            float t = (voltage - v_pts[i + 1]) / (v_pts[i] - v_pts[i + 1]);
            return p_pts[i + 1] + t * (p_pts[i] - p_pts[i + 1]);
        }
    }
    return 0.0f;
}
