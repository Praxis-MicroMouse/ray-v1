#include "battery.h"

#include <Arduino.h>

#define BATTERY_DIVIDER_RATIO \
    (BATTERY_DIVIDER_R2_OHM / (BATTERY_DIVIDER_R1_OHM + BATTERY_DIVIDER_R2_OHM))
#define BATTERY_SAMPLE_COUNT 8

void battery_init(void) {
    // Max range (~0-3.3V), needed since the divided battery voltage
    // (~1.2-1.7V for a 3.0-4.2V single cell) sits well inside it.
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    Serial.printf("[BATTERY] init OK (adc_pin=%d, divider_ratio=%.3f)\n",
                  BATTERY_ADC_PIN, BATTERY_DIVIDER_RATIO);
}

float battery_read_voltage(void) {
    uint32_t sum_mv = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        sum_mv += analogReadMilliVolts(BATTERY_ADC_PIN);
    }
    float adc_v = (sum_mv / (float)BATTERY_SAMPLE_COUNT) / 1000.0f;
    float battery_v = adc_v / BATTERY_DIVIDER_RATIO;

    Serial.printf("[BATTERY] adc=%.3fV battery=%.2fV\n", adc_v, battery_v);
    if (battery_v < BATTERY_LOW_VOLTAGE) {
        Serial.printf("[BATTERY] WARNING: low battery (%.2fV < %.2fV)\n",
                      battery_v, BATTERY_LOW_VOLTAGE);
    }

    return battery_v;
}
