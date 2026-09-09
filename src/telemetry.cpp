#include "telemetry.h"

#include <Arduino.h>

void telemetry_send(const sensor_reading_t *reading) {
    Serial.printf("DATA,%lu,%u,%u,%u\n",
                  (unsigned long)millis(),
                  reading->front_mm,
                  reading->right_mm,
                  reading->left_mm);
}
