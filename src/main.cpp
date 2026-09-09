#include <Arduino.h>
#include "sensor.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting...");

    if (!sensor_init()) {
        Serial.println("[MAIN] one or more ToF sensors failed to init");
    }
}

void loop() {
    sensor_reading_t reading;
    sensor_read_all(&reading);

    delay(200);
}
