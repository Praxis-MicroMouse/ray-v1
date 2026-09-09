#include <Arduino.h>
#include "sensor.h"
#include "telemetry.h"

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
    telemetry_send(&reading);

    // High-accuracy ranging already takes ~200ms per sensor (~600ms/loop),
    // so no extra delay is needed to keep the bus/host happy.
    delay(10);
}
