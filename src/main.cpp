#include <Arduino.h>
#include "sensor.h"
#include "telemetry.h"
#include "motor.h"
#include "drive.h"
#include "battery.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting...");

    if (!sensor_init()) {
        Serial.println("[MAIN] one or more ToF sensors failed to init");
    }

    battery_init();
    battery_read_voltage();

    motor_init();

    // One-shot motion test: forward, then a pivot turn each way, then
    // stop. Robot WILL move on boot — place it in a clear area before
    // powering on. Speeds/durations are untuned guesses; adjust once
    // you've seen how it actually moves.
    Serial.println("[MAIN] motion test starting...");
    drive_forward(DRIVE_DEFAULT_SPEED);
    delay(800);
    drive_stop();
    delay(500);

    drive_turn_left(DRIVE_DEFAULT_SPEED);
    delay(400);
    drive_stop();
    delay(500);

    drive_turn_right(DRIVE_DEFAULT_SPEED);
    delay(400);
    drive_stop();
    Serial.println("[MAIN] motion test done");
}

void loop() {
    sensor_reading_t reading;
    sensor_read_all(&reading);
    telemetry_send(&reading);
    battery_read_voltage();

    // High-accuracy ranging already takes ~200ms per sensor (~600ms/loop),
    // so no extra delay is needed to keep the bus/host happy.
    delay(10);
}
