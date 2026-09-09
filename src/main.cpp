#include <Arduino.h>
#include "motor.h"
#include "drive.h"

// Motors-only build for isolated testing (sensor/battery/telemetry
// modules are untouched in the tree — just not called from here).

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (motors only)...");
    motor_init();
}

void loop() {
    // Robot WILL move — place it in a clear area. Speeds/durations are
    // untuned guesses; adjust once you've seen how it actually moves.
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

    delay(2000);  // pause before repeating
}
