#include <Arduino.h>

#include "motor.h"
#include "drive.h"
#include "encoder.h"
#include "sensor.h"
#include "control.h"

// Drives forward 2m through a maze corridor using control.h's ToF wall-
// centering PID loop (control_run_straight_centered()) to keep the robot
// balanced between the left/right walls as it drives - unlike a pure
// encoder-synced straight run (control_run_straight()), which only keeps
// the two wheels' speeds matched to each other and drifts sideways if
// the two motors/wheels aren't perfectly identical, this steers off the
// actual left/right wall distances so it tracks the corridor's
// centerline instead. Encoder ticks (not the ToF sensors) are still
// what measures the 2m distance and stops the run. One-shot: runs once
// in setup(), then loop() idles.
// Robot WILL move - place it in a maze corridor (or between two parallel
// walls) at least 2m long, with walls on both sides for the ToF sensors
// to center against.

#define RUN_TARGET_MM 2000.0f
#define RUN_SPEED 50 // out of 255

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (2m sensor-balanced run)...");
    motor_init();
    encoder_init();
    control_init();
    if (!sensor_init())
    {
        Serial.println("[MAIN] WARNING - one or more ToF sensors failed to init; centering may misbehave");
    }

    Serial.println("[MAIN] driving 2m, balanced between the walls...");
    control_run_straight_centered(RUN_TARGET_MM, RUN_SPEED, nullptr);

    control_debug_t dbg = control_get_debug();
    Serial.printf("[MAIN] run done - measured travel %.1fmm (target %.1fmm)\n",
                  dbg.measurement, RUN_TARGET_MM);
}

void loop()
{
    delay(1000); // one-shot test - nothing to do here
}
