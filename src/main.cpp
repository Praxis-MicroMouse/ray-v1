#include <Arduino.h>

#include "motor.h"
#include "drive.h"
#include "encoder.h"
#include "control.h"
#include "turns.h"

// Drives forward a fixed distance using control.h's encoder-based dual-
// wheel straight-line PID loop (control_run_straight()) - it corrects for
// left/right wheel-speed mismatch as it drives, unlike an open-loop full-
// PWM run, so it tracks straight and stops right at the target distance
// instead of just running both motors at the same PWM and hoping. Base
// speed is full PWM (255); the PID only trims the *difference* between
// the two wheels around that, so running at max doesn't remove its
// correction headroom the way it would for a fixed/uncorrected reference
// motor. Then pivots 90 degrees right using turns.h's encoder-based turn
// (no MPU9250 fitted - kept in its own file/module, separate from this
// sequencing, either way). One-shot: runs once in setup(), then loop()
// idles.
// Robot WILL move - place it in a clear 1m+ runway with turning room.

#define STRAIGHT_TARGET_MM 720.0f
#define STRAIGHT_SPEED 200 // out of 255 - full PWM base speed

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (straight + 90deg turn test)...");
    motor_init();
    encoder_init();
    control_init();
    turns_init();

    Serial.println("[MAIN] driving forward...");
    control_run_straight(STRAIGHT_TARGET_MM, STRAIGHT_SPEED, nullptr);

    control_debug_t dbg = control_get_debug();
    Serial.printf("[MAIN] straight done - measured travel %.1fmm (target %.1fmm)\n",
                  dbg.measurement, STRAIGHT_TARGET_MM);

    Serial.println("[MAIN] turning 90 degrees right...");
    turn_right_90(TURNS_DEFAULT_SPEED);
}

void loop()
{
    delay(1000); // one-shot test - nothing to do here
}
