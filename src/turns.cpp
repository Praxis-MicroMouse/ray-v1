#include "turns.h"

#include <Arduino.h>
#include <math.h>

#include "motor.h"
#include "drive.h"
#include "encoder.h"
#include "control.h"

bool turns_init(void)
{
    return true; // no MPU9250 fitted - nothing to bring up
}

void turn_degrees(float degrees, int16_t speed)
{
    float target_arc_mm = fabsf(degrees) * (float)M_PI / 180.0f * (TURNS_WHEEL_TRACK_MM / 2.0f);
    bool turning_right = degrees >= 0.0f;

    Serial.printf("[TURNS] turning %.1f degrees (target arc %.1fmm/wheel) at speed=%d\n",
                  degrees, target_arc_mm, speed);

    encoder_reset(ENCODER_LEFT);
    encoder_reset(ENCODER_RIGHT);

    int16_t left_speed = turning_right ? speed : -speed;
    int16_t right_speed = turning_right ? -speed : speed;
    motor_set_speed(DRIVE_LEFT_MOTOR, (int16_t)(DRIVE_LEFT_SIGN * left_speed));
    motor_set_speed(DRIVE_RIGHT_MOTOR, (int16_t)(DRIVE_RIGHT_SIGN * right_speed));

    uint32_t start_ms = millis();
    float measured_arc_mm = 0.0f;
    while (true)
    {
        if (millis() - start_ms > TURNS_MAX_RUN_MS)
        {
            Serial.println("[TURNS] safety time ceiling hit before reaching target");
            break;
        }

        float left_mm = fabsf(control_ticks_to_mm(encoder_get_ticks(ENCODER_LEFT)));
        float right_mm = fabsf(control_ticks_to_mm(encoder_get_ticks(ENCODER_RIGHT)));
        measured_arc_mm = (left_mm + right_mm) / 2.0f;

        if (measured_arc_mm >= target_arc_mm)
        {
            break;
        }
        delay(1);
    }

    drive_stop();

    Serial.printf("[TURNS] done - measured arc %.1fmm/wheel (target %.1fmm)\n",
                  measured_arc_mm, target_arc_mm);
}

void turn_right_90(int16_t speed)
{
    turn_degrees(90.0f, speed);
}

void turn_left_90(int16_t speed)
{
    turn_degrees(-90.0f, speed);
}
