#include <Arduino.h>

#include "drive.h"
#include "sensor.h"

// Simple obstacle-avoidance test: drive forward, stop if anything is
// within OBSTACLE_THRESHOLD_MM ahead, then steer away from whichever
// side (left/right) sees the obstacle closer, and resume forward.

#define OBSTACLE_THRESHOLD_MM 50 // 5cm
#define DRIVE_SPEED 50           // PWM out of 255 - kept low since we're running off 3.7V
#define TURN_DURATION_MS 400     // how long to pivot away before re-checking

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] obstacle-avoidance test starting...");
    motor_init();
    sensor_init();
}

void loop()
{
    sensor_reading_t reading;
    sensor_read_all(&reading);

    if (reading.front_mm > OBSTACLE_THRESHOLD_MM)
    {
        drive_forward(DRIVE_SPEED);
        return;
    }

    Serial.println("[MAIN] obstacle ahead - stopping");
    drive_stop();

    if (reading.left_mm < reading.right_mm)
    {
        Serial.println("[MAIN] obstacle closer on left - turning right");
        drive_turn_right(DRIVE_SPEED);
    }
    else
    {
        Serial.println("[MAIN] obstacle closer on right - turning left");
        drive_turn_left(DRIVE_SPEED);
    }

    delay(TURN_DURATION_MS);
    drive_stop();
}
