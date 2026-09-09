#include "drive.h"
#include "motor.h"

#include <Arduino.h>

#define DRIVE_LEFT_MOTOR  MOTOR_A
#define DRIVE_RIGHT_MOTOR MOTOR_B

// If a wheel spins backward for a positive speed during testing, flip
// its sign here rather than rewiring the driver.
#define DRIVE_LEFT_SIGN   1
#define DRIVE_RIGHT_SIGN  1

void drive_forward(int16_t speed) {
    Serial.printf("[DRIVE] forward speed=%d\n", speed);
    motor_set_speed(DRIVE_LEFT_MOTOR,  (int16_t)(DRIVE_LEFT_SIGN  * speed));
    motor_set_speed(DRIVE_RIGHT_MOTOR, (int16_t)(DRIVE_RIGHT_SIGN * speed));
}

// Pivot turns: wheels spin in opposite directions in place.
void drive_turn_left(int16_t speed) {
    Serial.printf("[DRIVE] turn left speed=%d\n", speed);
    motor_set_speed(DRIVE_LEFT_MOTOR,  (int16_t)(-DRIVE_LEFT_SIGN  * speed));
    motor_set_speed(DRIVE_RIGHT_MOTOR, (int16_t)( DRIVE_RIGHT_SIGN * speed));
}

void drive_turn_right(int16_t speed) {
    Serial.printf("[DRIVE] turn right speed=%d\n", speed);
    motor_set_speed(DRIVE_LEFT_MOTOR,  (int16_t)( DRIVE_LEFT_SIGN  * speed));
    motor_set_speed(DRIVE_RIGHT_MOTOR, (int16_t)(-DRIVE_RIGHT_SIGN * speed));
}

void drive_stop(void) {
    Serial.println("[DRIVE] stop");
    motor_stop_all();
}
