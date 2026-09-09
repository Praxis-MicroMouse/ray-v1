#include "motor.h"

#include <Arduino.h>
#include <stdlib.h>

#define MOTOR_PWM_FREQ_HZ         20000  // above audible range
#define MOTOR_PWM_RESOLUTION_BITS 8
#define MOTOR_PWM_MAX              255

// LEDC PWM channels dedicated to each motor's speed pin.
#define MOTOR_A_PWM_CHANNEL 0
#define MOTOR_B_PWM_CHANNEL 1

typedef struct {
    uint8_t pwm_pin;
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t pwm_channel;
} motor_pins_t;

static const motor_pins_t s_motor[MOTOR_COUNT] = {
    { MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM_CHANNEL },
    { MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM_CHANNEL }
};
static const char *s_name[MOTOR_COUNT] = { "A", "B" };

void motor_init(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pinMode(s_motor[i].in1_pin, OUTPUT);
        pinMode(s_motor[i].in2_pin, OUTPUT);
        digitalWrite(s_motor[i].in1_pin, LOW);
        digitalWrite(s_motor[i].in2_pin, LOW);

        ledcSetup(s_motor[i].pwm_channel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
        ledcAttachPin(s_motor[i].pwm_pin, s_motor[i].pwm_channel);
        ledcWrite(s_motor[i].pwm_channel, 0);

        Serial.printf("[MOTOR] %s init OK (pwm=%d, in1=%d, in2=%d)\n",
                      s_name[i], s_motor[i].pwm_pin,
                      s_motor[i].in1_pin, s_motor[i].in2_pin);
    }
}

void motor_set_speed(motor_id_t motor, int16_t speed) {
    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    const motor_pins_t *m = &s_motor[motor];

    if (speed > 0) {
        digitalWrite(m->in1_pin, HIGH);
        digitalWrite(m->in2_pin, LOW);
    } else if (speed < 0) {
        digitalWrite(m->in1_pin, LOW);
        digitalWrite(m->in2_pin, HIGH);
    } else {
        digitalWrite(m->in1_pin, LOW);
        digitalWrite(m->in2_pin, LOW);
    }

    ledcWrite(m->pwm_channel, (uint32_t)abs(speed));

    Serial.printf("[MOTOR] %s speed=%d\n", s_name[motor], speed);
}

void motor_stop_all(void) {
    motor_set_speed(MOTOR_A, 0);
    motor_set_speed(MOTOR_B, 0);
}
