#include "motor.h"

#include <Arduino.h>
#include <stdlib.h>

#include "battery.h"
#include "config/pins.h"
#include "config/robot_physical.h"
#include "config/motion_tuning.h"

#define MOTOR_PWM_FREQ_HZ         20000  // above audible range
#define MOTOR_PWM_RESOLUTION_BITS 8

// LEDC PWM channels dedicated to each motor's speed pin.
#define MOTOR_LEFT_PWM_CHANNEL  0
#define MOTOR_RIGHT_PWM_CHANNEL 1

typedef struct {
    uint8_t pwm_pin;
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t pwm_channel;
    int8_t  polarity;
} motor_pins_t;

static const motor_pins_t s_motor[MOTOR_COUNT] = {
    { PIN_MOTOR_LEFT_PWM,  PIN_MOTOR_LEFT_IN1,  PIN_MOTOR_LEFT_IN2,  MOTOR_LEFT_PWM_CHANNEL,  MOTOR_LEFT_POLARITY },
    { PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_IN1, PIN_MOTOR_RIGHT_IN2, MOTOR_RIGHT_PWM_CHANNEL, MOTOR_RIGHT_POLARITY },
};
static const char *s_name[MOTOR_COUNT] = { "LEFT", "RIGHT" };
static int16_t s_last_pwm[MOTOR_COUNT] = { 0, 0 };
static float s_last_volts[MOTOR_COUNT] = { 0.0f, 0.0f };

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

void motor_set_pwm(motor_id_t motor, int16_t pwm) {
    if (pwm > MOTOR_MAX_PWM) pwm = MOTOR_MAX_PWM;
    if (pwm < -MOTOR_MAX_PWM) pwm = -MOTOR_MAX_PWM;

    const motor_pins_t *m = &s_motor[motor];
    int16_t signed_pwm = m->polarity * pwm;

    if (signed_pwm > 0) {
        digitalWrite(m->in1_pin, HIGH);
        digitalWrite(m->in2_pin, LOW);
    } else if (signed_pwm < 0) {
        digitalWrite(m->in1_pin, LOW);
        digitalWrite(m->in2_pin, HIGH);
    } else {
        digitalWrite(m->in1_pin, LOW);
        digitalWrite(m->in2_pin, LOW);
    }

    ledcWrite(m->pwm_channel, (uint32_t)abs(signed_pwm));
    s_last_pwm[motor] = pwm;
}

void motor_set_volts(motor_id_t motor, float volts) {
    if (volts > MOTOR_MAX_VOLTS) volts = MOTOR_MAX_VOLTS;
    if (volts < -MOTOR_MAX_VOLTS) volts = -MOTOR_MAX_VOLTS;
    s_last_volts[motor] = volts;

    float battery_v = battery_voltage();
    if (battery_v < 1.0f) battery_v = 1.0f; // guard against div-by-~0 before the first real reading

    int pwm = (int)(MOTOR_MAX_PWM * volts / battery_v);
    motor_set_pwm(motor, (int16_t)pwm);
}

int16_t motor_get_pwm(motor_id_t motor) {
    return s_last_pwm[motor];
}

float motor_get_volts(motor_id_t motor) {
    return s_last_volts[motor];
}

void motor_stop_all(void) {
    motor_set_pwm(MOTOR_LEFT, 0);
    motor_set_pwm(MOTOR_RIGHT, 0);
}

void motor_brake(motor_id_t motor) {
    const motor_pins_t *m = &s_motor[motor];

    digitalWrite(m->in1_pin, HIGH);
    digitalWrite(m->in2_pin, HIGH);
    ledcWrite(m->pwm_channel, MOTOR_MAX_PWM);
    s_last_pwm[motor] = 0;
    s_last_volts[motor] = 0.0f;
}

void motor_brake_all(void) {
    motor_brake(MOTOR_LEFT);
    motor_brake(MOTOR_RIGHT);
}
