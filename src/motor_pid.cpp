#include "motor_pid.h"

#include <Arduino.h>

#include "motor.h"

namespace
{

    constexpr uint8_t ENCODER_A_C1_PIN = 4;
    constexpr uint8_t ENCODER_B_C1_PIN = 23;
    constexpr float PULSES_PER_MOTOR_REVOLUTION = 7.0f;
    constexpr float RPM_MATCH_KP = 0.5f;
    constexpr int16_t REFERENCE_MOTOR_PWM = 170;
    constexpr int16_t MINIMUM_RUNNING_PWM = 150;

    volatile uint32_t pulseCountA = 0;
    volatile uint32_t pulseCountB = 0;

    float latestRpmA = 0.0f;
    float latestRpmB = 0.0f;
    int16_t latestPwmA = REFERENCE_MOTOR_PWM;
    int16_t latestPwmB = REFERENCE_MOTOR_PWM;

    void IRAM_ATTR encoderAISR()
    {
        pulseCountA++;
    }

    void IRAM_ATTR encoderBISR()
    {
        pulseCountB++;
    }

}

void motor_pid_init(void)
{
    motor_init();
    motor_set_speed(MOTOR_A, REFERENCE_MOTOR_PWM);
    motor_set_speed(MOTOR_B, REFERENCE_MOTOR_PWM);

    pinMode(ENCODER_A_C1_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_C1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_C1_PIN), encoderAISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_C1_PIN), encoderBISR, RISING);
}

void motor_pid_update(void)
{
    static uint32_t lastTime = 0;
    uint32_t now = millis();

    if (now - lastTime < 1000)
    {
        return;
    }

    noInterrupts();
    uint32_t pulsesA = pulseCountA;
    uint32_t pulsesB = pulseCountB;
    pulseCountA = 0;
    pulseCountB = 0;
    interrupts();

    float motorRPM_A = (pulsesA / PULSES_PER_MOTOR_REVOLUTION) * 60.0f;
    float motorRPM_B = (pulsesB / PULSES_PER_MOTOR_REVOLUTION) * 60.0f;

    int16_t speedB = motor_get_speed(MOTOR_B);
    speedB += (int16_t)(RPM_MATCH_KP * (motorRPM_A - motorRPM_B));
    speedB = constrain(speedB, MINIMUM_RUNNING_PWM, 255);
    motor_set_speed(MOTOR_B, speedB);

    latestRpmA = motorRPM_A;
    latestRpmB = motorRPM_B;
    latestPwmA = REFERENCE_MOTOR_PWM;
    latestPwmB = speedB;

    lastTime = now;
}

float motor_pid_get_rpm(motor_id_t motor)
{
    return motor == MOTOR_A ? latestRpmA : latestRpmB;
}

int16_t motor_pid_get_pwm(motor_id_t motor)
{
    return motor == MOTOR_A ? latestPwmA : latestPwmB;
}