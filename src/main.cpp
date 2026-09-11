#include <Arduino.h>

#include "motor.h"
#include "drive.h"
#include "ota.h"
#include "telemetry.h"
#include "battery.h"

// Manual motor rotation-matching tool: both motors start at full PWM, and
// you nudge each one independently over serial while watching live
// battery voltage + cumulative rotation counts, until each motor's
// rotation count climbs at the same rate - no automatic PID/proportional
// correction involved, you're finding and hardcoding the matching PWM
// values yourself.
//
// Serial commands (single character, no newline needed):
//   1 / 2   -> motor A PWM up / down by PWM_STEP
//   3 / 4   -> motor B PWM up / down by PWM_STEP
//   0       -> stop both motors
//   r       -> resume both motors at STARTING_PWM (also resets rotation counts)
//
// Battery voltage + cumulative rotations + current PWM for both motors
// print every UPDATE_INTERVAL_MS, over Serial and (once ota_init_sta()
// joins your WiFi network - fill in wifi_credentials.h first) over UDP
// broadcast too, so you can watch the numbers untethered while the robot
// moves. Commands still come in over Serial only - USB stays plugged in
// if you need to send them.
// Robot WILL move as soon as it boots - place it in a clear area.

namespace
{

    constexpr uint8_t ENCODER_A_C1_PIN = 4;
    constexpr uint8_t ENCODER_B_C1_PIN = 16;
    constexpr float PULSES_PER_MOTOR_REVOLUTION = 7.0f;
    constexpr int16_t STARTING_PWM = 255;
    constexpr int16_t PWM_STEP = 5;
    constexpr uint32_t UPDATE_INTERVAL_MS = 500;

    volatile uint32_t pulseCountA = 0;
    volatile uint32_t pulseCountB = 0;

    int16_t pwmA = STARTING_PWM;
    int16_t pwmB = STARTING_PWM;

    void IRAM_ATTR encoderAISR()
    {
        pulseCountA++;
    }

    void IRAM_ATTR encoderBISR()
    {
        pulseCountB++;
    }

    void apply_speeds()
    {
        motor_set_speed(DRIVE_LEFT_MOTOR, (int16_t)(DRIVE_LEFT_SIGN * pwmA));
        motor_set_speed(DRIVE_RIGHT_MOTOR, (int16_t)(DRIVE_RIGHT_SIGN * pwmB));
    }

    void handle_serial_commands()
    {
        while (Serial.available())
        {
            char c = Serial.read();
            switch (c)
            {
            case '1':
                pwmA = constrain(pwmA + PWM_STEP, 0, 255);
                break;
            case '2':
                pwmA = constrain(pwmA - PWM_STEP, 0, 255);
                break;
            case '3':
                pwmB = constrain(pwmB + PWM_STEP, 0, 255);
                break;
            case '4':
                pwmB = constrain(pwmB - PWM_STEP, 0, 255);
                break;
            case '0':
                pwmA = 0;
                pwmB = 0;
                break;
            case 'r':
            case 'R':
                pwmA = STARTING_PWM;
                pwmB = STARTING_PWM;
                noInterrupts();
                pulseCountA = 0;
                pulseCountB = 0;
                interrupts();
                break;
            default:
                continue; // ignore newlines/unrecognized input
            }
            apply_speeds();
            Serial.printf("[TUNE] PWM A=%d B=%d\n", pwmA, pwmB);
        }
    }

}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (manual rotation-match tuning)...");
    ota_init_sta();
    telemetry_init();
    motor_init();
    battery_init();

    pinMode(ENCODER_A_C1_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_C1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_C1_PIN), encoderAISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_C1_PIN), encoderBISR, RISING);

    Serial.println("[TUNE] commands: 1/2 motor A pwm up/down, 3/4 motor B pwm up/down, 0 stop, r resume full PWM");
    apply_speeds();
}

void loop()
{
    ota_handle();
    handle_serial_commands();

    static uint32_t lastTime = 0;
    uint32_t now = millis();
    if (now - lastTime < UPDATE_INTERVAL_MS)
    {
        return;
    }

    noInterrupts();
    uint32_t pulsesA = pulseCountA; // cumulative since boot/last 'r' reset - not cleared here
    uint32_t pulsesB = pulseCountB;
    interrupts();

    float rotationsA = pulsesA / PULSES_PER_MOTOR_REVOLUTION;
    float rotationsB = pulsesB / PULSES_PER_MOTOR_REVOLUTION;
    float batteryVoltage = battery_read_voltage();

    char line[128];
    snprintf(line, sizeof(line), "[TUNE] BATTERY=%.2fV ROTATIONS A=%.2f B=%.2f | PWM A=%d B=%d",
             batteryVoltage, rotationsA, rotationsB, pwmA, pwmB);
    telemetry_send_line(line);

    lastTime = now;
}
