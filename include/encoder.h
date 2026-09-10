#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Quadrature (2-channel A/B) wheel encoders, decoded via pin-change
// interrupts on each channel-A pin. Not wired up yet, so every pin below
// is left at -1 ("unset") — encoder_init() logs and skips any encoder
// whose pins aren't set instead of touching undefined hardware, so the
// rest of the firmware keeps working while the encoders are still being
// mounted/tuned.
//
// Suggested pins (avoid strapping pins 0/2/12/15 and pins already used by
// motor.h/sensor.h/battery.h):
//   Left  A -> GPIO 4     Left  B -> GPIO 16
//   Right A -> GPIO 17    Right B -> GPIO 23
// All four are regular digital GPIOs (interrupt-capable, internal
// pull-ups available) free on a standard esp32dev DevKit. Note: on a
// WROVER module (with PSRAM) GPIO16/17 are reserved for PSRAM — swap
// those two for another free pair (e.g. GPIO 15/2) in that case.
//
// Fill in the four defines below once wired, e.g.:
//   #define ENCODER_LEFT_A_PIN  4
#define ENCODER_LEFT_A_PIN 4   // suggested: GPIO 4
#define ENCODER_LEFT_B_PIN 16  // suggested: GPIO 16
#define ENCODER_RIGHT_A_PIN 17 // suggested: GPIO 17
#define ENCODER_RIGHT_B_PIN 23 // suggested: GPIO 23

// Motor-shaft encoder spec (GA12-N20 magnetic Hall encoder datasheet):
// pulses per motor-shaft revolution, on ONE channel. encoder_isr() below
// fires on every CHANGE of channel A (both rising and falling edges),
// so one motor-shaft revolution works out to 2x this many ticks.
#define ENCODER_PULSES_PER_MOTOR_REV 7
#define ENCODER_TICKS_PER_MOTOR_REV  (2 * ENCODER_PULSES_PER_MOTOR_REV)

// Gearbox reduction ratio between the motor shaft (where the encoder
// actually is - NOT the output/wheel shaft) and the output shaft, e.g.
// 30.0f for a 1:30 GA12-N20. TODO: set this to the ratio printed on/
// listed for the motors actually in use - until then,
// encoder_get_output_rpm() and control.h's derived ticks-per-wheel-
// revolution are both wrong by whatever the real ratio turns out to be.
#define ENCODER_GEARBOX_RATIO 1.0f

typedef enum
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_COUNT
} encoder_id_t;

// Configures pins (INPUT_PULLUP) and attaches a CHANGE interrupt on
// channel A for every encoder whose pins are set. Encoders left at -1
// are logged as "not configured" and skipped.
void encoder_init(void);

// Signed tick count accumulated since boot or the last encoder_reset().
// Positive = forward rotation, given channel A/B aren't swapped -
// swap them in wiring (or the pin defines above) if a wheel counts
// backward while driving forward.
int32_t encoder_get_ticks(encoder_id_t encoder);

void encoder_reset(encoder_id_t encoder);

// Motor-shaft speed, in RPM, averaged over the time since this encoder's
// last encoder_get_motor_rpm() call (or since encoder_init(), whichever
// is more recent) - so call it on a regular cadence (e.g. once per
// telemetry tick); the window it measures is simply "since last call".
float encoder_get_motor_rpm(encoder_id_t encoder);

// Output/wheel-shaft speed, in RPM: encoder_get_motor_rpm() divided by
// ENCODER_GEARBOX_RATIO. Only as accurate as that ratio is set.
float encoder_get_output_rpm(encoder_id_t encoder);

#endif // ENCODER_H
