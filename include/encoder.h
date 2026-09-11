#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Quadrature (2-channel A/B) wheel encoders, decoded via pin-change
// interrupts on each channel-A pin. Pins live in config/pins.h
// (PIN_ENCODER_*) - encoder_init() logs and skips any encoder whose A
// pin is left at -1 instead of touching undefined hardware.

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_COUNT
} encoder_id_t;

// Configures pins (INPUT_PULLUP) and attaches a CHANGE interrupt on
// channel A for every encoder whose pins are set.
void encoder_init(void);

// Signed tick count accumulated since boot or the last encoder_reset().
// Positive = forward rotation, given channel A/B aren't swapped - swap
// them in wiring, or flip the corresponding sign in
// config/robot_physical.h (ENCODER_LEFT_POLARITY/ENCODER_RIGHT_POLARITY),
// if a wheel counts backward while driving forward.
//
// NOTE: odometry.cpp is the only expected caller during normal operation
// - it tracks its own "ticks since last update" baseline rather than
// using encoder_reset(), so a maneuver's timing doesn't race against
// anything else that might also want to reset. Only call encoder_reset()
// for standalone bench tests (with odometry not running).
int32_t encoder_get_ticks(encoder_id_t encoder);

void encoder_reset(encoder_id_t encoder);

#endif // ENCODER_H
