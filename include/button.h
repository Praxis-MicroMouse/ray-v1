#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>

// Single push-button, active-low (wired to GND, INPUT_PULLUP idles high).
// Used to abort an in-progress run from any blocking wait loop (see
// motion.h's wait_until_*()/move()/turn()) and, optionally, to hold the
// robot in mouse_panic()'s blink loop until acknowledged - mirrors
// ukmarsbots' switches.button_pressed()/wait_for_button_release(), minus
// the resistor-ladder function-switch decoding (out of scope for now;
// this is just a single GO/STOP button).
//
// If config/pins.h's PIN_BUTTON is left at -1 (not wired), every function
// here degrades to "never pressed" rather than touching undefined
// hardware - the rest of the firmware works the same either way, just
// without a way to abort a run early.

void button_init(void);

bool button_pressed(void);

// Busy-waits (with a small delay) until the button is released, then
// returns. No-op if the button isn't wired or isn't currently pressed.
void button_wait_for_release(void);

#endif // BUTTON_H
