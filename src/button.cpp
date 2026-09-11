#include "button.h"

#include <Arduino.h>

#include "config/pins.h"

void button_init(void) {
    if (PIN_BUTTON < 0) {
        Serial.println("[BUTTON] not configured (PIN_BUTTON unset) - abort-by-button disabled");
        return;
    }
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    Serial.printf("[BUTTON] init OK (pin=%d)\n", PIN_BUTTON);
}

bool button_pressed(void) {
    if (PIN_BUTTON < 0) {
        return false;
    }
    return digitalRead(PIN_BUTTON) == LOW;
}

void button_wait_for_release(void) {
    if (PIN_BUTTON < 0) {
        return;
    }
    while (button_pressed()) {
        delay(10);
    }
}
