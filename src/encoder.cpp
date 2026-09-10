#include "encoder.h"

#include <Arduino.h>
#include <stdint.h>

typedef struct {
    int8_t pin_a;
    int8_t pin_b;
} encoder_pins_t;

static const encoder_pins_t s_pins[ENCODER_COUNT] = {
    { ENCODER_LEFT_A_PIN,  ENCODER_LEFT_B_PIN  },
    { ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN }
};
static const char *s_name[ENCODER_COUNT] = { "LEFT", "RIGHT" };

static volatile int32_t s_ticks[ENCODER_COUNT] = { 0, 0 };

// Runs on every edge of channel A. Channel B's level at that instant
// tells us which way the wheel is turning (standard 1x quadrature
// decode - only counts A edges, so 1 tick per encoder slot/pole pair).
static void IRAM_ATTR encoder_isr(void *arg) {
    encoder_id_t id = (encoder_id_t)(intptr_t)arg;
    bool a = digitalRead(s_pins[id].pin_a);
    bool b = digitalRead(s_pins[id].pin_b);
    s_ticks[id] += (a == b) ? 1 : -1;
}

void encoder_init(void) {
    for (int i = 0; i < ENCODER_COUNT; i++) {
        const encoder_pins_t *p = &s_pins[i];
        if (p->pin_a < 0 || p->pin_b < 0) {
            Serial.printf("[ENCODER] %s not configured (pins unset) - skipping\n", s_name[i]);
            continue;
        }

        pinMode(p->pin_a, INPUT_PULLUP);
        pinMode(p->pin_b, INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(p->pin_a), encoder_isr,
                            (void *)(intptr_t)i, CHANGE);

        Serial.printf("[ENCODER] %s init OK (a=%d, b=%d)\n", s_name[i], p->pin_a, p->pin_b);
    }
}

int32_t encoder_get_ticks(encoder_id_t encoder) {
    noInterrupts();
    int32_t ticks = s_ticks[encoder];
    interrupts();
    return ticks;
}

void encoder_reset(encoder_id_t encoder) {
    noInterrupts();
    s_ticks[encoder] = 0;
    interrupts();
}
