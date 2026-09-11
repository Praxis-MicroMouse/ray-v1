#ifndef SYNC_H
#define SYNC_H

#include <freertos/FreeRTOS.h>

// ===========================================================================
// Cross-core-safe critical section.
//
// mazerunner-core (single-core AVR) guards shared ISR/foreground state with
// an ATOMIC{} macro built on noInterrupts()/interrupts(). That is NOT
// sufficient here: the ESP32 is dual-core, and disabling interrupts on one
// core does nothing to stop the *other* core writing the same variable at
// the same time. portMUX_TYPE is the ESP32-correct equivalent - a real
// spinlock both cores respect.
//
// Usage - one portMUX_TYPE per independently-protected piece of state:
//   static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
//   float get_thing() {
//       float v;
//       SYNC(s_mux) { v = s_thing; }
//       return v;
//   }
//
// Keep the protected block short (a handful of assignments) - it disables
// interrupts on the calling core for its duration.
// ===========================================================================
#define SYNC(mux) \
    for (int _sync_once_ = (portENTER_CRITICAL(&(mux)), 0); _sync_once_ < 1; \
         _sync_once_++, portEXIT_CRITICAL(&(mux)))

#endif // SYNC_H
