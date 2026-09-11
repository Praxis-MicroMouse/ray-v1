#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stddef.h>

#include "sensor.h"

// UDP port telemetry broadcasts to once telemetry_init() has run - see
// ota.h for the AP/station link that makes this reachable.
#define TELEMETRY_UDP_PORT 4210

// Starts the UDP socket used to broadcast telemetry to whatever laptop is
// on the same link as the ESP32 (either connected to its OTA access point,
// or joined to the same WiFi network via ota_init_sta()), in addition to
// Serial. Call once from setup(), after ota_init()/ota_init_sta(). Safe to
// skip - telemetry_send()/telemetry_send_line() still work over Serial
// alone if this is never called.
void telemetry_init(void);

// Emits one sensor reading as a machine-parseable line:
//   DATA,<millis>,<front_mm>,<right_mm>,<left_mm>
// over Serial always, and over UDP broadcast (TELEMETRY_UDP_PORT) if
// telemetry_init() has run and a laptop is reachable. Kept separate from
// sensor.cpp's [SENSOR] debug logs so a host tool (e.g. MATLAB) can filter
// for lines starting with "DATA," and ignore everything else.
void telemetry_send(const sensor_reading_t *reading);

// Same idea as telemetry_send(), but for any pre-formatted line instead of
// a sensor_reading_t specifically - e.g. main.cpp's motor RPM/PWM tuning
// printout. Emits over Serial always, and over UDP broadcast if
// telemetry_init() has run and a laptop is reachable. `line` need not be
// newline-terminated; one is appended before sending.
void telemetry_send_line(const char *line);

#endif // TELEMETRY_H
