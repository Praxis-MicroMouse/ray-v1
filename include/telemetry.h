#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "sensor.h"

// UDP port telemetry_send() broadcasts DATA, lines to once telemetry_init()
// has run - see ota.h for the access point that makes this reachable.
#define TELEMETRY_UDP_PORT 4210

// Starts the UDP socket used to broadcast telemetry to whatever laptop is
// connected to the ESP32's OTA access point, in addition to Serial. Call
// once from setup(), after ota_init(). Safe to skip - telemetry_send()
// still works over Serial alone if this is never called.
void telemetry_init(void);

// Emits one sensor reading as a machine-parseable line:
//   DATA,<millis>,<front_mm>,<right_mm>,<left_mm>
// over Serial always, and over UDP broadcast (TELEMETRY_UDP_PORT) if
// telemetry_init() has run and a laptop is connected to the AP. Kept
// separate from sensor.cpp's [SENSOR] debug logs so a host tool (e.g.
// MATLAB) can filter for lines starting with "DATA," and ignore
// everything else.
void telemetry_send(const sensor_reading_t *reading);

#endif // TELEMETRY_H
