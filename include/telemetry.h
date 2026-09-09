#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "sensor.h"

// Emits one sensor reading as a machine-parseable line over Serial:
//   DATA,<millis>,<front_mm>,<right_mm>,<left_mm>
// Kept separate from sensor.cpp's [SENSOR] debug logs so a host tool
// (e.g. MATLAB) can filter for lines starting with "DATA," and ignore
// everything else on the port.
void telemetry_send(const sensor_reading_t *reading);

#endif // TELEMETRY_H
