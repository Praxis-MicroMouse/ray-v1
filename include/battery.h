#ifndef BATTERY_H
#define BATTERY_H

#define BATTERY_ADC_PIN 34

// Voltage divider: R1 from battery+ to the ADC pin, R2 from the ADC pin to
// GND. Vadc = Vbat * R2/(R1+R2)  =>  Vbat = Vadc * (R1+R2)/R2
#define BATTERY_DIVIDER_R1_OHM 14100.0f
#define BATTERY_DIVIDER_R2_OHM 9400.0f

// Single-cell Li-ion/LiPo nominal 3.7V; warn once discharged.
#define BATTERY_LOW_VOLTAGE 3.3f

void battery_init(void);

// Reads the ADC (averaged), converts through the divider, and returns
// battery voltage in volts. Also logs the reading, with a warning if low.
float battery_read_voltage(void);

#endif // BATTERY_H
