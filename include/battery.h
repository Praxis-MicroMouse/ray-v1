#ifndef BATTERY_H
#define BATTERY_H

// Voltage divider: R1 from battery+ to the ADC pin, R2 from the ADC pin
// to GND. Vadc = Vbat * R2/(R1+R2)  =>  Vbat = Vadc * (R1+R2)/R2
#define BATTERY_DIVIDER_R1_OHM 14100.0f
#define BATTERY_DIVIDER_R2_OHM 9400.0f

// Single-cell Li-ion/LiPo nominal 3.7V; warn once discharged.
#define BATTERY_LOW_VOLTAGE 3.3f

void battery_init(void);

// Reads the ADC (averaged), converts through the divider, logs the
// result (with a warning if low), and updates the cached value returned
// by battery_voltage(). Call periodically (SENSOR_LOOP_HZ is plenty -
// battery voltage doesn't change fast) - NOT from the fast control loop;
// this does a multi-sample ADC read plus a Serial.printf and would blow
// the control loop's timing budget at CONTROL_LOOP_HZ.
void battery_update(void);

// Cross-core-safe cached read of the last battery_update() result, in
// volts. Cheap enough to call every control-loop tick (motor.cpp does,
// for battery-compensated PWM). Returns MOTOR_NOMINAL_BATTERY_VOLTS
// (config/motion_tuning.h) until the first battery_update() call.
float battery_voltage(void);

// Rough 1S LiPo state-of-charge estimate (0-100) from a piecewise-linear
// discharge curve. Good enough for a "roughly how much is left" readout,
// not a calibrated fuel gauge - LiPo voltage sag under load will make
// this read a bit low while driving.
float battery_get_percent(float voltage);

#endif // BATTERY_H
