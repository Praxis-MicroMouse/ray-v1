#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

// ===========================================================================
// EVERY GPIO ASSIGNMENT ON THE ROBOT - ONE PLACE.
// If you rewire something, change it here only; nothing else in the
// codebase should hardcode a pin number.
// ===========================================================================

// ---- Motor driver (TB6612FNG-style: PWM + 2 direction pins per motor) ----
// STBY is assumed tied high in hardware (not software controlled here).
#define PIN_MOTOR_LEFT_PWM  26
#define PIN_MOTOR_LEFT_IN1  33
#define PIN_MOTOR_LEFT_IN2  25

#define PIN_MOTOR_RIGHT_PWM 27
#define PIN_MOTOR_RIGHT_IN1 14
#define PIN_MOTOR_RIGHT_IN2 32

// ---- Quadrature wheel encoders (2-channel A/B, decoded on A's CHANGE) ----
#define PIN_ENCODER_LEFT_A  4
#define PIN_ENCODER_LEFT_B  16
#define PIN_ENCODER_RIGHT_A 17
#define PIN_ENCODER_RIGHT_B 23

// ---- ToF (VL53L0X) sensors: shared I2C bus + one XSHUT pin each ----
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define PIN_TOF_XSHUT_FRONT 18
#define PIN_TOF_XSHUT_RIGHT 19
#define PIN_TOF_XSHUT_LEFT  5

// ---- Battery voltage divider ----
#define PIN_BATTERY_ADC 34

// ---- User button: abort a run / start a bench routine ----
// Set to -1 if not wired - button.cpp then degrades to "never pressed"
// instead of touching undefined hardware, same pattern encoder.cpp
// already used for unwired encoder pins.
#define PIN_BUTTON -1

#endif // CONFIG_PINS_H
