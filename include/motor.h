#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// TB6612FNG-style dual motor driver: one PWM (speed) pin + two direction
// pins per motor (config/pins.h). STBY is assumed tied high in hardware
// (not software controlled here).

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
} motor_id_t;

// Configures direction pins and attaches PWM to both motors' speed pins.
// Motors start stopped.
void motor_init(void);

// speed: -MOTOR_MAX_PWM (full reverse) .. 0 (stop) .. MOTOR_MAX_PWM (full
// forward) (config/motion_tuning.h). Out-of-range values are clamped.
// Applies config/robot_physical.h's MOTOR_*_POLARITY. speed=0 coasts
// (IN1/IN2 both low - outputs go high-impedance) - use motor_brake()/
// motor_brake_all() instead when momentum carrying the robot past a
// target is the problem.
void motor_set_pwm(motor_id_t motor, int16_t pwm);

// Voltage-domain command: clamps to +-MOTOR_MAX_VOLTS, then converts to
// a PWM duty compensated for the current battery voltage (battery.h) so
// a given `volts` produces roughly the same wheel speed regardless of
// how discharged the battery is - see drive_controller.cpp, the only
// expected caller during normal (closed-loop) operation.
void motor_set_volts(motor_id_t motor, float volts);

// Last value actually applied via motor_set_pwm()/motor_set_volts()
// (post-clamp), for telemetry/display purposes.
int16_t motor_get_pwm(motor_id_t motor);
float motor_get_volts(motor_id_t motor);

void motor_stop_all(void); // coast - see motor_set_pwm()'s speed=0 note

// Short-brake (TB6612FNG: IN1=IN2=HIGH) - shorts the motor's terminals so
// its own back-EMF resists the remaining spin, stopping it much faster
// than coasting. Safe to hold indefinitely (it's not a stall condition -
// no drive current, just the motor's own kinetic energy dissipating).
void motor_brake(motor_id_t motor);
void motor_brake_all(void);

#endif // MOTOR_H
