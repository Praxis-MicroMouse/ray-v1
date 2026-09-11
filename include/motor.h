#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// TB6612FNG-style dual motor driver: one PWM (speed) pin + two direction
// pins per motor. STBY is assumed tied high in hardware (not software
// controlled here).

#define MOTOR_A_PWM 26
#define MOTOR_A_IN1 33
#define MOTOR_A_IN2 25

#define MOTOR_B_PWM 27
#define MOTOR_B_IN1 14
#define MOTOR_B_IN2 32

typedef enum {
    MOTOR_A = 0,
    MOTOR_B,
    MOTOR_COUNT
} motor_id_t;

// Configures direction pins and attaches PWM to both motors' speed pins.
// Motors start stopped.
void motor_init(void);

// speed: -255 (full reverse) .. 0 (stop) .. 255 (full forward).
// Out-of-range values are clamped. speed=0 coasts (IN1/IN2 both low -
// outputs go high-impedance, momentum just spins the motor down on its
// own) - use motor_brake()/motor_brake_all() instead when momentum
// carrying the robot past a target is the problem.
void motor_set_speed(motor_id_t motor, int16_t speed);

// Last speed actually applied via motor_set_speed() (post-clamp), for
// telemetry/display purposes. 0 until the first motor_set_speed() call.
int16_t motor_get_speed(motor_id_t motor);

void motor_stop_all(void); // coast - see motor_set_speed()'s speed=0 note

// Short-brake (TB6612FNG: IN1=IN2=HIGH) - shorts the motor's terminals so
// its own back-EMF resists the remaining spin, stopping it much faster
// than coasting. Safe to hold indefinitely (it's not a stall condition -
// no drive current, just the motor's own kinetic energy dissipating).
void motor_brake(motor_id_t motor);
void motor_brake_all(void);

#endif // MOTOR_H
