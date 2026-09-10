#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include "motor.h"

// Open-loop wheel-speed sync: motor A is held at a fixed reference PWM,
// and motor B's PWM is nudged (proportional control) each update so its
// pulse rate matches A's - keeps both wheels turning at the same RPM
// despite motor-to-motor variance, without needing a target/setpoint of
// its own (there's no RUN command - this just runs continuously once
// started).
void motor_pid_init(void);
void motor_pid_update(void);

// Latest computed motor-shaft RPM / currently-applied PWM for either
// motor, for telemetry - both update once per motor_pid_update() cycle
// (every ~1s), not on every call.
float motor_pid_get_rpm(motor_id_t motor);
int16_t motor_pid_get_pwm(motor_id_t motor);

#endif // MOTOR_PID_H