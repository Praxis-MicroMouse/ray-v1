#ifndef CONFIG_MOTION_TUNING_H
#define CONFIG_MOTION_TUNING_H

// ===========================================================================
// EVERY PD GAIN, FEEDFORWARD CONSTANT, SPEED AND ACCELERATION IN THE
// MOTION SYSTEM. This is the file to open when tuning how the robot
// drives. Every PD controller in the codebase (forward position,
// rotation/heading, and cross-track steering) gets its Kp/Kd from here -
// there is exactly one generic PD implementation (pd.h/pd.cpp); nothing
// hardcodes a gain anywhere else.
//
// All values below marked "PLACEHOLDER" are untuned starting guesses -
// they will not make the robot drive well as-is. Tune in this order:
//   1. Feedforward (FF_*) - open-loop, controller disabled.
//   2. Forward PD (PD_FORWARD_*) - closed-loop straight-line tracking.
//   3. Rotation PD (PD_ROTATION_*) - closed-loop turning.
//   4. Steering PD (PD_STEERING_*) - closed-loop wall-centering.
// Tuning an earlier stage well makes every later stage much easier - a
// good feedforward means the PD terms only have to correct small errors,
// not do all the work against motor stiction.
// ===========================================================================

// ---- Control loop timing ----
#define CONTROL_LOOP_HZ          500
#define CONTROL_LOOP_INTERVAL_S  (1.0f / CONTROL_LOOP_HZ)

// How often (Hz) the slower sensor/steering loop polls the ToF sensors
// and recomputes the steering correction. Independent of CONTROL_LOOP_HZ
// because I2C reads can't keep up with a 500Hz motor loop - see
// sensor_loop.h.
#define SENSOR_LOOP_HZ 100

// ---- Motor electrical limits ----
#define MOTOR_MAX_PWM               255
#define MOTOR_MAX_VOLTS             6.0f  // clamp on any single commanded motor voltage
#define MOTOR_NOMINAL_BATTERY_VOLTS 3.7f  // used as battery.cpp's cache value before the first real ADC read

// ---- Per-wheel feedforward ----
// voltage = speed_mm_s * FF_KV + sign(speed_mm_s) * FF_BIAS + accel_mm_s2 * FF_KA
//
// Determine FF_<SIDE>_KV / FF_<SIDE>_BIAS per wheel:
//   1. Disable the PD controller (drive_controller_disable()) so only
//      feedforward is driving the motor.
//   2. Command a fixed voltage (motor_set_volts) from 0.5V up to
//      MOTOR_MAX_VOLTS in 0.5V steps; at each step let the wheel reach
//      steady speed and log it from odometry.
//   3. Plot steady-state speed (mm/s) against commanded voltage. The
//      slope is FF_KV (well, 1/slope - FF_KV is Volts per mm/s). The
//      x-intercept (voltage needed before the wheel starts moving at
//      all - stiction) is FF_BIAS.
// FF_KA (accel feedforward) can stay at 0 initially; the PD term covers
// small transient errors well enough without it, tune it later if you
// want tighter step response.
#define FF_LEFT_KV    0.010f // PLACEHOLDER - Volts per mm/s
#define FF_LEFT_BIAS  0.35f  // PLACEHOLDER - Volts
#define FF_LEFT_KA    0.0f   // PLACEHOLDER - Volts per mm/s/s

#define FF_RIGHT_KV   0.010f // PLACEHOLDER
#define FF_RIGHT_BIAS 0.35f  // PLACEHOLDER
#define FF_RIGHT_KA   0.0f   // PLACEHOLDER

// ---- PD gains: one Kp/Kd pair per controller instance ----
// Forward: tracks motion.cpp's forward profile position (mm) against
// odometry's measured distance travelled (mm).
#define PD_FORWARD_KP 0.35f // PLACEHOLDER
#define PD_FORWARD_KD 3.0f  // PLACEHOLDER

// Rotation: tracks (profile angular rate + steering rate) integrated
// against odometry's measured heading change (deg).
#define PD_ROTATION_KP 0.28f // PLACEHOLDER
#define PD_ROTATION_KD 2.2f  // PLACEHOLDER

// Steering: continuous cross-track correction from the left/right ToF
// sensors while driving straight. Output is a heading RATE (deg/s) fed
// into the rotation controller's target - see steering.h.
#define PD_STEERING_KP 0.9f // PLACEHOLDER
#define PD_STEERING_KD 4.0f // PLACEHOLDER
#define STEERING_ADJUST_LIMIT_DEG_S 90.0f

// ---- Speed / acceleration profile (mm/s, mm/s/s, deg/s, deg/s/s) ----
#define SEARCH_SPEED_MM_S       300
#define SEARCH_ACCEL_MM_S2      1500
#define SEARCH_TURN_SPEED_MM_S  250

#define SPIN_TURN_OMEGA_DEG_S    360
#define SPIN_TURN_ALPHA_DEG_S2   1800

// Speed run (post-exploration, full map trusted) - can be pushed higher
// than search speed once the PD/FF above is solid.
#define FAST_RUN_SPEED_MM_S   500
#define FAST_RUN_ACCEL_MM_S2  2000

#endif // CONFIG_MOTION_TUNING_H
