#include "drive_controller.h"

#include "pd.h"
#include "motor.h"
#include "motion.h"
#include "odometry.h"
#include "steering.h"
#include "config/motion_tuning.h"
#include "config/robot_physical.h"

static pd_ctrl_t s_pd_forward;
static pd_ctrl_t s_pd_rotation;

static float s_rot_error_accum = 0.0f;
static float s_prev_left_speed = 0.0f;
static float s_prev_right_speed = 0.0f;

static bool s_enabled = false;
static bool s_feedforward_enabled = true;

void drive_controller_init(void) {
    pd_init(&s_pd_forward, PD_FORWARD_KP, PD_FORWARD_KD);
    pd_init(&s_pd_rotation, PD_ROTATION_KP, PD_ROTATION_KD);
    drive_controller_reset();
    s_enabled = false;
}

void drive_controller_enable(void) {
    s_enabled = true;
}

void drive_controller_disable(void) {
    s_enabled = false;
    motor_stop_all();
}

void drive_controller_reset(void) {
    pd_reset(&s_pd_forward);
    pd_reset(&s_pd_rotation);
    s_rot_error_accum = 0.0f;
    s_prev_left_speed = 0.0f;
    s_prev_right_speed = 0.0f;
}

// voltage = speed*Kv + sign(speed)*bias + accel*Ka
static float feedforward(float speed_mm_s, float accel_mm_s2, float kv, float bias, float ka) {
    float ff = speed_mm_s * kv;
    if (speed_mm_s > 0.0f) {
        ff += bias;
    } else if (speed_mm_s < 0.0f) {
        ff -= bias;
    }
    ff += accel_mm_s2 * ka;
    return ff;
}

void drive_controller_update(void) {
    if (!s_enabled) {
        motor_stop_all();
        return;
    }

    const float dt_s = CONTROL_LOOP_INTERVAL_S;

    // ---- Forward: target vs. actual are both running totals, so a
    // plain setpoint-minus-measurement PD reproduces the same tracking
    // ukmarsbots gets from its incremental error accumulator. ----
    float pos_output = pd_update(&s_pd_forward, motion_position(), odometry_robot_distance_mm(), dt_s);

    // ---- Rotation: the target here is a moving SUM of the rotation
    // profile's commanded rate (0 while just driving straight) plus
    // steering's continuous small correction - not a single clean
    // profile position - so it needs the incremental accumulator form,
    // same as ukmarsbots' angle_controller(). ----
    float target_omega = motion_omega() + steering_get_adjustment();
    s_rot_error_accum += target_omega * dt_s - odometry_rot_change_deg();
    float rot_output = pd_update_error(&s_pd_rotation, s_rot_error_accum, dt_s);

    float left_output = pos_output - rot_output;
    float right_output = pos_output + rot_output;

    if (s_feedforward_enabled) {
        // Feedforward uses the PURE profile omega (no steering) - steering
        // only ever acts through the closed-loop PD term above, so it
        // can't get amplified by being fed through feedforward too.
        float tangent_speed = motion_omega() * TURN_RADIUS_MM * RADIANS_PER_DEGREE;
        float left_speed = motion_velocity() - tangent_speed;
        float right_speed = motion_velocity() + tangent_speed;

        float left_accel = (left_speed - s_prev_left_speed) / dt_s;
        float right_accel = (right_speed - s_prev_right_speed) / dt_s;
        s_prev_left_speed = left_speed;
        s_prev_right_speed = right_speed;

        left_output += feedforward(left_speed, left_accel, FF_LEFT_KV, FF_LEFT_BIAS, FF_LEFT_KA);
        right_output += feedforward(right_speed, right_accel, FF_RIGHT_KV, FF_RIGHT_BIAS, FF_RIGHT_KA);
    }

    motor_set_volts(MOTOR_LEFT, left_output);
    motor_set_volts(MOTOR_RIGHT, right_output);
}

void drive_controller_set_forward_gains(float kp, float kd) {
    pd_set_gains(&s_pd_forward, kp, kd);
}

void drive_controller_get_forward_gains(float *kp, float *kd) {
    pd_get_gains(&s_pd_forward, kp, kd);
}

void drive_controller_set_rotation_gains(float kp, float kd) {
    pd_set_gains(&s_pd_rotation, kp, kd);
}

void drive_controller_get_rotation_gains(float *kp, float *kd) {
    pd_get_gains(&s_pd_rotation, kp, kd);
}
