#include "motion.h"

#include <Arduino.h>

#include "profile.h"
#include "odometry.h"
#include "drive_controller.h"
#include "motor.h"
#include "button.h"
#include "sync.h"
#include "config/motion_tuning.h"

static profile_t s_forward;
static profile_t s_rotation;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void motion_init(void) {
    SYNC(s_mux) {
        profile_reset(&s_forward);
        profile_reset(&s_rotation);
    }
}

void motion_reset_drive_system(void) {
    drive_controller_disable();
    odometry_reset();
    SYNC(s_mux) {
        profile_reset(&s_forward);
        profile_reset(&s_rotation);
    }
    drive_controller_reset();
    drive_controller_enable();
}

void motion_stop(void) {
    SYNC(s_mux) {
        profile_set_target_speed(&s_forward, 0.0f);
        profile_set_target_speed(&s_rotation, 0.0f);
    }
}

void motion_disable_drive(void) {
    drive_controller_disable();
}

void motion_emergency_stop(void) {
    // Hard stop first (short-brake, not coast) - this is the panic path,
    // don't let momentum carry the robot any further than it has to.
    motor_brake_all();
    motion_reset_drive_system();
    motion_disable_drive();
}

void motion_update(void) {
    SYNC(s_mux) {
        profile_update(&s_forward, CONTROL_LOOP_INTERVAL_S);
        profile_update(&s_rotation, CONTROL_LOOP_INTERVAL_S);
    }
}

float motion_position(void) {
    float v;
    SYNC(s_mux) { v = profile_position(&s_forward); }
    return v;
}

float motion_velocity(void) {
    float v;
    SYNC(s_mux) { v = profile_speed(&s_forward); }
    return v;
}

float motion_acceleration(void) {
    float v;
    SYNC(s_mux) { v = profile_acceleration(&s_forward); }
    return v;
}

float motion_angle(void) {
    float v;
    SYNC(s_mux) { v = profile_position(&s_rotation); }
    return v;
}

float motion_omega(void) {
    float v;
    SYNC(s_mux) { v = profile_speed(&s_rotation); }
    return v;
}

float motion_alpha(void) {
    float v;
    SYNC(s_mux) { v = profile_acceleration(&s_rotation); }
    return v;
}

void motion_set_target_velocity(float velocity_mm_s) {
    SYNC(s_mux) { profile_set_target_speed(&s_forward, velocity_mm_s); }
}

void motion_start_move(float distance_mm, float top_speed, float final_speed, float acceleration) {
    SYNC(s_mux) { profile_start(&s_forward, distance_mm, top_speed, final_speed, acceleration); }
}

bool motion_move_finished(void) {
    bool v;
    SYNC(s_mux) { v = profile_is_finished(&s_forward); }
    return v;
}

void motion_move(float distance_mm, float top_speed, float final_speed, float acceleration) {
    motion_start_move(distance_mm, top_speed, final_speed, acceleration);
    while (!motion_move_finished()) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        delay(2);
    }
}

void motion_start_turn(float angle_deg, float top_speed, float final_speed, float acceleration) {
    SYNC(s_mux) { profile_start(&s_rotation, angle_deg, top_speed, final_speed, acceleration); }
}

bool motion_turn_finished(void) {
    bool v;
    SYNC(s_mux) { v = profile_is_finished(&s_rotation); }
    return v;
}

void motion_turn(float angle_deg, float top_speed, float final_speed, float acceleration) {
    motion_start_turn(angle_deg, top_speed, final_speed, acceleration);
    while (!motion_turn_finished()) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        delay(2);
    }
}

void motion_spin_turn(float angle_deg, float omega_deg_s, float alpha_deg_s2) {
    motion_set_target_velocity(0.0f);
    while (motion_velocity() != 0.0f) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        delay(2);
    }
    SYNC(s_mux) { profile_reset(&s_rotation); }
    motion_turn(angle_deg, omega_deg_s, 0.0f, alpha_deg_s2);
}

void motion_set_position(float position_mm) {
    SYNC(s_mux) { profile_set_position(&s_forward, position_mm); }
}

void motion_adjust_forward_position(float delta_mm) {
    SYNC(s_mux) { profile_adjust_position(&s_forward, delta_mm); }
}

void motion_wait_until_position(float position_mm) {
    while (motion_position() < position_mm) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        delay(2);
    }
}

void motion_wait_until_distance(float distance_mm) {
    motion_wait_until_position(motion_position() + distance_mm);
}

void motion_stop_at(float position_mm) {
    float remaining = position_mm - motion_position();
    motion_move(remaining, motion_velocity(), 0.0f, motion_acceleration());
}

void motion_stop_after(float distance_mm) {
    motion_move(distance_mm, motion_velocity(), 0.0f, motion_acceleration());
}
