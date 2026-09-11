#include "odometry.h"

#include "encoder.h"
#include "sync.h"
#include "config/robot_physical.h"

static int32_t s_last_left_ticks = 0;
static int32_t s_last_right_ticks = 0;

static float s_robot_distance_mm = 0.0f;
static float s_robot_angle_deg = 0.0f;
static float s_fwd_change_mm = 0.0f;
static float s_rot_change_deg = 0.0f;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_distance_snapshot_mm = 0.0f;
static float s_angle_snapshot_deg = 0.0f;

void odometry_init(void) {
    odometry_reset();
}

void odometry_reset(void) {
    s_last_left_ticks = encoder_get_ticks(ENCODER_LEFT);
    s_last_right_ticks = encoder_get_ticks(ENCODER_RIGHT);
    s_robot_distance_mm = 0.0f;
    s_robot_angle_deg = 0.0f;
    s_fwd_change_mm = 0.0f;
    s_rot_change_deg = 0.0f;
    SYNC(s_mux) {
        s_distance_snapshot_mm = 0.0f;
        s_angle_snapshot_deg = 0.0f;
    }
}

void odometry_update(void) {
    int32_t left_ticks = encoder_get_ticks(ENCODER_LEFT);
    int32_t right_ticks = encoder_get_ticks(ENCODER_RIGHT);

    int32_t left_delta = (left_ticks - s_last_left_ticks) * ENCODER_LEFT_POLARITY;
    int32_t right_delta = (right_ticks - s_last_right_ticks) * ENCODER_RIGHT_POLARITY;
    s_last_left_ticks = left_ticks;
    s_last_right_ticks = right_ticks;

    float left_change_mm = left_delta * MM_PER_TICK_LEFT;
    float right_change_mm = right_delta * MM_PER_TICK_RIGHT;

    s_fwd_change_mm = 0.5f * (left_change_mm + right_change_mm);
    s_robot_distance_mm += s_fwd_change_mm;

    s_rot_change_deg = (right_change_mm - left_change_mm) * DEG_PER_MM_DIFFERENCE;
    s_robot_angle_deg += s_rot_change_deg;

    SYNC(s_mux) {
        s_distance_snapshot_mm = s_robot_distance_mm;
        s_angle_snapshot_deg = s_robot_angle_deg;
    }
}

float odometry_robot_distance_mm(void) {
    return s_robot_distance_mm;
}

float odometry_robot_angle_deg(void) {
    return s_robot_angle_deg;
}

float odometry_fwd_change_mm(void) {
    return s_fwd_change_mm;
}

float odometry_rot_change_deg(void) {
    return s_rot_change_deg;
}

float odometry_robot_speed_mm_s(float dt_s) {
    return (dt_s > 0.0f) ? (s_fwd_change_mm / dt_s) : 0.0f;
}

float odometry_robot_omega_deg_s(float dt_s) {
    return (dt_s > 0.0f) ? (s_rot_change_deg / dt_s) : 0.0f;
}

float odometry_distance_snapshot_mm(void) {
    float v;
    SYNC(s_mux) { v = s_distance_snapshot_mm; }
    return v;
}

float odometry_angle_snapshot_deg(void) {
    float v;
    SYNC(s_mux) { v = s_angle_snapshot_deg; }
    return v;
}
