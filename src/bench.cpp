#include "bench.h"

#include <Arduino.h>

#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "motion.h"
#include "odometry.h"
#include "steering.h"
#include "sensor.h"
#include "config/config.h"

#define BENCH_FF_STEP_VOLTS  0.5f
#define BENCH_FF_SETTLE_MS   500
#define BENCH_FF_MEASURE_MS  500

void bench_feedforward(motor_id_t motor) {
    encoder_id_t enc = (motor == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT;
    float mm_per_tick = (motor == MOTOR_LEFT) ? MM_PER_TICK_LEFT : MM_PER_TICK_RIGHT;

    Serial.println("[BENCH] feedforward characterization");
    Serial.println("[BENCH] volts,speed_mm_s");

    for (float v = BENCH_FF_STEP_VOLTS; v <= MOTOR_MAX_VOLTS + 0.001f; v += BENCH_FF_STEP_VOLTS) {
        battery_update(); // keep voltage compensation reasonably fresh as the pack sags under load
        motor_set_volts(motor, v);
        delay(BENCH_FF_SETTLE_MS); // let it reach steady speed before measuring

        int32_t t0 = encoder_get_ticks(enc);
        uint32_t start_ms = millis();
        delay(BENCH_FF_MEASURE_MS);
        int32_t t1 = encoder_get_ticks(enc);
        uint32_t elapsed_ms = millis() - start_ms;

        float mm = (t1 - t0) * mm_per_tick;
        float speed_mm_s = mm / (elapsed_ms / 1000.0f);
        Serial.printf("[BENCH] %.2f,%.1f\n", v, speed_mm_s);
    }

    motor_set_pwm(motor, 0);
    Serial.println("[BENCH] feedforward characterization done");
}

#define BENCH_STRAIGHT_DISTANCE_MM 720.0f
#define BENCH_STRAIGHT_REPEATS     4

void bench_straight_test(void) {
    Serial.println("[BENCH] straight test - place the mouse with room to move ~720mm each way");
    Serial.println("[BENCH] commanded_mm,measured_mm,error_mm");

    for (int i = 0; i < BENCH_STRAIGHT_REPEATS; i++) {
        motion_reset_drive_system();
        steering_set_mode(STEERING_OFF);

        float sign = (i % 2 == 0) ? 1.0f : -1.0f;
        float commanded = sign * BENCH_STRAIGHT_DISTANCE_MM;
        motion_move(commanded, (float)SEARCH_SPEED_MM_S, 0.0f, SEARCH_ACCEL_MM_S2);

        float measured = odometry_distance_snapshot_mm();
        Serial.printf("[BENCH] %.1f,%.1f,%.1f\n", commanded, measured, measured - commanded);
        delay(500);
    }

    motion_disable_drive();
    Serial.println("[BENCH] straight test done");
}

#define BENCH_TURN_REPEATS 4

void bench_turn_test(void) {
    Serial.println("[BENCH] turn test - one 360-degree spin per line");
    Serial.println("[BENCH] commanded_deg,measured_deg,error_deg");

    motion_reset_drive_system();
    steering_set_mode(STEERING_OFF);

    float commanded = 0.0f;
    for (int i = 0; i < BENCH_TURN_REPEATS; i++) {
        motion_spin_turn(360.0f, SPIN_TURN_OMEGA_DEG_S, SPIN_TURN_ALPHA_DEG_S2);
        commanded += 360.0f;

        float measured = odometry_angle_snapshot_deg();
        Serial.printf("[BENCH] %.1f,%.1f,%.1f\n", commanded, measured, measured - commanded);
        delay(500);
    }

    motion_disable_drive();
    Serial.println("[BENCH] turn test done");
}

#define BENCH_WALLCENTER_DISTANCE_MM 900.0f // ~5 cells - needs a straight walled corridor this long
#define BENCH_WALLCENTER_LOG_MS      100

void bench_wallcenter_test(void) {
    Serial.println("[BENCH] wall-center test - place in a straight corridor with walls both sides");
    Serial.println("[BENCH] pos_mm,left_mm,right_mm,steer_adjust_deg_s");

    motion_reset_drive_system();
    steering_set_mode(STEERING_NORMAL);
    motion_start_move(BENCH_WALLCENTER_DISTANCE_MM, (float)SEARCH_SPEED_MM_S, 0.0f, SEARCH_ACCEL_MM_S2);

    uint32_t last_log_ms = millis();
    while (!motion_move_finished()) {
        if (millis() - last_log_ms >= BENCH_WALLCENTER_LOG_MS) {
            last_log_ms = millis();
            sensor_reading_t r;
            sensor_get_latest(&r);
            Serial.printf("[BENCH] %.0f,%u,%u,%.1f\n",
                          motion_position(), r.left_mm, r.right_mm, steering_get_adjustment());
        }
        delay(5);
    }

    steering_set_mode(STEERING_OFF);
    motion_disable_drive();
    Serial.println("[BENCH] wall-center test done");
}
