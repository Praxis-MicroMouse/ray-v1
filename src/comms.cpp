#include "comms.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#include "control.h"
#include "drive.h"
#include "motor.h"
#include "sensor.h"
#include "battery.h"
#include "encoder.h"
#include "mpu9250.h"

#define COMMS_LINE_MAX 96

static char s_line_buf[COMMS_LINE_MAX];
static uint8_t s_line_len = 0;

static const char *loop_name(control_loop_id_t loop) {
    switch (loop) {
        case CONTROL_LOOP_STRAIGHT:   return "straight";
        case CONTROL_LOOP_TURN:       return "turn";
        case CONTROL_LOOP_WALLCENTER: return "wallcenter";
        default:                      return "?";
    }
}

static bool loop_from_name(const char *name, control_loop_id_t *out) {
    if (strcmp(name, "straight") == 0)   { *out = CONTROL_LOOP_STRAIGHT;   return true; }
    if (strcmp(name, "turn") == 0)       { *out = CONTROL_LOOP_TURN;       return true; }
    if (strcmp(name, "wallcenter") == 0) { *out = CONTROL_LOOP_WALLCENTER; return true; }
    return false;
}

static void send_telemetry_line(void) {
    sensor_reading_t sr = { 0, 0, 0 };
    sensor_read_all(&sr); // stale/max-range readings if sensor_init() never succeeded - still safe to print

    float batt_v = battery_read_voltage();
    float batt_pct = battery_get_percent(batt_v);

    int32_t enc_l = encoder_get_ticks(ENCODER_LEFT);
    int32_t enc_r = encoder_get_ticks(ENCODER_RIGHT);
    float dl_mm = control_ticks_to_mm(enc_l);
    float dr_mm = control_ticks_to_mm(enc_r);

    int16_t pwm_l = motor_get_speed(DRIVE_LEFT_MOTOR);
    int16_t pwm_r = motor_get_speed(DRIVE_RIGHT_MOTOR);

    mpu9250_data_t imu;
    memset(&imu, 0, sizeof(imu));
    mpu9250_read(&imu);

    control_debug_t dbg = control_get_debug();

    Serial.printf(
        "{\"t\":%lu,"
        "\"tof\":{\"f\":%u,\"r\":%u,\"l\":%u},"
        "\"batt\":{\"v\":%.2f,\"pct\":%.0f},"
        "\"enc\":{\"l\":%ld,\"r\":%ld,\"dl_mm\":%.1f,\"dr_mm\":%.1f},"
        "\"pwm\":{\"l\":%d,\"r\":%d},"
        "\"imu\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"temp\":%.1f},"
        "\"pid\":{\"loop\":\"%s\",\"sp\":%.1f,\"meas\":%.1f,\"out\":%.1f,\"active\":%s}}\n",
        (unsigned long) millis(),
        sr.front_mm, sr.right_mm, sr.left_mm,
        batt_v, batt_pct,
        (long) enc_l, (long) enc_r, dl_mm, dr_mm,
        (int) pwm_l, (int) pwm_r,
        imu.accel_g[0], imu.accel_g[1], imu.accel_g[2],
        imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2], imu.temp_c,
        loop_name(dbg.loop), dbg.setpoint, dbg.measurement, dbg.output,
        dbg.active ? "true" : "false");
}

static void handle_line(char *line) {
    char *saveptr;
    char *cmd = strtok_r(line, " ", &saveptr);
    if (!cmd) return;

    if (strcmp(cmd, "PID") == 0) {
        char *loop_str = strtok_r(NULL, " ", &saveptr);
        char *kp_str   = strtok_r(NULL, " ", &saveptr);
        char *ki_str   = strtok_r(NULL, " ", &saveptr);
        char *kd_str   = strtok_r(NULL, " ", &saveptr);
        control_loop_id_t loop;
        if (loop_str && kp_str && ki_str && kd_str && loop_from_name(loop_str, &loop)) {
            control_set_gains(loop, (float) atof(kp_str), (float) atof(ki_str), (float) atof(kd_str));
            Serial.printf("{\"ack\":\"PID\",\"loop\":\"%s\"}\n", loop_str);
        }
        return;
    }

    if (strcmp(cmd, "GETPID") == 0) {
        char *loop_str = strtok_r(NULL, " ", &saveptr);
        control_loop_id_t loop;
        if (loop_str && loop_from_name(loop_str, &loop)) {
            float kp, ki, kd;
            control_get_gains(loop, &kp, &ki, &kd);
            Serial.printf("{\"pidcfg\":{\"loop\":\"%s\",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f}}\n",
                          loop_str, kp, ki, kd);
        }
        return;
    }

    if (strcmp(cmd, "RUN") == 0) {
        char *maneuver = strtok_r(NULL, " ", &saveptr);
        char *arg_str  = strtok_r(NULL, " ", &saveptr);
        if (!maneuver) return;

        if (strcmp(maneuver, "stop") == 0) {
            control_request_abort();
            return;
        }

        float arg = arg_str ? (float) atof(arg_str) : 0.0f;
        if (strcmp(maneuver, "straight") == 0) {
            control_run_straight(arg, DRIVE_DEFAULT_SPEED, comms_tick);
        } else if (strcmp(maneuver, "turn") == 0) {
            control_run_turn(arg, DRIVE_DEFAULT_SPEED, comms_tick);
        } else if (strcmp(maneuver, "wallcenter") == 0) {
            control_run_wallcenter((uint32_t) arg, DRIVE_DEFAULT_SPEED, comms_tick);
        }
        return;
    }
}

void comms_init(void) {
    s_line_len = 0;
}

void comms_poll(void) {
    while (Serial.available() > 0) {
        char c = (char) Serial.read();
        if (c == '\n' || c == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                handle_line(s_line_buf);
                s_line_len = 0;
            }
        } else if (s_line_len < COMMS_LINE_MAX - 1) {
            s_line_buf[s_line_len++] = c;
        }
    }
}

void comms_tick(void) {
    send_telemetry_line();
    comms_poll(); // lets a "RUN stop" line abort an in-progress maneuver
}
