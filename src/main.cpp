#include <Arduino.h>

#include "motor.h"
#include "encoder.h"
#include "button.h"
#include "sensor.h"
#include "battery.h"
#include "mouse.h"
#include "bench.h"
#include "tasks.h"

// Build-time run mode - flip the #define below and reflash. See
// TUNING.md for the full tuning procedure each bench mode belongs to.
//
//   RUN_MODE_MAZE (default)      - the real thing: full search + speed run.
//   RUN_MODE_BRINGUP             - motors off; streams ToF/encoder/battery
//                                   readings so you can sanity-check the
//                                   sensors before trusting the mouse to
//                                   drive on them. (TUNING.md Step 1)
//   RUN_MODE_FEEDFORWARD_LEFT/RIGHT - open-loop volts-vs-speed sweep on
//                                   one wheel, for fitting FF_*_KV/BIAS.
//                                   (TUNING.md Step 4)
//   RUN_MODE_STRAIGHT_TEST        - repeated straight moves, reports
//                                   commanded vs. odometry-measured
//                                   distance. (TUNING.md Steps 3 & 5)
//   RUN_MODE_TURN_TEST            - repeated 360-degree spins, reports
//                                   commanded vs. odometry-measured
//                                   angle. (TUNING.md Steps 3 & 6)
//   RUN_MODE_WALLCENTER_TEST      - drives a corridor with steering
//                                   engaged, reports cross-track error.
//                                   (TUNING.md Step 7)
#define RUN_MODE_MAZE               1
#define RUN_MODE_BRINGUP            2
#define RUN_MODE_FEEDFORWARD_LEFT   3
#define RUN_MODE_FEEDFORWARD_RIGHT  4
#define RUN_MODE_STRAIGHT_TEST      5
#define RUN_MODE_TURN_TEST          6
#define RUN_MODE_WALLCENTER_TEST    7

#define RUN_MODE RUN_MODE_MAZE

#if RUN_MODE == RUN_MODE_MAZE

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[MAIN] booting (maze run)...");

    motor_init();
    encoder_init();
    button_init();

    mouse_init();
    tasks_start(mouse_run);
}

void loop() {
    delay(1000); // the whole run happens inside tasks.cpp's three tasks
}

#elif RUN_MODE == RUN_MODE_BRINGUP

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[MAIN] booting (bringup check - motors idle)...");

    encoder_init();
    button_init();
    battery_init();
    sensor_init();
}

void loop() {
    sensor_poll();
    battery_update();

    sensor_reading_t r;
    sensor_get_latest(&r);

    Serial.printf("[BRINGUP] front=%u(%d) right=%u(%d) left=%u(%d) mm  encL=%ld encR=%ld  batt=%.2fV\n",
                  r.front_mm, (int)r.front_in_range,
                  r.right_mm, (int)r.right_in_range,
                  r.left_mm, (int)r.left_in_range,
                  (long)encoder_get_ticks(ENCODER_LEFT),
                  (long)encoder_get_ticks(ENCODER_RIGHT),
                  battery_voltage());
    delay(200);
}

#elif RUN_MODE == RUN_MODE_FEEDFORWARD_LEFT || RUN_MODE == RUN_MODE_FEEDFORWARD_RIGHT

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[MAIN] booting (feedforward characterization - STANDALONE, no tasks_start())...");
    Serial.println("[MAIN] give the mouse a long clear runway - it WILL drive at increasing speed");

    motor_init();
    encoder_init();
    battery_init();

#if RUN_MODE == RUN_MODE_FEEDFORWARD_LEFT
    bench_feedforward(MOTOR_LEFT);
#else
    bench_feedforward(MOTOR_RIGHT);
#endif
}

void loop() {
    delay(1000);
}

#elif RUN_MODE == RUN_MODE_STRAIGHT_TEST

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[MAIN] booting (straight test)...");

    motor_init();
    encoder_init();
    button_init();

    tasks_start(bench_straight_test);
}

void loop() {
    delay(1000);
}

#elif RUN_MODE == RUN_MODE_TURN_TEST

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[MAIN] booting (turn test)...");

    motor_init();
    encoder_init();
    button_init();

    tasks_start(bench_turn_test);
}

void loop() {
    delay(1000);
}

#elif RUN_MODE == RUN_MODE_WALLCENTER_TEST

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[MAIN] booting (wall-center test)...");

    motor_init();
    encoder_init();
    button_init();

    tasks_start(bench_wallcenter_test);
}

void loop() {
    delay(1000);
}

#else
#error "Unknown RUN_MODE"
#endif
