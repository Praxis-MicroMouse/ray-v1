#include <Arduino.h>
#include "motor.h"
#include "drive.h"
#include "encoder.h"
#include "mpu9250.h"
// #include "sensor.h"     // uncomment to bring the ToF sensors back in
// #include "telemetry.h"  // uncomment along with sensor.h
// #include "battery.h"    // uncomment to bring battery monitoring back in
// #include "maze.h"       // uncomment along with solver.h, to run the maze solver
// #include "solver.h"     // uncomment (needs sensor.h too - the solver senses walls)

// Motors + encoders + IMU bring-up build. ToF sensor/battery/telemetry/
// maze-solver modules are untouched in the tree but not wired into main
// below — the calls are left commented out at each site; uncomment the
// matching lines (and the #includes above) to bring a module back in.
//
// Encoder pins are still unset (see encoder.h) so encoder_init() will log
// "not configured" and skip them until they're wired up; encoder_get_ticks()
// stays at 0 until then. MPU9250 is wired per the suggested pins in
// mpu9250.h - if it's not physically connected yet, mpu9250_init() will
// fail and this build logs that instead of reading it every loop.

static bool s_mpu_ready = false;
// static bool s_sensors_ready = false;  // uncomment with sensor.h

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (motors + encoders + IMU)...");
    motor_init();
    encoder_init();
    s_mpu_ready = mpu9250_init();

    // s_sensors_ready = sensor_init();  // uncomment with sensor.h
    // battery_init();                  // uncomment with battery.h
}

static void print_encoder_ticks(void) {
    Serial.printf("[MAIN] encoder ticks L=%ld R=%ld\n",
                  (long)encoder_get_ticks(ENCODER_LEFT),
                  (long)encoder_get_ticks(ENCODER_RIGHT));
}

void loop() {
    // Uncomment to run the full maze-solving algorithm (search to
    // center, then back to start) instead of the canned motion test
    // below - needs sensor.h/maze.h/solver.h included above and
    // s_sensors_ready wired in setup(). Comment out the motion test
    // block that follows when enabling this, since both drive the
    // motors and would otherwise fight each other:
    // if (s_sensors_ready) {
    //     solver_run();
    // }
    // for (;;) { delay(1000); }  // halt once solved - solver_run() blocks until done

    // Robot WILL move — place it in a clear area. Speeds/durations are
    // untuned guesses; adjust once you've seen how it actually moves.
    Serial.println("[MAIN] motion test starting...");
    drive_forward(DRIVE_DEFAULT_SPEED);
    delay(800);
    drive_stop();
    print_encoder_ticks();
    delay(500);

    drive_turn_left(DRIVE_DEFAULT_SPEED);
    delay(400);
    drive_stop();
    print_encoder_ticks();
    delay(500);

    drive_turn_right(DRIVE_DEFAULT_SPEED);
    delay(400);
    drive_stop();
    print_encoder_ticks();
    Serial.println("[MAIN] motion test done");

    if (s_mpu_ready) {
        mpu9250_data_t imu;
        mpu9250_read(&imu);
    }

    // Uncomment to read all three ToF sensors and stream them out over
    // telemetry (needs sensor.h/telemetry.h included above and
    // s_sensors_ready declared):
    // if (s_sensors_ready) {
    //     sensor_reading_t reading;
    //     sensor_read_all(&reading);
    //     telemetry_send(&reading);
    // }

    // Uncomment to log battery voltage each loop (needs battery.h included
    // above):
    // battery_read_voltage();

    delay(2000);  // pause before repeating
}
