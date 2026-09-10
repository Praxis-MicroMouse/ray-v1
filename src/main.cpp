#include <Arduino.h>

#include "motor.h"
#include "drive.h"
#include "sensor.h"
#include "telemetry.h"
#include "battery.h"
#include "encoder.h"
#include "mpu9250.h"
#include "motor_pid.h"
#include "control.h"
#include "comms.h"
#include "maze.h"
#include "solver.h"
#include "tasks.h"

// ============================================================================
// BUILD MODE SELECT
//
// Uncomment exactly ONE of the MODE_* defines below and leave the rest
// commented out - each one is a complete, independent setup()/loop() pair
// further down this file, wrapped in "#if defined(MODE_...)". The #if/#error
// block right after this list won't compile if zero or more than one is
// active, so a stray uncomment can't silently build the wrong thing.
//
// All module headers are included above unconditionally (they're cheap/
// include-guarded) so switching modes never needs an #include edited too -
// only the one line below.
// ============================================================================

// #define MODE_MOTORS_ONLY          // raw per-motor spin/direction check (motor.h only)
// #define MODE_DRIVE_TEST           // drive.h forward + pivot motion test, with encoder printout
// #define MODE_SENSOR_TELEMETRY     // stream ToF readings as DATA, lines (e.g. for MATLAB)
#define MODE_OBSTACLE_AVOID          // reactive drive-forward-and-steer-away test (current default)
// #define MODE_BATTERY_IMU_BRINGUP  // log battery voltage + IMU readings, no motors
// #define MODE_MOTOR_PID_SYNC       // continuous open-loop wheel-speed sync (motor_pid.h)
// #define MODE_PID_DASHBOARD        // full stack talking to tools/dashboard over serial
// #define MODE_MAZE_SOLVER          // single-core flood-fill maze solve (solver.h)
// #define MODE_MAZE_SOLVER_RTOS     // dual-core FreeRTOS maze solve (tasks.h)

#define MODE_COUNT (defined(MODE_MOTORS_ONLY) + defined(MODE_DRIVE_TEST) +          \
                    defined(MODE_SENSOR_TELEMETRY) + defined(MODE_OBSTACLE_AVOID) + \
                    defined(MODE_BATTERY_IMU_BRINGUP) + defined(MODE_MOTOR_PID_SYNC) + \
                    defined(MODE_PID_DASHBOARD) + defined(MODE_MAZE_SOLVER) +       \
                    defined(MODE_MAZE_SOLVER_RTOS))

#if MODE_COUNT == 0
#error "main.cpp: no MODE_* selected - uncomment exactly one near the top of the file"
#elif MODE_COUNT > 1
#error "main.cpp: more than one MODE_* selected - comment out all but one"
#endif

#if defined(MODE_MOTORS_ONLY)
// Raw motor bring-up test: exercises each motor directly through motor.h,
// bypassing drive.h's left/right + polarity mapping entirely. Use this
// first when wiring a new motor driver, to confirm each motor spins the
// right way before trusting drive.h's DRIVE_LEFT_SIGN/DRIVE_RIGHT_SIGN.
// Robot WILL move - place it in a clear area.

#define MOTORS_ONLY_SPEED 150 // out of 255

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (motors only)...");
    motor_init();
}

void loop()
{
    Serial.println("[MAIN] MOTOR_A forward");
    motor_set_speed(MOTOR_A, MOTORS_ONLY_SPEED);
    delay(800);
    motor_set_speed(MOTOR_A, 0);
    delay(500);

    Serial.println("[MAIN] MOTOR_A reverse");
    motor_set_speed(MOTOR_A, -MOTORS_ONLY_SPEED);
    delay(800);
    motor_set_speed(MOTOR_A, 0);
    delay(500);

    Serial.println("[MAIN] MOTOR_B forward");
    motor_set_speed(MOTOR_B, MOTORS_ONLY_SPEED);
    delay(800);
    motor_set_speed(MOTOR_B, 0);
    delay(500);

    Serial.println("[MAIN] MOTOR_B reverse");
    motor_set_speed(MOTOR_B, -MOTORS_ONLY_SPEED);
    delay(800);
    motor_stop_all();

    delay(2000); // pause before repeating
}
#endif // MODE_MOTORS_ONLY

#if defined(MODE_DRIVE_TEST)
// drive.h motion test: forward, then a pivot turn each way, then stop -
// confirms drive.h's left/right + polarity mapping and prints encoder
// ticks after each leg (meaningful only once encoder.h's pins are wired;
// they default to -1/"unset", in which case ticks just read 0).
// Robot WILL move - place it in a clear area.

static void print_encoder_ticks(void)
{
    Serial.printf("[MAIN] encoder ticks L=%ld R=%ld\n",
                  (long)encoder_get_ticks(ENCODER_LEFT),
                  (long)encoder_get_ticks(ENCODER_RIGHT));
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (drive test)...");
    motor_init();
    encoder_init();
}

void loop()
{
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

    delay(2000); // pause before repeating
}
#endif // MODE_DRIVE_TEST

#if defined(MODE_SENSOR_TELEMETRY)
// Streams all three ToF sensors as DATA, lines for a host tool (e.g.
// MATLAB) to plot live - see telemetry.h. No motor/drive code runs.

static bool s_sensors_ready = false;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (sensor telemetry)...");
    s_sensors_ready = sensor_init();
    if (!s_sensors_ready)
    {
        Serial.println("[MAIN] one or more ToF sensors failed to init");
    }
}

void loop()
{
    sensor_reading_t reading;
    sensor_read_all(&reading);
    telemetry_send(&reading);

    // High-accuracy ranging already takes ~200ms per sensor (~600ms/loop),
    // so no extra delay is needed to keep the bus/host happy.
    delay(10);
}
#endif // MODE_SENSOR_TELEMETRY

#if defined(MODE_OBSTACLE_AVOID)
// Simple reactive obstacle avoidance: drive forward, and once anything is
// within OBSTACLE_THRESHOLD_MM ahead, steer away from whichever side
// (left/right) sees the obstacle closer, then resume forward.

#define OBSTACLE_THRESHOLD_MM 50 // 5cm
#define DRIVE_SPEED 255          // PWM out of 255
#define TURN_DURATION_MS 400     // how long to pivot away before re-checking

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] obstacle-avoidance test starting...");
    motor_init();
    sensor_init();
}

void loop()
{
    sensor_reading_t reading;
    sensor_read_all(&reading);

    if (reading.front_mm < OBSTACLE_THRESHOLD_MM)
    {
        if (reading.left_mm < reading.right_mm)
        {
            Serial.println("[MAIN] obstacle closer on left - turning right");
            drive_turn_right(DRIVE_SPEED);
        }
        else
        {
            Serial.println("[MAIN] obstacle closer on right - turning left");
            drive_turn_left(DRIVE_SPEED);
        }
    }
    else
    {
        Serial.println("[MAIN] path clear - driving forward");
        drive_forward(DRIVE_SPEED);
    }

    delay(TURN_DURATION_MS);
    drive_stop();
}
#endif // MODE_OBSTACLE_AVOID

#if defined(MODE_BATTERY_IMU_BRINGUP)
// Battery voltage + IMU bring-up: no motors/sensors involved, just logs
// battery voltage and accel/gyro/mag readings each loop. Useful for
// confirming both are wired correctly in isolation.

static bool s_mpu_ready = false;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (battery + IMU bring-up)...");
    battery_init();
    s_mpu_ready = mpu9250_init();
    if (!s_mpu_ready)
    {
        Serial.println("[MAIN] WARNING: MPU9250 init failed");
    }
}

void loop()
{
    battery_read_voltage();

    if (s_mpu_ready)
    {
        mpu9250_data_t imu;
        mpu9250_read(&imu);
    }

    delay(1000);
}
#endif // MODE_BATTERY_IMU_BRINGUP

#if defined(MODE_MOTOR_PID_SYNC)
// Continuous open-loop wheel-speed sync test - see motor_pid.h. Motor A
// holds a fixed reference PWM and motor B is nudged each second to match
// its pulse rate; runs forever once started, no target/RUN command like
// control.h's loops. motor_pid_init() calls motor_init() and starts both
// motors itself - robot WILL move as soon as it boots.

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (motor PID sync test)...");
    motor_pid_init();
}

void loop()
{
    motor_pid_update();
    delay(50);
}
#endif // MODE_MOTOR_PID_SYNC

#if defined(MODE_PID_DASHBOARD)
// Full sensor/actuator stack talking to tools/dashboard (see its README)
// over serial via comms.h's line protocol - live telemetry out, PID
// gains + test maneuvers in. Flash this while tuning the physical robot.
//
// Encoder pins are still unset (see encoder.h), so control.h's
// encoder-based loops (straight-line sync, and the distance/speed
// telemetry fields) won't do anything meaningful until they're wired up.

#define TELEMETRY_PERIOD_MS 50

static bool s_sensors_ready = false;
static uint32_t s_last_telemetry_ms = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (PID tuning dashboard)...");
    motor_init();
    encoder_init();
    mpu9250_init();
    s_sensors_ready = sensor_init();
    control_init();
    comms_init();

    if (!s_sensors_ready)
    {
        Serial.println("[MAIN] WARNING: ToF sensors failed to init - wall readings will be stale/max-range");
    }
}

void loop()
{
    comms_poll(); // dispatches PID/RUN/GETPID commands; RUN blocks internally until done/aborted

    uint32_t now = millis();
    if (now - s_last_telemetry_ms >= TELEMETRY_PERIOD_MS)
    {
        s_last_telemetry_ms = now;
        comms_tick(); // one telemetry line; also re-polls, harmless when idle
    }
}
#endif // MODE_PID_DASHBOARD

#if defined(MODE_MAZE_SOLVER)
// Single-pass flood-fill maze solve: search to center, then back to
// start - see solver.h/maze.h. Blocks inside solver_run() until done,
// then halts. Robot WILL move - place it in the maze before powering on.

static bool s_sensors_ready = false;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (maze solver)...");
    motor_init();
    s_sensors_ready = sensor_init();
    if (!s_sensors_ready)
    {
        Serial.println("[MAIN] WARNING: ToF sensors failed to init");
    }
}

void loop()
{
    if (s_sensors_ready)
    {
        solver_run();
    }

    for (;;)
    {
        delay(1000); // halt once solved (or if sensors never came up)
    }
}
#endif // MODE_MAZE_SOLVER

#if defined(MODE_MAZE_SOLVER_RTOS)
// Dual-core FreeRTOS maze solve - see tasks.h. tasks_start() spawns the
// planning (core 0) and control (core 1) tasks and returns immediately;
// the maze run happens entirely inside them, so loop() stays idle. The
// control task calls sensor_init() itself. Robot WILL move - place it in
// the maze before powering on.

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (maze solver, dual-core)...");
    motor_init();
    tasks_start();
}

void loop()
{
    delay(1000);
}
#endif // MODE_MAZE_SOLVER_RTOS
