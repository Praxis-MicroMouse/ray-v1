#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "mpu9250.h"
#include "sensor.h"
#include "control.h"
#include "comms.h"
// #include "drive.h"      // uncomment for the plain motion-test build below instead
// #include "telemetry.h"  // uncomment to bring the [SENSOR]-independent DATA, stream back
// #include "battery.h"    // only needed directly for battery_init(); battery.h's other
                            // half (battery_get_percent/read_voltage) is already used by comms.cpp
// #include "maze.h"       // uncomment along with solver.h, to run the maze solver instead
// #include "solver.h"     // uncomment (needs sensor.h too - the solver senses walls)
// #include "tasks.h"      // uncomment for the dual-core RTOS version of the solver instead of solver.h

// PID-tuning dashboard build (the current default): brings up every
// sensor/actuator module and talks to tools/dashboard (see its README)
// over serial using comms.h's line protocol - live telemetry out, PID
// gains + test maneuvers in. Flash this while tuning the physical robot;
// swap in the maze-solver blocks above (mutually exclusive with each
// other and with this) once PID is dialed in and you're ready to run the
// actual maze.

#define TELEMETRY_PERIOD_MS 50

static bool s_sensors_ready = false;
static uint32_t s_last_telemetry_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("[MAIN] booting (PID tuning dashboard)...");
    motor_init();
    encoder_init();
    mpu9250_init();
    s_sensors_ready = sensor_init();
    control_init();
    comms_init();

    if (!s_sensors_ready) {
        Serial.println("[MAIN] WARNING: ToF sensors failed to init - wall readings will be stale/max-range");
    }
}

void loop() {
    comms_poll(); // dispatches PID/RUN/GETPID commands; RUN blocks internally until done/aborted

    uint32_t now = millis();
    if (now - s_last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
        s_last_telemetry_ms = now;
        comms_tick(); // one telemetry line; also re-polls, harmless when idle
    }
}
