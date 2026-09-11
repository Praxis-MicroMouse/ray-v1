#include "sensor_loop.h"

#include "sensor.h"
#include "steering.h"
#include "battery.h"
#include "config/motion_tuning.h"

// Battery voltage barely moves tick to tick - refresh it far less often
// than the sensor/steering work above (see battery.cpp's own
// BATTERY_LOG_INTERVAL_MS for the separate log-throttling).
#define BATTERY_UPDATE_EVERY_N_TICKS (SENSOR_LOOP_HZ / 10) // ~10 times/sec

static int s_tick_count = 0;

bool sensor_loop_init(void) {
    bool ok = sensor_init();
    steering_init();
    battery_init();
    s_tick_count = 0;
    return ok;
}

void sensor_loop_tick(void) {
    sensor_poll();
    steering_update();

    s_tick_count++;
    if (s_tick_count >= BATTERY_UPDATE_EVERY_N_TICKS) {
        s_tick_count = 0;
        battery_update();
    }
}
