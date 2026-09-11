#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "control_loop.h"
#include "sensor_loop.h"
#include "config/motion_tuning.h"

#define PLANNING_CORE 0
#define CONTROL_CORE  1

// Bytes, not words - ESP-IDF's FreeRTOS port takes stack depth in bytes.
#define CONTROL_TASK_STACK_BYTES 4096
#define SENSOR_TASK_STACK_BYTES  4096
#define TOP_TASK_STACK_BYTES     8192 // deepest call chain: mouse.cpp -> maze's Dijkstra scratch arrays are `static`, but the call stack itself still runs deeper here than the other two tasks

#define CONTROL_TASK_PRIORITY 3
#define SENSOR_TASK_PRIORITY  2
#define TOP_TASK_PRIORITY     1

static tasks_top_level_fn_t s_top_level_fn = nullptr;

static void control_task(void *pv) {
    (void)pv;
    control_loop_init();
    Serial.printf("[TASKS] control task started on core %d (%dHz)\n", xPortGetCoreID(), CONTROL_LOOP_HZ);

    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        control_loop_tick();
    }
}

static void sensor_task(void *pv) {
    (void)pv;
    bool ok = sensor_loop_init();
    Serial.printf("[TASKS] sensor task started on core %d (%dHz, sensors_ok=%d)\n",
                  xPortGetCoreID(), SENSOR_LOOP_HZ, (int)ok);

    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        sensor_loop_tick();
    }
}

static void top_task(void *pv) {
    (void)pv;
    Serial.printf("[TASKS] top task started on core %d\n", xPortGetCoreID());

    // Give the other two tasks a moment to bring up sensors/controllers
    // before the top-level function starts issuing moves.
    vTaskDelay(pdMS_TO_TICKS(250));

    s_top_level_fn();

    Serial.println("[TASKS] top task idle (run complete)");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tasks_start(tasks_top_level_fn_t top_level_fn) {
    s_top_level_fn = top_level_fn;
    xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK_BYTES, NULL, CONTROL_TASK_PRIORITY, NULL, CONTROL_CORE);
    xTaskCreatePinnedToCore(sensor_task, "sensor", SENSOR_TASK_STACK_BYTES, NULL, SENSOR_TASK_PRIORITY, NULL, PLANNING_CORE);
    xTaskCreatePinnedToCore(top_task, "top", TOP_TASK_STACK_BYTES, NULL, TOP_TASK_PRIORITY, NULL, PLANNING_CORE);
}
