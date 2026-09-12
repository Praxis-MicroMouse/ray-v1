#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "maze.h"
#include "sensor.h"
#include "drive.h"
#include "control.h"
#include "turns.h"
#include "solver.h" // reuses SOLVER_WALL_THRESHOLD_MM - see solver.h

#define PLANNING_CORE 0
#define CONTROL_CORE  1

// Bytes, not words - ESP-IDF's FreeRTOS port takes stack depth in bytes.
// Both tasks keep their large state in `static` locals (see maze.cpp's
// flood-fill/Dijkstra scratch arrays), so actual stack use is small;
// this is a generous safety margin, mainly for Serial.printf.
#define TASK_STACK_BYTES 4096

typedef enum {
    CMD_SENSE,
    CMD_MOVE_FORWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_HALT
} control_cmd_type_t;

typedef struct {
    control_cmd_type_t type;
} control_cmd_t;

typedef struct {
    bool wall_front;
    bool wall_right;
    bool wall_left;
} control_event_t;

static QueueHandle_t s_cmd_queue;
static QueueHandle_t s_event_queue;

// ---------------------------------------------------------------------
// Control task (core 1): the only task that touches a sensor or motor.
// ---------------------------------------------------------------------

static void control_task(void *pv) {
    (void) pv;

    bool sensors_ready = sensor_init();
    Serial.printf("[CONTROL] task started on core %d (sensors_ready=%d)\n",
                  xPortGetCoreID(), (int) sensors_ready);

    control_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (cmd.type) {
            case CMD_MOVE_FORWARD:
                // Closed-loop dual-wheel encoder PID (control.h) instead of
                // open-loop timing - stops at the actual MAZE_CELL_SIZE_MM.
                control_run_straight(MAZE_CELL_SIZE_MM, DRIVE_DEFAULT_SPEED, nullptr);
                break;
            case CMD_TURN_LEFT:
                // Encoder-measured pivot (turns.h) instead of a timed guess.
                turn_left_90(TURNS_DEFAULT_SPEED);
                break;
            case CMD_TURN_RIGHT:
                turn_right_90(TURNS_DEFAULT_SPEED);
                break;
            case CMD_HALT:
                drive_stop();
                Serial.println("[CONTROL] halted");
                continue; // no event expected after halt
            case CMD_SENSE:
            default:
                break;
        }

        control_event_t event = { false, false, false };
        if (sensors_ready) {
            sensor_reading_t reading;
            sensor_read_all(&reading);
            event.wall_front = reading.front_mm < SOLVER_WALL_THRESHOLD_MM;
            event.wall_right = reading.right_mm < SOLVER_WALL_THRESHOLD_MM;
            event.wall_left  = reading.left_mm  < SOLVER_WALL_THRESHOLD_MM;
        }
        xQueueSend(s_event_queue, &event, portMAX_DELAY);
    }
}

// ---------------------------------------------------------------------
// Planning task (core 0): maze bookkeeping + search algorithms only.
// ---------------------------------------------------------------------

static void send_cmd(control_cmd_type_t type) {
    control_cmd_t cmd = { type };
    xQueueSend(s_cmd_queue, &cmd, portMAX_DELAY);
}

static void recv_event(control_event_t *event) {
    xQueueReceive(s_event_queue, event, portMAX_DELAY);
}

static void apply_sense(maze_t *maze, int x, int y, maze_dir_t heading, const control_event_t *event) {
    maze_dir_t dirs[3]   = { heading, maze_turn_right(heading), maze_turn_left(heading) };
    bool       sensed[3] = { event->wall_front, event->wall_right, event->wall_left };

    for (int i = 0; i < 3; i++) {
        int nx = x;
        int ny = y;
        maze_step(&nx, &ny, dirs[i]);

        if (sensed[i] || !maze_in_bounds(nx, ny)) {
            maze_set_wall(maze, x, y, dirs[i]);
        } else {
            maze_clear_wall(maze, x, y, dirs[i]);
        }
    }
}

// Turns one 90-degree step at a time toward `target`, re-sensing after
// every step (the map doesn't change from turning alone, but this keeps
// the request/reply protocol with the control task uniform).
static void turn_to(maze_t *maze, int x, int y, maze_dir_t *heading, maze_dir_t target) {
    control_event_t event;
    while (*heading != target) {
        int diff = (target - *heading + 4) % 4;
        if (diff == 1) {
            send_cmd(CMD_TURN_RIGHT);
            *heading = maze_turn_right(*heading);
        } else {
            send_cmd(CMD_TURN_LEFT);
            *heading = maze_turn_left(*heading);
        }
        recv_event(&event);
        apply_sense(maze, x, y, *heading, &event);
    }
}

static void planning_task(void *pv) {
    (void) pv;
    Serial.printf("[PLANNING] task started on core %d\n", xPortGetCoreID());

    static maze_t maze;
    maze_init(&maze);

    maze_cell_t start = { 0, 0 };
    maze_cell_t goals[4] = {
        { (MAZE_WIDTH - 1) / 2, (MAZE_HEIGHT - 1) / 2 },
        { MAZE_WIDTH / 2,       (MAZE_HEIGHT - 1) / 2 },
        { (MAZE_WIDTH - 1) / 2, MAZE_HEIGHT / 2 },
        { MAZE_WIDTH / 2,       MAZE_HEIGHT / 2 }
    };
    maze_cell_t target_goals[4] = { goals[0], goals[1], goals[2], goals[3] };
    int target_goal_count = 4;
    bool returning_to_start = false;

    int x = 0;
    int y = 0;
    maze_dir_t heading = MAZE_NORTH;
    control_event_t event;

    // Prime the map with a sense at the start cell before making any move.
    send_cmd(CMD_SENSE);
    recv_event(&event);
    apply_sense(&maze, x, y, heading, &event);

    Serial.println("[PLANNING] exploring...");

    while (true) {
        bool at_center = (x == goals[0].x || x == goals[1].x) && (y == goals[0].y || y == goals[2].y);
        if (!returning_to_start && at_center) {
            Serial.println("[PLANNING] reached center - returning to start");
            returning_to_start = true;
            target_goals[0] = start;
            target_goal_count = 1;
        }
        if (returning_to_start && x == start.x && y == start.y) {
            break;
        }

        maze_flood_fill(&maze, target_goals, target_goal_count);
        maze_dir_t next_dir = maze_choose_next_direction(&maze, x, y, heading);

        turn_to(&maze, x, y, &heading, next_dir);

        send_cmd(CMD_MOVE_FORWARD);
        recv_event(&event);
        maze_step(&x, &y, heading);
        apply_sense(&maze, x, y, heading, &event);

        Serial.printf("[PLANNING] at (%d,%d) heading=%d\n", x, y, (int) heading);
    }

    Serial.println("[PLANNING] exploration complete - planning min-turn speed-run path");

    static maze_action_t path[MAZE_MAX_PATH_LEN];
    int path_len = maze_plan_min_turn_path(&maze, start, MAZE_NORTH, goals, 4, path, MAZE_MAX_PATH_LEN);
    Serial.printf("[PLANNING] speed-run path: %d actions\n", path_len);

    // Map is trusted from here on, so the speed run doesn't need to wait
    // on sensing to decide anything - it only waits on each action
    // finishing before requesting the next (see tasks.h for the note on
    // pipelining this further by queueing several actions ahead).
    for (int i = 0; i < path_len; i++) {
        switch (path[i]) {
            case MAZE_ACTION_FORWARD:   send_cmd(CMD_MOVE_FORWARD); break;
            case MAZE_ACTION_TURN_LEFT: send_cmd(CMD_TURN_LEFT);    break;
            case MAZE_ACTION_TURN_RIGHT: send_cmd(CMD_TURN_RIGHT);  break;
        }
        recv_event(&event); // discarded - map is trusted during the speed run
    }

    send_cmd(CMD_HALT);
    Serial.println("[PLANNING] speed run complete");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tasks_start(void) {
    s_cmd_queue = xQueueCreate(4, sizeof(control_cmd_t));
    s_event_queue = xQueueCreate(4, sizeof(control_event_t));

    xTaskCreatePinnedToCore(control_task, "control", TASK_STACK_BYTES, NULL, 2, NULL, CONTROL_CORE);
    xTaskCreatePinnedToCore(planning_task, "planning", TASK_STACK_BYTES, NULL, 1, NULL, PLANNING_CORE);
}
