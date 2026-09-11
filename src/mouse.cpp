#include "mouse.h"

#include <Arduino.h>
#include <stdint.h>

#include "maze.h"
#include "motion.h"
#include "steering.h"
#include "sensor.h"
#include "button.h"
#include "config/config.h"

#define FULL_CELL MAZE_CELL_SIZE_MM
#define HALF_CELL (MAZE_CELL_SIZE_MM / 2.0f)

static maze_t s_maze;
static int s_x, s_y;
static maze_dir_t s_heading;

static bool cell_in_set(int x, int y, const maze_cell_t *set, int n) {
    for (int i = 0; i < n; i++) {
        if (set[i].x == x && set[i].y == y) return true;
    }
    return false;
}

// Bring the robot to a full, safe halt and require operator
// intervention (button press, if wired - otherwise a power-cycle) before
// anything else in mouse.cpp runs again. Called on any unrecoverable
// error: no route to target, or a flood/search bug.
static void panic(void) {
    motion_emergency_stop();
    steering_set_mode(STEERING_OFF);
    Serial.println("[MOUSE] PANIC - halted. Press the button (if wired) to acknowledge, or power-cycle.");
    for (;;) {
        if (button_pressed()) {
            button_wait_for_release();
            return;
        }
        delay(200);
    }
}

// Marks walls of the cell the mouse currently occupies from the current
// sensor reading, absolute-direction-aware.
static void update_map(void) {
    sensor_walls_t w;
    sensor_get_walls(&w);

    maze_dir_t dirs[3]   = { s_heading, maze_turn_right(s_heading), maze_turn_left(s_heading) };
    bool       sensed[3] = { w.front, w.right, w.left };

    for (int i = 0; i < 3; i++) {
        int nx = s_x, ny = s_y;
        maze_step(&nx, &ny, dirs[i]);
        if (sensed[i] || !maze_in_bounds(nx, ny)) {
            maze_set_wall(&s_maze, s_x, s_y, dirs[i]);
        } else {
            maze_clear_wall(&s_maze, s_x, s_y, dirs[i]);
        }
    }
}

// A 90-degree turn taken WHILE MOVING: keeps driving at the turn's speed,
// starts pivoting either at the geometry offset in config/turn_params.h
// or earlier if the front sensor sees a wall closer than expected, then
// drives out to the next cell's sensing point. Updates s_heading.
static void turn_smooth(turn_type_t type) {
    const turn_params_t *params = &TURN_PARAMS[type];

    steering_set_mode(STEERING_OFF); // never steer through a turn
    motion_set_target_velocity((float)params->speed_mm_s);

    float turn_point = FULL_CELL + HALF_CELL - params->entry_offset_mm;
    while (motion_position() < turn_point) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        sensor_reading_t r;
        sensor_get_latest(&r);
        if (r.front_mm < (uint16_t)params->trigger_mm) {
            motion_set_target_velocity(motion_velocity()); // freeze speed, don't keep accelerating toward the wall
            break;
        }
    }

    motion_turn(params->angle_deg, params->omega_deg_s, 0.0f, params->alpha_deg_s2);

    float end_point = HALF_CELL + params->exit_offset_mm;
    motion_move(SENSING_POSITION_MM - end_point, motion_velocity(), (float)SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
    motion_set_position(SENSING_POSITION_MM);

    s_heading = (type == TURN_SS90_LEFT) ? maze_turn_left(s_heading) : maze_turn_right(s_heading);
}

static void mouse_turn_left(void) {
    turn_smooth(TURN_SS90_LEFT);
}

static void mouse_turn_right(void) {
    turn_smooth(TURN_SS90_RIGHT);
}

// Bring the mouse to a halt at approximately the cell center. If a front
// wall is present, it approaches slowly and lets the front ToF sensor
// (not the profile) decide exactly where to stop - see
// TOF_FRONT_REFERENCE_MM. Call adjust_position() afterward to fine-tune.
static void stop_at_centre(void) {
    sensor_walls_t walls;
    sensor_get_walls(&walls);
    bool has_wall = walls.front;

    steering_set_mode(STEERING_OFF);
    float remaining = (FULL_CELL + HALF_CELL) - motion_position();
    float final_speed = has_wall ? 30.0f : 0.0f;
    motion_start_move(remaining, motion_velocity(), final_speed, motion_acceleration());

    uint32_t timeout_ms = has_wall ? 500 : 1000;
    uint32_t start = millis();
    while (true) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        if (millis() - start > timeout_ms) break;
        if (has_wall) {
            sensor_reading_t r;
            sensor_get_latest(&r);
            if (r.front_mm <= TOF_FRONT_REFERENCE_MM) break;
        } else if (motion_move_finished()) {
            break;
        }
        delay(2);
    }
    motion_reset_drive_system();
}

// Fine-tunes stopping position against a front wall, using the ToF
// reading as ground truth. No-op if there's no front wall.
static void adjust_position(void) {
    sensor_walls_t walls;
    sensor_get_walls(&walls);
    if (!walls.front) return;

    const int tolerance_mm = 5;
    const float correction_mm = 10.0f;
    const float adjust_speed = 100.0f;
    const float adjust_accel = 1000.0f;

    sensor_reading_t r;
    sensor_get_latest(&r);
    int error = (int)r.front_mm - TOF_FRONT_REFERENCE_MM; // positive = too far from the wall
    if (error > tolerance_mm) {
        motion_move(correction_mm, adjust_speed, 0.0f, adjust_accel);
    } else if (error < -tolerance_mm) {
        motion_move(-correction_mm, adjust_speed, 0.0f, adjust_accel);
    }
}

static void mouse_turn_back(void) {
    stop_at_centre();
    adjust_position();
    motion_spin_turn(180.0f, SPIN_TURN_OMEGA_DEG_S, SPIN_TURN_ALPHA_DEG_S2);
    float distance = SENSING_POSITION_MM - HALF_CELL;
    motion_move(distance, (float)SEARCH_SPEED_MM_S, (float)SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
    motion_set_position(SENSING_POSITION_MM);
    s_heading = maze_opposite(s_heading);
}

// The robot is already moving; carry on to the next cell's sensing
// point without stopping - "subtracting a cell" from the tracked
// position tricks the profile into thinking it's back at a cell start.
static void mouse_move_ahead(void) {
    motion_adjust_forward_position(-FULL_CELL);
    motion_wait_until_position(SENSING_POSITION_MM);
}

// In-place turn to an absolute heading (used between exploration and the
// speed run, and to face the goal after arriving) - NOT a search turn,
// the robot is stationary throughout.
static void turn_to_face(maze_dir_t new_heading) {
    int diff = (new_heading - s_heading + 4) % 4;
    switch (diff) {
        case 0: // already facing that way
            break;
        case 1: // right
            motion_spin_turn(-(90.0f + TURN_TRIM_DEG_RIGHT), SPIN_TURN_OMEGA_DEG_S, SPIN_TURN_ALPHA_DEG_S2);
            break;
        case 2: // back
            motion_spin_turn(180.0f, SPIN_TURN_OMEGA_DEG_S, SPIN_TURN_ALPHA_DEG_S2);
            break;
        case 3: // left
            motion_spin_turn(90.0f + TURN_TRIM_DEG_LEFT, SPIN_TURN_OMEGA_DEG_S, SPIN_TURN_ALPHA_DEG_S2);
            break;
    }
    s_heading = new_heading;
}

// Explores (mapping walls as it goes) from the mouse's current position
// to whichever cell in `goals` the flood fill finds cheapest, using
// safe search speeds and smooth search turns. On entry the mouse is
// assumed backed up against the wall behind it, centered, at rest.
static void search_to(const maze_cell_t *goals, int goal_count) {
    maze_flood_fill(&s_maze, goals, goal_count);
    if (s_maze.dist[maze_index(s_x, s_y)] == INT16_MAX) {
        Serial.println("[MOUSE] ERR: no route to target");
        panic();
        return;
    }

    steering_set_mode(STEERING_OFF); // never steer from zero speed
    motion_move(BACK_WALL_TO_CENTER_MM, (float)SEARCH_SPEED_MM_S, (float)SEARCH_SPEED_MM_S, SEARCH_ACCEL_MM_S2);
    motion_set_position(HALF_CELL);
    motion_wait_until_position(SENSING_POSITION_MM);

    while (!cell_in_set(s_x, s_y, goals, goal_count)) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }

        steering_set_mode(STEERING_NORMAL);
        maze_step(&s_x, &s_y, s_heading); // the cell we're about to enter (see mouse.h's SENSING_POSITION note)
        update_map();
        maze_flood_fill(&s_maze, goals, goal_count);
        if (s_maze.dist[maze_index(s_x, s_y)] == INT16_MAX) {
            Serial.println("[MOUSE] ERR: no route to target");
            panic();
            return;
        }

        if (cell_in_set(s_x, s_y, goals, goal_count)) {
            break;
        }

        maze_dir_t new_heading = maze_choose_next_direction(&s_maze, s_x, s_y, s_heading);
        int diff = (new_heading - s_heading + 4) % 4;
        switch (diff) {
            case 0: mouse_move_ahead(); break;
            case 1: mouse_turn_right(); break;
            case 2: mouse_turn_back(); break;
            case 3: mouse_turn_left(); break;
        }
    }

    stop_at_centre();
    adjust_position();
    steering_set_mode(STEERING_OFF);
}

void mouse_init(void) {
    maze_init(&s_maze);
    s_x = 0;
    s_y = 0;
    s_heading = MAZE_NORTH;
}

void mouse_run(void) {
    maze_cell_t start = { 0, 0 };
    maze_cell_t goals[4] = {
        { (MAZE_WIDTH - 1) / 2, (MAZE_HEIGHT - 1) / 2 },
        { MAZE_WIDTH / 2,       (MAZE_HEIGHT - 1) / 2 },
        { (MAZE_WIDTH - 1) / 2, MAZE_HEIGHT / 2 },
        { MAZE_WIDTH / 2,       MAZE_HEIGHT / 2 }
    };

    Serial.println("[MOUSE] exploring to center...");
    motion_reset_drive_system();
    search_to(goals, 4);

    Serial.println("[MOUSE] returning to start...");
    motion_reset_drive_system();
    search_to(&start, 1);

    turn_to_face(MAZE_NORTH);
    motion_stop();
    motion_disable_drive();

    Serial.println("[MOUSE] planning turn-minimizing speed-run path...");
    static maze_action_t path[MAZE_MAX_PATH_LEN];
    int path_len = maze_plan_min_turn_path(&s_maze, start, MAZE_NORTH, goals, 4, path, MAZE_MAX_PATH_LEN);
    Serial.printf("[MOUSE] speed-run path: %d actions\n", path_len);

    Serial.println("[MOUSE] speed run...");
    s_x = 0;
    s_y = 0;
    s_heading = MAZE_NORTH;
    motion_reset_drive_system();
    steering_set_mode(STEERING_OFF);
    motion_move(BACK_WALL_TO_CENTER_MM, (float)FAST_RUN_SPEED_MM_S, (float)FAST_RUN_SPEED_MM_S, FAST_RUN_ACCEL_MM_S2);
    motion_set_position(HALF_CELL);
    motion_wait_until_position(SENSING_POSITION_MM);

    for (int i = 0; i < path_len; i++) {
        if (button_pressed()) {
            motion_emergency_stop();
            return;
        }
        switch (path[i]) {
            case MAZE_ACTION_FORWARD:
                mouse_move_ahead();
                maze_step(&s_x, &s_y, s_heading);
                break;
            case MAZE_ACTION_TURN_LEFT:
                mouse_turn_left();
                break;
            case MAZE_ACTION_TURN_RIGHT:
                mouse_turn_right();
                break;
        }
    }

    stop_at_centre();
    adjust_position();
    steering_set_mode(STEERING_OFF);
    motion_disable_drive();
    Serial.println("[MOUSE] speed run complete");
}
