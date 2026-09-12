#include "solver.h"

#include <Arduino.h>

#include "maze.h"
#include "sensor.h"
#include "drive.h"
#include "control.h"
#include "turns.h"

// A wall is "sensed" on a side if its ToF reading is under the threshold.
// Absolute wall direction = heading rotated to match front/right/left.
static void sense_walls(maze_t *maze, int x, int y, maze_dir_t heading) {
    sensor_reading_t reading;
    sensor_read_all(&reading);

    int sensed_front = reading.front_mm < SOLVER_WALL_THRESHOLD_MM;
    int sensed_right = reading.right_mm < SOLVER_WALL_THRESHOLD_MM;
    int sensed_left  = reading.left_mm  < SOLVER_WALL_THRESHOLD_MM;

    maze_dir_t dirs[3]   = { heading, maze_turn_right(heading), maze_turn_left(heading) };
    int        sensed[3] = { sensed_front, sensed_right, sensed_left };

    for (int i = 0; i < 3; i++) {
        maze_dir_t dir = dirs[i];
        int nx = x;
        int ny = y;
        maze_step(&nx, &ny, dir);

        if (sensed[i] || !maze_in_bounds(nx, ny)) {
            maze_set_wall(maze, x, y, dir);
        } else {
            maze_clear_wall(maze, x, y, dir);
        }
    }
}

// Closed-loop move via control.h's dual-wheel encoder PID (control_run_straight())
// instead of open-loop timing - it corrects for left/right wheel-speed
// mismatch as it drives, so it actually stops at MAZE_CELL_SIZE_MM
// instead of "however far the timing guess happened to carry it".
static void move_forward_one_cell(void) {
    Serial.println("[SOLVER] move forward 1 cell");
    control_run_straight(MAZE_CELL_SIZE_MM, DRIVE_DEFAULT_SPEED, nullptr);
}

// Pivots one 90-degree step at a time toward `target`, always taking a
// single right turn for a 90-degree gap and a left turn otherwise (two
// lefts covers a 180, matching the mms-c reference algorithm's turn_to).
// Each step uses turns.h's encoder-measured pivot instead of a timed
// guess, so it stops at the actual 90 degrees regardless of battery sag
// or surface friction changing the turn rate.
static void turn_to(maze_dir_t *heading, maze_dir_t target) {
    while (*heading != target) {
        int diff = (target - *heading + 4) % 4;

        if (diff == 1) {
            turn_right_90(TURNS_DEFAULT_SPEED);
            *heading = maze_turn_right(*heading);
        } else {
            turn_left_90(TURNS_DEFAULT_SPEED);
            *heading = maze_turn_left(*heading);
        }
    }
}

bool solver_run(void) {
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

    Serial.println("[SOLVER] starting maze solve");

    while (true) {
        bool at_center = (x == goals[0].x || x == goals[1].x) && (y == goals[0].y || y == goals[2].y);
        if (!returning_to_start && at_center) {
            Serial.println("[SOLVER] reached center - returning to start");
            returning_to_start = true;
            target_goals[0] = start;
            target_goal_count = 1;
        }

        if (returning_to_start && x == start.x && y == start.y) {
            Serial.println("[SOLVER] back at start - solve complete");
            break;
        }

        sense_walls(&maze, x, y, heading);
        maze_flood_fill(&maze, target_goals, target_goal_count);

        maze_dir_t next_dir = maze_choose_next_direction(&maze, x, y, heading);
        turn_to(&heading, next_dir);
        move_forward_one_cell();
        maze_step(&x, &y, heading);

        Serial.printf("[SOLVER] at (%d,%d) heading=%d\n", x, y, (int)heading);
    }

    drive_stop();
    return true;
}
