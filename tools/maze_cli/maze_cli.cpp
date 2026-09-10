// Host-side CLI that runs the exact same maze-solving code the robot
// runs (../../src/maze.cpp - zero Arduino dependencies) against a maze,
// so the algorithm can be validated/visualized without needing to
// script the mms simulator's GUI. See ../dashboard for the frontend
// that drives this.
//
// Input: a whitespace/line-separated list of KEY VALUE... tokens on
// stdin (deliberately not JSON - keeps this file dependency-free; the
// dashboard's Python backend translates the frontend's JSON request
// into this before piping it in):
//
//   MODE search|speedrun
//   START <x> <y>                          (default 0 0)
//   GOALS <n> <x1> <y1> ... <xn> <yn>       (default: standard 4-cell center)
//   RANDOM 0|1                              (search mode: generate a maze instead of using WALLS)
//   SEED <uint>                             (RANDOM's RNG seed)
//   WALLS <n> <w1> <w2> ... <wn>            (n = width*height cells, row-major
//                                            y*width+x, each a MAZE_WALL_* bitmask -
//                                            ground truth for search, or the known
//                                            map for speedrun)
//
// Output: one JSON object on stdout - see README.md in this directory
// for the exact schema.
//
// Unrecognized keys are ignored (forward-compatible with a newer
// dashboard); WIDTH/HEIGHT are accepted and ignored since maze size is
// fixed at compile time by MAZE_WIDTH/MAZE_HEIGHT (see ../../include/maze.h).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "maze.h"
#include "solver.h" // only for the SOLVER_*_TIME_MS estimate constants - no Arduino dependency

using namespace std;

static const char *dir_name(maze_dir_t d) {
    switch (d) {
        case MAZE_NORTH: return "N";
        case MAZE_EAST:  return "E";
        case MAZE_SOUTH: return "S";
        default:         return "W";
    }
}

static const char *action_name(maze_action_t a) {
    switch (a) {
        case MAZE_ACTION_FORWARD:    return "FWD";
        case MAZE_ACTION_TURN_LEFT:  return "LEFT";
        default:                     return "RIGHT";
    }
}

// Recursive-backtracker perfect maze (no loops, one path between any two
// cells) - always solvable from (0,0). Built entirely out of maze.h's
// own set/clear-wall functions so it can't drift from how the real
// algorithm interprets walls.
static void generate_random_maze(maze_t *maze, unsigned seed) {
    maze_init(maze);
    for (int y = 0; y < MAZE_HEIGHT; y++) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            maze_set_wall(maze, x, y, MAZE_NORTH);
            maze_set_wall(maze, x, y, MAZE_EAST);
            maze_set_wall(maze, x, y, MAZE_SOUTH);
            maze_set_wall(maze, x, y, MAZE_WEST);
        }
    }

    vector<bool> visited(MAZE_CELL_COUNT, false);
    mt19937 rng(seed);
    vector<pair<int, int>> stack;
    stack.push_back({0, 0});
    visited[maze_index(0, 0)] = true;

    while (!stack.empty()) {
        int x = stack.back().first;
        int y = stack.back().second;

        vector<maze_dir_t> dirs = { MAZE_NORTH, MAZE_EAST, MAZE_SOUTH, MAZE_WEST };
        shuffle(dirs.begin(), dirs.end(), rng);

        bool advanced = false;
        for (maze_dir_t d : dirs) {
            int nx = x, ny = y;
            maze_step(&nx, &ny, d);
            if (!maze_in_bounds(nx, ny) || visited[maze_index(nx, ny)]) continue;

            maze_clear_wall(maze, x, y, d);
            visited[maze_index(nx, ny)] = true;
            stack.push_back({nx, ny});
            advanced = true;
            break;
        }
        if (!advanced) stack.pop_back();
    }
}

static void sense_from_ground_truth(maze_t *known, const maze_t *truth, int x, int y, maze_dir_t heading) {
    maze_dir_t dirs[3] = { heading, maze_turn_right(heading), maze_turn_left(heading) };
    for (int i = 0; i < 3; i++) {
        maze_dir_t d = dirs[i];
        int nx = x, ny = y;
        maze_step(&nx, &ny, d);
        bool wall_present = (truth->walls[maze_index(x, y)] & (1 << d)) != 0;
        if (wall_present || !maze_in_bounds(nx, ny)) {
            maze_set_wall(known, x, y, d);
        } else {
            maze_clear_wall(known, x, y, d);
        }
    }
}

typedef tuple<int, int, maze_dir_t> path_point_t;

struct SearchResult {
    vector<path_point_t> path;
    vector<maze_action_t> actions;
    maze_t discovered;
    int reached_center_at_step = -1;
};

// Mirrors ../../src/solver.cpp's solver_run() / ../../src/tasks.cpp's
// planning_task() move-by-move, except walls come from sense_from_ground_truth()
// instead of real ToF sensors.
static void run_search(const maze_t &truth, maze_cell_t start, const vector<maze_cell_t> &goals,
                        SearchResult &result) {
    maze_t known;
    maze_init(&known);

    vector<maze_cell_t> target_goals = goals;
    bool returning = false;

    int x = start.x;
    int y = start.y;
    maze_dir_t heading = MAZE_NORTH;

    sense_from_ground_truth(&known, &truth, x, y, heading);
    result.path.push_back({x, y, heading});

    int step = 0;
    int guard = MAZE_CELL_COUNT * 8; // safety cap against a logic bug looping forever
    while (step < guard) {
        bool at_center = goals.size() == 4
            && (x == goals[0].x || x == goals[1].x)
            && (y == goals[0].y || y == goals[2].y);

        if (!returning && at_center) {
            returning = true;
            target_goals = { start };
            result.reached_center_at_step = step;
        }
        if (returning && x == start.x && y == start.y) break;

        maze_flood_fill(&known, target_goals.data(), (int) target_goals.size());
        maze_dir_t next_dir = maze_choose_next_direction(&known, x, y, heading);

        while (heading != next_dir) {
            int diff = (next_dir - heading + 4) % 4;
            if (diff == 1) {
                result.actions.push_back(MAZE_ACTION_TURN_RIGHT);
                heading = maze_turn_right(heading);
            } else {
                result.actions.push_back(MAZE_ACTION_TURN_LEFT);
                heading = maze_turn_left(heading);
            }
            result.path.push_back({x, y, heading});
        }

        result.actions.push_back(MAZE_ACTION_FORWARD);
        maze_step(&x, &y, heading);
        sense_from_ground_truth(&known, &truth, x, y, heading);
        result.path.push_back({x, y, heading});

        step++;
    }

    result.discovered = known;
}

static void run_speedrun(const maze_t &walls, maze_cell_t start, const vector<maze_cell_t> &goals,
                          vector<maze_action_t> &actions, vector<path_point_t> &path) {
    static maze_action_t buf[MAZE_MAX_PATH_LEN];
    int n = maze_plan_min_turn_path(&walls, start, MAZE_NORTH, goals.data(), (int) goals.size(),
                                     buf, MAZE_MAX_PATH_LEN);

    int x = start.x;
    int y = start.y;
    maze_dir_t heading = MAZE_NORTH;
    path.push_back({x, y, heading});

    for (int i = 0; i < n; i++) {
        actions.push_back(buf[i]);
        if (buf[i] == MAZE_ACTION_FORWARD) {
            maze_step(&x, &y, heading);
        } else if (buf[i] == MAZE_ACTION_TURN_LEFT) {
            heading = maze_turn_left(heading);
        } else {
            heading = maze_turn_right(heading);
        }
        path.push_back({x, y, heading});
    }
}

static vector<maze_cell_t> default_goals() {
    return {
        { (int8_t) ((MAZE_WIDTH - 1) / 2), (int8_t) ((MAZE_HEIGHT - 1) / 2) },
        { (int8_t) (MAZE_WIDTH / 2),       (int8_t) ((MAZE_HEIGHT - 1) / 2) },
        { (int8_t) ((MAZE_WIDTH - 1) / 2), (int8_t) (MAZE_HEIGHT / 2) },
        { (int8_t) (MAZE_WIDTH / 2),       (int8_t) (MAZE_HEIGHT / 2) }
    };
}

static void print_path(const vector<path_point_t> &path) {
    printf("[");
    for (size_t i = 0; i < path.size(); i++) {
        int px, py; maze_dir_t ph;
        tie(px, py, ph) = path[i];
        printf("%s{\"x\":%d,\"y\":%d,\"heading\":\"%s\"}", i ? "," : "", px, py, dir_name(ph));
    }
    printf("]");
}

static void print_actions(const vector<maze_action_t> &actions) {
    printf("[");
    for (size_t i = 0; i < actions.size(); i++) {
        printf("%s\"%s\"", i ? "," : "", action_name(actions[i]));
    }
    printf("]");
}

static void count_moves_turns(const vector<maze_action_t> &actions, int *moves, int *turns) {
    *moves = 0;
    *turns = 0;
    for (maze_action_t a : actions) {
        if (a == MAZE_ACTION_FORWARD) (*moves)++; else (*turns)++;
    }
}

int main() {
    string mode = "search";
    maze_cell_t start = { 0, 0 };
    vector<maze_cell_t> goals;
    bool random_maze = false;
    unsigned seed = 42;
    bool have_walls = false;
    maze_t input_walls;
    maze_init(&input_walls);

    string key;
    while (cin >> key) {
        if (key == "MODE") {
            cin >> mode;
        } else if (key == "WIDTH" || key == "HEIGHT") {
            int tmp; cin >> tmp; // accepted, ignored - maze size is compile-time fixed
        } else if (key == "START") {
            int x, y; cin >> x >> y;
            start = { (int8_t) x, (int8_t) y };
        } else if (key == "GOALS") {
            int n; cin >> n;
            goals.clear();
            for (int i = 0; i < n; i++) {
                int x, y; cin >> x >> y;
                goals.push_back({ (int8_t) x, (int8_t) y });
            }
        } else if (key == "RANDOM") {
            int v; cin >> v;
            random_maze = (v != 0);
        } else if (key == "SEED") {
            cin >> seed;
        } else if (key == "WALLS") {
            int n; cin >> n;
            have_walls = true;
            for (int i = 0; i < n && i < MAZE_CELL_COUNT; i++) {
                int w; cin >> w;
                input_walls.walls[i] = (uint8_t) w;
            }
        }
        // unknown keys ignored - forward compatible
    }

    if (goals.empty()) goals = default_goals();

    if (mode == "search") {
        maze_t truth;
        if (random_maze || !have_walls) {
            generate_random_maze(&truth, seed);
        } else {
            truth = input_walls;
        }

        SearchResult result;
        run_search(truth, start, goals, result);

        int moves, turns;
        count_moves_turns(result.actions, &moves, &turns);
        long est_ms = (long) moves * SOLVER_CELL_MOVE_TIME_MS + (long) turns * SOLVER_TURN_90_TIME_MS;

        printf("{\"ok\":true,\"width\":%d,\"height\":%d,", MAZE_WIDTH, MAZE_HEIGHT);
        printf("\"start\":{\"x\":%d,\"y\":%d},", start.x, start.y);
        printf("\"goals\":[");
        for (size_t i = 0; i < goals.size(); i++) {
            printf("%s{\"x\":%d,\"y\":%d}", i ? "," : "", goals[i].x, goals[i].y);
        }
        printf("],");
        printf("\"ground_truth_walls\":[");
        for (int i = 0; i < MAZE_CELL_COUNT; i++) printf("%s%d", i ? "," : "", truth.walls[i]);
        printf("],");
        printf("\"search\":{\"path\":");
        print_path(result.path);
        printf(",\"actions\":");
        print_actions(result.actions);
        printf(",\"discovered_walls\":[");
        for (int i = 0; i < MAZE_CELL_COUNT; i++) printf("%s%d", i ? "," : "", result.discovered.walls[i]);
        printf("],");
        printf("\"reached_center_at_step\":%d,\"total_moves\":%d,\"total_turns\":%d,\"est_time_ms\":%ld}}\n",
               result.reached_center_at_step, moves, turns, est_ms);
        return 0;
    }

    if (mode == "speedrun") {
        if (!have_walls) {
            printf("{\"ok\":false,\"error\":\"speedrun mode requires WALLS (a fully-known map)\"}\n");
            return 1;
        }

        vector<maze_action_t> actions;
        vector<path_point_t> path;
        run_speedrun(input_walls, start, goals, actions, path);

        int moves, turns;
        count_moves_turns(actions, &moves, &turns);
        long est_ms = (long) moves * SOLVER_CELL_MOVE_TIME_MS + (long) turns * SOLVER_TURN_90_TIME_MS;

        printf("{\"ok\":true,\"width\":%d,\"height\":%d,\"speedrun\":{\"path\":", MAZE_WIDTH, MAZE_HEIGHT);
        print_path(path);
        printf(",\"actions\":");
        print_actions(actions);
        printf(",\"total_moves\":%d,\"total_turns\":%d,\"est_time_ms\":%ld}}\n", moves, turns, est_ms);
        return 0;
    }

    printf("{\"ok\":false,\"error\":\"unknown mode '%s'\"}\n", mode.c_str());
    return 1;
}
