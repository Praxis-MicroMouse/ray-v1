#include "maze.h"

#include <limits.h>
#include <stdint.h>

maze_dir_t maze_turn_left(maze_dir_t dir) {
    return (maze_dir_t)((dir + 3) % 4);
}

maze_dir_t maze_turn_right(maze_dir_t dir) {
    return (maze_dir_t)((dir + 1) % 4);
}

maze_dir_t maze_opposite(maze_dir_t dir) {
    return (maze_dir_t)((dir + 2) % 4);
}

void maze_step(int *x, int *y, maze_dir_t dir) {
    if (dir == MAZE_NORTH) {
        (*y)++;
    } else if (dir == MAZE_EAST) {
        (*x)++;
    } else if (dir == MAZE_SOUTH) {
        (*y)--;
    } else {
        (*x)--;
    }
}

int maze_in_bounds(int x, int y) {
    return x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT;
}

int maze_index(int x, int y) {
    return y * MAZE_WIDTH + x;
}

static int wall_mask(maze_dir_t dir) {
    return 1 << dir;
}

void maze_init(maze_t *maze) {
    for (int i = 0; i < MAZE_CELL_COUNT; i++) {
        maze->walls[i] = 0;
        maze->dist[i] = INT16_MAX;
    }

    for (int y = 0; y < MAZE_HEIGHT; y++) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            int index = maze_index(x, y);
            if (y == MAZE_HEIGHT - 1) maze->walls[index] |= MAZE_WALL_NORTH;
            if (x == MAZE_WIDTH - 1)  maze->walls[index] |= MAZE_WALL_EAST;
            if (y == 0)               maze->walls[index] |= MAZE_WALL_SOUTH;
            if (x == 0)               maze->walls[index] |= MAZE_WALL_WEST;
        }
    }
}

void maze_set_wall(maze_t *maze, int x, int y, maze_dir_t dir) {
    int nx = x;
    int ny = y;
    maze_step(&nx, &ny, dir);

    maze->walls[maze_index(x, y)] |= wall_mask(dir);
    if (maze_in_bounds(nx, ny)) {
        maze->walls[maze_index(nx, ny)] |= wall_mask(maze_opposite(dir));
    }
}

void maze_clear_wall(maze_t *maze, int x, int y, maze_dir_t dir) {
    int nx = x;
    int ny = y;
    maze_step(&nx, &ny, dir);

    maze->walls[maze_index(x, y)] &= ~wall_mask(dir);
    if (maze_in_bounds(nx, ny)) {
        maze->walls[maze_index(nx, ny)] &= ~wall_mask(maze_opposite(dir));
    }
}

void maze_flood_fill(maze_t *maze, const maze_cell_t *goals, int goal_count) {
    static maze_cell_t queue[MAZE_CELL_COUNT];

    for (int i = 0; i < MAZE_CELL_COUNT; i++) {
        maze->dist[i] = INT16_MAX;
    }

    int head = 0;
    int tail = 0;
    for (int i = 0; i < goal_count; i++) {
        int idx = maze_index(goals[i].x, goals[i].y);
        if (maze->dist[idx] > 0) {
            maze->dist[idx] = 0;
            queue[tail++] = goals[i];
        }
    }

    while (head < tail) {
        maze_cell_t current = queue[head++];
        int current_index = maze_index(current.x, current.y);
        int current_dist = maze->dist[current_index];

        for (maze_dir_t dir = MAZE_NORTH; dir <= MAZE_WEST; dir = (maze_dir_t)(dir + 1)) {
            if (maze->walls[current_index] & wall_mask(dir)) {
                continue;
            }

            int nx = current.x;
            int ny = current.y;
            maze_step(&nx, &ny, dir);
            if (!maze_in_bounds(nx, ny)) {
                continue;
            }

            int next_index = maze_index(nx, ny);
            if (maze->dist[next_index] > current_dist + 1) {
                maze->dist[next_index] = (int16_t)(current_dist + 1);
                queue[tail++] = (maze_cell_t){ (int8_t)nx, (int8_t)ny };
            }
        }
    }
}

maze_dir_t maze_choose_next_direction(const maze_t *maze, int x, int y, maze_dir_t heading) {
    maze_dir_t best_dir = heading;
    int best_dist = INT16_MAX;
    int current_index = maze_index(x, y);

    for (maze_dir_t dir = MAZE_NORTH; dir <= MAZE_WEST; dir = (maze_dir_t)(dir + 1)) {
        if (maze->walls[current_index] & wall_mask(dir)) {
            continue;
        }

        int nx = x;
        int ny = y;
        maze_step(&nx, &ny, dir);
        if (!maze_in_bounds(nx, ny)) {
            continue;
        }

        int next_dist = maze->dist[maze_index(nx, ny)];
        if (next_dist < best_dist) {
            best_dist = next_dist;
            best_dir = dir;
        }
    }

    return best_dir;
}

#define MAZE_STATE_COUNT (MAZE_CELL_COUNT * 4)

static int state_index(int cell_index, maze_dir_t heading) {
    return cell_index * 4 + heading;
}

int maze_plan_min_turn_path(const maze_t *maze, maze_cell_t start, maze_dir_t start_heading,
                             const maze_cell_t *goals, int goal_count,
                             maze_action_t *out_actions, int max_actions) {
    static int32_t dist[MAZE_STATE_COUNT];
    static int32_t prev_state[MAZE_STATE_COUNT];
    static maze_action_t prev_action[MAZE_STATE_COUNT];
    static uint8_t visited[MAZE_STATE_COUNT];

    for (int i = 0; i < MAZE_STATE_COUNT; i++) {
        dist[i] = INT32_MAX;
        prev_state[i] = -1;
        visited[i] = 0;
    }

    int start_state = state_index(maze_index(start.x, start.y), start_heading);
    dist[start_state] = 0;

    // Plain O(V^2) Dijkstra (V = MAZE_STATE_COUNT, at most 1024 for a
    // 16x16 maze) - no heap needed at this size, and it's a few
    // milliseconds at most on an ESP32.
    for (int iter = 0; iter < MAZE_STATE_COUNT; iter++) {
        int u = -1;
        int32_t best = INT32_MAX;
        for (int i = 0; i < MAZE_STATE_COUNT; i++) {
            if (!visited[i] && dist[i] < best) {
                best = dist[i];
                u = i;
            }
        }
        if (u < 0) {
            break; // everything left unvisited is unreachable
        }
        visited[u] = 1;

        int cell_idx = u / 4;
        maze_dir_t heading = (maze_dir_t)(u % 4);
        int x = cell_idx % MAZE_WIDTH;
        int y = cell_idx / MAZE_WIDTH;

        if (!(maze->walls[cell_idx] & (1 << heading))) {
            int nx = x;
            int ny = y;
            maze_step(&nx, &ny, heading);
            if (maze_in_bounds(nx, ny)) {
                int v = state_index(maze_index(nx, ny), heading);
                int32_t nd = dist[u] + MAZE_MOVE_COST;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    prev_state[v] = u;
                    prev_action[v] = MAZE_ACTION_FORWARD;
                }
            }
        }

        int v_left = state_index(cell_idx, maze_turn_left(heading));
        int32_t nd_left = dist[u] + MAZE_TURN_COST;
        if (nd_left < dist[v_left]) {
            dist[v_left] = nd_left;
            prev_state[v_left] = u;
            prev_action[v_left] = MAZE_ACTION_TURN_LEFT;
        }

        int v_right = state_index(cell_idx, maze_turn_right(heading));
        int32_t nd_right = dist[u] + MAZE_TURN_COST;
        if (nd_right < dist[v_right]) {
            dist[v_right] = nd_right;
            prev_state[v_right] = u;
            prev_action[v_right] = MAZE_ACTION_TURN_RIGHT;
        }
    }

    int best_goal_state = -1;
    int32_t best_goal_dist = INT32_MAX;
    for (int g = 0; g < goal_count; g++) {
        int cell_idx = maze_index(goals[g].x, goals[g].y);
        for (maze_dir_t h = MAZE_NORTH; h <= MAZE_WEST; h = (maze_dir_t)(h + 1)) {
            int s = state_index(cell_idx, h);
            if (dist[s] < best_goal_dist) {
                best_goal_dist = dist[s];
                best_goal_state = s;
            }
        }
    }

    if (best_goal_state < 0 || best_goal_dist == INT32_MAX) {
        return 0;
    }

    static maze_action_t reversed[MAZE_MAX_PATH_LEN];
    int count = 0;
    int s = best_goal_state;
    while (prev_state[s] != -1 && count < MAZE_MAX_PATH_LEN) {
        reversed[count++] = prev_action[s];
        s = prev_state[s];
    }

    int n = (count < max_actions) ? count : max_actions;
    for (int i = 0; i < n; i++) {
        out_actions[i] = reversed[count - 1 - i];
    }
    return n;
}
