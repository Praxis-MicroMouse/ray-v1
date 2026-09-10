#include "maze.h"

#include <limits.h>

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
