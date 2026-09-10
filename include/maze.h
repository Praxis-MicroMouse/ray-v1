#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

// Maze grid representation + flood-fill path search, ported from the
// mms-c simulator reference algorithm (../mms-c/Main.c) so the same
// search logic runs on the real robot. Pure grid math here - no sensor
// or motor calls - so it stays testable independent of hardware. See
// solver.h for the piece that drives the real robot using this module.

// Standard full-size micromouse maze. Change to match the maze actually
// being run (e.g. 8x8 for a quarter-maze/half-size contest).
#define MAZE_WIDTH  16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)

typedef enum {
    MAZE_NORTH = 0,
    MAZE_EAST  = 1,
    MAZE_SOUTH = 2,
    MAZE_WEST  = 3
} maze_dir_t;

enum {
    MAZE_WALL_NORTH = 1 << MAZE_NORTH,
    MAZE_WALL_EAST  = 1 << MAZE_EAST,
    MAZE_WALL_SOUTH = 1 << MAZE_SOUTH,
    MAZE_WALL_WEST  = 1 << MAZE_WEST
};

typedef struct {
    int8_t x;
    int8_t y;
} maze_cell_t;

typedef struct {
    uint8_t walls[MAZE_CELL_COUNT]; // bitmask of MAZE_WALL_* per cell
    int16_t dist[MAZE_CELL_COUNT];  // flood-fill distance-to-goal, cell steps
} maze_t;

maze_dir_t maze_turn_left(maze_dir_t dir);
maze_dir_t maze_turn_right(maze_dir_t dir);
maze_dir_t maze_opposite(maze_dir_t dir);

// Advances (x, y) one cell in the given direction.
void maze_step(int *x, int *y, maze_dir_t dir);

int maze_in_bounds(int x, int y);
int maze_index(int x, int y);

// Clears all cell walls except the maze's outer boundary.
void maze_init(maze_t *maze);

// Marks the wall on (x, y)'s `dir` side, and the matching wall on the
// neighboring cell (if any), as present/absent.
void maze_set_wall(maze_t *maze, int x, int y, maze_dir_t dir);
void maze_clear_wall(maze_t *maze, int x, int y, maze_dir_t dir);

// Multi-source BFS: fills maze->dist[] with each cell's shortest known
// distance (in cell-steps) to the nearest goal cell, respecting walls
// discovered so far. Unreachable cells are left at INT16_MAX.
void maze_flood_fill(maze_t *maze, const maze_cell_t *goals, int goal_count);

// Picks the open neighbor of (x, y) with the lowest flood-fill distance
// (call maze_flood_fill() first). Falls back to `heading` if every
// neighbor is walled off or unreachable.
maze_dir_t maze_choose_next_direction(const maze_t *maze, int x, int y, maze_dir_t heading);

#endif // MAZE_H
