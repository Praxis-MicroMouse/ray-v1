#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

#include "config/maze_layout.h"

// Maze grid representation + flood-fill path search, ported from the
// mms-c simulator reference algorithm (../mms-c/Main.c) so the same
// search logic runs on the real robot. Pure grid math here - no sensor
// or motor calls - so it stays testable independent of hardware. See
// mouse.h for the piece that drives the real robot using this module.
//
// MAZE_WIDTH/HEIGHT/CELL_SIZE_MM and friends now live in
// config/maze_layout.h alongside every other tuning constant - this
// header just uses them.

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

// ---- Turn-minimizing path planning (for a fully-explored maze) ----
//
// maze_flood_fill()/maze_choose_next_direction() greedily minimize cell
// count only, and work fine on a partially-explored maze (that's what
// they're for during exploration). Once the maze is fully mapped, a
// competition run wants the fastest path, and turns cost real time
// (decelerate, pivot, accelerate) that a cell-count-only metric ignores.
//
// maze_plan_min_turn_path() runs Dijkstra over an expanded state graph
// of (cell, heading) - not just cell - where "drive forward one cell"
// and "turn 90 degrees in place" are separate weighted edges. Weighting
// turns higher than a forward move biases the shortest *weighted* path
// toward fewer turns whenever a similarly-short alternative exists, at
// the cost of assuming the wall map is trustworthy (unlike flood fill,
// this won't self-correct if walls are still just guesses).

typedef enum {
    MAZE_ACTION_FORWARD = 0,
    MAZE_ACTION_TURN_LEFT,
    MAZE_ACTION_TURN_RIGHT
} maze_action_t;

// Finds the min-cost (fewest-turns-biased) path from (start, start_heading)
// to whichever goal cell/heading combination is cheapest to reach, given
// the walls currently known in `maze`. Writes up to max_actions actions
// into out_actions (in the order they should be executed) and returns
// how many were written - 0 if start is already a goal or no path exists.
int maze_plan_min_turn_path(const maze_t *maze, maze_cell_t start, maze_dir_t start_heading,
                             const maze_cell_t *goals, int goal_count,
                             maze_action_t *out_actions, int max_actions);

#endif // MAZE_H
