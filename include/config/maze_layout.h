#ifndef CONFIG_MAZE_LAYOUT_H
#define CONFIG_MAZE_LAYOUT_H

// ===========================================================================
// MAZE SIZE + PER-CELL POSITIONING CONSTANTS.
// Pure numbers, no Arduino dependency, so this is safe to include from
// native (host-built) unit tests too.
// ===========================================================================

// Standard full-size micromouse maze. Change to match whatever's actually
// being run (e.g. 8x8 for a quarter-maze/half-size contest, or a small
// practice maze on the bench).
#define MAZE_WIDTH  16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)

// Physical size of one cell (measured: 18cm x 18cm).
#define MAZE_CELL_SIZE_MM 180.0f

// Edge weights for maze_plan_min_turn_path()'s Dijkstra planner - turning
// costs more than a move so the planner prefers fewer turns over a path
// that's only marginally shorter in cells. Tune the ratio to taste.
#define MAZE_MOVE_COST 2
#define MAZE_TURN_COST 3

// Generous upper bound on actions in a planned path (worst case: no state
// revisited).
#define MAZE_MAX_PATH_LEN (MAZE_CELL_COUNT * 2)

// ---- Per-cell positioning (mirrors ukmarsbots' BACK_WALL_TO_CENTER /
//      SENSING_POSITION) ----

// With the mouse backed up against the wall behind it (start of a run),
// how far (mm) is its position-tracking origin from the current cell's
// center? TODO: measure - back the mouse up to a wall, note wheel-axle
// position relative to the cell boundary.
#define BACK_WALL_TO_CENTER_MM 48.0f

// Position (mm, measured from the cell entry boundary) at which mouse.cpp
// samples the ToF sensors to decide the next move. Must be comfortably
// less than MAZE_CELL_SIZE_MM so the decision is made before the mouse
// reaches the next cell boundary.
#define SENSING_POSITION_MM 90.0f

#endif // CONFIG_MAZE_LAYOUT_H
