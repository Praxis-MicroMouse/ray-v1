#ifndef SOLVER_H
#define SOLVER_H

#include <stdbool.h>

// Physical maze-solving run: search to the center, then back to the
// start, using maze.h for the flood-fill search and sensor.h/drive.h to
// sense walls and move the real robot. Ported from the mms-c simulator
// reference algorithm's main() (../mms-c/Main.c) - same search logic,
// swapping the simulator's API_* calls for the real hardware modules.
//
// Movement is open-loop (time-based) by default, same as the untuned
// drive.h motion test - SOLVER_CELL_MOVE_TIME_MS / SOLVER_TURN_90_TIME_MS
// below are guesses that need tuning against the real maze's cell size
// and the robot's actual speed. If encoder.h's pins get wired up and
// SOLVER_CELL_TICKS is set to a measured (non-zero) ticks-per-cell
// value, solver.cpp uses encoder-counted movement instead - see
// move_forward_one_cell() there.

// A wall is reported "present" if a ToF reading comes back under this
// threshold (mm) - half the real 180mm (18cm x 18cm) cell size (see
// MAZE_CELL_SIZE_MM in maze.h). Re-tune if sensor mounting changes.
#define SOLVER_WALL_THRESHOLD_MM 90

#define SOLVER_CELL_MOVE_TIME_MS 500  // time to drive exactly one cell forward - TUNE
#define SOLVER_TURN_90_TIME_MS   350  // time to pivot exactly 90 degrees - TUNE
#define SOLVER_CELL_TICKS        0    // encoder ticks per 180mm (MAZE_CELL_SIZE_MM) cell; 0 = use timed movement above - TODO once encoders are wired and calibrated

// Runs one full search-to-center-then-return-to-start pass, driving the
// real robot the whole way. Blocks until the run finishes. Requires
// sensor_init() (and, for tick-based movement, encoder_init()) to have
// been called already.
bool solver_run(void);

#endif // SOLVER_H
