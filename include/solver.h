#ifndef SOLVER_H
#define SOLVER_H

#include <stdbool.h>

// Physical maze-solving run: search to the center, then back to the
// start, using maze.h for the flood-fill search and sensor.h to sense
// walls, with control.h/turns.h driving the real robot via closed-loop
// encoder feedback (dual-wheel PID sync per cell/turn, not open-loop
// timing). Ported from the mms-c simulator reference algorithm's main()
// (../mms-c/Main.c) - same search logic, swapping the simulator's API_*
// calls for the real hardware modules.

// A wall is reported "present" if a ToF reading comes back under this
// threshold (mm) - half the real 180mm (18cm x 18cm) cell size (see
// MAZE_CELL_SIZE_MM in maze.h). Re-tune if sensor mounting changes.
#define SOLVER_WALL_THRESHOLD_MM 90

// Runs one full search-to-center-then-return-to-start pass, driving the
// real robot the whole way. Blocks until the run finishes. Requires
// sensor_init() and encoder_init() to have been called already.
bool solver_run(void);

#endif // SOLVER_H
