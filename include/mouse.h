#ifndef MOUSE_H
#define MOUSE_H

// High-level maze-solving behavior, mirroring ukmarsbots' Mouse class:
// search to the center, search back to the start, then plan and execute
// a turn-minimizing speed run - all built on motion.h (profile-based
// moves/turns), sensor.h/steering.h (wall sensing + continuous
// centering), and maze.h (flood fill + Dijkstra planner, unchanged from
// before this rewrite).
//
// Runs entirely on the "mouse" task (tasks.cpp) - a plain blocking
// sequence of motion_move()/motion_turn() calls, exactly like
// ukmarsbots' Mouse methods block the AVR's single core. The actual
// hardware I/O and control math happen on the other two tasks
// (control_loop.h, sensor_loop.h) in the background; this module just
// waits on their shared, lock-protected state (see motion.h's threading
// note).
//
// Positive rotation angles mean a LEFT (counter-clockwise) turn
// throughout this module and config/turn_params.h - same convention
// ukmarsbots uses. If a turn goes the wrong way on real hardware, fix it
// in config/robot_physical.h (flip a *_POLARITY), not by renegating
// angles here.
//
// Scope note: this is a from-scratch v1 of the search/speed-run logic,
// not a port of every ukmarsbots Mouse method - hand-start detection
// (wait_for_user_start()), wall-following/wander test modes, and the
// bench calibration routines (conf_log_front_sensor(), edge detection,
// sensor-spin calibration) are deliberately left out for now. Every run
// currently assumes the mouse starts backed up against the wall behind
// it at (0,0) facing north.

void mouse_init(void);

// Blocking: explores to the maze center, returns to the start, plans the
// fastest known path, and drives it. Returns once the speed run
// finishes (or panic() halts on an unrecoverable error - see mouse.cpp).
void mouse_run(void);

#endif // MOUSE_H
