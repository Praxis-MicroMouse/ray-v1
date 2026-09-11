#ifndef ODOMETRY_H
#define ODOMETRY_H

// Turns raw encoder tick counts (encoder.h) into robot-frame forward
// distance and heading, updated once per control tick.
//
// Threading note: odometry_update() and the plain odometry_robot_*()/
// odometry_*_change_*() accessors below must only ever be called from
// the fast control-loop task (see control_loop.h) - that's the hot path
// drive_controller.cpp uses every tick, kept lock-free since it's all
// single-task. odometry_distance_snapshot_mm()/odometry_angle_snapshot_deg()
// are the cross-core-safe exception: a locked copy taken at the end of
// every odometry_update(), for other tasks (bench.cpp's tests, mainly)
// that need to read actual measured position/heading rather than
// motion.h's commanded profile targets.

void odometry_init(void);

// Re-baselines tick tracking to the current encoder counts and zeroes the
// accumulated distance/angle. Call at the start of a run or maneuver
// sequence (mirrors motion_reset_drive_system()) - NOT every tick.
void odometry_reset(void);

// Reads the encoders, computes this tick's forward/rotation change, and
// folds it into the running totals. Call exactly once per control tick.
void odometry_update(void);

// Accumulated forward distance (mm) since the last odometry_reset().
float odometry_robot_distance_mm(void);

// Accumulated heading change (deg) since the last odometry_reset().
// Positive = clockwise/rightward, matching maze.h's turn-right convention.
float odometry_robot_angle_deg(void);

// This tick's forward/rotation change (mm / deg) - the raw per-tick
// deltas odometry_update() just folded into the running totals above.
float odometry_fwd_change_mm(void);
float odometry_rot_change_deg(void);

// Convenience rate accessors (this tick's change / dt).
float odometry_robot_speed_mm_s(float dt_s);
float odometry_robot_omega_deg_s(float dt_s);

// Cross-core-safe snapshots of odometry_robot_distance_mm()/
// odometry_robot_angle_deg(), safe to call from any task.
float odometry_distance_snapshot_mm(void);
float odometry_angle_snapshot_deg(void);

#endif // ODOMETRY_H
