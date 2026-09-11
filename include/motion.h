#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

// High-level locomotion, built on two trapezoidal profile_t instances
// (profile.h) - one for forward motion, one for rotation - mirroring
// ukmarsbots' Motion class. Converts "move this far at this speed" /
// "turn this many degrees" requests into continuously-updated targets;
// drive_controller.cpp is what actually turns those targets into motor
// voltages every control tick.
//
// Threading model: motion_update() must only be called from the control
// task (control_loop.h), once per tick, after odometry_update(). Every
// other function here is cross-core-safe and may be called from any
// task - in particular, mouse.cpp (running on the mouse/planning task)
// calls motion_move()/motion_turn()/etc, which busy-wait on this
// module's locked accessors while the control task does the real work
// in the background.

void motion_init(void);

// Stops the robot, resets odometry + both profiles, and re-enables the
// drive controller - call before starting a new sequence of moves.
void motion_reset_drive_system(void);

void motion_stop(void);           // both profiles target zero speed
void motion_disable_drive(void);  // drive_controller_disable() - motors go idle
void motion_emergency_stop(void); // reset_drive_system() + disable_drive()

// Advances both profiles by one control-loop tick. CONTROL-TASK ONLY.
void motion_update(void);

float motion_position(void);     // forward profile position, mm
float motion_velocity(void);     // forward profile speed, mm/s
float motion_acceleration(void); // forward profile acceleration, mm/s/s

float motion_angle(void);        // rotation profile position, deg
float motion_omega(void);        // rotation profile speed, deg/s
float motion_alpha(void);        // rotation profile acceleration, deg/s/s

void motion_set_target_velocity(float velocity_mm_s);

void motion_start_move(float distance_mm, float top_speed, float final_speed, float acceleration);
bool motion_move_finished(void);
// Blocking: starts the move and waits for it to finish (button-abortable).
void motion_move(float distance_mm, float top_speed, float final_speed, float acceleration);

void motion_start_turn(float angle_deg, float top_speed, float final_speed, float acceleration);
bool motion_turn_finished(void);
void motion_turn(float angle_deg, float top_speed, float final_speed, float acceleration);

// In-place spin turn: waits for forward speed to reach zero first, then
// pivots. Blocking.
void motion_spin_turn(float angle_deg, float omega_deg_s, float alpha_deg_s2);

void motion_set_position(float position_mm);
void motion_adjust_forward_position(float delta_mm);

// Busy-waits (button-abortable) until the forward profile reaches the
// given absolute position / has moved the given further distance.
void motion_wait_until_position(float position_mm);
void motion_wait_until_distance(float distance_mm);

// Robot is assumed moving; re-targets the current move to stop exactly
// at `position`/after `distance`, using the current speed/acceleration.
void motion_stop_at(float position_mm);
void motion_stop_after(float distance_mm);

#endif // MOTION_H
