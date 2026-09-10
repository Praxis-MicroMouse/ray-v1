#ifndef COMMS_H
#define COMMS_H

// Line-based serial protocol between the firmware and the tuning
// dashboard (tools/dashboard - see its README). Every line is either
// JSON (telemetry, out) or plain space-separated tokens (commands, in) -
// kept deliberately simple so both ends can parse it without a JSON
// parsing library (only formatting, on the firmware side).
//
// Incoming commands (one per line, newline-terminated):
//   PID <loop> <kp> <ki> <kd>   - loop in {straight, turn, wallcenter}
//   GETPID <loop>               - firmware replies with one {"pidcfg":...} line
//   RUN <maneuver> <arg>        - maneuver in {straight, turn, wallcenter, stop}
//                                   straight's arg    = target distance, mm
//                                   turn's arg        = target angle, degrees
//                                   wallcenter's arg  = duration, ms
//                                   stop takes no arg and aborts an in-progress RUN
//
// Outgoing telemetry (one JSON object per line - roughly every
// COMMS_TELEMETRY_PERIOD_MS via main.cpp's idle loop, and once per
// control-loop iteration during a RUN so the dashboard can plot the
// maneuver live):
//   {"t":<millis>,"tof":{"f":..,"r":..,"l":..},"batt":{"v":..,"pct":..},
//    "enc":{"l":..,"r":..,"dl_mm":..,"dr_mm":..},
//    "pwm":{"l":..,"r":..},
//    "imu":{"ax":..,"ay":..,"az":..,"gx":..,"gy":..,"gz":..,"temp":..},
//    "pid":{"loop":"straight","sp":..,"meas":..,"out":..,"active":true}}
// "pwm" is each wheel's last-commanded signed motor speed (-255..255,
// via motor_get_speed()) - what's actually driving the wheel right now,
// whether idle, mid-RUN-maneuver, or driven some other way.

void comms_init(void);

// Reads and dispatches any complete command lines currently buffered on
// Serial (non-blocking - returns immediately if nothing is available).
// A RUN command blocks inside this call until the maneuver finishes or
// is aborted (see control.h), during which comms_tick() keeps firing.
void comms_poll(void);

// Sends one telemetry line using the latest sensor/battery/encoder/IMU
// readings plus whatever control_get_debug() reports, then polls for an
// incoming "RUN stop" so a maneuver's Stop button works while it's still
// running. Pass this directly as the tick_cb to control_run_*(), and
// also call it periodically from loop() when idle.
void comms_tick(void);

#endif // COMMS_H
