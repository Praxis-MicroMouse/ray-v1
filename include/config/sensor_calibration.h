#ifndef CONFIG_SENSOR_CALIBRATION_H
#define CONFIG_SENSOR_CALIBRATION_H

// ===========================================================================
// PER-SENSOR ToF (VL53L0X) CALIBRATION.
//
// Unlike ukmarsbots' analog IR sensors, a VL53L0X reports absolute
// distance directly - there's no inverse-square linearization curve to
// fit, and no per-venue lighting recalibration needed. What's still worth
// calibrating per sensor is a small linear correction for that specific
// unit/mounting:
//
//   corrected_mm = raw_mm * TOF_<X>_SCALE + TOF_<X>_OFFSET_MM
//
// Calibration procedure:
//   1. Place the mouse so the sensor in question is exactly NEAR_MM from
//      a flat wall (use a ruler/spacer block). Read raw_near.
//   2. Move it back to exactly FAR_MM. Read raw_far.
//   3. TOF_<X>_SCALE      = (FAR_MM - NEAR_MM) / (raw_far - raw_near)
//      TOF_<X>_OFFSET_MM  = NEAR_MM - TOF_<X>_SCALE * raw_near
// Defaults below (scale=1, offset=0) assume the sensor is already accurate
// out of the box, which VL53L0X usually is within a few mm - only bother
// tuning these if you measure a consistent bias.
// ===========================================================================

#define TOF_FRONT_SCALE      1.0f
#define TOF_FRONT_OFFSET_MM  0.0f
#define TOF_RIGHT_SCALE      1.0f
#define TOF_RIGHT_OFFSET_MM  0.0f
#define TOF_LEFT_SCALE       1.0f
#define TOF_LEFT_OFFSET_MM   0.0f

// Distance (mm) from each sensor's optical center to the robot's true
// front/side edge - subtract this from a corrected reading if you need
// "distance from robot body" rather than "distance from sensor" (used by
// mouse.cpp's stopAtCentre()-style front-wall referencing).
// TODO: measure with calipers.
#define TOF_FRONT_MOUNT_OFFSET_MM 25.0f
#define TOF_RIGHT_MOUNT_OFFSET_MM 35.0f
#define TOF_LEFT_MOUNT_OFFSET_MM  35.0f

// A wall is "present" if the corrected reading is under this many mm.
// Defaults to half a cell (see MAZE_CELL_SIZE_MM in maze_layout.h) -
// re-tune if sensor mounting or cell size changes.
#define TOF_WALL_THRESHOLD_FRONT_MM 90
#define TOF_WALL_THRESHOLD_SIDE_MM  90

// Expected front reading (mm, corrected) when the mouse is centered in a
// cell with a wall directly ahead - used by mouse.cpp's adjustPosition()
// to fine-tune stopping position against that wall.
// TODO: measure - center the mouse by eye/jig against a front wall and
// read sensor_get_latest().front_mm.
#define TOF_FRONT_REFERENCE_MM 45

// Above this front reading, side-sensor readings are masked out of
// steering/wall-detection: at very close range a VL53L0X's narrow-FoV
// beam can still pick up cross-talk off a wall dead ahead, corrupting the
// side channels. (VL53L0X is far less prone to this than reflectance IR,
// which is what ukmarsbots' FRONT_WALL_RELIABILITY_LIMIT exists for - this
// is a cheap defensive threshold, not a hard requirement.)
#define TOF_FRONT_RELIABILITY_LIMIT_MM 60

#define TOF_MAX_RANGE_MM 2000

// Despike + EMA smoothing (see filter.h) - shared across all three ToF
// channels.
#define TOF_FILTER_SPIKE_THRESHOLD_MM 150.0f
#define TOF_FILTER_EMA_ALPHA          0.35f

// Time to wait after releasing a sensor's XSHUT (powering it on) before
// it's safe to talk to it over I2C - only one sensor is ever powered on
// at a time (see sensor.cpp's service_one_sensor()), so this delay is
// paid on every single reading, not just once at boot. 10ms is a
// conservative starting point (VL53L0X boot time is normally much
// shorter); if a sensor shows "init FAILED" only during the round-robin
// cycle - never at sensor_init()'s first pass - try raising this before
// suspecting the sensor itself.
#define TOF_XSHUT_BOOT_DELAY_MS 10

#endif // CONFIG_SENSOR_CALIBRATION_H
