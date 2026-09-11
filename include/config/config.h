#ifndef CONFIG_CONFIG_H
#define CONFIG_CONFIG_H

// Single include point for every tuning/config file. Application code
// should generally just `#include "config/config.h"` rather than reaching
// for the individual files below.
#include "config/pins.h"
#include "config/robot_physical.h"
#include "config/maze_layout.h"
#include "config/sensor_calibration.h"
#include "config/motion_tuning.h"
#include "config/turn_params.h"

#endif // CONFIG_CONFIG_H
