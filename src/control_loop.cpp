#include "control_loop.h"

#include "odometry.h"
#include "motion.h"
#include "drive_controller.h"

void control_loop_init(void) {
    odometry_init();
    motion_init();
    drive_controller_init();
}

void control_loop_tick(void) {
    odometry_update();
    motion_update();
    drive_controller_update();
}
