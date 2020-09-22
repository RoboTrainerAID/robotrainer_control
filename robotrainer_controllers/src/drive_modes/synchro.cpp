#include "drive_modes/synchro.h"

#include <limits>

SynchroMode::SynchroMode() {
    ROS_INFO("synchro mode constructor");
}

void SynchroMode::apply(double& vx, double& vy, double& va) {
    va = 0;
}
