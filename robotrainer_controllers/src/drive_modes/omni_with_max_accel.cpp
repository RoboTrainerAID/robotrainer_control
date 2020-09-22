#include "drive_modes/omni_with_max_accel.h"

OmniModeWithMaxAccel::OmniModeWithMaxAccel() {
    ROS_INFO("OmniModeWithMaxAccel constructor");
    this->last_vx = 0.0;
    this->last_vy = 0.0;
}

void OmniModeWithMaxAccel::apply(double& vx, double& vy, double& va) {
    double dif_x = (vx - this->last_vx);
    double dif_y = (vy - this->last_vy);
    double requested_accel = norm(dif_x, dif_y, 0.0);

    if (requested_accel > this->max_accel) {
        vx = this->last_vx + dif_x * (this->max_accel) / requested_accel;
        vy = this->last_vy + dif_y * (this->max_accel) / requested_accel;
    }
}

void OmniModeWithMaxAccel::set_parameters(
robotrainer_controllers::DriveModeParameters& dm_params) {
    // THIS IS AFFECTED BY THE PLATFORM'S UPDATE FREQUENCY.
    this->max_accel = dm_params.p;
    if (this->max_accel < 0) this->max_accel = 0;
}
