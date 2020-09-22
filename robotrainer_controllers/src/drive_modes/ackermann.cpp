/*
Beware: This implementation only works for virtual axes parallel to the y-axis,
centered on the x-axis.
*/

#include "drive_modes/ackermann.h"

#include <cmath>

void AckermannMode::set_axle(double x, double y, double a) {
    // Contrary to DifferentialMode, AckermannMode needs to keep track of the
    // axle definition in order to ensure minimum turning radius is respected.
    this->axle_x = x;
    this->axle_y = y;
    this->axle_a = a;
    DifferentialMode::set_axle(x, y, a);
}

void AckermannMode::set_parameters(robotrainer_controllers::DriveModeParameters&
dm_params) {
    this->set_axle(dm_params.virtual_axle.x,
                   dm_params.virtual_axle.y,
                   dm_params.virtual_axle.a);
    this->r_min = dm_params.r_min;
}

void AckermannMode::apply(double& vx, double& vy, double& va) {
    DifferentialMode::apply(vx, vy, va);

    double old_angular_speed = va;
    double max_angular_speed = std::abs(vx * std::cos(axle_a)
                                      + vy * std::sin(axle_a)) / r_min;

    if (max_angular_speed < va) {
        old_angular_speed = va;
        va = max_angular_speed;

        vy *= (max_angular_speed/old_angular_speed);
    }
    else if (-max_angular_speed > va) {
        old_angular_speed = va;
        va = -max_angular_speed;

        vy *= (-max_angular_speed/old_angular_speed);
    }
}
