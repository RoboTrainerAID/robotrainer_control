// TODO


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

void AckermannMode::set_parameters(struct drive_mode_parameters& dm_params) {
    this->set_axle(dm_params.x, dm_params.y, dm_params.a);
    this->r_min = dm_params.r_min;
}

void AckermannMode::apply(double& vx, double& vy, double& va) {
    DifferentialMode::apply(vx, vy, va);

    double max_angular_speed = std::abs(vx * std::sin(axle_a)
                                      + vy * std::cos(axle_a));

    if (max_angular_speed < va) {
        va = max_angular_speed;
    }
    else if (-max_angular_speed > va) {
        va = -max_angular_speed;
    }
}
