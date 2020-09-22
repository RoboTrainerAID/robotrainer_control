#include "drive_modes/ackermann_smoother.h"

void AckermannSmoother::apply(double& vx, double& vy, double& va) {
    DifferentialMode::apply(vx, vy, va);

    double old_angular_speed = va;
    
    // Max angular speed possible with THIS particular linear speed.
    double max_angular_speed = std::abs(vx * std::cos(axle_a)
                                      + vy * std::sin(axle_a)) / r_min;

    va *= max_angular_speed / VA_MAX;
    vy *= max_angular_speed / VA_MAX;
}
