#include "drive_modes/differential.h"
#include<cmath>
DifferentialMode::DifferentialMode() {
    ROS_INFO("DifferentialMode constructor");
    this->set_velocity_point(0.0, 0.0);
}

void DifferentialMode::set_axle(double x, double y, double a) {
    // Find allowed subspace of the 3 dimensional velocity vector space.
    // 0 is always in the subspace, as the robot must be able stop. [nx ny nr]
    // is the normal vector of the subspace.
    
    nx = -std::sin(a);
    ny = std::cos(a);
    nr = (x - this->x) * std::cos(a) + (y - this->y) * std::sin(a);

    double n = this->norm(nx, ny, nr);
    nx /= n;
    ny /= n;
    nr /= n;
}

void DifferentialMode::apply(double& vx, double& vy, double& va) {
    this->project_onto_plane(vx, vy, va, nx, ny, nr);
}

void DifferentialMode::get_normal(double& x, double& y, double& r) {
    x = this->nx;
    y = this->ny;
    r = this->nr;
}

void
DifferentialMode::set_parameters(robotrainer_controllers::DriveModeParameters&
dm_params) {
    this->set_velocity_point(dm_params.velocity_point_x,
                             dm_params.velocity_point_y);

    this->set_axle(dm_params.virtual_axle.x,
                   dm_params.virtual_axle.y,
                   dm_params.virtual_axle.a);

    // Respond with the normal. This is not necessary. Just for debugging.
    // It is ugly. They are different vector spaces.
    this->get_normal(dm_params.virtual_axle.x,
                     dm_params.virtual_axle.y,
                     dm_params.virtual_axle.a);
}
