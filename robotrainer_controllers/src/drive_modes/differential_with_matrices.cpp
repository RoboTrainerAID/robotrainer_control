#include "drive_modes/differential_with_matrices.h"
#include<cmath>
DifferentialModeMat::DifferentialModeMat() {
    ROS_INFO("DifferentialModeMat constructor");
    this->set_velocity_point(0.0, 0.0);
}

void DifferentialModeMat::apply(double& vx, double& vy, double& va) {
    // Matrix multiplication.
    double tmp;
    tmp = m11 * vx + m12 * vy + m13 * va;
    vy  = m21 * vx + m22 * vy + m23 * va;
    vx  = tmp;
}

void
DifferentialModeMat::set_parameters(robotrainer_controllers::DriveModeParameters&
dm_params) {
    const double x = dm_params.virtual_axle.x;
    const double y = dm_params.virtual_axle.y;
    const double a = dm_params.virtual_axle.a;
    const double c = std::cos(a);
    const double s = std::sin(a);

    m11 = c*c;
    m12 = c*s;
    m13 = y - y*c*c + x*c*s;
    
    m21 = c*s;
    m22 = s*s;
    m23 = x*s*s - x - y*c*s;
   
}
