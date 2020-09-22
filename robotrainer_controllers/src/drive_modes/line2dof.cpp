#include "drive_modes/line2dof.h"

#include <cmath>

void Line2DOF::set_parameters(robotrainer_controllers::DriveModeParameters& 
    dm_params) {
    X = dm_params.velocity_point_x;
    Y = dm_params.velocity_point_y;
    p = dm_params.p;
}
void Line2DOF::dr_apply(double& vx, double& vy, double& va) {

    double c = std::cos(p-pos_a);
    double s = std::sin(p-pos_a);

    double ct = std::abs(std::cos(pos_a)); // cos(theta)
    double new_vx = va*(Y - Y*c*c*ct + X*c*s*ct) + vx*c*c*ct + vy*c*s*ct;
    double new_vy = vy*s*s*ct - va*(X - X*s*s*ct + Y*c*s*ct) + vx*c*s*ct;
    ROS_INFO("%lf pos_a ", pos_a);
    
    vx = new_vx;
    vy = new_vy;
}
