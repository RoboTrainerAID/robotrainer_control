#include "drive_modes/AxPLUSb.h"

void AxPLUSbMode::set_parameters(robotrainer_controllers::DriveModeParameters&
dm_params) {
    this->A << 
    dm_params.a_and_b.a11, dm_params.a_and_b.a12, dm_params.a_and_b.a13, 
    dm_params.a_and_b.a21, dm_params.a_and_b.a22, dm_params.a_and_b.a23, 
    dm_params.a_and_b.a31, dm_params.a_and_b.a32, dm_params.a_and_b.a33; 
    this->b << 
    dm_params.a_and_b.b1, dm_params.a_and_b.b2, dm_params.a_and_b.b3;
}

void AxPLUSbMode::apply(double& vx, double& vy, double& va) {
    Eigen::Vector3d xi;
    xi << vx, vy, va;
    xi = A * xi + b;
    vx = xi[0];
    vy = xi[1];
    va = xi[2];
}
