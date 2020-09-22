/*
This mode allows you to modify the drive command "x" using linear algebra:
x_new = Ax+b
*/

#ifndef AXPLUSB_MODE_H
#define AXPLUSB_MODE_H

#include "drive_modes/drive_mode_base_class.h"
#include <Eigen/Geometry>

class AxPLUSbMode : public DriveMode {
private:
    Eigen::Matrix<double, 3, 3> A;
    Eigen::Vector3d b;
public:
    void set_parameters(robotrainer_controllers::DriveModeParameters& dm_params);
    void apply(double& vx, double& vy, double& va);
};

#endif
