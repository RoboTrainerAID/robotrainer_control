/*
Constrains the robot to 2 DOF. It is as if one point on the platform were
attached on a linear rail by a swivel joint.
*/

#ifndef LINE_2_DOF_H
#define LINE_2_DOF_H

#include "drive_modes/dead_reckon.h"

class Line2DOF : public DeadReckonMode {
private:
    double X, Y, p;
public:
    void dr_apply(double& vx, double& vy, double& va);
    void set_parameters(robotrainer_controllers::DriveModeParameters& 
    dm_params);
};

#endif
