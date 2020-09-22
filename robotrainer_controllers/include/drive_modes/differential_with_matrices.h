#ifndef DIFFERENTIAL_MODE_WITH_MATRICES_H
#define DIFFERENTIAL_MODE_WITH_MATRICES_H

#include "drive_modes/drive_mode_base_class.h"

// Simulates behaviour of a platform with differential drive.
class DifferentialModeMat : public DriveMode {

private:
    double m11, m12, m13,
           m21, m22, m23;
         // m31, m32, m33
         // No need for third row. It is 0 0 1.

public:
    DifferentialModeMat();

    void apply(double& vx, double& vy, double& va);

    void set_parameters(robotrainer_controllers::DriveModeParameters&
    dm_params);
};

#endif
