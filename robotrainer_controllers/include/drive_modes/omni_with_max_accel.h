#ifndef OMNI_MODE_WITH_MAX_ACCEL_H
#define OMNI_MODE_WITH_MAX_ACCEL_H

#include "drive_modes/drive_mode_base_class.h"

class OmniModeWithMaxAccel : public DriveMode {
public:
    OmniModeWithMaxAccel();
    void apply(double& vx, double& vy, double& va);
    void set_parameters(robotrainer_controllers::DriveModeParameters& 
    dm_params);
private:
    double max_accel;

    double last_vx;
    double last_vy;
};

#endif
