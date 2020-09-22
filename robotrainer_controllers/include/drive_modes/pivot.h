#ifndef PIVOT_MODE_H
#define PIVOT_MODE_H

#include "drive_modes/drive_mode_base_class.h"

// The platform can only drive around a point. Single degree of freedom.
class PivotMode : public DriveMode {
public:
    PivotMode();

    void apply(double& vx, double& vy, double& va);

    void set_parameters(robotrainer_controllers::DriveModeParameters& dm_params);

    void set_focus(double focus_x, double focus_y);

private:
    double nx, ny, na;
};

#endif
