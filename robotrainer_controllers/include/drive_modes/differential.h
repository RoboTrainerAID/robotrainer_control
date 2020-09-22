#ifndef DIFFERENTIAL_MODE_H
#define DIFFERENTIAL_MODE_H

#include "drive_modes/drive_mode_base_class.h"

// Simulates behaviour of a platform with differential drive.
class DifferentialMode : public DriveMode {

private:
    // Normal vector of the allowed plane of velocity vectors. Has unit size.
    double nx, ny, nr;

protected:
    // x, y are w.r.t. platform center.
    // a: The angle the normal of the axle makes with the platform coordinate
    // system's longitudinal axis (x) in the counter-clockwise direction
    // (conventional right handed coordinate system).
    void set_axle(double x, double y, double a);

public:
    DifferentialMode();

    void apply(double& vx, double& vy, double& va);

    void set_parameters(robotrainer_controllers::DriveModeParameters&
    dm_params);

    // For debugging. Returns nx, ny, nr.
    void get_normal(double& x, double& y, double& r);
};

#endif
