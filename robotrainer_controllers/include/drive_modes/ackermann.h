#ifndef ACKERMANN_MODE_H
#define ACKERMANN_MODE_H

#include "drive_modes/differential.h"
#include "drive_modes/pivot.h"

// AckermannMode inherits from DifferentialMode, because it is essentially
// DifferentialMode with a non-zero minimum turning radius.
class AckermannMode : public DifferentialMode {

protected:
    // The next three parameters define an axis. axle_x and axle_y specify a
    // point on the axis (in platform coordinates). axle_a specifies the normal
    // of the axis, i.e. the direction in which the robot would move if wheels
    // were turned with equal speed around the axis.
    //
    // As opposed to DifferentialMode, AckermannMode needs to keep track of the
    // axle in order to check if the minimum radius constraint is satisfied.
    double axle_x, axle_y, axle_a;

    // Minimum turning radius, measured from axle center.
    double r_min;

    // Axle definition. Axle center, and an angle for the normal vector of the
    // angle.
    // Note that the angle is measured from the y axis of the platform in the
    // counter-clockwise direction (when you look from above).
    void set_axle(double x, double y, double a);
    
public:
    void apply(double& r_vx, double& r_vy, double& r_va);
    void set_parameters(robotrainer_controllers::DriveModeParameters& dm_params);
};

#endif
