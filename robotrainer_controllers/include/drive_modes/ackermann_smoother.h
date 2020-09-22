/*
With AckermannMode, the front wheels are somewhat "jumpy": Front wheels end up
at the limits too often. With this mode, it should be less often.
*/

#ifndef ACKERMANN_SMOOTHER_H
#define ACKERMANN_SMOOTHER_H

#include "drive_modes/ackermann.h"

class AckermannSmoother : public AckermannMode {

private:
    // Maximum rotation (in radians per second) the platform allows.
    // This should ideally come from the ROS service as a parameter.
    const double VA_MAX = 1.0;

public:
    void apply(double& vx, double& vy, double& va);
};

#endif
