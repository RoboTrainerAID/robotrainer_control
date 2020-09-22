/*
Pivot mode with moving focus, i.e. moving ICR. This was the first attempt at
non-ideal castor wheel emulation. The idea was to move the ICR along an axis,
slower than it would with an ideal platform. It does not work. Conceptually, 
unnecessarily complex.
*/

#ifndef PIVOT_MODE_WITH_MOVING_FOCUS_H
#define PIVOT_MODE_WITH_MOVING_FOCUS_H

#include "drive_modes/pivot.h"

#include <fstream>

// Just for visualization
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

// The platform can only drive around a point. Single degree of freedom.
class PivotModeWithMovingFocus : public PivotMode {
public:
    PivotModeWithMovingFocus();
    ~PivotModeWithMovingFocus();

    void apply(double& vx, double& vy, double& va);

    void set_parameters(robotrainer_controllers::DriveModeParameters& dm_params);

private:
    double fx; // focus x (axle distance, actually)
    double param; // how slow the castor angle changes. ∈[0,1]

    double C_beta; // of castor

    double function(double a);
    double solver(double beg, double end, double target);
    double input_rotation(double vy, double va);
    double find_angle_should(double vx, double rot);

	double weighted_middle(double angle1, double angle2);

    std::ofstream of;
    double hx;
    double d;

    // Just for visualization
    geometry_msgs::Vector3 msg_xi;
    std_msgs::Float64 msg_beta;
    ros::NodeHandle nh;
    ros::Publisher pub_xi;
    ros::Publisher pub_beta;
};

#endif
