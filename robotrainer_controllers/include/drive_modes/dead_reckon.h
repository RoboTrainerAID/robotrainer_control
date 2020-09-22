/*
Keeps a running sum of drive mode commands. Intended as a base class for modes
that might need this information: Line2DOF and Castor.
Note: If you inherit from this class, you should overwrite dr_apply() instead
of apply().
*/

#ifndef DEAD_RECKON_MODE_H
#define DEAD_RECKON_MODE_H

#include "drive_modes/drive_mode_base_class.h"

// Just for visualization
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>

class DeadReckonMode : public DriveMode {
protected:
    // ξ in global coordinate system.
    double pos_x;
    double pos_y;
    double pos_a;

    // dξ in global coordinate system.
    double dx;
    double dy;
    double da;

    // Just for visualization
    geometry_msgs::Vector3 msg_xi; // msg_ξ: sends current pose to visualizer
    ros::Publisher pub_xi;

	// Updates time_diff (time since it's last call).
	void calculateTimeDiff();

	double time_diff;

    // Just for visualization
    ros::NodeHandle nh;
    void publishOrientation();

    // Called at the end of apply.
    void keepTrack(double& vx, double& vy, double& va);

    // Children should override.
    virtual void dr_apply(double& vx, double& vy, double& va);
    
public:
    DeadReckonMode();
    void apply(double& vx, double& vy, double& va);
};

#endif
