#ifndef CASTOR_MODE_H
#define CASTOR_MODE_H

#include "drive_modes/dead_reckon.h"
#include <std_msgs/Float64.h>

#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>

#include "drive_modes/differential.h"

class CastorMode : public DeadReckonMode {
public:
    CastorMode();

    void dr_apply(double& vx, double& vy, double& va);

    void set_parameters(robotrainer_controllers::DriveModeParameters& dm_params);

private:
	// Castor position in platform polar coordinates
	double a; // castor's alpha
	double l; // joint distance to center

	double d; // castor offset 
	double r; // castor wheel radius

    double b; // castor's beta

	double p; // castor's "reactiveness": the smaller, the stiffer
    double p2;

    // Virtual axle
    double VAx;
    double VAy;
    double VAa;

    DifferentialMode diff_mode;
    void helper(const Eigen::Vector3d& dxi, Eigen::Vector3d& newdxi, Eigen::Vector3d& dCxi);

    // Just for visualization
    std_msgs::Float64 msg_beta;
    ros::Publisher pub_beta;
	ros::Publisher pub_castor_init;
    visualization_msgs::Marker icr_marker;
    ros::Publisher pub_icr_marker;
};

#endif
