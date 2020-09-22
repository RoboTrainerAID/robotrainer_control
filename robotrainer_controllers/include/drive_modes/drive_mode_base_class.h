#ifndef DRIVE_MODE_BASE_CLASS_H
#define DRIVE_MODE_BASE_CLASS_H

#include <ros/ros.h>

// Everything inside drive_modes/ used to be pure C++.
// Then Denis Stogl said something along the lines of "I've been working with
// ROS for 10 years that puristic approach leads to thousand adapters, and you
// only use those modules in ROS anyway".
// So I decided it would be better to send ROS messages directly to the
// different modes. Therefore the only real external dependency (ROS) is due to
// this include. To test drive_modes/ in an isolated manner, you can create a
// struct for the message, put it in the correct namespace and file, and you
// would be set. You also need to #define ROS_INFO printf in that file.
#include "robotrainer_controllers/DriveModeParameters.h"

// Base class for all drive modes.
class DriveMode {

public:
    DriveMode();

    // Accepts a platform velocity, finds the most "similar" velocity possible
    // for the given drive mode. Then overwrites the given parameters with the
    // output. Each child class defines what "similar" means in its own terms.
    //
    // Note that the platform velocity is the velocity of the point set by
    // set_velocity_point, expressed in global coordinates.
    //
    // Velocity components are: 
    // vx: Longitudinal platform velocity
    // vy: Latitudinal platform velocity
    // va: Angular platform velocity
    //
    // The default implementation does not do anything.
    virtual void apply(double& vx, double& vy, double& va);

    // Sets specifics for the drive mode. For example, if the current mode is
    // differential drive, this method will let you set where the axle is.
    //
    // The default implementation does not do anything.
    virtual void set_parameters(robotrainer_controllers::DriveModeParameters& 
    dm_params);
    
    // Sets the point where the velocity is to be expressed at. x, y in
    // platform coordinates.
    void set_velocity_point(double x, double y);

protected: 
    // Set by set_velocity_point.
    double x, y;


    // I did not want to introduce dependencies. Boost did not help much
    // either. So, this class ended up with some manually implemented linear
    // algebra in R^3.

    // Dot product.
    double dot(double v1, double v2, double v3,
               double w1, double w2, double w3);

    // Finds norm of the vector.
    double norm(double v1, double v2, double v3);

    // Finds unit vector in the same direction. Assumption: At least one
    // component is non-zero.
    void normalize(double& v1, double& v2, double& v3);

    // These methods overwrite the projection onto the input vector. Please
    // note: In both of these methods, [u1 u2 u3] is assumed to be of unit
    // length. Principle of Least Surprise is broken for performance, ease of
    // programming, and/or shorter method names.
    void project_onto_plane(double& v1, double& v2, double& v3,
                            double u1, double u2, double u3);
    void project_onto_line(double& v1, double& v2, double& v3,
                           double u1, double u2, double u3);

    // Multiply [x,y]^T by 2D rotation matrix R(angle).
    void rotate(double angle, double& x, double& y);
};

#endif
