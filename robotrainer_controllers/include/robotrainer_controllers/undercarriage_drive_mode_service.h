/*

This is an "adapter" between ROS and the "drive modes" "module", which is
(almost) pure C++.

- Defines the ROS service "/base/undercarriage_drive_mode".
- Interprets incoming messages.
- Uses the "drive modes" "module" to respond to requests.

These requests are for "filter" operations on a velocity vector with three
components: Longitudinal (vx), latitudinal (vy) and rotational (vr), expressed
in platform coordinates. The filtering performed by the "apply" method.

*/

#ifndef ROBOTRAINER_CONTROLLERS_UNDERCARRIAGE_DRIVE_MODE
#define ROBOTRAINER_CONTROLLERS_UNDERCARRIAGE_DRIVE_MODE

#include <ros/ros.h>
#include "robotrainer_controllers/DriveModeParameters.h"
#include "robotrainer_controllers/ucdm_cmd.h"

#include "drive_modes/drive_modes.h"

class UndercarriageDriveMode {
public:
    UndercarriageDriveMode(ros::NodeHandle& root_nh);

    // Change the given velocity so that the set drive mode behaviour is
    // observed. Method named by D.S. Don't change.
    void apply(double& vx, double& vy, double& vr);

    // Service callback. This changes the drive mode and its parameters. Method
    // named by D.S. Don't change.
    bool config(robotrainer_controllers::ucdm_cmd::Request& req,
                robotrainer_controllers::ucdm_cmd::Response& res);

    ~UndercarriageDriveMode();
                
private:
    // This has to do with aligning wheels nicely upon receiving a new drive
    // mode. The idea is to send a miniscule drive command upon initialization, 
    // so that the wheels align, showing what the platform is about to do.
    // This either stays at -1 or if it is not becoming -1, it cycles [0,6].
    // Possible states:
    // -1: Initialized.
    // [0,2]: Send small, linearly independent drive commands. 
    // [3,5]: Negate [0,2] commands.
    // All this is to deal with the fact that apply() does not get called for
    // small drive commands. See "target_.update = false;" in fts_base_controller.cpp.
    int init_new_dm_state;

    // Ideally the minimum speed that the platform would execute. Assumed to be
    // the same value for all three components of the drive command.
    const double SMALL_V = 0.005;
    bool isTooSmall(double a);
    // But it does not work because "too small" drive commands such as zero are 
    // not sent. So update() is in fact not called every 5ms or so.

    DriveMode* dm = NULL;

    bool set_mode(int mode);

    ros::ServiceServer srv;

    int currentMode;
    robotrainer_controllers::DriveModeParameters currentParams;
};

#endif
