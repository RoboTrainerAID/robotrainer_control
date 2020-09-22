#include "robotrainer_controllers/undercarriage_drive_mode_service.h"

#define UCDM UndercarriageDriveMode

UCDM::UCDM(ros::NodeHandle& root_nh) {
    this->set_mode(1);
    this->srv = root_nh.advertiseService("undercarriage_drive_mode",
                                         &UCDM::config, this);
}

bool UCDM::config(robotrainer_controllers::ucdm_cmd::Request& req,
                  robotrainer_controllers::ucdm_cmd::Response& res) {
    if (req.mode == 0) {
        res.mode = this->currentMode;
        res.params = this->currentParams;
        return true;
    }

    if (!this->set_mode(req.mode)) {
        res.mode = -1;
        return false;
    }

    // Do this before dm->set_parameters, as some DriveModes might overwrite the
    // parameters.
    this->currentParams = req.params;

    // It might have been useful to receive feedback if the parameters were
    // invalid. However, for the drive modes we currently have, there are no
    // wrong parameters. 
    this->dm->set_parameters(req.params);

    res.mode = req.mode;
    res.params = req.params;
    return true;
}

bool UCDM::isTooSmall(double a) {
    if (a > SMALL_V) return false;
    if (a < -SMALL_V) return false;
    return true;
}

void UCDM::apply(double& vx, double& vy, double& vr) {
    if (this->init_new_dm_state != -1) {
        // First calls of apply with the newly set drive mode.

        // For the initialization of a new mode, we want the wheels to move so 
        // that the wheels get in position. If the drive command sent is too
        // small, we should set a small value.

        if (!isTooSmall(vx) || !isTooSmall(vy) || !isTooSmall(vr)) {
            // In case the sent drive command is not small, skip this
            // initialization.
            this->init_new_dm_state = -1;
        }
        else {
            vx = vy = vr = 0;
            switch(this->init_new_dm_state) {
                case 0: vx = SMALL_V; break;
                case 1: vy = SMALL_V; break;
                case 2: vr = SMALL_V; break;
                case 3: vx = -SMALL_V; break;
                case 4: vy = -SMALL_V; break;
                case 5: vr = -SMALL_V; break;
            }
            this->init_new_dm_state++;
            this->init_new_dm_state %= 6;
        }
    }

    this->dm->apply(vx, vy, vr);
}

bool UCDM::set_mode(int mode) {
    DriveMode* new_dm = NULL;

    switch (mode) {
        case 1: new_dm = new OmnidirectionalMode(); break;
        case 11: new_dm = new OmniModeWithMaxAccel(); break;
        case 2: new_dm = new DifferentialMode(); break;
        case 21: new_dm = new DifferentialModeMat(); break;
        case 3: new_dm = new SynchroMode(); break;
        case 4: new_dm = new AckermannMode(); break;
        case 41: new_dm = new AckermannSmoother(); break;
        case 5: new_dm = new PivotMode(); break;
        case 6: new_dm = new CastorMode(); break;
        case 7: new_dm = new Line2DOF(); break;
        case 8: new_dm = new AxPLUSbMode(); break;
        default: 
            ROS_INFO("Invalid mode: %d", mode);
            return false;
    }

    this->currentMode = mode;

    ROS_INFO("In set_mode: %d", this->init_new_dm_state);
    this->init_new_dm_state = 0;

    if (this->dm) delete this->dm;
    this->dm = new_dm;

    return true;
}

UCDM::~UCDM() {
    delete this->dm;
}

#undef UCDM
