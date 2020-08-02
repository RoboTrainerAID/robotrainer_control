#ifndef ROBOTRAINER_CONTROLLERS_FTS_CONTROLLER_H
#define ROBOTRAINER_CONTROLLERS_FTS_CONTROLLER_H

#include "robotrainer_controllers/fts_base_controller.h"
#include <hardware_interface/robot_hw.h>

namespace robotrainer_controllers {
        
    class FTSController : public FTSBaseController {

    public:
        FTSController();
        virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
        virtual void update(const ros::Time& time, const ros::Duration& period);
        ~FTSController() { delete chain_ptr_;};
    };

}
#endif
