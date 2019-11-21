#include <robotrainer_controllers/fts_controller.h>
#include <pluginlib/class_list_macros.h>

namespace robotrainer_controllers
{

FTSController::FTSController(){}
/**
        * \brief Init function of the FTSController
        */
bool FTSController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {   
        ROS_INFO("init fts_controller");
        bool base_ctrl_initalized = FTSBaseController::init(robot_hw, root_nh, controller_nh);
        return base_ctrl_initalized;
}


/**
        * \brief Update loop of the FTSController
        */
void FTSController::update(const ros::Time& time, const ros::Duration& period) {  
        
        FTSBaseController::setForceInput(FTSBaseController::getScaledLimitedFTSInput(FTSBaseController::getFTSInput(time)));
        FTSBaseController::update(time, period);
}

        
}
PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::FTSController, controller_interface::ControllerBase)
// 
