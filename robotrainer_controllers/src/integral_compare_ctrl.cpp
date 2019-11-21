#include <robotrainer_controllers/integral_compare_ctrl.h>
#include <pluginlib/class_list_macros.h>

namespace robotrainer_controllers
{

IntegralCompareCtrl::IntegralCompareCtrl() {}
/**
 * \brief Init function of the IntegralCompareCtrl
 */
bool IntegralCompareCtrl::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {
    ROS_INFO("init integralCompareController");
    bool base_ctrl_initalized = FTSBaseController::init(robot_hw, root_nh, controller_nh);

    //init topics
    integral_raw_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/Integral_raw",1);
    integral_std_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/Integral_std",1);
    integral_chuy_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/Integral_chuy",1);
    integral_sliding_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/Integral_sliding",1);
    integral_sliding_weight_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/Integral_sliding_weight",1);

    integral_raw_MD_pub_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/integral_compare_ctrl/Integral_raw_MD",1);
    integral_std_MD_pub_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/integral_compare_ctrl/Integral_std_MD",1);
    integral_chuy_MD_pub_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/integral_compare_ctrl/Integral_chuy_MD",1);
    integral_sliding_MD_pub_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/integral_compare_ctrl/Integral_sliding_MD",1);
    integral_sliding_weight_MD_pub_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/integral_compare_ctrl/Integral_sliding_weight_MD",1);

    max_integral_val_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/max_integral_val",1);
    max_integral_val_3d_pub_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/integral_compare_ctrl/max_integral_val_3d",1);



    //set up dynamic reconfigure
    ros::NodeHandle integral_compare_nh(controller_nh, "IntegralCompareCtrl");

//     integral_compare_nh.param<int>("window_size", window_size_, 1024);
    integral_compare_nh.param<bool>("use_raw_integral", use_raw_integral_, false);
    integral_compare_nh.param<bool>("use_std_integral", use_std_integral_, false);
    integral_compare_nh.param<bool>("use_chuy_integral", use_chuy_integral_, false);
    integral_compare_nh.param<bool>("use_sliding_integral", use_sliding_integral_, false);
    integral_compare_nh.param<bool>("use_sliding_integral_linearWeight", use_sliding_integral_linearWeight_, false);

    integral_compare_nh.param<bool>("use_raw_integral_3d", use_raw_integral_3d_, false);
    integral_compare_nh.param<bool>("use_std_integral_3d", use_std_integral_3d_, false);
    integral_compare_nh.param<bool>("use_chuy_integral_3d", use_chuy_integral_3d_, false);
    integral_compare_nh.param<bool>("use_sliding_integral_3d", use_sliding_integral_3d_, false);
    integral_compare_nh.param<bool>("use_sliding_integral_linearWeight_3d", use_sliding_integral_linearWeight_3d_, false);

    //get pointer for dynamic reconfigure
    p_dysrv_ = new dynamic_reconfigure::Server<robotrainer_controllers::IntegralCompareCtrlConfig>(integral_compare_nh);
    dynamic_reconfigure::Server<robotrainer_controllers::IntegralCompareCtrlConfig>::CallbackType dycb =boost::bind(&IntegralCompareCtrl::reconfigureCallback, this, _1, _2);
    p_dysrv_->setCallback(dycb);

    return base_ctrl_initalized;
}

void IntegralCompareCtrl::starting(const ros::Time& time)
{

    FTSBaseController::starting(time);

}

/**
 * \brief Update loop of the IntegralCompareCtrl
 */
void IntegralCompareCtrl::update(const ros::Time& time, const ros::Duration& period) {

    FTSBaseController::setForceInput(FTSBaseController::getScaledLimitedFTSInput(FTSBaseController::getFTSInput(time)));
    FTSBaseController::update(time, period);

    old_velocity_= FTSBaseController::getOldVelocity();
    old_force_ = FTSBaseController::getOldForce();

    if(use_raw_integral_) {
        integral_raw_msg_.data = raw_integral_.calculateIntegral(old_force_, old_velocity_, time);
        integral_raw_pub_.publish(integral_raw_msg_);
    }
    if(use_std_integral_) {
        integral_std_msg_.data = std_integral_.calculateIntegral(old_force_, old_velocity_, time);
        integral_std_pub_.publish(integral_std_msg_);
    }
    if(use_chuy_integral_) {
        integral_chuy_msg_.data = chuy_integral_.calculateIntegral(old_force_, old_velocity_, time);
        integral_chuy_pub_.publish(integral_chuy_msg_);
    }
    if(use_sliding_integral_) {
        integral_sliding_msg_.data = sliding_integral_ptr_->calculateIntegral(old_force_, old_velocity_, time);
        integral_sliding_pub_.publish(integral_sliding_msg_);
        max_integral_val_msg_.data = max_integral_val_;
        max_integral_val_pub_.publish(max_integral_val_3d_msg_);
    }
    if(use_sliding_integral_linearWeight_) {
        integral_sliding_weight_msg_.data = sliding_integral_weight_ptr_->calculateIntegral(old_force_, old_velocity_, time);
        integral_sliding_weight_pub_.publish(integral_sliding_weight_msg_);
    }


    if(use_raw_integral_3d_) {
        std::array<double, DIM> integralValues = raw_integral_MD_.calculateIntegralMultiDim(old_force_, old_velocity_, time);
        integral_raw_MD_msg_.x = integralValues[0];
        integral_raw_MD_msg_.y = integralValues[1];
        integral_raw_MD_msg_.z = integralValues[2];
        integral_raw_MD_pub_.publish(integral_raw_MD_msg_);
    }
    if(use_std_integral_3d_) {
        std::array<double, DIM> integralValues = std_integral_MD_.calculateIntegralMultiDim(old_force_, old_velocity_, time);
        integral_std_MD_msg_.x = integralValues[0];
        integral_std_MD_msg_.y = integralValues[1];
        integral_std_MD_msg_.z = integralValues[2];
        integral_std_MD_pub_.publish(integral_std_MD_msg_);
    }
    if(use_chuy_integral_3d_) {
        std::array<double, DIM> integralValues = chuy_integral_MD_.calculateIntegralMultiDim(old_force_, old_velocity_, time);
        integral_chuy_MD_msg_.x = integralValues[0];
        integral_chuy_MD_msg_.y = integralValues[1];
        integral_chuy_MD_msg_.z = integralValues[2];
        integral_chuy_MD_pub_.publish(integral_chuy_MD_msg_);
    }
    if(use_sliding_integral_3d_) {
        std::array<double, DIM> integralValues = sliding_integral_MD_ptr_->calculateIntegralMultiDim(old_force_, old_velocity_, time);
        integral_sliding_MD_msg_.x = integralValues[0];
        integral_sliding_MD_msg_.y = integralValues[1];
        integral_sliding_MD_msg_.z = integralValues[2];
        integral_sliding_MD_pub_.publish(integral_sliding_MD_msg_);

        max_integral_val_3d_msg_.x = max_integral_val_3d_[0];
        max_integral_val_3d_msg_.y = max_integral_val_3d_[1];
        max_integral_val_3d_msg_.z = max_integral_val_3d_[2];
        max_integral_val_3d_pub_.publish(max_integral_val_3d_msg_);

    }
    if(use_sliding_integral_linearWeight_3d_) {
        std::array<double, DIM> integralValues = sliding_integral_weight_MD_ptr_->calculateIntegralMultiDim(old_force_, old_velocity_, time);
        integral_sliding_weight_MD_msg_.x = integralValues[0];
        integral_sliding_weight_MD_msg_.y = integralValues[1];
        integral_sliding_weight_MD_msg_.z = integralValues[2];
        integral_sliding_weight_MD_pub_.publish(integral_sliding_weight_MD_msg_);
    }

}


void IntegralCompareCtrl::reconfigureCallback(robotrainer_controllers::IntegralCompareCtrlConfig &config, uint32_t level)
{
    stopController();

    ROS_INFO("Dyn reconfigure integralCompareCtrl");

//     window_size_ = config.window_size;


    use_raw_integral_ = config.use_raw_integral;
    use_std_integral_ = config.use_std_integral;
    use_chuy_integral_ = config.use_chuy_integral;
    use_sliding_integral_ = config.use_sliding_integral;
    use_sliding_integral_linearWeight_ = config.use_sliding_integral_linearWeight;
    sliding_integral_ptr_.reset( new robotrainer_helper_types::SlidingIntegral{config.sliding_integral_window_size, config.use_reset_sliding});
    sliding_integral_weight_ptr_.reset( new robotrainer_helper_types::SlidingIntegralLineraWeigth{config.sliding_integral_linearWeight_window_size, config.use_reset_sliding_linearWeight});

    use_raw_integral_3d_ = config.use_raw_integral_3d;
    use_std_integral_3d_ = config.use_std_integral_3d;
    use_chuy_integral_3d_ = config.use_chuy_integral_3d;
    use_sliding_integral_3d_ = config.use_sliding_integral_3d;
    use_sliding_integral_linearWeight_3d_ = config.use_sliding_integral_linearWeight_3d;
    sliding_integral_MD_ptr_.reset( new robotrainer_helper_types::SlidingIntegralMultiDim{config.sliding_integral_3d_window_size,config.use_reset_sliding_3d});
    sliding_integral_weight_MD_ptr_.reset( new robotrainer_helper_types::SlidingIntegralLineraWeigthMultiDim{config.sliding_integral_linearWeight3d_window_size,config.use_reset_sliding_linearWeight_3d});


    std::array<double, DIM> tmpFT = FTSBaseController::getMaxFt();
    std::array<double, DIM> tmpVel = FTSBaseController::getMaxVel();
    ROS_INFO("getMaxFt [%.2f][%.2f][%.2f]",tmpFT[0],tmpFT[1],tmpFT[2]);
    ROS_INFO("getMaxVel [%.2f][%.2f][%.2f]",tmpVel[0],tmpVel[1],tmpVel[2]);
    for(int i = 0; i < DIM; ++i) {
        max_integral_val_3d_[i] = tmpFT[i] * tmpVel[i] * config.sliding_integral_window_size * (-1.0);
    }
    ROS_INFO("max_integral_val_3d [%.2f][%.2f][%.2f]",max_integral_val_3d_[0],max_integral_val_3d_[1],max_integral_val_3d_[2]);
    max_integral_val_ = sumArray(max_integral_val_3d_);
    ROS_INFO("max_integral_val [%.2f]",max_integral_val_);

    //reset the IntegralValues on reconfigure, this way changes are compareable
    resetIntegrals();

    resetController();

}

void IntegralCompareCtrl::resetIntegrals()
{
    ROS_INFO("Reset Integrals");
    std_integral_.resetIntegralValue();
    chuy_integral_.resetIntegralValue();

    std_integral_MD_.resetIntegralValues();
    chuy_integral_MD_.resetIntegralValues();

}

void IntegralCompareCtrl::delete_pointers()
{
    delete chain_ptr_;

}

}
PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::IntegralCompareCtrl, controller_interface::ControllerBase)
