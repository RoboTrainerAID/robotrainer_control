#include "robotrainer_controllers/fts_base_controller.h"

#include "robotrainer_controllers/fts_controllers_led_defines.hpp"
#include <pluginlib/class_loader.h>

namespace robotrainer_controllers 
{
    
bool FTSBaseController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    ROS_INFO("Init FTSBaseController!");
    ctrl_nh_ = controller_nh;
    ros::NodeHandle wheels_nh(root_nh, "wheel_controller");
    wheel_ctrl_nh_ = wheels_nh;

    diagnostic_.add("RoboTrainer Controller - Base", this, &FTSBaseController::diagnostics);
    diagnostic_.setHardwareID("FTS_Base_Controller");
    diagnostic_.broadcast(0, "Initializing FTS Base Controller");

    /* get Parameters from rosparam server (stored on yaml file) */
    ros::NodeHandle fts_base_ctrl_nh(ctrl_nh_, "FTSBaseController");
    fts_base_ctrl_nh.param<bool>("no_hw_output", no_hw_output_, true);
    fts_base_ctrl_nh.param<bool>("debug", debug_, false);
    fts_base_ctrl_nh.param<bool>("use_twist_input", use_twist_input_, false);
    fts_base_ctrl_nh.param<double>("update_rate", controllerUpdateRate_, 1.0);
    fts_base_ctrl_nh.param<std::string>("frame_id", controllerFrameId_, "base_link");
    fts_base_ctrl_nh.param<bool>("y_reversed", yReversed_, false);
    fts_base_ctrl_nh.param<bool>("rot_reversed", rotReversed_, false);
    fts_base_ctrl_nh.param<double>("reversed_max_force_scale", reversedMaxForceScale_, 0.25);
    fts_base_ctrl_nh.param<double>("reversed_max_vel_scale", reversedMaxVelScale_, 0.25);
    fts_base_ctrl_nh.param<double>("backwards_max_force_scale", backwardsMaxForceScale_, 0.25);
    fts_base_ctrl_nh.param<double>("backwards_max_vel_scale", backwardsMaxVelScale_, 0.25);
    fts_base_ctrl_nh.param<bool>("x_force_controller", use_controller_[0], false);
    fts_base_ctrl_nh.param<bool>("y_force_controller", use_controller_[1], false);
    fts_base_ctrl_nh.param<bool>("rot_controller", use_controller_[2], false);
    fts_base_ctrl_nh.param<double>("x_min_force", min_ft_[0], 1.0);
    fts_base_ctrl_nh.param<double>("x_max_force", max_ft_[0], 1.0);
    fts_base_ctrl_nh.param<double>("x_max_vel", max_vel_[0], 0.0);
    fts_base_ctrl_nh.param<double>("x_gain", gain_[0], 1.0);
    fts_base_ctrl_nh.param<double>("x_time_const", time_const_[0], 1000000.0);
    fts_base_ctrl_nh.param<double>("y_min_force", min_ft_[1], 1.0);
    fts_base_ctrl_nh.param<double>("y_max_force", max_ft_[1], 1.0);
    fts_base_ctrl_nh.param<double>("y_max_vel", max_vel_[1], 0.0);
    fts_base_ctrl_nh.param<double>("y_gain", gain_[1], 1.0);
    fts_base_ctrl_nh.param<double>("y_time_const", time_const_[1], 1000000.0);
    fts_base_ctrl_nh.param<double>("rot_min_torque", min_ft_[2], 1.0);
    fts_base_ctrl_nh.param<double>("rot_max_torque", max_ft_[2], 1.0);
    fts_base_ctrl_nh.param<double>("rot_max_rot_vel", max_vel_[2], 0.0);
    fts_base_ctrl_nh.param<double>("rot_gain", gain_[2], 1.0);
    fts_base_ctrl_nh.param<double>("rot_time_const", time_const_[2], 1000000.0);
    
    default_min_ft_ = min_ft_;
    default_max_ft_ = max_ft_;
    default_max_vel_ = max_vel_;

    /* Control actions */
    //Adapting center of rotation
    fts_base_ctrl_nh.param<bool>("global_control_actions/adaptive_cor/adapt_cor", adapt_center_of_rotation_, false);
    fts_base_ctrl_nh.param<double>("global_control_actions/adaptive_cor/cor_x", cor_x_, 0.0);
    fts_base_ctrl_nh.param<double>("global_control_actions/adaptive_cor/cor_y", cor_y_, 0.0);
    //global counterforce enabled by default
    fts_base_ctrl_nh.param<bool>("global_control_actions/global_counterforce/enabled", enableCounterForce_, false);
    fts_base_ctrl_nh.param<double>("global_control_actions/global_counterforce/counterforce_x", staticCounterForce_[0], 0.0);
    fts_base_ctrl_nh.param<double>("global_control_actions/global_counterforce/counterforce_y", staticCounterForce_[1], 0.0);
    fts_base_ctrl_nh.param<double>("global_control_actions/global_counterforce/countertorque_z", staticCounterForce_[2], 0.0);


    base_reconfigured_flag_ = false;

    /* Debug messages */
    pub_platform_hw_velocity_.reset(
      new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(
        ctrl_nh_, "debug/platform_hw_velocity", 1));
    pub_admittance_velocity_.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(
        ctrl_nh_, "debug/admittance_velocity", 1));
    pub_final_velocity_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(
        ctrl_nh_, "debug/velocity_output", 1));
    pub_force_input_raw_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(
        ctrl_nh_, "debug/force_input_raw", 1));
    pub_force_input_scaled_limited_.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(
        ctrl_nh_, "debug/force_input_scaled_limited", 1));
    pub_resulting_force_after_counterforce_.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(
        ctrl_nh_, "debug/after_counterforce_modality", 1));
    pub_resulting_force_after_cor_.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(
        ctrl_nh_, "debug/after_cor_modality", 1));
    
    /* LED Output */
    pub_input_force_for_led_.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(
        root_nh, "/leds_rectangle/led_force", 1));
    led_ac_.reset(new actionlib::SimpleActionClient<iirob_led::BlinkyAction>(
        "/leds_rectangle/blinky", true));
    if (led_ac_->waitForServer(ros::Duration(2))) {
        ROS_DEBUG("[BASE-C - INIT] LED action client registered");
    }
    else {
        ROS_WARN("Action server for LED-Rectangle not started and it will not be used!");
    }

    /* For Users GUI */
    pub_input_force_force_for_gui_.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(
            root_nh, "force_for_gui", 1));
    
    /* Subscriber */
    //subscriber to the spatial control action for counterforce, which pushes a message with the current distance whenever the robot enters the counterforce area
    counterforce_area_sub_ = root_nh.subscribe("virtual_areas/counterforce/center_dist_percent", 10, &FTSBaseController::counterforce_area_callback, this);
    /* Services */
    fts_client_ = root_nh.serviceClient<force_torque_sensor::CalculateSensorOffset>("CalculateOffsets");

    /* Initialize Arrays for dimension on/off switching */
    all_inactive_ = { false, false, false };
    x_active_ = {true, false, false };
    y_active_ = {false, true, false };
    rot_active_ = {false, false, true };
    x_y_active_ = {true, true, false };
    x_rot_active_ = {true, false, true };
    y_rot_active_ = {false, true, true };
    all_active_ = {true, true, true };
    timeSinceReleasing_ = ros::Time::now();

    zeroForce_ = {0.0, 0.0, 0.0};

    /* Initialize FTS */
    if(robot_hw==NULL) {
            std::cout<<"null hw ptr"<<std::endl;
            return false;
    }
    hw_fts_ = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>()->getHandle("ATI_45_Mini");
    hardware_interface::VelocityJointInterface* hw_vel = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if(!cob_omni_drive_controller::parseWheelParams(wheel_params_, wheel_ctrl_nh_) || !GeomController<UndercarriageCtrl>::init(hw_vel, wheel_params_)) return false;

    if (controllerFrameId_.compare(hw_fts_.getFrameId()) != 0) {
        ROS_WARN("'frame_id' of input data is not equal to controllers frame!! Your controller will probably not work as expected! \nFTSBaseController is expecting data in '%s' frame, but input data frame is '%s'! You should probably correct this in configuration of your ForceTorqueSensorHandle.", controllerFrameId_.c_str(), hw_fts_.getFrameId().c_str());
    }


    /* Set dynamic reconfigure */
    base_dysrv_ = new dynamic_reconfigure::Server<robotrainer_controllers::FTSBaseControllerConfig>(fts_base_ctrl_nh);
    dynamic_reconfigure::Server<robotrainer_controllers::FTSBaseControllerConfig>::CallbackType base_dycb = boost::bind(&FTSBaseController::reconfigureCallback, this, _1, _2);
    base_dysrv_->setCallback(base_dycb);

    dysrv_callback_service_ = fts_base_ctrl_nh.serviceClient<dynamic_reconfigure::Reconfigure>("/base/fts_adaptive_force_controller/FTSBaseController/set_parameters");

    /* Initialize Position control */
    pos_ctrl_.init(wheel_params_, wheel_ctrl_nh_);

    ROS_WARN_COND(no_hw_output_, "SIMULATION MODE, NO OUTPUT TO REAL ROBOT!");

    // Controller Mode Service
    use_twist_input_srv_ = root_nh.advertiseService("set_use_twist_input", &FTSBaseController::setUseTwistInputCallback, this);

    // Kinematics services
    update_kinematics_service_srv_ = root_nh.advertiseService("update_kinematics", &FTSBaseController::updateWheelParamsCallback, this);

    /* Initialize Modalities */
    configure_modalities_srv_ = root_nh.advertiseService("configure_modalities", &FTSBaseController::configureModalitiesCallback, this);
    modalities_loaded_ = false;
    modalities_configured_ = false;
    
    loadModalityInstances();
    
    drive_mode_used_ = drive_mode_type::NONE;
    ucdm_ = new UndercarriageDriveMode(root_nh);

    apply_areal_counterforce_ = false;

    base_initialized_ = WheelControllerBase::setup(root_nh, wheel_ctrl_nh_);

    diagnostic_.force_update();

    return base_initialized_;
}

/**
 * \brief start controller
 */
void FTSBaseController::starting(const ros::Time& time)
{
    WheelControllerBase::starting(time);
    controller_started_ = true;

    // Calculate FTS offsets and reset with reorienting wheels
    std::string lock = "starting";
    protectedToggleControllerRunning(false, lock);
    bool ret = unsafeRecalculateFTSOffsets();
    setOrientWheels(std::array<double, 3>({1, 0, 0}));
    protectedToggleControllerRunning(ret, lock);
    setLEDPhase(controller_led_phases::UNLOCKED);
}

/**
 * \brief Update robot's internal state before executing controller
 */
void FTSBaseController::updateRobotState()
{

//   if (internal_state_updated_) {
//     return;
//   }

  updateState();  // fts_GeomController call
  geom_->calcDirect(platform_state_);
  platform_linear_vel_ = std::pow(platform_state_.getVelX(), 2) +
                         std::pow(platform_state_.getVelY(), 2);
  platfrom_velocity_ = platform_linear_vel_ + std::pow(platform_state_.dRotRobRadS, 2);

  platform_is_moving_ = ( platfrom_velocity_ > 0.001 );

  internal_state_updated_ = true;
}

/**
* \brief Update loop, which is called periodically to calculate the next target for the robot and set it on the robot hardware-chain
*
*  Be aware that this is only the base class for control.
*
* THIS CONTROLLER IS NOT INTENTED FOR STANDALONE USE!
*
* For using this base class you have to derive from it and set the forceinput by calling the
*          void FTSBaseController::setForceInput(std::array<double,3> force_input_for_control)
*  and than call the update function
*          void FTSBaseController::update(const ros::Time& time, const ros::Duration& period)
*
*/
void FTSBaseController::update(const ros::Time& time, const ros::Duration& period) {

    // First read states from the hardware
    updateRobotState();
    updateState();
    
    if (debug_) {
        convertToTwistAndPublish(time, getVelocity(), pub_platform_hw_velocity_);
    }

    std::array<double, 3> new_vel;
    bool set_new_commands = true;
    bool running = getRunning();

    boost::mutex::scoped_lock controller_internal_states_lock(controller_internal_states_mutex_);
    
    sendLEDOutput(); // LED output to robot (if LEDPhase has changed or is set to controller_led_phases::SHOW_FORCE)
    if (running and not use_twist_input_) {
        if ( force_input_[0] < -max_ft_[0]*backwardsMaxForceScale_) force_input_[0] = -max_ft_[0]*backwardsMaxForceScale_;
        
        if (debug_) {
            convertToWrenchAndPublish(time, force_input_, pub_force_input_scaled_limited_);
        }

//                 TODO: This should go into a modality
        if (enableCounterForce_ && userIsGripping() ) {
            force_input_ = applyGlobalCounterforce(force_input_);
        } else if (apply_areal_counterforce_ && userIsGripping()) {
            force_input_ = applyAreaCounterforce(force_input_);
            apply_areal_counterforce_ = false;
        }

        if (debug_) {
            convertToWrenchAndPublish(time, force_input_, pub_resulting_force_after_counterforce_);
        }

        if (adapt_center_of_rotation_ && userIsGripping()) {
            force_input_ = adaptCenterOfRotation(force_input_);
        }
        if (debug_) {
            convertToWrenchAndPublish(time, force_input_, pub_resulting_force_after_cor_);
        }

        // calculate velocity using spring-mass-damping
        for (int i = 0; i < 3; i++) {
            if (!use_controller_[i]) {
                ROS_DEBUG_THROTTLE(5, "NO Controller set for Dimension %d", i);
                new_vel[i] = 0.0;
            } else {
                new_vel[i] = (b1_[i] * force_old_[i] + a1_[i]*velocity_old_[i]); //in scale 0-1
                force_old_[i] = force_input_[i];
                velocity_old_[i] = new_vel[i];
                new_vel[i] *= max_vel_[i]; //scaled back to max vel
                // stop robot when small velocity
                if (std::fabs(new_vel[i]) < 0.001 ) {
                    new_vel[i] = 0.0;
                }
            }
        }

        if (debug_) {
          convertToTwistAndPublish(time, new_vel, pub_admittance_velocity_);
        }

        // handle special cases for each dimension seperately
        // TODO: Do I want to influence dynamics? if so than the maximal velocities should be changed
        if ( new_vel[0] < -max_vel_[0]*backwardsMaxVelScale_) {
            new_vel[0] = -max_vel_[0]*backwardsMaxVelScale_; //backwards x scale
        }
        if (yReversed_) {
            new_vel[1] = -new_vel[1];  //inverse y
        }
        if (rotReversed_) {
            new_vel[2] = -new_vel[2];  //inverse rotation
        }

        delta_vel_ = std::pow(new_vel[0], 2) + std::pow(new_vel[1], 2) + std::pow(new_vel[2], 2);

        //apply modalities
        if (modalities_used_ != none && userIsGripping()) {
            new_vel = applyModalities(new_vel, force_input_);
        }
        
        if (drive_mode_used_ != drive_mode_type::NONE) {
            ucdm_->apply(new_vel[0], new_vel[1], new_vel[2]);
        }
        
        if (debug_) {
          convertToTwistAndPublish(time, new_vel, pub_final_velocity_);
        }

        if (no_hw_output_) {
            ROS_WARN_THROTTLE(5.0, "[FTS_BASE_CTRLR] SIMULATION MODE, NO OUTPUT TO ROBOT, BUT SENDING VELOCITIES TO DEBUG TOPIC!");
            set_new_commands = false;
        }

    } else {
        set_new_commands = false;
        if (!platform_is_moving_ and getCanBeRunning() and !userIsGripping()) {
            if (orient_wheels_ > 0 and orient_wheels_ < 100) {
                ROS_DEBUG_ONCE("Orienting wheels with values: %f, %f, %f", orient_wheels_vel_[0], orient_wheels_vel_[1], orient_wheels_vel_[2]);
                new_vel = orient_wheels_vel_;
                set_new_commands = true;
                orient_wheels_++;
            } else if (!running) {
                setRunning(true);
                orient_wheels_ = 0;
                ROS_DEBUG("FTSBaseController is running!");
            }
        }
    }

    if (platform_is_moving_ and (!userIsGripping() || !running || no_hw_output_) ) {
        for (int i = 0; i < 3; i++) {
            new_vel[i] = 0.0;
            force_old_[i] = 0.0;
            velocity_old_[i] = new_vel[i];
        }
        set_new_commands = true;
    }

    if ((running and orient_wheels_ == 0) and use_twist_input_) {
        boost::mutex::scoped_lock lock(twist_mutex_);
        new_vel[0] = twist_command_.linear.x;
        new_vel[1] = twist_command_.linear.y;
        new_vel[2] = twist_command_.angular.z;
        set_new_commands = true;
    }

    if (set_new_commands) {
        if (std::isnan(new_vel[0]) || std::isnan(new_vel[1]) || std::isnan(new_vel[2])) {
            ROS_FATAL("Received NaN-value in Twist message. Reset target to zero.");
            target_.state = PlatformState();
        } else {
            //TODO check if this is necessary
            target_.state.setVelX(limitValue(new_vel[0], max_vel_trans_));
            target_.state.setVelY(limitValue(new_vel[1], max_vel_trans_));
            target_.state.dRotRobRadS = limitValue(new_vel[2], max_vel_rot_);
        }
        target_.updated = true;
        target_.stamp = ros::Time::now();
    }
    else {
        target_.updated = false;
    }

    pos_ctrl_.try_configure(*geom_);
    WheelControllerBase::updateCtrl(time, period);
    for (unsigned i=0; i<wheel_commands_.size(); i++){
        steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
        drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
    }
    internal_state_updated_ = false;
    diagnostic_.update();
}

/**
 * \brief Stops the controller
 */
void FTSBaseController::stopping(const ros::Time& /*time*/) {
    std::string lock = "stopping";
    protectedToggleControllerRunning(false, lock);
    controller_started_ = false;
    protectedToggleControllerRunning(false, lock);
    setLEDPhase(controller_led_phases::STOPPED);
}

/**
* \brief Resets the controller by setting old force and velocities to zero and re-discretizing the pt1-element parameters
*/
void FTSBaseController::restartController()
{
    std::string lock = "restart_controller";
    protectedToggleControllerRunning(false, lock);
    protectedToggleControllerRunning(true, lock);
    setLEDPhase(controller_led_phases::UNLOCKED);

}

void FTSBaseController::resetController()
{
    std::string lock = "reset_controller";
    protectedToggleControllerRunning(false, lock);
    setOrientWheels({1, 0, 0});
//     bool ret = unsafeRecalculateFTSOffsets();
//     protectedToggleControllerRunning(ret, lock);
    protectedToggleControllerRunning(true, lock);
}

//TODO: Temp function should be renamed to resetController, when all child classes are clarified
void FTSBaseController::resetControllerNew()
{
    std::string lock = "reset_controller_new";
    protectedToggleControllerRunning(false, lock);
    protectedToggleControllerRunning(true, lock);

}

void FTSBaseController::restartControllerAndOrientWheels(std::array<double,3> direction)
{
    std::string lock = "restart_controller_and_orient_wheels";
    protectedToggleControllerRunning(false, lock);
    setOrientWheels(direction);
    protectedToggleControllerRunning(true, lock);
    setLEDPhase(controller_led_phases::UNLOCKED);
}

std::string FTSBaseController::setUseTwistInput(bool use_twist_input)
{
    std::string lock = "set_use_twist_input";
    protectedToggleControllerRunning(false, lock);
    std::string message = internalSetUseTwistInput(use_twist_input);
    protectedToggleControllerRunning(true, lock);
    setLEDPhase(controller_led_phases::UNLOCKED);
    return message;
}

/* ______CONTROLLER RUNNING CONTROL_______*/
void FTSBaseController::protectedToggleControllerRunning(const bool start_stop, const std::string locking_string)
{
    boost::mutex::scoped_lock lock(locking_mutex_);
    if (!start_stop) {
        ROS_DEBUG("Requested Locking Nr: '%s'", locking_string.c_str());
        if (locking_string_.compare(LOCKING_NONE) == 0) {
            ROS_DEBUG("Running Control: Current Locking not set - locking!");
            locking_string_ = locking_string;
            stopController();
        } else if (locking_string_.compare(locking_string) == 0) {
            ROS_WARN("Requested unlock without start the controller for '%s'!",
                locking_string.c_str()
            );
            locking_string_ = LOCKING_NONE;
        } else {
            ROS_FATAL("Request lock for not allowed '%s'; allowed key is %s",
                      locking_string.c_str(), locking_string_.c_str());
        }
    }
    if (start_stop) {
        if (locking_string_.compare(locking_string) == 0) {
            ROS_DEBUG("Request unlock for '%s' - unlocking!", locking_string.c_str());
            locking_string_ = LOCKING_NONE;
            startController();
        } else if (locking_string_.compare(LOCKING_NONE) == 0) {
            ROS_WARN("Trying to unlock not locked toggle!");
        } else {
            ROS_FATAL("Request unlock for not allowed '%s'; allowed key is %s",
                      locking_string.c_str(), locking_string_.c_str());
        }
    }
    diagnostic_.force_update();
}

/**
 * \brief Stops the controller
 */
void FTSBaseController::stopController()
{
    setCanBeRunning(false);
    setRunning(false);
}

void FTSBaseController::startController()
{
    discretizeController();
    // initialize variables
    force_old_[0] = 0.0;
    force_old_[1] = 0.0;
    force_old_[2] = 0.0;
    velocity_old_[0] = 0.0;
    velocity_old_[1] = 0.0;
    velocity_old_[2] = 0.0;
    setCanBeRunning(true);
}

/* ______CONTROLLER INTERNALS________ - Controller has to be stoped before calling this functions! */
void FTSBaseController::setOrientWheels(std::array<double,3> direction) {
    boost::mutex::scoped_try_lock lock(controller_internal_states_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    for (uint i=0; i < orient_wheels_vel_.size(); i++ ) {
        orient_wheels_vel_[i] = direction[i] * 0.001;
    }
    orient_wheels_ = 1;
}

bool FTSBaseController::unsafeRecalculateFTSOffsets()
{
    force_torque_sensor::CalculateSensorOffset srv;
    srv.request.apply_after_calculation = true;

    if (fts_client_.call(srv)) {
        ROS_DEBUG("Called FTS with CalculateOffsets, retrieved new offset as force:[%.4f, %.4f, %.4f] torque:[%.4f, %.4f, %.4f]",
                  srv.response.offset.force.x, srv.response.offset.force.y, srv.response.offset.force.z, srv.response.offset.torque.x, srv.response.offset.torque.y, srv.response.offset.torque.z);
        return true;
    } else {
        ROS_ERROR("Failed to call Service to recalculate FTSOffsets!");
        return false;
    }
}

std::string FTSBaseController::internalSetUseTwistInput(bool use_twist_input) {
    boost::mutex::scoped_try_lock lock(controller_internal_states_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    use_twist_input_ = use_twist_input;

    std::string message = "RoboTrainer Controller uses _force_ input!";
    if (use_twist_input_) {
        std::string message = "RoboTrainer Controller uses _twist_ input!";
    }
    ROS_DEBUG("%s", message.c_str());
    return message;
}

void FTSBaseController::internalSetNoHWOutput(bool no_hw_output) {
    boost::mutex::scoped_try_lock lock(controller_internal_states_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    no_hw_output_ = no_hw_output;

    std::string message = "RoboTrainer Controller will be output to hardware!";
    if (no_hw_output_) {
        std::string message = "RoboTrainer Controller will NOT be output to hardware!";
    }
    ROS_DEBUG("%s", message.c_str());
}

/// \brief Publishes diagnostics and status
void FTSBaseController::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (getRunning()) {
        status.summary(0, "Controller is running");
    } else {
        status.summary(1, "Cotroller is not running");
    }

    status.add("running", getRunning());
    status.add("can be running", getCanBeRunning());
    status.add("lock", locking_string_);
    status.add("user is gripping", userIsGripping());
    status.add("platform is moving", platform_is_moving_);
    status.add("use twist input", use_twist_input_);
    status.add("no hardware output", no_hw_output_);
    status.add("modalities used", modality_type_names_[modalities_used_]);
    status.add("counter force", enableCounterForce_);
    status.add("adapt center of rotation", adapt_center_of_rotation_);
    status.add("drive modes", drive_mode_type_names_.at(drive_mode_used_));
    status.add("orient wheels", orient_wheels_);
    status.add("controller frame id", controllerFrameId_);
}

/*_____PROTECTED GETTERS AND SETTERS_____*/

void FTSBaseController::setRunning(bool value)
{
    boost::unique_lock<boost::shared_mutex> lock(running_mutex_);
    running_ = value;
}

bool FTSBaseController::getRunning()
{
    boost::shared_lock<boost::shared_mutex> lock(running_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    return running_;
}

void FTSBaseController::setCanBeRunning(bool value)
{
    boost::unique_lock<boost::shared_mutex> lock(can_be_running_mutex_);
    can_be_running_ = value;
}

bool FTSBaseController::getCanBeRunning()
{
    boost::shared_lock<boost::shared_mutex> lock(can_be_running_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    return can_be_running_;
}


/*______MODALITIES FUNCTIONS FROM HERE ON______*/

/**
 * \brief This function creates an instance for all modalities one after another, and returns true only if all were created successfully. If all are setup correctly, then the trigger modalities_loaded_ is set to true.
 */
bool FTSBaseController::loadModalityInstances() {

        ROS_INFO("Trying to load modalities so that they can be configured!");
        modalities_loaded_ = ( loadBaseModalityInstances() && loadControllerModalityInstances() );
        return modalities_loaded_;
}

/**
 * \brief This function creates an instance for all base-type-modalities one after another, and returns true only if all were created successfully.
 */
bool FTSBaseController::loadBaseModalityInstances() {

//     pluginlib::ClassLoader<robotrainer_modalities::ModalityBase<geometry_msgs::Twist>> modalities_loader("robotrainer_modalities", "robotrainer_modalities::ModalityBase<geometry_msgs::Twist>");
    modalities_loader_.reset(new pluginlib::ClassLoader<
      robotrainer_modalities::ModalityBase<geometry_msgs::Twist>>(
          "robotrainer_modalities", "robotrainer_modalities::ModalityBase<geometry_msgs::Twist>"));
    //Virtual Force
    try {
        force_modality_ptr_ = modalities_loader_->createInstance("robotrainer_modalities/VirtualForces");
        ROS_INFO_ONCE("[fts_base_controller.cpp] Force_Modality loaded");
    } catch(pluginlib::PluginlibException& e) {
        ROS_ERROR_STREAM("[fts_base_controller.cpp] Force_Modality plugin failed to load:" << e.what());
        return false;
    }
    // TODO(denis): Commented output for the RoSy-Study in Oct. 2020
//     //Walls
//     try {
//         walls_modality_ptr_ = modalities_loader_->createInstance("robotrainer_modalities/VirtualWalls");
//         ROS_INFO_ONCE("[fts_base_controller.cpp] Walls_Modality loaded");
//     } catch(pluginlib::PluginlibException& e) {
//         ROS_ERROR_STREAM("[fts_base_controller.cpp] Walls_Modality plugin failed to load:" << e.what());
//         return false;
//     }
//     //Path Tracking
//     try {
//         pathtracking_modality_ptr_ = modalities_loader_->createInstance("robotrainer_modalities/PathTracking");
//         ROS_INFO_ONCE("[fts_base_controller.cpp] PathTracking_Modality loaded");
//     } catch(pluginlib::PluginlibException& e) {
//         ROS_ERROR_STREAM("[fts_base_controller.cpp] PathTracking_Modality plugin failed to load:" << e.what());
//         return false;
//     }
//     //Virtual Area
//     try {
//         area_modality_ptr_ = modalities_loader_->createInstance("robotrainer_modalities/VirtualAreas");
//         ROS_INFO_ONCE("[fts_base_controller.cpp] Areas_Modality loaded");
//     } catch(pluginlib::PluginlibException& e) {
//         ROS_ERROR_STREAM("[fts_base_controller.cpp] Areas_Modality plugin failed to load:" << e.what());
//         return false;
//     }
    return true;
}

/**
 * \brief This function creates an instance for all controller-type-modalities one after another, and returns true only if all were created successfully.
 */
bool FTSBaseController::loadControllerModalityInstances() {

//     pluginlib::ClassLoader<robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>> modality_controllers_loader("robotrainer_modalities", "robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>");
    modality_controllers_loader_.reset(new pluginlib::ClassLoader<
      robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>>(
        "robotrainer_modalities",
        "robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>"));
    //force_controller
    try {
        force_controller_modality_ptr_ = modality_controllers_loader_->createInstance("robotrainer_modalities/ModalitiesVirtualForcesController");
        ROS_INFO_ONCE("[fts_base_controller.cpp] Force_Controller_Modality loaded");
    }
    catch(pluginlib::PluginlibException& e) {
        ROS_ERROR_STREAM("[fts_base_controller.cpp] Force_Controller_Modality plugin failed to load:" << e.what());
        return false;
    }
    // TODO(denis): Commented output for the RoSy-Study in Oct. 2020
//     //virtual_walls_controller
//     try {
//         walls_controller_modality_ptr_ = modality_controllers_loader_->createInstance("robotrainer_modalities/ModalitiesVirtualWallsController");
//         ROS_INFO_ONCE("[fts_base_controller.cpp] Walls_Controller_Modality loaded");
//     }
//     catch(pluginlib::PluginlibException& e) {
//         ROS_ERROR_STREAM("[fts_base_controller.cpp] Walls_Controller_Modality plugin failed to load:" << e.what());
//         return false;
//     }
//     //path_tracking_controller
//     try {
//         pathtracking_controller_modality_ptr_ = modality_controllers_loader_->createInstance("robotrainer_modalities/ModalitiesPathTrackingController");
//         ROS_INFO_ONCE("[fts_base_controller.cpp] Pathtracking_Controller_Modality loaded");
//     }
//     catch(pluginlib::PluginlibException& e) {
//         ROS_ERROR_STREAM("[fts_base_controller.cpp] Pathtracking_Controller_Modality plugin failed to load:" << e.what());
//         return false;
//     }

    return true;
}

/**
 * \brief (Re)configures all modalities that are set on the parameter server.
 *
 * The configure-functions of all modalities will get called. This way, the scenario of walls, areas,
 * force-areas and the path will get (re)loaded from the parameter server by the modalities.
 *
 * Returns true when the configure-functions of all modalities return true.
 * Returns false when a configure-function of one of the modalities returns false.
 */
bool FTSBaseController::configureModalities() {

    if ( !modalities_loaded_  && !loadModalityInstances() ) {
        ROS_WARN("Modalities could not be configured as they cannot be loaded by the pluginClassLoader!");
        return false;
    } else {
        modalities_configured_ = ( configureBaseModalities() && configureControllerModalities() );
        ROS_INFO_COND(modalities_configured_, "All modalities were successfully configured!");
        return modalities_configured_;
    }
}

/**
 * \brief This function loads the configurations for all Base-type-modalities one after another, and returns true only if all configurations were loaded successfully.
 */
bool FTSBaseController::configureBaseModalities() {

    bool success = true;
    if ( !force_modality_ptr_->configure() ) {
            ROS_ERROR("Unable to configure force_modality!");
            success = false;
    } else {
            ROS_DEBUG("Force_modality configured!");
    }
    // TODO(denis): Commented output for the RoSy-Study in Oct. 2020
//     if ( !walls_modality_ptr_->configure() ) {
//             ROS_ERROR("Unable to configure walls_modality!");
//             success = false;
//     } else {
//             ROS_DEBUG("Walls_modality configured!");
//     }
//     if ( !pathtracking_modality_ptr_->configure() ) {
//             ROS_ERROR("Unable to configure pathtracking_modality!");
//             success = false;
//     } else {
//             ROS_DEBUG("Pathtracking_modality configured!");
//     }
//     if ( !area_modality_ptr_->configure() ) {
//             ROS_ERROR("Unable to configure area_modality!");
//             success = false;
//     } else {
//             ROS_DEBUG("Area_modality configured!");
//     }
    return success;
}

/**
 * \brief This function loads the configurations for all Controller-type-modalities one after another, and returns true only if all configurations were loaded successfully.
 */
bool FTSBaseController::configureControllerModalities() {
    bool success = true;
    if ( !force_controller_modality_ptr_->configure() ) {
            ROS_ERROR("Unable to configure force_controller_modality!");
            success = false;
    } else {
            ROS_DEBUG("Force_controller_modality configured!");
    }
    // TODO(denis): Commented output for the RoSy-Study in Oct. 2020
//     if ( !walls_controller_modality_ptr_->configure() ) {
//             ROS_ERROR("Unable to configure walls_controller_modality!");
//             success = false;
//     } else {
//             ROS_DEBUG("Walls_controller_modality configured!");
//     }
//     if ( !pathtracking_controller_modality_ptr_->configure() ) {
//             ROS_ERROR("Unable to configure pathtracking_controller_modality!");
//             success = false;
//     } else {
//             ROS_DEBUG("Pathtracking_controller_modality configured!");
//     }
    return success;
}

bool FTSBaseController::setUseTwistInputCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

//     stopController();
    ROS_DEBUG("Called service to set twist input on/off");

    resp.success = true;
    resp.message = setUseTwistInput(!use_twist_input_);

//     restartController();

    return true;
}

bool FTSBaseController::updateWheelParamsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

    std::string lock = "update_wheel_params_callback";
    protectedToggleControllerRunning(false, lock);
    
    ROS_DEBUG("Called service to update kinematic configuration");
    resp.success = true;
    resp.success &= cob_omni_drive_controller::parseWheelParams(wheel_params_, wheel_ctrl_nh_);
    resp.success &= GeomController<UndercarriageCtrl>::update(wheel_params_);
    ROS_INFO_COND(resp.success, "Kinematics sucessfully updated!");
    ROS_ERROR_COND(!resp.success, "Kinematics update faild!");
    
    protectedToggleControllerRunning(true, lock);
    restartControllerAndOrientWheels({1, 0, 0});

    return true;
}

/**
* \brief Callback function for service "configure_modalities"
*/
bool FTSBaseController::configureModalitiesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    return configureModalities();
}

/**
 * \brief Applies the preset modalities to input base velocity and returns the resulting velocity
 */
std::array<double, 3> FTSBaseController::applyModalities(
    const std::array<double, 3> & base_vel, 
    const std::array<double, 3> & force_input)
{
    std::array<double, 3> vel_after_modalities = base_vel;
    geometry_msgs::Twist msg_before_modality;
    geometry_msgs::Twist after_force_mod, after_walls_mod, after_pathtrack_mod, after_area_mod;


    switch (modalities_used_) {
        case base_modalities:

            msg_before_modality.linear.x=base_vel[0];
            msg_before_modality.linear.y=base_vel[1];
            msg_before_modality.linear.z= 0.0;
            msg_before_modality.angular.x= 0.0;
            msg_before_modality.angular.y= 0.0;
            msg_before_modality.angular.z=base_vel[2];

            force_modality_ptr_->update(msg_before_modality, after_force_mod);
            // TODO(denis): Commented output for the RoSy-Study in Oct. 2020
//             walls_modality_ptr_->update(after_force_mod, after_walls_mod);
//             pathtracking_modality_ptr_->update(after_walls_mod, after_pathtrack_mod);
//             area_modality_ptr_->update(after_pathtrack_mod, after_area_mod);

            vel_after_modalities[0] = after_area_mod.linear.x;
            vel_after_modalities[1] = after_area_mod.linear.y;
            vel_after_modalities[2] = after_area_mod.angular.z;
            break;

        case controller_modalities:
            msg_before_modality.linear.x=base_vel[0];
            msg_before_modality.linear.y=base_vel[1];
            msg_before_modality.linear.z= 0.0;
            msg_before_modality.angular.x= 0.0;
            msg_before_modality.angular.y= 0.0;
            msg_before_modality.angular.z=base_vel[2];

            robotrainer_helper_types::wrench_twist input_msg_before_modality;
            robotrainer_helper_types::wrench_twist output_msg_after_modality;

            input_msg_before_modality.twist_ = msg_before_modality;
            input_msg_before_modality.wrench_.force.x = force_input[0];
            input_msg_before_modality.wrench_.force.y = force_input[1];
            input_msg_before_modality.wrench_.force.z = 0.0;
            input_msg_before_modality.wrench_.torque.x = 0.0;
            input_msg_before_modality.wrench_.torque.y = 0.0;
            input_msg_before_modality.wrench_.torque.z = force_input[2];

            force_controller_modality_ptr_->update(input_msg_before_modality, output_msg_after_modality);
            // TODO(denis): Commented output for the RoSy-Study in Oct. 2020
//             walls_controller_modality_ptr_->update(output_msg_after_modality, output_msg_after_modality);
//             pathtracking_controller_modality_ptr_->update(output_msg_after_modality, output_msg_after_modality);

            vel_after_modalities[0] = output_msg_after_modality.twist_.linear.x;
            vel_after_modalities[1] = output_msg_after_modality.twist_.linear.y;
            vel_after_modalities[2] = output_msg_after_modality.twist_.angular.z;
            break;
    }
    return vel_after_modalities;
}

/*___MORE COMPLEX GLOBAL MODALITIES___*/

/**
 * \brief generic function which applies the scaledCounterforce input to the scaledInputForce, and reeturns the resulting force. The counterforce is in every case directed against the current movement direction. This function is subsequently called by either the applyGlobalCounterforce, or the applyAreaCounterforce, as this control action can both be used globally or spatial.
 */
std::array<double, 3> FTSBaseController::applyCounterforce( std::array<double, 3> scaledInputForce, std::array<double, 3> scaledCounterforce) {

    std::array<double, 3> resultingForce = scaledInputForce;

    //counter adaption in respective dimension only if result would be positive, else scale to zero
    for (int i = 0; i < 3; i++) {
        if ( (scaledInputForce[i] < 0.0) && (scaledInputForce[i] + scaledCounterforce[i] < 0.0) ) {
            resultingForce[i] = scaledInputForce[i] + scaledCounterforce[i];
        } else if ( (scaledInputForce[i] > 0.0) && (scaledInputForce[i] - scaledCounterforce[i] > 0.0) ) {
            resultingForce[i] = scaledInputForce[i] - scaledCounterforce[i];
        } else { //doNothing as it is negative
            resultingForce[i] = 0.0;
        }
    }

    ROS_DEBUG("[CounterForceModality] - ScaledInputForce: [%.2f, %.2f, %.2f] - ScaledCounterForce: [%.2f, %.2f, %.2f] - ScaledResultingForce: [%.2f, %.2f, %.2f]",
        scaledInputForce[0], scaledInputForce[1], scaledInputForce[2], scaledCounterforce[0], scaledCounterforce[1], scaledCounterforce[2], resultingForce[0], resultingForce[1], resultingForce[2] );
    return resultingForce;
}

/**
 * \brief This function applies the global counterforce to the robot (which essentially calls the generic applyCounterforce function together with the full counterforce, as no fraction has to be calculated -> difference to areaCounterforce, where the counterforce gets scaled up to 100% when being close enough to the center and 0% when leaving the area)
 */
std::array<double, 3> FTSBaseController::applyGlobalCounterforce( std::array<double, 3> scaledInputForce ) {

        return applyCounterforce( scaledInputForce, getScaledLimitedFTSInput(staticCounterForce_) );
}

/**
 * \brief This function applies the area counterforce with the scaled input into the generic applyCounterforce function (which checks the distance towards the center and ultimately returns the resulting remaining velocity)
 */
std::array<double, 3> FTSBaseController::applyAreaCounterforce(std::array<double, 3> scaledInputForce) {

        std::array<double, 3> scaledCounterforce = getScaledLimitedFTSInput(areaCounterForce_);
        double scaleFactor = (counterforce_distance_to_center_ < begin_scaledown_at_this_dist_) ? 1.0 : ((counterforce_distance_to_center_ > 1.0) ? 0.0 : (1.0 - counterforce_distance_to_center_) * 2.0);
        ROS_DEBUG("CurrentDist: %.2f, ScaleRange: [%.1f, 1.0], currentScale: %.2f", counterforce_distance_to_center_, begin_scaledown_at_this_dist_, scaleFactor);

        for (int i = 0; i < 3; i++) {
                scaledCounterforce[i] *= scaleFactor;
        }

        return applyCounterforce( scaledInputForce, scaledCounterforce );
}

/**
 / ** \brief This function adapts the center of rotation (CoR) for the robot by the given global values of cor_x_ and cor_y_. This effectively allowes the robot to follow a circle with defined radius when pushing it in x-direction only (when modifying cor_y_ to values other than zero.). This also allowes the robot to tip around its front or back when moving it around its rotation axis (when changing the cor_x_ value). Furthermore, a slight change of the CoR allowes the robot to adapt to users having a slightly varying force between both hands by compensating the weaker hand by shifting the cor_y_ towards the stronger hand, effectively reducing its influence.
 */
std::array<double, 3> FTSBaseController::adaptCenterOfRotation(std::array<double, 3> fts_input_raw)
{
        std::array<double, 3> changedInput;
        changedInput[2] = fts_input_raw[2] + 
                               fts_input_raw[0] * cor_y_ - fts_input_raw[1] * cor_x_;
        changedInput[0] = fts_input_raw[0];
        changedInput[1] = fts_input_raw[1];
        // REMOVE: This under probably does not make any sense
        // X
//         if (std::fabs(cor_y_) < 0.1) {            
//             changedInput[0] = fts_input_raw[0];
//         } else {
//             changedInput[0] = fts_input_raw[0] - fts_input_raw[2]/cor_y_;
//         }
        // Y
//         if (std::fabs(cor_x_) < 0.1) {
//             changedInput[1] = fts_input_raw[1];
//         } else {
//             changedInput[1] = fts_input_raw[1] - fts_input_raw[2]/cor_x_;
//         }
        return changedInput;
}

/**
* \brief Discretizes the input values of the PT1-element so that they can be used in the update loop
*/
void FTSBaseController::discretizeController()
{
        for (int i = 0; i < 3; i++) {
                a1_[i] = exp( -1.0 / (controllerUpdateRate_ * time_const_[i]) );
                b1_[i] = gain_[i] * (1.0 - a1_[i]);
        }
        ROS_DEBUG_THROTTLE(0.1, "Parameters of discrete controller calculates as a1: [%.5f, %.5f, %.5f], b1: [%.5f, %.5f, %.5f]", a1_[0], a1_[1], a1_[2], b1_[0], b1_[1], b1_[2]);
}

/**
 * \brief This function calls the internal function to discretize the input valuesw of the PT1-element so that is can be used in the update loop
 */
void FTSBaseController::discretizeWithNewParameters( std::array<double,3> time_const_for_control, std::array<double,3> gain_for_control) {

        for (int i = 0; i < 3; i++) {
                time_const_[i] = time_const_for_control[i];
                gain_[i] = gain_for_control[i];
        }
        discretizeController();
}

/**
 * \brief This function calls the internal function to discretize the input values of the PT1-element with mass and damping
 */
void FTSBaseController::discretizeWithNewMassDamping( std::array<double,3> virtual_mass, std::array<double,3> virtual_damping) {

         std::array<double,3> time_const, gain_for_control;

         for (int i = 0; i < 3; i++) {
                gain_for_control[i] = 1.0/virtual_damping[i];
                time_const[i] = virtual_mass[i]/virtual_damping[i];
        }
        discretizeWithNewParameters(time_const, gain_for_control);
}

/**
* \brief Dynamic Reconfigure Callback of the FTSBaseController class
*/
void FTSBaseController::reconfigureCallback(robotrainer_controllers::FTSBaseControllerConfig &config, uint32_t level) {
  // Update values in GUI only
  config.x_damping = calculatevirtualdamping(config.x_max_force, config.x_max_vel, config.x_gain);
  config.x_mass = calculatevirtualmass(config.x_time_const, config.x_damping);
  config.y_damping = calculatevirtualdamping(config.y_max_force, config.y_max_vel, config.y_gain);
  config.y_mass = calculatevirtualmass(config.y_time_const, config.y_damping);
  config.rot_damping = calculatevirtualdamping(config.rot_max_torque, config.rot_max_rot_vel, config.rot_gain);
  config.rot_intertia = calculatevirtualmass(config.rot_time_const, config.rot_damping);

  if (!controller_started_) {
    ROS_WARN("Controller is not started, therefore, the parameters can not be set! \n \
              Returning the parameters from controllers");

    config.reset_controller = false;
    config.recalculate_FTS_offsets = false;
    config.apply_base_controller_params = false;
    config.no_hw_output = no_hw_output_;
    config.use_twist_input = use_twist_input_;
    config.debug_output = debug_;

    config.x_force_controller = use_controller_[0];
    config.y_force_controller = use_controller_[1];
    config.rot_controller = use_controller_[2];
    //Force-torque limits
    config.x_min_force = min_ft_[0];
    config.x_max_force = max_ft_[0];
    config.y_min_force = min_ft_[1];
    config.y_max_force = max_ft_[1];
    config.rot_min_torque = min_ft_[2];
    config.rot_max_torque = max_ft_[2];
    // velocity limits
    config.x_max_vel = max_vel_[0];
    config.y_max_vel = max_vel_[1];
    config.rot_max_rot_vel = max_vel_[2];
    // controller parameters
    config.x_gain = gain_[0];
    config.x_time_const = time_const_[0];
    config.y_gain = gain_[1];
    config.y_time_const = time_const_[1];
    config.rot_gain = gain_[2];
    config.rot_time_const = time_const_[2];
    config.backwards_max_force_scale = backwardsMaxForceScale_;
    config.backwards_max_vel_scale = backwardsMaxVelScale_;


    config.apply_control_actions = false;
    config.spatial_control_action_type = 0;
    config.counterforce_area_scaledown_dist = begin_scaledown_at_this_dist_;
    config.area_counter_force_x = areaCounterForce_[0];
    config.area_counter_force_y = areaCounterForce_[1];
    config.area_counter_torque_rot = areaCounterForce_[2];

    //global modalities
    config.reversed_max_force_scale = reversedMaxForceScale_;
    config.reversed_max_vel_scale = reversedMaxVelScale_;
    config.y_reversed = yReversed_;

    rotReversed_ = config.rot_reversed;

    config.enable_counter_force = enableCounterForce_;
    config.counter_force_x = staticCounterForce_[0];
    config.counter_force_y = staticCounterForce_[1];
    config.counter_torque_rot = staticCounterForce_[2];

    config.adapt_center_of_rotation = adapt_center_of_rotation_ = config.adapt_center_of_rotation;
    config.cor_x = cor_x_ = config.cor_x;
    config.cor_y = cor_y_;

    config.drive_mode_type = static_cast<int>(drive_mode_used_);

    config.apply_control_actions = false;

    return;
  }

    // First check if anything changed, if not go out an do not stop controller
    bool needs_processing = config.reset_controller || config.recalculate_FTS_offsets ||
                            config.apply_base_controller_params || config.apply_control_actions ||
                            no_hw_output_ != config.no_hw_output || 
                            use_twist_input_ != config.use_twist_input ||
                            debug_ != config.debug_output;

    if (!needs_processing) {
        return;
    }

    ROS_DEBUG("[FTS_Base_Ctrlr]: In dyn reconfigure.");

    if (config.reset_controller) {
        ROS_DEBUG("[FTS_Base]: Called reset controller.");
        config.reset_controller = false;
        resetController();
        return;
    }

    if (config.recalculate_FTS_offsets) {
        ROS_DEBUG("[FTS_Base]: Called recalculate offse.");
        config.recalculate_FTS_offsets = false;
        recalculateFTSOffsets();
        return;
    }

    std::string lock = "base_reconfigure_callback";
    protectedToggleControllerRunning(false, lock);
    
    debug_ = config.debug_output;

    if (no_hw_output_ != config.no_hw_output) {
        internalSetNoHWOutput(config.no_hw_output);
        ROS_WARN_COND(!no_hw_output_, "[FTS_Base]: Simulation OFF - Robot _will_ move");
        ROS_WARN_COND(no_hw_output_, "[FTS_Base]: Simulation ON - Robot _will not_ move!!!");
    }
    if (use_twist_input_ != config.use_twist_input) {
        internalSetUseTwistInput(config.use_twist_input);
    }

    if (config.apply_base_controller_params) {
        ROS_INFO("[FTS_Base_Ctrlr]: Applying base controller parameters as set in dynamic reconfigure!");
        //Use different dimensions
        use_controller_[0] = config.x_force_controller;
        ROS_INFO_COND(!use_controller_[0], "X Dimension switched off");
        use_controller_[1] = config.y_force_controller;
        ROS_INFO_COND(!use_controller_[1], "Y Dimension switched off");
        use_controller_[2] = config.rot_controller;
        ROS_INFO_COND(!use_controller_[2], "Rotational Dimension switched off");
        //Force-torque limits
        min_ft_[0] = config.x_min_force;
        max_ft_[0] = config.x_max_force;
        min_ft_[1] = config.y_min_force;
        max_ft_[1] = config.y_max_force;
        min_ft_[2] = config.rot_min_torque;
        max_ft_[2] = config.rot_max_torque;
        default_min_ft_ = min_ft_;
        default_max_ft_ = max_ft_;
        // velocity limits
        max_vel_[0] = config.x_max_vel;
        max_vel_[1] = config.y_max_vel;
        max_vel_[2] = config.rot_max_rot_vel;
        default_max_vel_ = max_vel_;
        // controller parameters
        gain_[0] = config.x_gain;
        time_const_[0] = config.x_time_const;
        gain_[1] = config.y_gain;
        time_const_[1] = config.y_time_const;
        gain_[2] = config.rot_gain;
        time_const_[2] = config.rot_time_const;
        backwardsMaxForceScale_ = config.backwards_max_force_scale;
        backwardsMaxVelScale_ = config.backwards_max_vel_scale;
        config.apply_base_controller_params = false;
    }


    if (config.apply_control_actions) {
        ROS_INFO("[FTS_Base_Ctrlr]: Applying Control actions as set in dynamic reconfigure!");

        // TODO(denis): Enalbed modalities to be on also if not configured
        //              modalities configuraiton should than also stop controller
        int modality_type = config.spatial_control_action_type;
        if (modality_type == 1) {
            if (modalities_configured_) {
                modalities_used_ = base_modalities;
                begin_scaledown_at_this_dist_ = config.counterforce_area_scaledown_dist;
                areaCounterForce_[0] = config.area_counter_force_x;
                areaCounterForce_[1] = config.area_counter_force_y;
                areaCounterForce_[2] = config.area_counter_torque_rot;
            } else {
                ROS_ERROR("Request to use base_modalities although none have been configured yet. Please send a configuration first!");
                modalities_used_ = none;
        }
        } else if (modality_type == 2) {
            if (modalities_configured_) {
                modalities_used_ = controller_modalities;
            } else {
                ROS_ERROR("Request to use controller_modalities although none have been configured yet. Please send a configuration first!");
                modalities_used_ = none;
            }
        } else {
            modalities_used_ = none;
        }

        //global modalities
        reversedMaxForceScale_ = config.reversed_max_force_scale;
        reversedMaxVelScale_ = config.reversed_max_vel_scale;
        yReversed_ = config.y_reversed;
        if (yReversed_) {
            setMaxFtAxis(1, default_max_ft_[1]*reversedMaxForceScale_);
            setMaxVelAxis(1, default_max_vel_[1]*reversedMaxVelScale_);
        } else {
            setMaxFtAxis(1, default_max_ft_[1]);
            setMaxVelAxis(1, default_max_vel_[1]);
        }
        rotReversed_ = config.rot_reversed;
        if (rotReversed_) {
            setMaxFtAxis(2, default_max_ft_[2]*reversedMaxForceScale_);
            setMaxVelAxis(2, default_max_vel_[2]*reversedMaxVelScale_);
        } else {
            setMaxFtAxis(2, default_max_ft_[2]);
            setMaxVelAxis(2, default_max_vel_[2]);
        }
        enableCounterForce_ = config.enable_counter_force;
        if (enableCounterForce_) {
            staticCounterForce_[0] = config.counter_force_x;
            staticCounterForce_[1] = config.counter_force_y;
            staticCounterForce_[2] = config.counter_torque_rot;
        }

        adapt_center_of_rotation_ = config.adapt_center_of_rotation;
        if (adapt_center_of_rotation_) {
            cor_x_ = config.cor_x;
            cor_y_ = config.cor_y;
            ROS_DEBUG("[ADAPT_CoR: ON] - CoR: (x:%.2f, y:%.2f)", cor_x_, cor_y_);
        }
        
        drive_mode_used_ = static_cast<drive_mode_type>(config.drive_mode_type);
        if (drive_mode_used_ != drive_mode_type::NONE) {
            drive_mode_request_.mode = config.drive_mode_type;
            drive_mode_request_.params.velocity_point_x = config.vel_point_x;
            drive_mode_request_.params.velocity_point_y = config.vel_point_y;
            drive_mode_request_.params.icr.x = config.icr_x;
            drive_mode_request_.params.icr.y = config.icr_y;
            drive_mode_request_.params.virtual_axle.x = config.virtual_axle_x;
            drive_mode_request_.params.virtual_axle.y = config.virtual_axle_y;
            drive_mode_request_.params.virtual_axle.a = config.virtual_axle_a;
            drive_mode_request_.params.r_min = config.ackermann_min_rad;
//             drive_mode_request_.params.p
//             drive_mode_request_.params.castor.alpha
//             drive_mode_request_.params.castor.beta
//             drive_mode_request_.params.castor.d
//             drive_mode_request_.params.castor.l
//             drive_mode_request_.params.castor.r
//             drive_mode_request_.params.a_and_b.a11 .. a33, b1 .. b3
            ucdm_->config(drive_mode_request_, drive_mode_response_);
        }
        setOrientWheels({1, 0, 0});
        
        config.apply_control_actions = false;
    }

    base_reconfigured_flag_ = true;

    protectedToggleControllerRunning(true, lock);
}

double FTSBaseController::calculatevirtualdamping(double maxforce, double maxvelocity, double gain)
{
    return (maxforce/maxvelocity) * (1/gain);
}

double FTSBaseController::calculatevirtualmass(double timeconstant, double damping)
{
    return timeconstant*damping;
}


/**
 * \brief Callback function for the counterforce Area message (which triggers the calculation of the counterforce depending on the distance towards the area center)
 */
void FTSBaseController::counterforce_area_callback( const std_msgs::Float64::ConstPtr& msg ) {

    counterforce_distance_to_center_ = msg->data;
    apply_areal_counterforce_ = true;
}


/*______GETTER METHODS FROM HERE ON______*/


/**
* \brief scales down the FTS input wrench as a percentage value of the maximum allowed Force/Torque
*
* This function scales down the FTS input wrench to a percentaged value of the maximum allowed Force/Torque for each dimension.
* If the input Force/Torque exceeds the maximum allowed value, it is set as maximum input, if it is less than the minimum detectable input, it is set as zero.
* An output value of 1.0 means maximum force input was generated for the dimension.
*/
std::array<double, 3> FTSBaseController::getScaledLimitedFTSInput(
    std::array<double, 3> raw_fts_input)
{
    std::array<double, 3> limited_force;

    for (int i = 0; i < 3; i++) {
        //histerese
        limited_force[i] = std::fabs( raw_fts_input[i] ) < min_ft_[i] ? 0.0 : raw_fts_input[i];
        //saturation
        limited_force[i] = limited_force[i] > max_ft_[i] ? max_ft_[i] : ( limited_force[i] < -max_ft_[i] ? -max_ft_[i] : limited_force[i] );
        //interval scaling
        limited_force[i] /= max_ft_[i];
    }
    return limited_force;
}

/**
* \brief Gets the last detected FTS input and returns it
*/
std::array<double, 3> FTSBaseController::getFTSInput(const ros::Time & time)
{
    std::array<double, 3> raw_fts_input;
    geometry_msgs::WrenchStamped force_input;

    force_input.header.stamp = time;
    force_input.header.frame_id = hw_fts_.getFrameId();
    force_input.wrench.force.x  = hw_fts_.getForce()[0];
    force_input.wrench.force.y  = hw_fts_.getForce()[1];
    force_input.wrench.torque.z = hw_fts_.getTorque()[2];
    
    if (debug_) {
        if (pub_force_input_raw_->trylock()) {
            pub_force_input_raw_->msg_ = force_input;
            pub_force_input_raw_->unlockAndPublish();
        }
    }

    raw_fts_input[0] = force_input.wrench.force.x;
    raw_fts_input[1] = force_input.wrench.force.y;
    raw_fts_input[2] = force_input.wrench.torque.z;

    if (std::isnan(raw_fts_input[0]) || std::isnan(raw_fts_input[1]) || std::isnan(raw_fts_input[2])) {
        ROS_FATAL("Received NaN-value in Force data. Reset target to zero.");
        raw_fts_input[0] = 0.0;
        raw_fts_input[1] = 0.0;
        raw_fts_input[2] = 0.0;
    }

    bool noInput = ( (raw_fts_input[0] == 0.0) && (raw_fts_input[1] == 0.0) && (raw_fts_input[2] == 0.0) );
    double factor_from_min_force = 3;
    bool noUserInput = ((std::fabs(raw_fts_input[0]) < min_ft_[0]/factor_from_min_force) && (std::fabs(raw_fts_input[1]) < min_ft_[1]/factor_from_min_force) && (std::fabs(raw_fts_input[2]) < min_ft_[2]/factor_from_min_force));
    bool standingRobotNoInput = ( (delta_vel_ < 0.001) && (std::fabs(raw_fts_input[0]) < min_ft_[0]) && (std::fabs(raw_fts_input[1]) < min_ft_[1]) && (std::fabs(raw_fts_input[2]) < min_ft_[2]) );
    // no user grip detected in one frame, could be coincidence so count upwards

    if ( noInput || standingRobotNoInput || noUserInput ) {
        if (userIsGripping()) {
            noGripTicks_++;
            if (noGripTicks_ > 3) {
                setUserIsGripping(false);
                timeSinceReleasing_ = time;
            } //after x consecutive no-grips set user as not gripped
        }

        raw_fts_input = zeroForce_;
        force_input.wrench.force.x  = 0;
        force_input.wrench.force.y  = 0;
        force_input.wrench.torque.z = 0;
    //user gripping
    } else {
        // TODO: Add here "last_gripped_time_"
        setUserIsGripping(true);
        noGripTicks_ = 0;
        timeSinceReleasing_ = time + ros::Duration(1.0);
    }
//         lock.release();
    forceInputForLED_ = force_input;
//     forceInputToLed( force_input );
    if (!getRunning()) {
        raw_fts_input = zeroForce_;
    }
    return raw_fts_input;
}

void FTSBaseController::sendLEDForceTopics()
{
    forceInputToLed(forceInputForLED_);
}

void FTSBaseController::forceInputToLed( const geometry_msgs::WrenchStamped force_input ) {
    if (pub_input_force_for_led_->trylock()) {
        pub_input_force_for_led_->msg_ = force_input;
        pub_input_force_for_led_->unlockAndPublish();
    }
    
    if (pub_input_force_force_for_gui_->trylock()) {
        pub_input_force_force_for_gui_->msg_ = force_input;
        pub_input_force_force_for_gui_->unlockAndPublish();
    }
}

/**
 * \brief Returns the time since the user has last released the robot (e.g. amount of time ungripped)
 */
double FTSBaseController::getTimeSinceReleasingRobot(const ros::Time& time)
{
    if (userIsGripping()) {
        return 0.0;
    } else {
        return (time - timeSinceReleasing_).toSec();
    }
}

/**
* \brief Returns the force which was used for the last update loop calculation
*/
std::array<double, 3> FTSBaseController:: getOldForce() {
    std::array<double, 3 > force_old;
    for(int i = 0 ; i < 3 ; ++i){
        force_old[i] = force_old_[i] * max_ft_[i];
    }

    return force_old;
}

/**
 * \brief Returns the percentual value of the last step input force vs the maximum allowed force
 */
std::array<double, 3> FTSBaseController:: getOldForcePercent() {
        return force_old_;
}

/**
 * \brief Returns the velocity which was used for the last update loop calculation
 */
std::array<double, 3> FTSBaseController::getOldVelocity() {
  std::array<double, 3> velocity_old;
  for(int i = 0; i < 3; ++i){
    velocity_old[i] = velocity_old_[i] * max_vel_[i];
  }
  return velocity_old;
}

/**
 * \brief Returns the percentual value from the velocity which was used for the last update loop calculation vs the maximum allowed velocity.
 */
std::array<double, 3> FTSBaseController::getOldVelocityPercent() {
        return velocity_old_;
}

std::array<double, 3> FTSBaseController::getVelocity() {
  return {platform_state_.getVelX(), platform_state_.getVelY(), platform_state_.dRotRobRadS};
}

/**
 * \brief Returns the current TimeConst values of the controller for each dimension.
 */
std::array<double, 3> FTSBaseController::getTimeConst() {
        return time_const_;
}

/**
 * \brief Returns the current Gain values of the controller for each dimension.
 */
std::array<double, 3> FTSBaseController::getGain() {
        return gain_;
}

/**
 * \brief Returns the current Damping values of the controller for each dimension.
 */
std::array<double, 3> FTSBaseController::getDamping() {
    std::array<double, 3> damping;
    for (int i=0; i<3; i++) {
        damping[i] = 1.0/gain_[i];
    }
    return damping;
}

/**
 * \brief Returns the current Mass values of the controller for each dimension.
 */
std::array<double, 3> FTSBaseController::getMass() {
    std::array<double, 3> mass;
    for (int i=0; i<3; i++) {
        mass[i] = time_const_[i]/gain_[i];
    }
    return mass;
}

/**
 * \brief Returns the current Maximum allowed Force-Torque values of the controller for each dimension.
 */
std::array<double, 3> FTSBaseController::getMaxFt() {
        return max_ft_;
}

/**
 * \brief Returns the current Maximum allowed Velocity of the controller for each dimension.
 */
std::array<double, 3> FTSBaseController::getMaxVel() {
        return max_vel_;
}

void FTSBaseController::setUserIsGripping(bool value)
{
    boost::unique_lock<boost::shared_mutex> lock(user_is_gripping_mutex_);
    userIsGripping_ = value;
}

/**
 * \brief Returns whether the user is gripping or not.
 */
bool FTSBaseController::userIsGripping() {
    boost::shared_lock<boost::shared_mutex> lock(user_is_gripping_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    return userIsGripping_;
}

/**
 * \brief Returns whether the robot is moving forward or not (if x-value is >0).
 */
bool FTSBaseController::robotIsMovingForward() {
    return (velocity_old_[0] > -0.00001);
}


/*_____SETTER_METHODS_FROM_HERE_ON_____*/
/**
* \brief sets the input value to the global force_input variable, which is used in the update loop to calculate robot movements
*/
void FTSBaseController::setForceInput(std::array<double,3> force_input_for_control) {
    force_input_[0] = force_input_for_control[0];
    force_input_[1] = force_input_for_control[1];
    force_input_[2] = force_input_for_control[2];
}

/**
* \brief sets the global maximum force-torque variable, which is used to determine percentual allowed force input
*/
void FTSBaseController::setMaxFt(std::array<double,3> max_ft_for_control) {
    max_ft_[0] = max_ft_for_control[0];
    max_ft_[1] = max_ft_for_control[1];
    max_ft_[2] = max_ft_for_control[2];
}

void FTSBaseController::setMaxFtAxis(uint axis, double max_ft_for_control) {
    max_ft_[axis] = max_ft_for_control;
}

/**
* \brief sets the global maximum velocity, to which the force is scaled
*/
void FTSBaseController::setMaxVel(std::array<double,3> max_vel_for_control) {
    max_vel_[0] = max_vel_for_control[0];
    max_vel_[1] = max_vel_for_control[1];
    max_vel_[2] = max_vel_for_control[2];
}

void FTSBaseController::setMaxVelAxis(uint axis, double max_vel_for_control) {
    max_vel_[axis] = max_vel_for_control;
}

/**
* \brief sets which controller dimensions are active
*/
void FTSBaseController::setActiveDimensions(std::array<bool, 3> enable_controller_dimension) {
    use_controller_[0] = enable_controller_dimension[0];
    use_controller_[1] = enable_controller_dimension[1];
    use_controller_[2] = enable_controller_dimension[2];
}

/**
 * \brief This function calls the "recalculateFTSOffset" service of the FTS interface. It is needed whenever the FTS is pressed to limit, because is can possibly mess up the previous offset. It is a necessary step while doing the maximum force parametrization, because it is possible that the user pushes the robot very strong against the virtual spring.
 */
bool FTSBaseController::recalculateFTSOffsets()
{

    std::string lock = "recalculate_FTS_offsets";
    protectedToggleControllerRunning(false, lock);

    bool ret = unsafeRecalculateFTSOffsets();

    protectedToggleControllerRunning(ret, lock);
    if (ret) {
        setLEDPhase(controller_led_phases::UNLOCKED);
    } else {
        setLEDPhase(controller_led_phases::STOPPED);
    }
    return ret;
}

/*_____HELPER FUNCTIONS_____*/

/**
 * \brief This function converts a std::array<double, 3> input vector into a geometry_msgs::Vector3 message
 */
geometry_msgs::Vector3 FTSBaseController::convertToMessage( std::array<double,3> input ) {
        geometry_msgs::Vector3 message;
        message.x = input[0];
        message.y = input[1];
        message.z = input[2];
        return message;
}

/**
 * \brief This function converts a std::array<double, 3> input vector into a geometry_msgs::WrenchStamped message
 */
void FTSBaseController::convertToWrenchAndPublish(
  const ros::Time & time,
  const std::array<double, 3> & input_vec,
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> & wrench_pub)
{
  if (wrench_pub->trylock()) {
    wrench_pub->msg_.header.stamp = time;
    wrench_pub->msg_.header.frame_id = controllerFrameId_;
    wrench_pub->msg_.wrench.force.x = input_vec[0];
    wrench_pub->msg_.wrench.force.y = input_vec[1];
    wrench_pub->msg_.wrench.torque.z = input_vec[2];
    wrench_pub->unlockAndPublish();
  }
}

/**
 * \brief This function converts a std::array<double, 3> input vector into a geometry_msgs::Twist message
 */
void FTSBaseController::convertToTwistAndPublish(
  const ros::Time & time,
  const std::array<double, 3> & input_vec,
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> & twist_pub)
{
  if (twist_pub->trylock()) {
    twist_pub->msg_.header.stamp = time;
    twist_pub->msg_.header.frame_id = controllerFrameId_;
    twist_pub->msg_.twist.linear.x = input_vec[0];
    twist_pub->msg_.twist.linear.y = input_vec[1];
    twist_pub->msg_.twist.angular.z = input_vec[2];
    twist_pub->unlockAndPublish();
  }
}

// TODO: We also need the twist publisher
// void FTSBaseController::publishTwist(
//   const ros::Time & time,
//   const std::array<double, 3> & input_vec,
//   std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> & twist_pub)
// {
//   if (twist_pub->trylock()) {
//     twist_pub->msg_.header.stamp = time;
//     twist_pub->msg_.header.frame_id = controllerFrameId_;
//     twist_pub->msg_.twist.linear.x = input_vec[0];
//     twist_pub->msg_.twist.linear.y = input_vec[1];
//     twist_pub->msg_.twist.angular.z = input_vec[2];
//     twist_pub->unlockAndPublish();
//   }
// }

/*_______LED STUFF______*/
/**
 * \brief Sets the LED phase (used for LED feedback messages)
 */

bool FTSBaseController::setLEDPhase(controller_led_phases requestPhase) {

    if (currentLEDPhase_ != requestPhase) { // to prevent from same message being shown multiple times
        led_ac_->cancelGoalsAtAndBeforeTime(ros::Time::now());
        currentLEDPhase_ = requestPhase;
        sendLEDGoal_ = true;
        ROS_DEBUG("LED PHASE HAS CHANGED!");
        return true;
    }
    return false;
}

/**
 * \brief sends the LED output which is defined by the currently active led_phase_ enum
 */
void FTSBaseController::sendLEDOutput() {

    if (currentLEDPhase_ == controller_led_phases::SHOW_FORCE) {
        sendLEDForceTopics();
        sendLEDGoal_ = false;
    } else if (sendLEDGoal_) {
        sendLEDGoal_ = false; //to only send it once the phase has changed
        if (currentLEDPhase_ == controller_led_phases::UNLOCKED) {
            setLEDPhase(controller_led_phases::SHOW_FORCE);
            ledGoalToSend_ = blinky_goals_.blinkyFreeMovementBlue_;
        }
        if (currentLEDPhase_ == controller_led_phases::STOPPED) {
            ledGoalToSend_ = blinky_goals_.blinkyStoppedRed_;
        }
        led_ac_->sendGoal(ledGoalToSend_);
    }
}

}  // namespce robotrainer_controllers

PLUGINLIB_EXPORT_CLASS(
    robotrainer_controllers::FTSBaseController, controller_interface::ControllerBase)
