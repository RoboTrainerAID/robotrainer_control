#include "robotrainer_controllers/fts_base_controller.h"
#include <pluginlib/class_loader.h>

namespace robotrainer_controllers {
FTSBaseController::FTSBaseController(){}
bool FTSBaseController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {   
        ROS_INFO("Init FTSBaseController!");
        wheel_params_t wheel_params;
        
        
        /* get Parameters from rosparam server (stored on yaml file) */
        ros::NodeHandle fts_base_ctrl_nh(controller_nh, "FTSBaseController");
        fts_base_ctrl_nh.param<bool>("no_hw_output", no_hw_output_, true);
        fts_base_ctrl_nh.param<double>("update_rate", controllerUpdateRate_, 1.0);
        fts_base_ctrl_nh.param<std::string>("frame_id", controllerFrameId_, "base_link");
        fts_base_ctrl_nh.param<bool>("y_reversed", yReversed_, false);
        fts_base_ctrl_nh.param<bool>("rot_reversed", rotReversed_, false);
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

        /* Control actions */
        //Adapting center of gravity
        fts_base_ctrl_nh.param<bool>("global_control_actions/adaptive_cog/adapt_cog", adapt_center_of_gravity_, false);
        fts_base_ctrl_nh.param<double>("global_control_actions/adaptive_cog/cog_x", cog_x_, 0.0);
        fts_base_ctrl_nh.param<double>("global_control_actions/adaptive_cog/cog_y", cog_y_, 0.0);
        //global counterforce enabled by default
        fts_base_ctrl_nh.param<bool>("global_control_actions/global_counterforce/enabled", enableCounterForce_, false);
        fts_base_ctrl_nh.param<double>("global_control_actions/global_counterforce/counterforce_x", staticCounterForce_[0], 0.0);
        fts_base_ctrl_nh.param<double>("global_control_actions/global_counterforce/counterforce_y", staticCounterForce_[1], 0.0);
        fts_base_ctrl_nh.param<double>("global_control_actions/global_counterforce/countertorque_z", staticCounterForce_[2], 0.0);

        
        base_reconfigured_flag_ = false;
        
        /* Debug messages */
        pub_cmd_ = root_nh.advertise<geometry_msgs::Twist>("robotrainer_controllers/fts_command_after_modalities", 1);
        pub_res_vel_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/base/velocity_output", 1);
        pub_force_lim_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/base/force_input_limited", 1);
        pub_wrench_lim_ = root_nh.advertise<geometry_msgs::WrenchStamped>("robotrainer_controllers/base/input_wrench_limited", 1);
        pub_resulting_force_after_counterforce_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/base/after_counterforce_modality", 1);
        
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
        if(!cob_omni_drive_controller::parseWheelParams(wheel_params, controller_nh) || !GeomController<UndercarriageCtrl>::init(hw_vel, wheel_params)) return false;

        if (controllerFrameId_.compare(hw_fts_.getFrameId()) != 0) {
            ROS_WARN("'frame_id' of input data is not equal to controllers frame!! Your controller will probably not work as expected! \nFTSBaseController is expecting data in '%s' frame, but input data frame is '%s'! You should probably correct this in configuration of your ForceTorqueSensorHandle.", controllerFrameId_.c_str(), hw_fts_.getFrameId().c_str());
        }

         
        /* Set dynamic reconfigure */
        base_dysrv_ = new dynamic_reconfigure::Server<robotrainer_controllers::FTSBaseControllerConfig>(fts_base_ctrl_nh);
        dynamic_reconfigure::Server<robotrainer_controllers::FTSBaseControllerConfig>::CallbackType base_dycb = boost::bind(&FTSBaseController::reconfigureCallback, this, _1, _2);
        base_dysrv_->setCallback(base_dycb);
        
        dysrv_callback_service_ = fts_base_ctrl_nh.serviceClient<dynamic_reconfigure::Reconfigure>("/base/fts_adaptive_force_controller/FTSBaseController/set_parameters");
        
        /* Initialize Position control */
        pos_ctrl_.init(wheel_params, controller_nh);  
        
        ROS_WARN_COND(no_hw_output_, "SIMULATION MODE, NO OUTPUT TO REAL ROBOT!");
        
        /* Initialize Modalities */
        service_server_ = root_nh.advertiseService("configure_modalities", &FTSBaseController::configureModalitiesCallback, this);        
        modalities_loaded_ = false;
        modalities_configured_ = false;
        
        apply_areal_counterforce_ = false;
        
        base_initialized_ = WheelControllerBase::setup(root_nh, controller_nh);
        return base_initialized_;
}

/**
 * \brief start controller 
 */
void FTSBaseController::starting(const ros::Time& time) {
    resetController();
    WheelControllerBase::starting(time);
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
        
        if (running_) {
                
                std::array<double, 3> new_vel;
                
                if ( force_input_[0] < -max_ft_[0]*backwardsMaxForceScale_) force_input_[0] = -max_ft_[0]*backwardsMaxForceScale_;
                if (enableCounterForce_ && userIsGripping_ ) {
                        force_input_ = applyGlobalCounterforce(force_input_);
                        pub_resulting_force_after_counterforce_.publish(convertToMessage(force_input_));
                } else if (apply_areal_counterforce_) {
                        if (userIsGripping_) {
                                force_input_ = applyAreaCounterforce(force_input_);
                                pub_resulting_force_after_counterforce_.publish(convertToMessage(force_input_));
                        }
                        apply_areal_counterforce_ = false;
                }
                if (adapt_center_of_gravity_) force_input_ = adaptCenterOfGravity(force_input_);                
                
                // calculate velocity using spring-mass-damping 
                for (int i = 0; i < 3; i++) {
                        if (!use_controller_[i]) {
                                ROS_INFO_ONCE("NO Controller set for Dimension %d", i);
                                new_vel[i] = 0.0;
                        } else {
                                new_vel[i] = (b1_[i] * force_old_[i] + a1_[i]*velocity_old_[i]); //in scale 0-1
                                
                                force_old_[i] = force_input_[i];
                                velocity_old_[i] = new_vel[i];
                                
                                new_vel[i] *= max_vel_[i]; //scaled back to max vel
                        }
                }
                
                
                
                //handle special cases for each dimension seperately
                if ( new_vel[0] < -max_vel_[0]*backwardsMaxVelScale_) new_vel[0] = -max_vel_[0]*backwardsMaxVelScale_; //backwards x scale
                if (yReversed_) new_vel[1] = -new_vel[1];  //inverse y
                if (rotReversed_) new_vel[2] = -new_vel[2];  //inverse rotation
                
                for (int i = 0; i < 3; i++) {
                        if (use_controller_[i] && std::fabs(new_vel[i] ) < 0.001 ) {
                                new_vel[i] = 0.0; //stop robot with too less velocity
                        }
                }
                delta_vel_ = std::pow(new_vel[0], 2) + std::pow(new_vel[1], 2) + std::pow(new_vel[2], 2); 
                
                
                if (modalities_used_ != none ) new_vel = applyModalities(new_vel); //apply modalities
                
                geometry_msgs::Vector3 vel_msg, force_msg;
                vel_msg.x = new_vel[0];
                vel_msg.y = new_vel[1];
                vel_msg.z = new_vel[2];
                force_msg.x = force_input_[0];
                force_msg.y = force_input_[1];
                force_msg.z = force_input_[2];
                pub_res_vel_.publish(vel_msg);
                pub_force_lim_.publish(force_msg);
                
                if((std::fabs(new_vel[0] - velocity_old_[0])<0.0001) && (std::fabs(new_vel[1] - velocity_old_[1])<0.0001) && (std::fabs(new_vel[2] - velocity_old_[2])<0.0001)){
                        target_.updated = false;
                } else if (no_hw_output_) {
                        ROS_WARN_THROTTLE(5.0, "[FTS_BASE_CTRLR] SIMULATION MODE, NO OUTPUT TO ROBOT, BUT SENDING VELOCITIES TO DEBUG TOPIC!");
                        target_.updated = false;
                } else {
                        
                        boost::mutex::scoped_lock lock(mutex_);
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
                
                updateState();
                pos_ctrl_.try_configure(*geom_);
                WheelControllerBase::updateCtrl(time, period);
                for (unsigned i=0; i<wheel_commands_.size(); i++){
                        steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
                        drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
                }
        } else {
                ROS_WARN_THROTTLE(0.05, "Base Controller is not running ATM");
        }
}

/**
 * \brief Stops the controller
 */
void FTSBaseController::stopController() {
        running_ = false;
}

/**
 * \brief Stops the controller
 */
void FTSBaseController::stopping(const ros::Time& /*time*/) {
}

/**
* \brief Resets the controller by setting old force and velocities to zero and re-discretizing the pt1-element parameters
*/
void FTSBaseController::resetController() {
        
        ROS_WARN_COND(!no_hw_output_, "[FTS_Base]: Reset (SIM OFF)");
        ROS_WARN_COND(no_hw_output_, "[FTS_Base]: Reset (SIM ON)!");
        
        discretizeController();
        // initialize variables
        force_old_[0] = 0.0;
        force_old_[1] = 0.0;
        force_old_[2] = 0.0;
        velocity_old_[0] = 0.0;
        velocity_old_[1] = 0.0;
        velocity_old_[2] = 0.0;
        running_ = true;
}


/*______MODALITIES FUNCTIONS FROM HERE ON______*/


/**
 * \brief This function creates an instance for all modalities one after another, and returns true only if all were created successfully. If all are setup correctly, then the trigger modalities_loaded_ is set to true.
 */
bool FTSBaseController::createModalityInstances() {
        
        ROS_INFO("Trying to load modalities so that they can be configured!");
        modalities_loaded_ = ( createBaseModalityInstances() && createControllerModalityInstances() );
        return modalities_loaded_;
}

/**
 * \brief This function creates an instance for all base-type-modalities one after another, and returns true only if all were created successfully.
 */
bool FTSBaseController::createBaseModalityInstances() {
        
        pluginlib::ClassLoader<robotrainer_modalities::ModalityBase<geometry_msgs::Twist>> modalities_loader("robotrainer_modalities", "robotrainer_modalities::ModalityBase<geometry_msgs::Twist>");
        //Virtual Force
        try {
                force_modality_ptr_ = modalities_loader.createInstance("robotrainer_modalities/VirtualForces");
                ROS_INFO_ONCE("[fts_base_controller.cpp] Force_Modality loaded");
        } catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] Force_Modality plugin failed to load:" << e.what());
                return false;
        }
        //Walls
        try {
                walls_modality_ptr_ = modalities_loader.createInstance("robotrainer_modalities/VirtualWalls");
                ROS_INFO_ONCE("[fts_base_controller.cpp] Walls_Modality loaded");
        } catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] Walls_Modality plugin failed to load:" << e.what());
                return false;
        }
        //Path Tracking
        try {
                pathtracking_modality_ptr_ = modalities_loader.createInstance("robotrainer_modalities/PathTracking");
                ROS_INFO_ONCE("[fts_base_controller.cpp] PathTracking_Modality loaded");
        } catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] PathTracking_Modality plugin failed to load:" << e.what());
                return false;
        }
        //Virtual Area
        try {
                area_modality_ptr_ = modalities_loader.createInstance("robotrainer_modalities/VirtualAreas");
                ROS_INFO_ONCE("[fts_base_controller.cpp] Areas_Modality loaded");
        } catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] Areas_Modality plugin failed to load:" << e.what());
                return false;
        }
        return true;
}

/**
 * \brief This function creates an instance for all controller-type-modalities one after another, and returns true only if all were created successfully.
 */
bool FTSBaseController::createControllerModalityInstances() {
        
        pluginlib::ClassLoader<robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>> modality_controllers_loader("robotrainer_modalities", "robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>");
        //force_controller
        try {
                force_controller_modality_ptr_ = modality_controllers_loader.createInstance("robotrainer_modalities/ModalitiesVirtualForcesController");
                ROS_INFO_ONCE("[fts_base_controller.cpp] Force_Controller_Modality loaded");
        }
        catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] Force_Controller_Modality plugin failed to load:" << e.what());
                return false;
        }
        //virtual_walls_controller
        try {
                walls_controller_modality_ptr_ = modality_controllers_loader.createInstance("robotrainer_modalities/ModalitiesVirtualWallsController");
                ROS_INFO_ONCE("[fts_base_controller.cpp] Walls_Controller_Modality loaded");
        }
        catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] Walls_Controller_Modality plugin failed to load:" << e.what());
                return false;
        }
        //path_tracking_controller
        try {
                pathtracking_controller_modality_ptr_ = modality_controllers_loader.createInstance("robotrainer_modalities/ModalitiesPathTrackingController");
                ROS_INFO_ONCE("[fts_base_controller.cpp] Pathtracking_Controller_Modality loaded");
        }
        catch(pluginlib::PluginlibException& e) {
                ROS_ERROR_STREAM("[fts_base_controller.cpp] Pathtracking_Controller_Modality plugin failed to load:" << e.what());
                return false;
        }
        
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
        
        if (!modalities_loaded_  && !createModalityInstances() ) {
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
        if ( !walls_modality_ptr_->configure() ) {
                ROS_ERROR("Unable to configure walls_modality!");
                success = false;
        } else {
                ROS_DEBUG("Walls_modality configured!");
        }
        if ( !pathtracking_modality_ptr_->configure() ) {
                ROS_ERROR("Unable to configure pathtracking_modality!");
                success = false;
        } else {
                ROS_DEBUG("Pathtracking_modality configured!");
        }
        if ( !area_modality_ptr_->configure() ) {
                ROS_ERROR("Unable to configure area_modality!");
                success = false;
        } else {
                ROS_DEBUG("Area_modality configured!");
        }
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
        if ( !walls_controller_modality_ptr_->configure() ) {
                ROS_ERROR("Unable to configure walls_controller_modality!");
                success = false;
        } else {
                ROS_DEBUG("Walls_controller_modality configured!");
        }
        if ( !pathtracking_controller_modality_ptr_->configure() ) {
                ROS_ERROR("Unable to configure pathtracking_controller_modality!");
                success = false;
        } else {
                ROS_DEBUG("Pathtracking_controller_modality configured!");
        }
        return success;
}

/**
* \brief Callback function for service "configure_modalities"
*/
bool FTSBaseController::configureModalitiesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        ROS_DEBUG("ConfigureModalitiesCallback called");
    return configureModalities();
}   

/**
 * \brief Applies the preset modalities to input base velocity and returns the resulting velocity
 */
std::array<double, 3> FTSBaseController::applyModalities( std::array<double, 3> base_vel) {
        
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
                        walls_modality_ptr_->update(after_force_mod, after_walls_mod);
                        pathtracking_modality_ptr_->update(after_walls_mod, after_pathtrack_mod);
                        area_modality_ptr_->update(after_pathtrack_mod, after_area_mod);

                        vel_after_modalities[0] = after_area_mod.linear.x;
                        vel_after_modalities[1] = after_area_mod.linear.y;
                        vel_after_modalities[2] = after_area_mod.angular.z;
                        pub_cmd_.publish(after_area_mod);
                        
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
                        input_msg_before_modality.wrench_.force.x = force_input_[0];
                        input_msg_before_modality.wrench_.force.y = force_input_[1];
                        input_msg_before_modality.wrench_.force.z = 0.0;
                        input_msg_before_modality.wrench_.torque.x = 0.0;
                        input_msg_before_modality.wrench_.torque.y = 0.0;
                        input_msg_before_modality.wrench_.torque.z = force_input_[2];
                        
                        force_controller_modality_ptr_->update(input_msg_before_modality, output_msg_after_modality);
                        walls_controller_modality_ptr_->update(output_msg_after_modality, output_msg_after_modality);
                        pathtracking_controller_modality_ptr_->update(output_msg_after_modality, output_msg_after_modality);
                        
                        vel_after_modalities[0] = output_msg_after_modality.twist_.linear.x;
                        vel_after_modalities[1] = output_msg_after_modality.twist_.linear.y;
                        vel_after_modalities[2] = output_msg_after_modality.twist_.angular.z;
                        pub_cmd_.publish(output_msg_after_modality.twist_);
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
 / ** \brief This function adapts the center of gravity (CoG) for the robot by the given global values of cog_x_ and cog_y_. This effectively allowes the robot to follow a circle with defined radius when pushing it in x-direction only (when modifying cog_y_ to values other than zero.). This also allowes the robot to tip around its front or back when moving it around its rotation axis (when changing the cog_x_ value). Furthermore, a slight change of the CoG allowes the robot to adapt to users having a slightly varying force between both hands by compensating the weaker hand by shifting the cog_y_ towards the stronger hand, effectively reducing its influence.
 */
std::array<double, 3> FTSBaseController::adaptCenterOfGravity(std::array<double, 3> fts_input_raw) {
        
        double changedTorque = fts_input_raw[0] * cog_y_ - fts_input_raw[1] * cog_x_ + fts_input_raw[2];
        std::array<double, 3> changedInput;
        ROS_DEBUG("Changed torque from: %.2f to %.2f", fts_input_raw[2], changedTorque);
        changedInput[0] = fts_input_raw[0];
        changedInput[1] = fts_input_raw[1];
        changedInput[2] = changedTorque;
        return changedInput;
}

/**
* \brief Discretizes the input values of the PT1-element so that they can be used in the update loop
*/
void FTSBaseController::discretizeController() {
        
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
* \brief Dynamic Reconfigure Callback of the FTSBaseController class
*/
void FTSBaseController::reconfigureCallback(robotrainer_controllers::FTSBaseControllerConfig &config, uint32_t level) {
                
        stopController();
        
        ROS_DEBUG("[FTS_Base_Ctrlr]: In dyn reconfigure.");
        
        no_hw_output_ = config.no_hw_output;

        if (config.apply_base_controller_params) {
                ROS_INFO("[FTS_Base_Ctrlr]: Applying base controller parameters as set in dynamic reconfigure!");
                //Use different dimensions
                use_controller_[0] = config.x_force_controller;
                ROS_INFO_COND(!use_controller_[0], "X Dimension switched off");
                use_controller_[1] = config.y_force_controller;
                ROS_INFO_COND(!use_controller_[0], "Y Dimension switched off");
                use_controller_[2] = config.rot_controller;
                ROS_INFO_COND(!use_controller_[0], "Rotational Dimension switched off");
                //Force-torque limits
                min_ft_[0] = config.x_min_force;
                max_ft_[0] = config.x_max_force;
                min_ft_[1] = config.y_min_force;
                max_ft_[1] = config.y_max_force;
                min_ft_[2] = config.rot_min_torque;
                max_ft_[2] = config.rot_max_torque;
                // velocity limits
                max_vel_[0] = config.x_max_vel;
                max_vel_[1] = config.y_max_vel;
                max_vel_[2] = config.rot_max_rot_vel;
                // controller parameters
                gain_[0] = config.x_gain;
                time_const_[0] = config.x_time_const;
                gain_[1] = config.y_gain;
                time_const_[1] = config.y_time_const;
                gain_[2] = config.rot_gain;
                time_const_[2] = config.rot_time_const;
                //untickDynamicReconfigureParam("apply_base_controller_params");
        }
        
        
        if (config.apply_control_actions) {
                ROS_INFO("[FTS_Base_Ctrlr]: Applying Control actions as set in dynamic reconfigure!");
                
                int modality_type = config.spatial_control_action_type;
                if (modality_type == 1) {
                        if (modalities_configured_) {
                                modalities_used_ = base_modalities;
                                begin_scaledown_at_this_dist_ = config.counterforce_area_scaledown_dist;
                                areaCounterForce_[0] = config.area_counter_force_x;
                                areaCounterForce_[1] = config.area_counter_force_y;
                                areaCounterForce_[2] = config.area_counter_torque_rot;
                        } else {
                                ROS_WARN("Request to use base_modalities although none have been configured yet. Please send a configuration first!");
                                modalities_used_ = none;
                        }
                } else if (modality_type == 2) {
                        if (modalities_configured_) {
                                modalities_used_ = controller_modalities;
                        } else {
                                ROS_WARN("Request to use controller_modalities although none have been configured yet. Please send a configuration first!");
                                modalities_used_ = none;
                        }
                } else {
                        modalities_used_ = none;
                }
                
                //global modalities
                yReversed_ = config.y_reversed;
                rotReversed_ = config.rot_reversed;
                backwardsMaxForceScale_ = config.backwards_max_force_scale;
                backwardsMaxVelScale_ = config.backwards_max_vel_scale;
                enableCounterForce_ = config.enable_counter_force;
                if (enableCounterForce_) {
                        staticCounterForce_[0] = config.counter_force_x;
                        staticCounterForce_[1] = config.counter_force_y;
                        staticCounterForce_[2] = config.counter_torque_rot;
                }
                
                adapt_center_of_gravity_ = config.adapt_center_of_gravity;
                if (adapt_center_of_gravity_) {
                        cog_x_ = config.cog_x;
                        cog_y_ = config.cog_y;
                        ROS_DEBUG("[ADAPT_CoG: ON] - Cog: (x:%.2f, y:%.2f)", cog_x_, cog_y_);
                }
                //untickDynamicReconfigureParam("apply_control_actions");
        }
        
        base_reconfigured_flag_ = true;
        
        resetController();
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
std::array<double, 3> FTSBaseController::getScaledLimitedFTSInput( std::array<double, 3> raw_fts_input ) {
        
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
std::array<double, 3> FTSBaseController::getFTSInput( const ros::Time& time ) {
        
        std::array<double, 3> raw_fts_input;
        geometry_msgs::WrenchStamped force_input;
        
        force_input.header.stamp = time;
        force_input.header.frame_id = hw_fts_.getFrameId();
        force_input.wrench.force.x  = hw_fts_.getForce()[0];
        force_input.wrench.force.y  = hw_fts_.getForce()[1];
        force_input.wrench.torque.z = hw_fts_.getTorque()[2];
        
        pub_wrench_lim_.publish(force_input);
        
        raw_fts_input[0] = force_input.wrench.force.x;
        raw_fts_input[1] = force_input.wrench.force.y;
        raw_fts_input[2] = force_input.wrench.torque.z;

        bool noInput = ( (raw_fts_input[0] == 0.0) && (raw_fts_input[1] == 0.0) && (raw_fts_input[2] == 0.0) );
        bool standingRobotNoInput = ( (delta_vel_ < 0.001) && (std::fabs(raw_fts_input[0]) < min_ft_[0]) && (std::fabs(raw_fts_input[1]) < min_ft_[1]) && (std::fabs(raw_fts_input[2]) < min_ft_[2]) );
        //no user grip detected in one frame, could be coincidence so count upwards
        if ( noInput || standingRobotNoInput ) {
                
                if (!noInput) ROS_DEBUG("Robot detected as standing and not gripped: Delta_v: [%.6f], Input: [%.6f, %.6f, %.6f]", delta_vel_, raw_fts_input[0], raw_fts_input[1], raw_fts_input[2]);
                
                if (userIsGripping_) {
                        noGripTicks_++; 
                        if (noGripTicks_ > 3) { 
                                userIsGripping_ = false; 
                                timeSinceReleasing_ = time;
                        } //after x consecutive no-grips set user as not gripped
                }
                
                return zeroForce_;
        //user gripping
        } else {
                userIsGripping_ = true;
                noGripTicks_ = 0;
                timeSinceReleasing_ = time + ros::Duration(1.0);
                return raw_fts_input;
        }
}

/**
 * \brief Returns the time since the user has last released the robot (e.g. amount of time ungripped)
 */
double FTSBaseController::getTimeSinceReleasingRobot( const ros::Time& time ) {
        
        if (userIsGripping_) {
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

/**
 * \brief Returns whether the user is gripping or not.
 */
bool FTSBaseController::userIsGripping() {
        
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

/**
* \brief sets the global maximum velocity, to which the force is scaled
*/
void FTSBaseController::setMaxVel(std::array<double,3> max_vel_for_control) {
        max_vel_[0] = max_vel_for_control[0];
        max_vel_[1] = max_vel_for_control[1];
        max_vel_[2] = max_vel_for_control[2];
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
bool FTSBaseController::recalculateFTSOffsets() {
        
        force_torque_sensor::CalculateSensorOffset srv;
        srv.request.apply_after_calculation = true;
        geometry_msgs::Wrench resultingOffset;
        
        if (fts_client_.call(srv)) {
                resultingOffset = srv.response.offset;
                ROS_DEBUG("Called FTS with CalculateOffsets, retrieved new offset as force:[%.4f, %.4f, %.4f] torque:[%.4f, %.4f, %.4f]", 
                         resultingOffset.force.x, resultingOffset.force.y, resultingOffset.force.z, resultingOffset.torque.x, resultingOffset.torque.y, resultingOffset.torque.z);
                return true;
        } else {
                ROS_ERROR("Failed to call Service to recalculate FTSOffsets!");
                 return false;
       }
        
}

/*_____HELPER FUNCTIONS_____*/

/**
 * \brief This function converts a std::array<double,3> input vector into a geometry_msgs::Vector3 message
 */
geometry_msgs::Vector3 FTSBaseController::convertToMessage( std::array<double,3> input ) {
        geometry_msgs::Vector3 message;
        message.x = input[0];
        message.y = input[1];
        message.z = input[2];
        return message;
}

        
}
PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::FTSBaseController, controller_interface::ControllerBase)
