#include <robotrainer_controllers/fts_adaptive_force_controller.h>

#include "robotrainer_controllers/fts_controllers_led_defines.hpp"
#include <pluginlib/class_list_macros.h>

namespace robotrainer_controllers 
{

bool FTSAdaptiveForceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {

    if (!FTSBaseController::init(robot_hw, root_nh, controller_nh)) { // init base controller if yet not initialized
        ROS_FATAL("Initializing FTSBaseController went wrong... Can not continue...");
        return false;
    }
    ROS_INFO("Init FTSAdaptiveForceController!");
    setBaseValues();
    
    diagnostic_.add("RoboTrainer Controller - Adaptive", this, &FTSAdaptiveForceController::diagnostics);
    diagnostic_.setHardwareID("FTS_Adaptive_Controller");
    diagnostic_.broadcast(0, "Initializing FTS Adaptive Controller");

    /* get Parameters from rosparam server (stored in yaml file) */
    ros::NodeHandle fts_a_ctrl_nh(controller_nh, "FTSAdaptiveForceController");
    fts_a_ctrl_nh.param<bool>("adaptive_force/use_passive_behavior_ctrlr", use_passive_behavior_ctrlr_, false);
    // Adaptive force parameters
    fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_zero_vel/x", force_scale_minvel_[0], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_zero_vel/y", force_scale_minvel_[1], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_zero_vel/rot", force_scale_minvel_[2], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_max_vel/x", force_scale_maxvel_[0], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_max_vel/y", force_scale_maxvel_[1], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_max_vel/rot", force_scale_maxvel_[2], 1.0);
    // damping adaption (yu2003)
    fts_a_ctrl_nh.param<double>("adaptive_force/damping_adaption/min/x",
                                damping_adaption_params_.min[0], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/damping_adaption/min/y",
                                damping_adaption_params_.min[1], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/damping_adaption/min/rot", 
                                damping_adaption_params_.min[2], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/damping_adaption/max/x",
                                damping_adaption_params_.max[0], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/damping_adaption/max/y",
                                damping_adaption_params_.max[1], 1.0);
    fts_a_ctrl_nh.param<double>("adaptive_force/damping_adaption/max/rot",
                                damping_adaption_params_.max[2], 1.0);
    // smooth transition
    fts_a_ctrl_nh.param<bool>("transition/use_smooth_transition", use_smooth_transition_, false);
    fts_a_ctrl_nh.param<double>("transition/transition_rate", transition_rate_, 1.0);
    // param MaxForce test
    fts_a_ctrl_nh.param<int>("parametrization/x_base/springConst", springConstant_x_, 200);
    fts_a_ctrl_nh.param<int>("parametrization/y_base/springConst", springConstant_y_, 150);
    fts_a_ctrl_nh.param<int>("parametrization/rot_base/springConst", springConstant_rot_, 60);

    fts_a_ctrl_nh.param<double>("parametrization/x_base/minForce", baseForce_minimumForce_x_, 40.0);
    fts_a_ctrl_nh.param<double>("parametrization/y_base/minForce", baseForce_minimumForce_y_, 40.0);
    fts_a_ctrl_nh.param<double>("parametrization/rot_base/minForce", baseForce_minimumTorque_, 10.0);

    fts_a_ctrl_nh.param<double>("parametrization/x_base/returnForce", returnForce_x_, 20.0);
    fts_a_ctrl_nh.param<double>("parametrization/y_base/returnForce", returnForce_y_, 15.0);
    fts_a_ctrl_nh.param<double>("parametrization/rot_base/returnForce", returnForce_rot_, 8.0);

    non_standard_max_forces_ = false;
            
    /* for parametrization */
    parametrization_active_ = false;
    parameterization_step_initalized_ = false;
    userParametrized_maxFT_ = getMaxFt();
    userParametrized_maxVel_ = getMaxVel();
    standard_maxFT_ = getMaxFt();
    standard_maxVel_ = getMaxVel();
    parameterization_current_step_ = baseX;
    sub_legtrack_ = root_nh.subscribe("/leg_detection/people_msg_stamped", 1, &FTSAdaptiveForceController::legTrackCallback, this );
    baseForce_allParamsStored_ = false;
    
    used_ft_type_ = standard_ft;
    used_vel_based_force_adaption_ = vel_based_adaption_none;
    pre_adaption_base_params_.gain = getGain();
    pre_adaption_base_params_.time_const = getTimeConst();
    pre_adaption_base_params_.damping = getDamping();
    pre_adaption_base_params_.mass = getMass();

    /* Parametrization debug topics */
    pub_base_currentDist_ = new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(controller_nh, "parametrization/base/currentDistanceFromStart", 1);
    pub_base_averageDist_ = new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(controller_nh, "parametrization/base/averageDist", 1);
    pub_base_currentRawForce_ = new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(controller_nh, "parametrization/base/currentRawForce", 1);
    pub_base_virtSpring_ = new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(controller_nh, "parametrization/base/virtualSpringForce", 1);
    pub_base_resForce_ = new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(controller_nh, "parametrization/base/resultingForce", 1);

    // leg tracking
    travelledDistance_ = 0.0;
    // adaption factors
    pub_adaptive_scale_x_ = new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "parametrization/adaptX/currentScale", 1);
    pub_adaptive_factor_min_ = new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "parametrization/adaptX/minVelFactor", 1);
    pub_adaptive_factor_max_ = new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "parametrization/adaptX/maxVelFactor", 1);
    pub_adaptive_distDiff_ = new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "parametrization/adaptX/distanceDiffToLast", 1);
    //force-torque
    pub_force_adapt_limited_input_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "adapted_limited_input_force", 1);

    pub_adaptive_scale_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "adaption/scale", 1);
    pub_adaptive_max_ft_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "adaption/maxFT", 1);

    /* Set dynamic reconfigure */
    fts_adaptive_dysrv_ = new dynamic_reconfigure::Server<robotrainer_controllers::FTSAdaptiveForceControllerConfig>(fts_a_ctrl_nh);
    dynamic_reconfigure::Server<robotrainer_controllers::FTSAdaptiveForceControllerConfig>::CallbackType adapt_dycb = boost::bind(&FTSAdaptiveForceController::reconfigureCallback, this, _1, _2);
    fts_adaptive_dysrv_->setCallback(adapt_dycb);

    /* Initialize passive behavior controller */
    ROS_INFO("[ADAPT INIT] - Starting initialization of passive behavior controller with gain:[%.2f, %.2f, %.2f] timeConst:[%.2f, %.2f, %.2f]",
            gain_[0], gain_[1], gain_[2], time_const_[0], time_const_[1], time_const_[2]);
    passive_behavior_ctrl_ = new PassiveBehaviorControllerBase();
    passive_behavior_ctrl_->setValuesFromBaseController(gain_, time_const_);
    passive_behavior_ctrl_->initInChain(root_nh, controller_nh);

    setLEDPhase(controller_led_phases::UNLOCKED); //Letting the robot blink blue to know initialization finished
    
    diagnostic_.force_update();
    return base_initialized_;
}


/**
 * \brief Update loop of the FTSAdaptiveForceController
 */
void FTSAdaptiveForceController::update(const ros::Time& time, const ros::Duration& period) {

    // to get reconfigured values as base_ values for adaption
    if (base_reconfigured_flag_) {
            setBaseValues();
            base_reconfigured_flag_ = false;
    }
    current_loop_time_ = time;
    //get inputs for the next update loop
//         ROS_WARN_COND(!running_, "Base Controller is not running at the moment!");
    std::array<double, 3> vel_percent = getOldVelocityPercent();
    std::array<double, 3> fts_input_raw = FTSBaseController::getFTSInput(current_loop_time_);

    if (parametrization_active_) {
        ROS_DEBUG_ONCE("[PARAMETRIZATION STARTED]");
        // If paramterization step has to be changed, wait for user to relese the handles
        if (getSwitchStepRequested()) {
            if (!userIsGripping()) {
                switchParametrizationStep();
            } else {
                setLEDPhase(controller_led_phases::STEP_AWAY_FROM_ROBOT);
                fts_input_raw = zeroForce_; // TODO: This should be already done in "getFTSInput"
                ROS_WARN_THROTTLE(2, "Parameterization is finished! User should relese the RoboTrainer!");
            }
        } else if (!parameterization_step_initalized_) {
            if (!userIsGripping()) {
                initParametrizationStep();
            } else {
                setLEDPhase(controller_led_phases::STEP_AWAY_FROM_ROBOT);
                fts_input_raw = zeroForce_; // TODO: This should be already done in "getFTSInput"
                ROS_WARN_THROTTLE(2, "Parameterization is finished! User should relese the RoboTrainer!");
            }
        } else if ( parameterization_step_initalized_ && !stepActivated_) { //step should be initialized, wait for user input to activate the step
            if ( (current_loop_time_ - step_finished_time_).toSec() < 1.0 ) { //wait until unlocking new phase
                    setLEDPhase(controller_led_phases::PHASE_FINISHED);
            } else {
                resetTravelledDistance();
                switch (parameterization_current_step_) {
                    case baseX: case baseYLeft: case baseYRight: case baseRotLeft: case baseRotRight:
                        resetBaseForceTest();
                        setLEDPhase(controller_led_phases::WAIT_FOR_INPUT);
                        stepActivated_ = true;
                        break;
                    case recordFeetDistance:
                    case adaptX:
                        if (userIsGripping()) {
                            setLEDPhase(controller_led_phases::WALK_FORWARD);
                            stepActivated_ = true;
                        } else {
                            setLEDPhase(controller_led_phases::WAIT_FOR_INPUT);
                        }
                        break;
                    case finished:
                        stepActivated_ = true;
                        break;
                }
            }
        //step is initialized and activated, so perform a loop of the corresponding function
        } else if (parameterization_step_initalized_ && stepActivated_) {
            updateTravelledDistance();
            switch (parameterization_current_step_) {
                case baseX:
                        ROS_WARN_ONCE("[PARAM BASE X] - STARTED");
                        fts_input_raw = baseForceTest(fts_input_raw);
                        break;
                case baseYLeft:
                        ROS_WARN_ONCE("[PARAM BASE Y LEFT] - STARTED");
                        fts_input_raw = baseForceTest(fts_input_raw);
                        break;
                case baseYRight:
                        ROS_WARN_ONCE("[PARAM BASE Y RIGHT] - STARTED");
                        fts_input_raw = baseForceTest(fts_input_raw);
                        break;
                case baseRotLeft:
                        ROS_WARN_ONCE("[PARAM BASE ROT LEFT] - STARTED");
                        fts_input_raw = baseForceTest(fts_input_raw);
                        break;
                case baseRotRight:
                    ROS_WARN_ONCE("[PARAM BASE ROT RIGHT] - STARTED");
                    fts_input_raw = baseForceTest(fts_input_raw);
                    break;
                case recordFeetDistance:
                    ROS_WARN_ONCE("[PARAM RECORD FEET DISTANCE] - STARTED");
                    fts_input_raw = recordBaseFeetDistance(fts_input_raw);
                    break;
                case adaptX:
                    ROS_WARN_ONCE("[PARAM ADAPTIVE X] - STARTED");
                    parametrizeAdaptForceX();
                    adaptForceScaleBasedOnVelocity();
                    break;
                case finished:
                    ROS_WARN("[PARAM FINISHED!] - Parametrization has finished, setting robot to active!");
                    recalculateFTSOffsets();
                    setSwitchStepRequested(true);
                    break;
                default:
                    ROS_WARN("Invalid parametrization step set, thus finishing parametrization now");
                    parameterization_current_step_ = finished;
            }
        } else {
            fts_input_raw = zeroForce_; //prevent robot from moving when nobody is gripping it
        }
    } else { // robot in normal use
        setActiveDimensions( all_active_);

        if (used_vel_based_force_adaption_ != vel_based_adaption_none) {
            adaptForceScaleBasedOnVelocity();
        }
    }

    std::array<double,3> scaledLimitedFTSInput = FTSBaseController::getScaledLimitedFTSInput(fts_input_raw);
    if (pub_force_adapt_limited_input_->trylock()) {
        pub_force_adapt_limited_input_->msg_ = convertToMessage(scaledLimitedFTSInput);
        pub_force_adapt_limited_input_->unlockAndPublish();
    }

    if (use_passive_behavior_ctrlr_) {
        double timeSinceRelease = FTSBaseController::getTimeSinceReleasingRobot(current_loop_time_);
        std::array<double, 3> passive_filtered_input = passive_behavior_ctrl_->updateWithInputs(current_loop_time_, period, scaledLimitedFTSInput, timeSinceRelease);
        FTSBaseController::setForceInput( passive_filtered_input );
        FTSBaseController::update(current_loop_time_, period);
        passive_behavior_ctrl_->updateBaseControllerValues(current_loop_time_, fts_input_raw, getOldVelocity() );
    } else {
        FTSBaseController::setForceInput( scaledLimitedFTSInput );
        FTSBaseController::update(current_loop_time_, period);
    }
    
    diagnostic_.update();
}

/// \brief Publishes diagnostics and status
void FTSAdaptiveForceController::diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (used_vel_based_force_adaption_ || used_ft_type_) {
        status.summary(0, "Cotroller is adapting parameters");
    } else {
        status.summary(1, "Controller is just calling Base");
    }

    status.add("parameterization", parametrization_active_);
    status.add("param. step", parameterization_step_names_[parameterization_current_step_]);
    status.add("param. step initalized", parameterization_step_initalized_);
    status.add("param. baseForce params stored", baseForce_allParamsStored_);

    status.add("Used Base force type", user_ft_names_[used_ft_type_]);
    
    std::string str; 
    str += std::string(" x: ") + std::to_string(userParametrized_maxFT_[0]);
    str += std::string(" y: ") + std::to_string(userParametrized_maxFT_[1]);
    str += std::string(" rot-z: ") + std::to_string(userParametrized_maxFT_[2]);
    status.add("User Base Force F max", str);
    str = "";
    str += std::string(" x: ") + std::to_string(userParametrized_maxVel_[0]);
    str += std::string(" y: ") + std::to_string(userParametrized_maxVel_[1]);
    str += std::string(" rot-z: ") + std::to_string(userParametrized_maxVel_[2]);
    status.add("User Base Force V max", str);
    
    status.add("Use Force Based Vel adaption", 
                velocity_based_adaption_names_[used_vel_based_force_adaption_]);
    str = "";
    str += std::string(" x: ") + std::to_string(force_scale_minvel_[0]);
    str += std::string(" y: ") + std::to_string(force_scale_minvel_[1]);
    str += std::string(" rot-z: ") + std::to_string(force_scale_minvel_[2]);
    status.add("Velocity-Based Force min scale", str);
    str = "";
    str += std::string(" x: ") + std::to_string(force_scale_maxvel_[0]);
    str += std::string(" y: ") + std::to_string(force_scale_maxvel_[1]);
    str += std::string(" rot-z: ") + std::to_string(force_scale_maxvel_[2]);
    status.add("Velocity-Based Force max scale", str);
    
}

void FTSAdaptiveForceController::forceInputToLed( const geometry_msgs::WrenchStamped force_input ) {
    if (not parametrization_active_) {
        FTSBaseController::forceInputToLed(force_input);
    }
}

/**
 * \brief This function initializes the parameters for the respective parametrization step, which is only executed after initialization is marked true
 */
void FTSAdaptiveForceController::initParametrizationStep() {

    std::array<double, 3> direction = {1, 0, 0};

    // All steps variables
    resetTravelledDistance();
    recalculateFTSOffsets();

    //initialize variables
    switch(parameterization_current_step_) {
        case baseX:
            resetBaseForceTest();
            baseForce_springConstant_ = springConstant_x_;
            baseForce_minimumForce_ = baseForce_minimumForce_x_;
            baseForce_testCompleted_ = false;
            direction = {1, 0, 0};
            ROS_DEBUG_THROTTLE(1, "[BASE X] - INIT - springConstant: %d, minForceNeeded: %.2f",
                                    baseForce_springConstant_, baseForce_minimumForce_ );
            break;
        case baseYLeft:
            resetBaseForceTest();
            baseForce_springConstant_ = springConstant_y_;
            baseForce_minimumForce_ = baseForce_minimumForce_y_;
            baseForce_testCompleted_ = false;
            direction = {0, 1, 0};
            ROS_DEBUG_THROTTLE(1, "[BASE Y LEFT] - INIT - springConstant: %d, minForceNeeded: %.2f",
                                    baseForce_springConstant_, baseForce_minimumForce_ );
            break;
        case baseYRight:
            resetBaseForceTest();
            baseForce_springConstant_ = springConstant_y_;
            baseForce_minimumForce_ = baseForce_minimumForce_y_;
            baseForce_testCompleted_ = false;
            direction = {0, 1, 0};
            ROS_DEBUG_THROTTLE(1, "[BASE Y RIGHT] - INIT - springConstant: %d, minForceNeeded: %.2f",
                                    baseForce_springConstant_, baseForce_minimumForce_ );
            break;
        case baseRotLeft:
            resetBaseForceTest();
            baseForce_springConstant_ = springConstant_rot_;
            baseForce_minimumForce_ = baseForce_minimumTorque_;
            baseForce_testCompleted_ = false;
            direction = {0, 0, 1};
            ROS_DEBUG_THROTTLE(1, "[BASE ROT LEFT] - INIT - springConstant: %d, minForceNeeded: %.2f",
                                    baseForce_springConstant_, baseForce_minimumForce_ );
            break;
        case baseRotRight:
            resetBaseForceTest();
            baseForce_springConstant_ = springConstant_rot_;
            baseForce_minimumForce_ = baseForce_minimumTorque_;
            baseForce_testCompleted_ = false;
            direction = {0, 0, 1};
            ROS_DEBUG_THROTTLE(1, "[BASE ROT RIGHT] - INIT - springConstant: %d, minForceNeeded: %.2f",
                                    baseForce_springConstant_, baseForce_minimumForce_ );
            break;
        case recordFeetDistance:
            footDistanceSum_ = 0.0;
            storedFootDistances_ = 0;
            fwd_footDistanceRecorded_ = false;
            bwd_footDistanceRecorded_ = false;
            debug_legTrackingStarted_ = current_loop_time_;
            footDistance_lastTravelledDist_ = 0.0;
            break;
        case adaptX:
            adaptX_startTime_ = current_loop_time_;
            adaptX_movingLegDistList_.clear();
            adaptX_movingLegDistList_.push_back(leg_dist_avg_x_); //reset stored distances
            adaptX_movementSpeedList_.clear();
            adaptX_movementSpeedList_.push_back(getOldVelocityPercent()[0]);
            adaptX_unchangedCtr_ = 0;
            adaptX_testIntervallDuration_ = 1.0;
            used_vel_based_force_adaption_ = vel_based_StoglZumkeller2020;
            use_smooth_transition_ = true;
            setLegTrackUpdated(false);
            force_scale_minvel_[0] = 1.5;
            force_scale_minvel_[1] = 1.5;
            force_scale_minvel_[2] = 1.0;
            force_scale_maxvel_[0] = 0.35;
            force_scale_maxvel_[1] = 0.35;
            force_scale_maxvel_[2] = 0.35;
            footDistance_lastTravelledDist_ = 0.0;
            // sendDebugTopicsParamAdaptX(1.0, 0.0);  //ACTIVATE FOR DEBUG ONLY
            ROS_DEBUG_THROTTLE(1, "[ADAPT X] - INIT - X-Adaptive initialized !");
            break;
        case finished:
            break;
        default:
            ROS_WARN("TRIED TO INITIALIZE AN INVALID CASE...");
    }

    //enable right dimension only and set starting led
    switch(parameterization_current_step_) {
        case baseX:
            setActiveDimensions(x_active_);
            break;
        case baseYLeft:
            setActiveDimensions(y_active_);
            break;
        case baseYRight:
            setActiveDimensions(y_active_);
            break;
        case baseRotLeft:
            setActiveDimensions(rot_active_);
            break;
        case baseRotRight:
            setActiveDimensions(rot_active_);
            break;
        case recordFeetDistance: case adaptX:
            setActiveDimensions(x_rot_active_);
            break;
        default:
            setActiveDimensions(all_inactive_);
    }
    FTSBaseController::restartControllerAndOrientWheels(direction);
    parameterization_step_initalized_ = true;
    return;
}

void FTSAdaptiveForceController::returnRobotAutonomouslyToStartPosition(bool finished_successfully)
{
    double secondsSinceLastGrip = (current_loop_time_ - last_gripped_time_).toSec();

    if (!autonomously_returning_ && secondsSinceLastGrip > 1.0) { // initalize returing
        setLEDPhase(controller_led_phases::ROBOT_IN_AUTOMATIC_MOVEMENT);
        FTSBaseController::setUseTwistInput(true);
        autonomously_returning_ = true;
        autonomously_needed_time_sec_ = travelledDistance_ / 0.1;
        autonomously_start_time_ = current_loop_time_;
        ROS_DEBUG("[BASE] - PHASE COMPLETED, RETURNING autonomously to start with following parameter: \n needed_time_sec: %f \n last_time_: %f \n velocity: %f", autonomously_needed_time_sec_, autonomously_start_time_.toSec(), 0.1);
    } else if ( autonomously_returning_ ) { //  move robot back to starting pointn
        autonomously_traveled_time_ = current_loop_time_ - autonomously_start_time_;

        if (autonomously_traveled_time_.toSec() < 0.995*autonomously_needed_time_sec_)
        {
            autonomously_returning_velocity = 0.1;
        } else {
            autonomously_returning_velocity = 0.0;
        }
        switch(parameterization_current_step_) {
            case baseX:
                twist_command_.linear.x = -autonomously_returning_velocity;
                break;
            case baseYLeft:
                twist_command_.linear.y = -autonomously_returning_velocity;
                break;
            case baseYRight:
                twist_command_.linear.y = autonomously_returning_velocity; // invert in order to use positive spring force as usual
                break;
            case baseRotLeft:
                twist_command_.angular.z = -autonomously_returning_velocity;
                break;
            case baseRotRight:
                twist_command_.angular.z = autonomously_returning_velocity;;// invert in order to use positive spring force as usual
                break;
        }
        if (autonomously_traveled_time_.toSec() >= autonomously_needed_time_sec_) //robot has returned
        {
            recalculateFTSOffsets();
            FTSBaseController::setUseTwistInput(false);
            autonomously_returning_ = false;
            autonomously_needed_time_sec_ = 0;
            ROS_INFO("[BASE TEST] - COMPLETED AND RETURNED! - Phase completed and robot returned to starting point.");
            baseForce_testCompleted_ = false;
            resetTravelledDistance();
            setLEDPhase(controller_led_phases::PHASE_FINISHED);
            if (finished_successfully) {
                setSwitchStepRequested(true);  // to trigger the next step
            }
        }
    }

}

/**
 * \brief This step is the first parametrization step. It is used to parametrize the base force in the current direction which the user can handle safely. This is done by adding a virtual spring with predefined spring constant in front of the robot, against which the user should push and hold the platform at an acceptable counterforce given by himself. For this reason, only the to-be-tested direction of the robot is set active, and upon gripping the robot (e.g. giving its FTS-sensor inputs) the current position is stored as 'home'. For each step the travelled distance from this home-point is calculated by integrating the current velocity, and the respective virtual spring force is calculated. It is then added against the user input in the same direction to steer the robot, to give the user the impression of a counterforce becoming lineary stronger the more you push the platform forward. At the same time, a rolling average of the travelled distance is calculated and compared against the current distance. If the user keeps holding the robot around a margin of the same distance over the course of the rolling average, and the input force exceeds a minimum force requirement (to ensure a reasonable resulting force), the average force during the rolling average is stored as the new baseline force, from which the velocity will be derived. The parametrization step is then marked finished, and the next step triggered.
 */
std::array<double, 3> FTSAdaptiveForceController::baseForceTest( std::array<double, 3> fts_input_raw) {

    if (userIsGripping()) {
        last_gripped_time_ = current_loop_time_;
    }

    if (baseForce_testCompleted_) { //return robot to start new phase
        ledForceInput_ = 0.0;
        if ( userIsGripping() /*|| userBehindRobot()*/ ) {
            setLEDPhase(controller_led_phases::STEP_AWAY_FROM_ROBOT);
            last_gripped_time_ = current_loop_time_;
            ROS_WARN_THROTTLE(10, "[BASE] - COMPLETED, USER STILL GRIPPING... Step away from Robot in order to reset its position.");
        } else {
            returnRobotAutonomouslyToStartPosition(true);
        }
        return zeroForce_;
    }

    // test not completed yet
    double currentVirtualSpringForce = travelledDistance_ * baseForce_springConstant_;
    double userInput = 0.0;
    switch(parameterization_current_step_) {
        case baseX:
            userInput = fts_input_raw[0];
            break;
        case baseYLeft:
            userInput = fts_input_raw[1];
            break;
        case baseYRight:
            userInput = -1.0 * fts_input_raw[1]; // invert in order to use positive spring force as usual
            break;
        case baseRotLeft:
            userInput = fts_input_raw[2];
            break;
        case baseRotRight:
            userInput = -1.0 * fts_input_raw[2];// invert in order to use positive spring force as usual
            break;
    }

    if ( !userIsGripping() ) {
        if (travelledDistance_ > 0.01 ) {
            double secondsSinceLastGrip = (current_loop_time_ - last_gripped_time_).toSec();
            if (secondsSinceLastGrip < 0.5) {
                setLEDPhase(controller_led_phases::STEP_AWAY_FROM_ROBOT);
            } else {
                returnRobotAutonomouslyToStartPosition(false);
            }
            ledForceInput_ = 0.0;
            ROS_WARN_THROTTLE(5, "[BASE] - AB ANDONED IN PHASE! - Slowly returning robot...");
            return zeroForce_;
        } else {
            resetBaseForceTest();
            setLEDPhase(controller_led_phases::WAIT_FOR_INPUT);
            ROS_INFO_THROTTLE(10, "[BASE] - WAIT FOR START- Grab the Robot and move in the shown direction to start recording !");
            return zeroForce_;
        }
    } else {
        if ( (userInput < 0.0) && (travelledDistance_ < 0.01)) {
            ROS_WARN_THROTTLE(0.5, "WRONG DIRECTION !");
            setLEDPhase(controller_led_phases::WAIT_FOR_INPUT);
            return zeroForce_;
        } else { //only case not returning
            ledForceInput_ = userInput;
            setLEDPhase(controller_led_phases::SHOW_FORCE);
        }
    }

    double secondsInPhase = (current_loop_time_ - baseForce_startingTime_).toSec();
    double effectiveForce = std::fmax(-50.0, userInput - currentVirtualSpringForce);

    //add new value to average distances
    baseForce_travelledDistanceList_.push_back(travelledDistance_);
    if (secondsInPhase > baseForce_movingAverageTimeframe_) { //erase elements longer than "maxTime" ago
            baseForce_travelledDistanceList_.pop_front();
    }

    //check if distance is kept over the time (using average distances) and minimum required force is met
    double averageDistance = std::accumulate(baseForce_travelledDistanceList_.begin(), baseForce_travelledDistanceList_.end(), 0.0) / baseForce_travelledDistanceList_.size();
    double diffToAvg = std::fabs(travelledDistance_ - averageDistance);
    if ((diffToAvg < baseForce_holdingDistance_) && (userInput > baseForce_minimumForce_ ) ) {
            baseForce_storeRawInput_.push_back(userInput);
            baseForce_stableForceCounter_++;
    } else {
            baseForce_stableForceCounter_ = 0;
            baseForce_storeRawInput_.clear();
    }
    
    if (debug_) {
        sendDebugTopicsParamBase(travelledDistance_, averageDistance, userInput,
                                 currentVirtualSpringForce, effectiveForce);
    }

    //Check phase completion and return spring-modified force input
    if (baseForce_stableForceCounter_ < 120) {
        switch(parameterization_current_step_) {
            case baseX:
                    return {effectiveForce, 0.0, 0.0};
            case baseYLeft:
                    return {0.0, effectiveForce, 0.0};
            case baseYRight:
                    return {0.0, -1.0 * effectiveForce, 0.0};
            case baseRotLeft:
                    return {0.0, 0.0, effectiveForce};
            case baseRotRight:
                    return {0.0, 0.0, -1.0 * effectiveForce};
            default:
                return zeroForce_;
    }
    } else { //phase completed

            double stableForce = std::accumulate(baseForce_storeRawInput_.begin(), baseForce_storeRawInput_.end(), 0.0) / baseForce_storeRawInput_.size();

            //set retrieved force as base force for the dimension
            switch (parameterization_current_step_) {
                case baseX:
                        userParametrized_maxFT_[0] = stableForce;
                        ROS_INFO("[BASE X] - COMPLETED, RECORD - Stable X-Force detected as %.2f (newBaseForce:[%.2f, %.2f, %.2f])",
                                    stableForce, userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2]);
                        break;
                case baseYLeft:
                        userParametrized_maxFT_[1] = stableForce;
                        ROS_INFO("[BASE Y LEFT] - COMPLETED, RECORD - Stable Y-Left-Force detected as %.2f (newBaseForce:[%.2f, %.2f, %.2f])",
                                    stableForce, userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2]);
                        break;
                case baseYRight:
                        userParametrized_maxFT_[1] = 0.5 * stableForce + 0.5 * userParametrized_maxFT_[1];
                        ROS_INFO("[BASE Y RIGHT] - COMPLETED, RECORD - Stable Y-Right-Force detected as %.2f (newBaseForce:[%.2f, %.2f, %.2f] - median of y-left + y-right)",
                                    stableForce, userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2]);
                        break;
                case baseRotLeft:
                        userParametrized_maxFT_[2] = std::fabs(stableForce);
                        ROS_INFO("[BASE ROT LEFT] - COMPLETED, RECORD - Stable Torque detected as %.2f (newBaseForce:[%.2f, %.2f, %.2f])",
                                    std::fabs(stableForce), userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2]);
                        break;
                case baseRotRight:
                        userParametrized_maxFT_[2] = 0.5 *std::fabs(stableForce) + 0.5 * userParametrized_maxFT_[2];
                        ROS_INFO("[BASE ROT RIGHT] - COMPLETED, RECORD - Stable Torque detected as %.2f (newBaseForce:[%.2f, %.2f, %.2f] - median of torque-left + torque-right)",
                                    std::fabs(stableForce), userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2]);
                        baseForce_allParamsStored_ = true;
                        break;
            }

            //adapt max velocity
            std::array<double,3> oldMaxFt = getMaxFt(); //get again in case preparametrized values get parametrized again
            std::array<double,3> oldMaxVel = getMaxVel();
            userParametrized_maxVel_ = oldMaxVel;
            double percentFromOldMax;
            switch (parameterization_current_step_) {
                    case baseX:
                            percentFromOldMax = userParametrized_maxFT_[0] / oldMaxFt[0];
                            if ( percentFromOldMax < 1.0 ) {
                                    userParametrized_maxVel_[0] = percentFromOldMax * oldMaxVel[0];
                                    ROS_INFO("[BASE X] - COMPLETED, CHANGE VELOCITY - New X-Force by factor %.2f lower than previous standard force! Lowering maxVelocity to [%.2f, %.2f, %.2f]",
                                                percentFromOldMax, userParametrized_maxVel_[0], userParametrized_maxVel_[1], userParametrized_maxVel_[2]);
                            }
                            break;
                    case baseYLeft:
                    case baseYRight:
                            percentFromOldMax = userParametrized_maxFT_[1] / oldMaxFt[1];
                            if ( percentFromOldMax < 1.0 ) {
                                    userParametrized_maxVel_[1] = percentFromOldMax * oldMaxVel[1];
                                    ROS_INFO("[BASE Y] - COMPLETED, CHANGE VELOCITY - New Y-Force by factor %.2f lower than previous standard force! Lowering maxVelocity to [%.2f, %.2f, %.2f]",
                                                percentFromOldMax, userParametrized_maxVel_[0], userParametrized_maxVel_[1], userParametrized_maxVel_[2]);
                            }
                            break;
                    case baseRotLeft:
                    case baseRotRight:
                            percentFromOldMax = userParametrized_maxFT_[2] / oldMaxFt[2];
                            if ( percentFromOldMax < 1.0 ) {
                                    userParametrized_maxVel_[2] = percentFromOldMax * oldMaxVel[2];
                                    ROS_INFO("[BASE ROT] - COMPLETED, CHANGE VELOCITY - New Torque by factor %.2f lower than previous standard torque! Lowering maxVelocity to [%.2f, %.2f, %.2f]",
                                                percentFromOldMax, userParametrized_maxVel_[0], userParametrized_maxVel_[1], userParametrized_maxVel_[2]);
                            }
                            break;
            }
            baseForce_testCompleted_ = true;
            return zeroForce_;
    }//Phase completed
}

/**
 * \brief This function records the median distance of the users feet while pushing the robot. The record is done by only allowing x-direction movement, and requesting the user to walk a straight path forward and then returning back to the starting point. The current direction is percieved by the travelledDistance (calculated as integral of the robot velocity, same as in parametrizeBaseX), and upon moving backwards, the whole median distances from each previous recording are averaged and stored as 'fwd_medianFootDistance_'. Upon returning back to the starting position, determined by reaching zero travelledDistance again, the measurements from the return path are likewise averaged and stored as 'bwd_medianFootDistance_'. These two values are then used in the next step to parametrize the adaption values for 'parametrizeAdaptForceX', so this step is marked completed and the next step started.
 */
std::array<double, 3> FTSAdaptiveForceController::recordBaseFeetDistance( std::array<double, 3> fts_input_raw ) {
    // assume forward movement with ever-increasing travelledDistance
    // assume backwards movement with ever-decreasing travelledDistance
    // assume task completion upon reaching zero travelledDistance again
    double lastStepDistance = (travelledDistance_ - footDistance_lastTravelledDist_);
    bool forwardMotion = (lastStepDistance > 0.0);

    if (getLegTrackUpdated() && userIsGripping() && (std::fabs(lastStepDistance) > 0.002) ) { //update stored distances
        footDistanceSum_ += leg_dist_avg_x_;
        storedFootDistances_++;
        footDistance_lastTravelledDist_ = travelledDistance_;
    }
    setLegTrackUpdated(false);

    if ( storedFootDistances_ < 200) { // to get a good amount of data for averaging
        if (!fwd_footDistanceRecorded_ ) {
                fts_input_raw = (fts_input_raw[0] < 0.0) ? zeroForce_ : fts_input_raw;
        } else {
                fts_input_raw = (fts_input_raw[0] > 0.0) ? zeroForce_ : fts_input_raw;
        }
    } else if (!fwd_footDistanceRecorded_ && !forwardMotion ) { // recording forward foot distance, robot now going back again -> store forward
        fwd_medianFootDistance_ = footDistanceSum_ / storedFootDistances_;
        fwd_footDistanceRecorded_ = true;
        footDistanceSum_ = 0.0; //reset stored distances
        storedFootDistances_ = 0; //reset stored distances
        ROS_WARN("[PARAM FOOT DIST] - Front median dist. recorded as %.3f m. from center.", fwd_medianFootDistance_);
        setLEDPhase(controller_led_phases::WALK_BACKWARDS);
    } else if (fwd_footDistanceRecorded_ && !bwd_footDistanceRecorded_ ) {
        if (travelledDistance_ < 0.5) {
            setLEDPhase(controller_led_phases::ALMOST_RETURNED);
            if (!userIsGripping() || travelledDistance_ < 0.05) {
                bwd_medianFootDistance_ = footDistanceSum_ / storedFootDistances_;
                bwd_footDistanceRecorded_ = true;
                ROS_WARN("[PARAM FOOT DIST] - Backwards median dist. recorded as %.3f m. from center.", bwd_medianFootDistance_);
                fts_input_raw = zeroForce_;
            } else {
                fts_input_raw = (fts_input_raw[0] > 0.0) ? zeroForce_ : fts_input_raw;
            }
        }
    } else if (fwd_footDistanceRecorded_ && bwd_footDistanceRecorded_) {
        setSwitchStepRequested(true);
        fts_input_raw = zeroForce_;
    }
    return fts_input_raw;
}

/**
 * \brief This function is used to parametrize the adaption parameters for the x-direction by using information about the averaged foot distance. Using the baseline-values 'fwd_medianFootDistance_' and 'bwd_medianFootDistance_' for forward and backwards direction, the difference towards these values is used as information about the current robot characteristics towards the user. If the robot is further away from the user than baseline, the robot reacts too fast. In this case, the reactiveness is reduced. In contrary, if the robot reacts too stiff and slow, the distance of the user towards it will decrease, and the parameters are tweaked towards a more adaptive behavior.
 */
void FTSAdaptiveForceController::parametrizeAdaptForceX() {
    if (adaptX_unchangedCtr_ >= 5) {
        if (!userIsGripping()) {
            setSwitchStepRequested(true);
        }
        return;
    }

    if (!getLegTrackUpdated()) {
        return;
    }
    if (!getLegTrackUpdated() || !userIsGripping()) {
        return;
    }
    setLegTrackUpdated(false);

    double lastStepDistance = (travelledDistance_ - footDistance_lastTravelledDist_);
    footDistance_lastTravelledDist_ = travelledDistance_;
    ROS_INFO_THROTTLE(2, "Calculating distance: %f", lastStepDistance);
    if (std::fabs(lastStepDistance) < 0.002) { //robot is not moving
        return;
    }

    adaptX_movingLegDistList_.push_back(leg_dist_avg_x_);
    adaptX_movementSpeedList_.push_back( std::fabs(getOldVelocityPercent()[0]) );

    if (adaptX_movingLegDistList_.size() < 50) {
        return; // wait until 50 recordings are made for a stable legDistance
    } else {
        adaptX_movingLegDistList_.pop_front();
        adaptX_movementSpeedList_.pop_front();
    }

    double secondsSinceLastParametrization = (current_loop_time_ - adaptX_lastTestTime_).toSec();

    /* to let the user recognize an adaption and the robot gain new insight on the parameter
     * changes with last adaption parametrization
     */
    if (secondsSinceLastParametrization < adaptX_testIntervallDuration_) {
        return;
    }

    double newAvgDist = std::accumulate(adaptX_movingLegDistList_.begin(), adaptX_movingLegDistList_.end(), 0.0) / adaptX_movingLegDistList_.size();
    double avgSpeed = std::fmin( std::accumulate(adaptX_movementSpeedList_.begin(), adaptX_movementSpeedList_.end(), 0.0) / adaptX_movementSpeedList_.size(), 1.0);
    double robotDistDiff = robotIsMovingForward() ? (newAvgDist - fwd_medianFootDistance_) : (newAvgDist - bwd_medianFootDistance_);
    double percentualOfMaxDiff = (std::fabs(robotDistDiff) < minLegDistance_) ? 0.0 : std::fmin( std::fabs(robotDistDiff) / maxLegDistance_, 1.0);
    double avgScale = force_scale_minvel_[0] + ( force_scale_maxvel_[0] - force_scale_minvel_[0] ) * avgSpeed;

    ROS_INFO("[PARAM AdaptX] [DEBUG] RobotDistDiff: %.2f, percentualOfMaxDiff: %.2f, avgSpeedFactor: %.2f, avgScale: %.2f (timeInPhase: %.2f)",
            robotDistDiff, percentualOfMaxDiff, avgSpeed, avgScale, (current_loop_time_ - adaptX_startTime_).toSec());

    if (percentualOfMaxDiff < 0.01) { // no need to adapt on this step
        adaptX_unchangedCtr_++;
        ROS_INFO("[PARAM AdaptX - NO CHANGE] Difference in last measurement less than minimal leg distance, thus not changing parameters now. (%d steps total)", adaptX_unchangedCtr_);
        if (adaptX_unchangedCtr_ >= 5) {
            //declare phase finished
            ROS_INFO("[PARAM AdaptX] Last five measurements were in range, which concludes parametrization of adaptive features\n Enabling velocity-adaption using force_scale_minvel: %.2f and force_scale_maxvel: %.2f as parameters now.", force_scale_minvel_[0], force_scale_maxvel_[0]);
            force_scale_maxvel_[1] = force_scale_maxvel_[0];
            force_scale_minvel_[1] = force_scale_minvel_[0];
            setLEDPhase(controller_led_phases::PHASE_FINISHED);
        }
    } else { //adaption needed
        adaptX_unchangedCtr_ = 0; //reset unchanged counter
        double quadraticPercentualChange = std::fmin(1.0, std::pow(percentualOfMaxDiff, adaptive_parametrization_adaptionRate_));

        if (robotDistDiff > minLegDistance_) { // distance too high = person too far away from robot -> robot reacts too fast -> decrease scales
            //TODO determine based on velocity, how much you need to adapt both parameters
            if (avgScale < 1.0) { //robot needed less than normal force for max velocity -> increase force_scale_maxvel_[0] to make it more force needed
                force_scale_maxvel_[0] += quadraticPercentualChange * (1.0 - force_scale_maxvel_[0]);
                ROS_INFO("[PARAM AdaptX - CHANGE] Distance %.2f Higher than baseline -> ROBOT TOO FAST -> avgScale < 1.0 -> adapting force_scale_maxvel_ to %.2f", robotDistDiff, force_scale_maxvel_[0]);
            } else {  //robot needed more than normal force, but was too fast -> increase force_scale_minvel_[0]
                force_scale_minvel_[0] += quadraticPercentualChange * (2.0 - force_scale_minvel_[0]);
                ROS_INFO("[PARAM AdaptX - CHANGE] Distance %.2f Higher than baseline -> ROBOT TOO FAST -> avgScale < 1.0 -> adapting force_scale_minvel_ to %.2f", robotDistDiff, force_scale_minvel_[0]);
            }
        } else if (robotDistDiff < -1.0 * minLegDistance_) { // distance too low == person too close to robot -> robot reacts too slow -> increase scales
            if (avgScale < 1.0) { // robot was too slow, but needed less than normal force for maxvel -> decrease force_scale_maxvel_[0] to make it even less force needed
                force_scale_maxvel_[0] -= quadraticPercentualChange * (force_scale_maxvel_[0] - 0.35);
                ROS_INFO("[PARAM AdaptX - CHANGE] Distance %.2f Lower than baseline -> ROBOT TOO SLOW -> avgScale < 1.0 -> adapting force_scale_maxvel_ to %.2f", robotDistDiff, force_scale_maxvel_[0]);
            } else { // robot was too slow, but needed more than normal force for maxvel -> decrease force_scale_minvel_[0] to make robot faster
                force_scale_minvel_[0] -= quadraticPercentualChange * (force_scale_minvel_[0] - 1.0);
                ROS_INFO("[PARAM AdaptX - CHANGE] Distance %.2f Lower than baseline -> ROBOT TOO SLOW -> avgScale > 1.0 -> adapting force_scale_minvel_ to %.2f", robotDistDiff, force_scale_minvel_[0]);
            }
        }//distanceTooLow
        setLEDPhase(controller_led_phases::SHOW_ADAPTED);
    }//adaption needed
    adaptX_lastTestTime_ = current_loop_time_;
        //sendDebugTopicsParamAdaptX(avgScale, robotDistDiff); // ACTIVATE FOR DEBUG ONLY
//     adaptForceScaleBasedOnVelocity();
}

/**
 * \brief Adapts the force scales depending on the velocity
 *
 * This function is for adaptive force inputs. It adapts the maximal needed force depending on the current velocity and the scaling factors defined by the user.
 * The goal is to have more static behavior when accelerating while allowing for easier keeping high velocities.
 *
 */
void FTSAdaptiveForceController::adaptForceScaleBasedOnVelocity() {

    std::array<double, 3> scale;
    std::array<double, 3> adaptive_max_ft;
    std::array<double, 3> vel_factor = getOldVelocityPercent();
    
    switch (used_vel_based_force_adaption_) {
        case vel_based_adaption_none:
            return;
        case vel_based_damping:
            std::array<double, 3> damping;
            for (int i=0; i<3; i++) {
                damping[i] = damping_adaption_params_.max[i] - (damping_adaption_params_.max[i] - 
                             damping_adaption_params_.min[i]) * getOldVelocityPercent()[i];
            }
            discretizeWithNewMassDamping(pre_adaption_base_params_.mass, damping);
            break;
        case vel_based_StoglZumkeller2020:
//             curr_scale = velocityBasedForceScaling_StoglZumkeller2020(force_scale_minvel_, force_scale_maxvel_);
//             break;
            for (int  i = 0; i < 3; i++) {
                vel_factor[i] = fmin( fabs(vel_factor[i]), 1.0); //velocity percent is now definitely between 0.0 and 1.0
                if (use_smooth_transition_) {
                    double steepness = -9.0;
                    double v_end = std::pow(transition_rate_ + 1.0, steepness);
                    vel_factor[i] = ( 1.0 - std::pow(vel_factor[i] * transition_rate_ + 1.0, steepness) ) / (1.0 - v_end);
                }
            }
            break;
        case vel_based_tanh:
            for (int  i = 0; i < 3; i++) {
                vel_factor[i] = std::tanh(vel_factor[i] * tanh_adaption_params_.scale[i] * 3.14159);
            }
            break;
    }
    
    for (int i = 0; i < 3; i++) {
        scale[i] = force_scale_minvel_[i] + ( force_scale_maxvel_[i] - force_scale_minvel_[i] ) * fmin( 1.0, vel_factor[i] );
        adaptive_max_ft[i] = scale[i] * base_max_ft_[i];
    }
    setMaxFt(adaptive_max_ft);
    if (pub_adaptive_scale_->trylock()) {
        pub_adaptive_scale_->msg_ = convertToMessage(scale);
        pub_adaptive_scale_->unlockAndPublish();
    }
    if (pub_adaptive_max_ft_->trylock()) {
        pub_adaptive_max_ft_->msg_ = convertToMessage(adaptive_max_ft);
        pub_adaptive_max_ft_->unlockAndPublish();
    }
    
//     } else {
//         ROS_WARN("[ADAPT_Force:OFF]");
//         resetToBaseValues();
//     }
}



//HELPER FUNCTIONS
/**
 * \brief Sets the base values from which the resulting dynamic values are derived
 */
void FTSAdaptiveForceController::setBaseValues() {
    base_max_ft_ = getMaxFt();
}

/**
 * \brief Resets controller values back to their base ones
 */
// void FTSAdaptiveForceController::resetToBaseValues() {
//         setMaxFt(base_max_ft_);
// }

/**
 * \brief Returns the current scaling factor based on current velocity and the input values for min and max scale factors
 * This function returns for each dimension of the robot the current adaptive scale factor between the input 'minScaleFactor' and 'maxScaleFactor' based on the current percentual velocity of the robot.
 * If the toggle 'use_smooth_transition_' is switched on, the scale is calculated on a different function which puts a higher change-rate towards the lower-end of the velocity (where the robot is normally pushed more often)
 *
 */
std::array<double, 3> FTSAdaptiveForceController::velocityBasedForceScaling_StoglZumkeller2020( std::array<double, 3> minScaleFactor, std::array<double, 3> maxScaleFactor ) {

    std::array<double, 3> vel_factor = getOldVelocityPercent();
    std::array<double, 3> scale;

    for (int  i = 0; i < 3; i++) {
        vel_factor[i] = fmin( fabs(vel_factor[i]), 1.0); //velocity percent is now definitely between 0.0 and 1.0
        if (use_smooth_transition_) {
            double steepness = -9.0;
            double v_end = std::pow(transition_rate_ + 1.0, steepness);
            vel_factor[i] = ( 1.0 - std::pow(vel_factor[i] * transition_rate_ + 1.0, steepness) ) / (1.0 - v_end);
        }
        scale[i] = minScaleFactor[i] + ( maxScaleFactor[i] - minScaleFactor[i] ) * fmin( 1.0, vel_factor[i] );
    }
    return scale;
}

/**
 * \brief This function calculates the distance towards the startingpoint by integrating the robot velocities. It is used for some of the parametrization steps, where information about the travelled distance arount x-direction is needed.
 *
 */
void FTSAdaptiveForceController::updateTravelledDistance() {

    double timeSinceLastUpdate = (current_loop_time_ - lastDistanceUpdate_).toSec();
    if (timeSinceLastUpdate > 0.0) { //update travelledDistance
        double newDistance = travelledDistance_;
        switch (parameterization_current_step_) {
            case baseX:
            case recordFeetDistance:
            case adaptX:
                    newDistance += getOldVelocity()[0] * timeSinceLastUpdate;
                    break;
            case baseYLeft:
                    newDistance += getOldVelocity()[1] * timeSinceLastUpdate;
                    break;
            case baseYRight:
                    newDistance += -1.0 * getOldVelocity()[1] * timeSinceLastUpdate;
                    break;
            case baseRotLeft:
                    newDistance += getOldVelocity()[2] * timeSinceLastUpdate;
                    break;
            case baseRotRight:
                    newDistance += -1.0 * getOldVelocity()[2] * timeSinceLastUpdate;
        }
        travelledDistance_ = (newDistance > 0.0) ? newDistance : 0.0;
        lastDistanceUpdate_ = current_loop_time_;
    }
}

/**
/*\brief This function resets the travelled distance
 */
void FTSAdaptiveForceController::resetTravelledDistance() {
        travelledDistance_ = 0.0;
        lastDistanceUpdate_ = current_loop_time_;
}

/**
 * \brief Resets the FTSAdaptiveForceController by first resetting the base controller, then storing the base values and finally adapting them by the configured dynamics
 *
 */
// void FTSAdaptiveForceController::resetController() {
//     ROS_DEBUG("[FTS_ADAPT:] Reset");
//     resetToBaseValues();
//     FTSBaseController::resetControllerNew();
// }

/**
 * \brief This function resets the parameters for the baseForceTest (when the user abandons the robot mid-test)
 *
 */
void FTSAdaptiveForceController::resetBaseForceTest() {

//     vel_based_force_adaption_active_ = false; //should be off here anyways, this is just for additional safety
    baseForce_startingTime_ = current_loop_time_;
    baseForce_travelledDistanceList_.clear();
    baseForce_stableForceCounter_ = 0;
    baseForce_storeRawInput_.clear();
}

/**
 * \brief This function is to layout which test follows each other. It is called at completion of a step to trigger the start of the next one
 */
void FTSAdaptiveForceController::switchParametrizationStep(){

    step_finished_time_ = current_loop_time_;
    ROS_DEBUG("[PARAM STEP] - Switch to new Phase");

    switch(parameterization_current_step_) {
        case baseX:
            parameterization_current_step_ = baseYLeft;
            ROS_INFO("[PARAM STEP - baseYLeft activated! ]");
            break;
        case baseYLeft:
            parameterization_current_step_ = baseYRight;
            ROS_INFO("[PARAM STEP - baseYRight activated! ]");
            break;
        case baseYRight:
            parameterization_current_step_ = baseRotLeft;
            ROS_INFO("[PARAM STEP - baseRotLeft activated! ]");
            break;
        case baseRotLeft:
            parameterization_current_step_ = baseRotRight;
            ROS_INFO("[PARAM STEP - baseRotRight activated! ]");
            break;
        case baseRotRight:
            parameterization_current_step_ = finished;
            ROS_INFO("[PARAM STEP - Base force parametrization finished! ]");
            break;
        case recordFeetDistance:
            parameterization_current_step_ = adaptX;
            ROS_INFO("[PARAM STEP - Feet Distance Recording finished! ]");
            break;
        case adaptX:
            parameterization_current_step_ = finished;
            ROS_INFO("[PARAM STEP - Velocity-based force adaption finished! ]");
            break;
        case finished:
            parametrization_active_ = false;
            parameterization_current_step_ = baseX;  // reset for the next time (is it needed?)
            ROS_INFO("[PARAM STEP - Parametrization finished!! All degrees of freedom are activated and resetting parametrization! ]");
            setLEDPhase(controller_led_phases::UNLOCKED);
            break;
        default:
            ROS_ERROR("[SWITCH STEP] No Valid Step defined!");
            return;
    }
    resetTravelledDistance();
    parameterization_step_initalized_ = false;
    stepActivated_ = false;
    setSwitchStepRequested(false);
}


//TOPIC PUBLISHING FUNCTIONS
/**
 * \brief Sends the 'input_force' as a wrench message, to which the 'iirob_led' node subscribes and prints on the LED strip
 */
void FTSAdaptiveForceController::sendLEDForceTopics() {

    geometry_msgs::WrenchStamped real_wrench_msg;
    real_wrench_msg.header.stamp = ros::Time::now();
    real_wrench_msg.header.frame_id = hw_fts_.getFrameId();
    real_wrench_msg.wrench.force.x = 0.0;
    real_wrench_msg.wrench.force.y = 0.0;
    real_wrench_msg.wrench.torque.z = 0.0;
    if (parametrization_active_) {
        switch (parameterization_current_step_) { // disable currently unavailable directions from showing
            case baseX:
                real_wrench_msg.wrench.force.x = ledForceInput_;
                break;
            case baseYLeft:
                real_wrench_msg.wrench.force.y = ledForceInput_;
                break;
            case baseYRight:
                real_wrench_msg.wrench.force.y = -1.0 * ledForceInput_;
                break;
            case baseRotLeft: // let the back left LED flash
                real_wrench_msg.wrench.force.x = ledForceInput_;
                real_wrench_msg.wrench.force.y = ledForceInput_;
                break;
            case baseRotRight: // let the back right
                real_wrench_msg.wrench.force.x = ledForceInput_;
                real_wrench_msg.wrench.force.y = -1.0 * ledForceInput_;
                break;
        }
    } else {
        real_wrench_msg = forceInputForLED_;
    }
    forceInputToLed(real_wrench_msg);
}

/**
 * \brief Sets the LED phase (used for LED feedback messages)
 */
bool FTSAdaptiveForceController::setLEDPhase(controller_led_phases requestPhase) {
    
    if (FTSBaseController::setLEDPhase(requestPhase)) {
    
        if (requestPhase != controller_led_phases::SHOW_FORCE) {
            switch (requestPhase) {
//                 case controller_led_phases::UNLOCKED:
    //                 led_ac_->sendGoal(blinkyFreeMovementBlue_);
//                     ledGoalToSend_ = blinky_goals_.blinkyFreeMovementBlue_;
//                     return true;
                case controller_led_phases::WAIT_FOR_INPUT:
                    switch (parameterization_current_step_) {
                        case baseX:
    //                         led_ac_->sendGoal(blinkyStart_x_);
                            ledGoalToSend_ = blinky_goals_.blinkyStart_x_;
                            return true;
                        case baseYLeft:
    //                         led_ac_->sendGoal(blinkyStart_y_left_);,
                            ledGoalToSend_ = blinky_goals_.blinkyStart_y_left_;
                            return true;
                        case baseYRight:
    //                         led_ac_->sendGoal(blinkyStart_y_right_);
                            ledGoalToSend_ = blinky_goals_.blinkyStart_y_right_;
                            return true;
                        case baseRotLeft:
    //                         led_ac_->sendGoal(blinkyStart_rot_left_);
                            ledGoalToSend_ = blinky_goals_.blinkyStart_rot_left_;
                            return true;
                        case baseRotRight:
    //                         led_ac_->sendGoal(blinkyStart_rot_right_);
                            ledGoalToSend_ = blinky_goals_.blinkyStart_rot_right_;
                            return true;
                        case recordFeetDistance:
                            if (!fwd_footDistanceRecorded_) {
    //                             led_ac_->sendGoal(blinkyStart_x_);
                                ledGoalToSend_ = blinky_goals_.blinkyStart_x_;
                            } else {
    //                             led_ac_->sendGoal(blinkyStart_x_back_);
                                ledGoalToSend_ = blinky_goals_.blinkyStart_x_back_;
                            }
                            return true;
                        case adaptX:
    //                         led_ac_->sendGoal(blinkyStartGreen_);
                            ledGoalToSend_ = blinky_goals_.blinkyStartGreen_;
                            return true;
                    }
                case controller_led_phases::WALK_FORWARD:
    //                 led_ac_->sendGoal(blinkyWalkForward_);
                    ledGoalToSend_ = blinky_goals_.blinkyWalkForward_;
                    return true;
                case controller_led_phases::WALK_BACKWARDS:
    //                 led_ac_->sendGoal(blinkyWalkBackwards_);
                    ledGoalToSend_ = blinky_goals_.blinkyWalkBackwards_;
                    return true;
                case controller_led_phases::SHOW_ADAPTED:
                    if (parameterization_current_step_ == adaptX) {
    //                     led_ac_->sendGoal(blinkyPhaseYellow_);
                        ledGoalToSend_ = blinky_goals_.blinkyPhaseYellow_;
                    }
                    break;
                case controller_led_phases::STEP_AWAY_FROM_ROBOT:
    //                 led_ac_->sendGoal(blinkyStepAway_);
                    ledGoalToSend_ = blinky_goals_.blinkyStepAway_;
                    return true;
                case controller_led_phases::ROBOT_IN_AUTOMATIC_MOVEMENT:
                    switch (parameterization_current_step_) {
                        case baseX:
    //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_x_);
                            ledGoalToSend_ = blinky_goals_.blinkyRobotAutomaticMovement_x_;
                            return true;
                        case baseYLeft:
    //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_y_left_);
                            ledGoalToSend_ = blinky_goals_.blinkyRobotAutomaticMovement_y_left_;
                            return true;
                        case baseYRight:
    //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_y_right_);
                            ledGoalToSend_ = blinky_goals_.blinkyRobotAutomaticMovement_y_right_;
                            return true;
                        case baseRotLeft:
    //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_rot_left_);
                            ledGoalToSend_ = blinky_goals_.blinkyRobotAutomaticMovement_rot_left_;
                            return true;
                        case baseRotRight:
    //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_rot_right_);
                            ledGoalToSend_ = blinky_goals_.blinkyRobotAutomaticMovement_rot_right_;
                            return true;
                        case recordFeetDistance:
                        case adaptX:
    //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_);
                            ledGoalToSend_ = blinky_goals_.blinkyRobotAutomaticMovement_;
                            return true;
                        }
                case controller_led_phases::ALMOST_RETURNED:
    //                 led_ac_->sendGoal(blinkyAlmostFinishedRed_);
                    ledGoalToSend_ = blinky_goals_.blinkyAlmostFinishedRed_;
                    return true;
                case controller_led_phases::PHASE_FINISHED:
    //                 led_ac_->sendGoal(blinkyFinishedRed_);
                    ledGoalToSend_ = blinky_goals_.blinkyFinishedRed_;
                    return true;
            }
        }
    }
}

/**
 * \brief sends the LED output which is defined by the currently active led_phase_ enum
 */
// void FTSAdaptiveForceController::sendLEDOutput() {
// 
//     if (currentLEDPhase_ == controller_led_phases::SHOW_FORCE) {
//         sendLEDForceTopics();
//     } else if (sendLEDGoal_) {
//         sendLEDGoal_ = false; //to only send it once the phase has changed
//         led_ac_->sendGoal(ledGoalToSend_);
// //         switch (currentLEDPhase_) {
// //             case controller_led_phases::UNLOCKED:
// //                 led_ac_->sendGoal(blinkyFreeMovementBlue_);
// //                 return;
// //             case controller_led_phases::WAIT_FOR_INPUT:
// //                 switch (parameterization_current_step_) {
// //                     case baseX:
// //                         led_ac_->sendGoal(blinkyStart_x_);
// //                         return;
// //                     case baseYLeft:
// //                             led_ac_->sendGoal(blinkyStart_y_left_);
// //                             return;
// //                     case baseYRight:
// //                             led_ac_->sendGoal(blinkyStart_y_right_);
// //                             return;
// //                     case baseRotLeft:
// //                             led_ac_->sendGoal(blinkyStart_rot_left_);
// //                             return;
// //                     case baseRotRight:
// //                             led_ac_->sendGoal(blinkyStart_rot_right_);
// //                             return;
// //                     case recordFeetDistance:
// //                         if (!fwd_footDistanceRecorded_) {
// //                                 led_ac_->sendGoal(blinkyStart_x_);
// //                         } else {
// //                                 led_ac_->sendGoal(blinkyStart_x_back_);
// //                         }
// //                         return;
// //                     case adaptX:
// //                         led_ac_->sendGoal(blinkyStartGreen_);
// //                         return;
// //                 }
// //             case controller_led_phases::WALK_FORWARD:
// //                     led_ac_->sendGoal(blinkyWalkForward_);
// //                     return;
// //             case controller_led_phases::WALK_BACKWARDS:
// //                     led_ac_->sendGoal(blinkyWalkBackwards_);
// //                     return;
// //             case controller_led_phases::SHOW_ADAPTED:
// //                 if (parameterization_current_step_ == adaptX) {
// //                     led_ac_->sendGoal(blinkyPhaseYellow_);
// //                 }
// //                 break;
// //             case controller_led_phases::STEP_AWAY_FROM_ROBOT:
// //                     led_ac_->sendGoal(blinkyStepAway_);
// //                     return;
// //             case controller_led_phases::ROBOT_IN_AUTOMATIC_MOVEMENT:
// //                 switch (parameterization_current_step_) {
// //                     case baseX:
// //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_x_);
// //                         return;
// //                     case baseYLeft:
// //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_y_left_);
// //                         return;
// //                     case baseYRight:
// //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_y_right_);
// //                         return;
// //                     case baseRotLeft:
// //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_rot_left_);
// //                         return;
// //                     case baseRotRight:
// //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_rot_right_);
// //                         return;
// //                     case recordFeetDistance:
// //                     case adaptX:
// //                         led_ac_->sendGoal(blinkyRobotAutomaticMovement_);
// //                         return;
// //                     }
// //             case controller_led_phases::ALMOST_RETURNED:
// //                     led_ac_->sendGoal(blinkyAlmostFinishedRed_);
// //                     return;
// //             case controller_led_phases::PHASE_FINISHED:
// //                     led_ac_->sendGoal(blinkyFinishedRed_);
// //                     return;
// //         }
//     }
// }

/**
 * \brief debug function which pushes debug messages regarding the process of 'parametrizeBaseX', e.g. current distance to startpoint or springforce and resulting force
 */
void FTSAdaptiveForceController::sendDebugTopicsParamBase( double travelledDist, double averageDist, double raw_input, double currentVirtSpringForce, double effectiveForce ) {

        //debug topic messages
        geometry_msgs::Twist averageDist_msg, virtSpring_msg, currentRawForce_msg, resForce_msg, currentDist_msg;

        switch(parameterization_current_step_) {
            case baseX:
                averageDist_msg.linear.x = averageDist;
                virtSpring_msg.linear.x = currentVirtSpringForce;
                currentRawForce_msg.linear.x = raw_input;
                resForce_msg.linear.x = effectiveForce;
                currentDist_msg.linear.x = travelledDist;
                break;
            case baseYLeft:
                averageDist_msg.linear.y = averageDist;
                virtSpring_msg.linear.y = currentVirtSpringForce;
                currentRawForce_msg.linear.y = raw_input;
                resForce_msg.linear.y = effectiveForce;
                currentDist_msg.linear.y = travelledDist;
                break;
            case baseYRight:
                averageDist_msg.linear.z = averageDist;
                virtSpring_msg.linear.z = currentVirtSpringForce;
                currentRawForce_msg.linear.z = raw_input;
                resForce_msg.linear.z = effectiveForce;
                currentDist_msg.linear.z = travelledDist;
                break;
            case baseRotLeft:
                averageDist_msg.angular.x = averageDist;
                virtSpring_msg.angular.x = currentVirtSpringForce;
                currentRawForce_msg.angular.x = raw_input;
                resForce_msg.angular.x = effectiveForce;
                currentDist_msg.angular.x = travelledDist;
                break;
            case baseRotRight:
                averageDist_msg.angular.y = averageDist;
                virtSpring_msg.angular.y = currentVirtSpringForce;
                currentRawForce_msg.angular.y = raw_input;
                resForce_msg.angular.y = effectiveForce;
                currentDist_msg.angular.y = travelledDist;
                break;
        }
        if (pub_base_averageDist_->trylock()) {
            pub_base_averageDist_->msg_ = averageDist_msg;
            pub_base_averageDist_->unlockAndPublish();
        }
        if (pub_base_virtSpring_->trylock()) {
            pub_base_virtSpring_->msg_ = virtSpring_msg;
            pub_base_virtSpring_->unlockAndPublish();
        }
        if (pub_base_currentRawForce_->trylock()) {
            pub_base_currentRawForce_->msg_ = currentRawForce_msg;
            pub_base_currentRawForce_->unlockAndPublish();
        }
        if (pub_base_resForce_->trylock()) {
            pub_base_resForce_->msg_ = resForce_msg;
            pub_base_resForce_->unlockAndPublish();
        }
        if (pub_base_currentDist_->trylock()) {
            pub_base_currentDist_->msg_ = currentDist_msg;
            pub_base_currentDist_->unlockAndPublish();
        }
}

/**
 * \brief Sends debug topics for paramerization
 */
void FTSAdaptiveForceController::sendDebugTopicsParamAdaptX( double currentScale, double robotDistDiff ) {

    if (pub_adaptive_scale_x_->trylock()){
        pub_adaptive_scale_x_->msg_.data = currentScale;
        pub_adaptive_scale_x_->unlockAndPublish();
    }
    if (pub_adaptive_factor_min_->trylock()){
        pub_adaptive_factor_min_->msg_.data = force_scale_minvel_[0];
        pub_adaptive_factor_min_->unlockAndPublish();
    }
    if (pub_adaptive_factor_max_->trylock()){
        pub_adaptive_factor_max_->msg_.data = force_scale_maxvel_[0];
        pub_adaptive_factor_max_->unlockAndPublish();
    }
    if (pub_adaptive_distDiff_->trylock()){
        pub_adaptive_distDiff_->msg_.data = robotDistDiff;
        pub_adaptive_distDiff_->unlockAndPublish();
    }
}


//CALLBACK FUNCTIONS
/**
 * \brief Dynamic reconfigure Callback function for the controller
 */
void FTSAdaptiveForceController::reconfigureCallback(robotrainer_controllers::FTSAdaptiveForceControllerConfig &config, uint32_t level) {
    
    std::string lock = "adaptive_reconfigure_callback";
    protectedToggleControllerRunning(false, lock);

    if (use_passive_behavior_ctrlr_ != config.use_passive_behavior_ctrlr) {
        use_passive_behavior_ctrlr_ = config.use_passive_behavior_ctrlr;
        ROS_INFO_COND(use_passive_behavior_ctrlr_, "[ADAPT] - Enabling passive behavior control!");
        ROS_INFO_COND(!use_passive_behavior_ctrlr_, "[ADAPT] - Disabling passive behavior control!");
    }

    //switch between standard ft/velocity values and user-parametrized values
    used_ft_type_ = static_cast<base_force_type>(config.max_ft_values_used);
    
    switch (used_ft_type_) {
        case standard_ft:
            useUserParametrizedValues(false);
            break;
        case user_ft:
            if (baseForce_allParamsStored_) {
                useUserParametrizedValues(true);
            } else {
                ROS_WARN("Please use ParametrizeBase before attempting to use the user-defined values!");
                config.max_ft_values_used = 0;
            }
            break;
    }
    
    if (used_vel_based_force_adaption_ != config.velocity_based_force_adaption_used)
    {
        if (used_vel_based_force_adaption_ != vel_based_adaption_none) {
            pre_adaption_base_params_.gain = getGain();
            pre_adaption_base_params_.time_const = getTimeConst();
            pre_adaption_base_params_.damping = getDamping();
            pre_adaption_base_params_.mass = getMass();
        }
        if (used_vel_based_force_adaption_ == vel_based_damping) {
            discretizeWithNewParameters(pre_adaption_base_params_.time_const,
                                        pre_adaption_base_params_.gain);
        }
        // velocity based force adaption configuraion
        used_vel_based_force_adaption_ = static_cast<velocity_based_adaption_type>(
            config.velocity_based_force_adaption_used);
    }
    
    if (config.apply_vel_adaption_params) {

        ROS_INFO("[ADAPT]: Changing parameters according to dynamic reconfigure inputs");
//         stopController();

        // vel_based_StoglZumkeller2020
        force_scale_minvel_[0] = config.force_scale_minvel_x;
        force_scale_minvel_[1] = config.force_scale_minvel_y;
        force_scale_minvel_[2] = config.force_scale_minvel_rot;
        force_scale_maxvel_[0] = config.force_scale_maxvel_x;
        force_scale_maxvel_[1] = config.force_scale_maxvel_y;
        force_scale_maxvel_[2] = config.force_scale_maxvel_rot;
        use_smooth_transition_ = config.use_smooth_transition;
        transition_rate_ = config.transition_rate;
//         resetToBaseValues();
//         resetController();
        
        // tanh
        tanh_adaption_params_.scale[0] = config.tanh_scale_x;
        tanh_adaption_params_.scale[1] = config.tanh_scale_y;
        tanh_adaption_params_.scale[2] = config.tanh_scale_z;
        
        // damping
        damping_adaption_params_.min[0] = config.damping_min_x;
        damping_adaption_params_.min[1] = config.damping_min_y;
        damping_adaption_params_.min[2] = config.damping_min_z;
        damping_adaption_params_.max[0] = config.damping_max_x;
        damping_adaption_params_.max[1] = config.damping_max_y;
        damping_adaption_params_.max[2] = config.damping_max_z;
        
        config.apply_vel_adaption_params = false;
    }

    //parametrization reconfigure part
    if (!parametrization_active_) { //enable change of parametrization parameters
        ROS_INFO("[ADAPT - Parametrization]: Updating parametrization parameters.");
        baseForce_movingAverageTimeframe_ = config.movingAverageTimeframe;
        baseForce_holdingDistance_ = config.holdingDistance;
        minLegDistance_ = config.minLegDistance;
        maxLegDistance_ = config.maxLegDistance;
        adaptive_parametrization_adaptionRate_ = config.adaptionRate;
    } else {
        ROS_DEBUG("[ADAPT - Parametrization]: Unable to update parametrization parameters because it is currently active!");
    }

    if (!parametrization_active_ && config.activate_force_parametrization) {
        ROS_INFO("[ADAPT - Parametrization] - Activating User Force Parametrization.");
        parametrization_active_ = true;
        config.parameterization_activated = true;
        parameterization_current_step_ = baseX;
        setSwitchStepRequested(false);
        parameterization_step_initalized_ = false;
    } else if (!parametrization_active_ && config.activate_adaptive_scale_parametrization) {
        ROS_INFO("[ADAPT - Parametrization]  - Activating Adaptive Scale parametrization.");
        parametrization_active_ = true;
        config.parameterization_activated = true;
        parameterization_current_step_ = recordFeetDistance;
        parameterization_step_initalized_ = false;
        setSwitchStepRequested(false);
    } else if (parametrization_active_ && !config.parameterization_activated) {
        parameterization_current_step_ = finished;
        parameterization_step_initalized_ = false;
        setSwitchStepRequested(false);
    }
    config.activate_force_parametrization = false;
    config.activate_adaptive_scale_parametrization = false;
    
    protectedToggleControllerRunning(true, lock);
}

/**
 * \brief Switch between userParametrized and standard values (initially used for quick switching during the evaluation, but leaving this function permanently active as it could be useful in general)
 */
void FTSAdaptiveForceController::useUserParametrizedValues(bool enable) {
    if (enable && baseForce_allParamsStored_) {
        ROS_INFO("Using userParametrized Force values: [%.2f, %.2f, %.2f], and Velocity values: [%.2f, %.2f, %.2f]", 
            userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2],
            userParametrized_maxVel_[0], userParametrized_maxVel_[1], userParametrized_maxVel_[2]          );
        setMaxFt(userParametrized_maxFT_);
        setMaxVel(userParametrized_maxVel_);
    } else {
        ROS_INFO("Using standard Force values: [%.2f, %.2f, %.2f], and Velocity values: [%.2f, %.2f, %.2f]",
            standard_maxFT_[0], standard_maxFT_[1], standard_maxFT_[2],
            standard_maxVel_[0], standard_maxVel_[1], standard_maxVel_[2]
        );
        setMaxFt(standard_maxFT_);
        setMaxVel(standard_maxVel_);
    }
}

/**
 * \brief Callback function for legTracking message type
 */
void FTSAdaptiveForceController::legTrackCallback(const leg_tracker::PersonMsg::ConstPtr& msg) {


    double fst_x = msg->leg1.position.x;
    double snd_x = msg->leg2.position.x;
    double leg_dist_fst_x = (fst_x > -0.3 || fst_x < -1.7) ? 0.0 : -1.0 * fst_x;
    double leg_dist_snd_x = (snd_x > -0.3 || snd_x < -1.7) ? 0.0 : -1.0 * snd_x;
    leg_dist_avg_x_ = 0.5 * leg_dist_fst_x + 0.5 * leg_dist_snd_x;
    setLegTrackUpdated(true);
}

void FTSAdaptiveForceController::setLegTrackUpdated(const bool value)
{
    boost::mutex::scoped_lock lock(leg_track_update_mutex_);
    legtrack_updated_ = value;
}

bool FTSAdaptiveForceController::getLegTrackUpdated()
{
    boost::mutex::scoped_try_lock lock(leg_track_update_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    return legtrack_updated_;
}

void FTSAdaptiveForceController::setSwitchStepRequested(const bool value)
{
    boost::unique_lock<boost::shared_mutex> lock(switch_step_update_mutex_);
    switchStepRequested_ = value;
}

bool FTSAdaptiveForceController::getSwitchStepRequested()
{
    boost::shared_lock<boost::shared_mutex> lock(switch_step_update_mutex_);
    while (!lock) {
        lock.try_lock();
    }
    return switchStepRequested_;
}

}  // namespace robotrainer_controllers

PLUGINLIB_EXPORT_CLASS(
    robotrainer_controllers::FTSAdaptiveForceController, controller_interface::ControllerBase)
