#include <robotrainer_controllers/fts_adaptive_force_controller.h>
#include <pluginlib/class_list_macros.h>

namespace robotrainer_controllers {
FTSAdaptiveForceController::FTSAdaptiveForceController(){}

bool FTSAdaptiveForceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {   
        
        if (!FTSBaseController::init(robot_hw, root_nh, controller_nh)) { // init base controller if yet not initialized
                ROS_FATAL("Initializing FTSBaseController went wrong... Can not continue...");
        }
        ROS_INFO("Init FTSAdaptiveForceController!");
        setBaseValues(); 
        
        /* get Parameters from rosparam server (stored on yaml file) */
        ros::NodeHandle fts_a_ctrl_nh(controller_nh, "FTSAdaptiveForceController");
        fts_a_ctrl_nh.param<bool>("adaptive_force/use_passive_behavior_ctrlr", use_passive_behavior_ctrlr_, false);
        fts_a_ctrl_nh.param<bool>("adaptive_force/use_adaptive_force", adaption_is_active_, false);
        // Adaptive force parameters
                fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_zero_vel/x", force_scale_minvel_[0], 1.0);
                fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_zero_vel/y", force_scale_minvel_[1], 1.0);
                fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_zero_vel/rot", force_scale_minvel_[2], 1.0);
                fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_max_vel/x", force_scale_maxvel_[0], 1.0);
                fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_max_vel/y", force_scale_maxvel_[1], 1.0);
                fts_a_ctrl_nh.param<double>("adaptive_force/force_scale_max_vel/rot", force_scale_maxvel_[2], 1.0);
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
        
        /* for parametrization */
                parametrization_active_ = false;
                stepInitialized_ = false;
                userParametrized_maxFT_ = getMaxFt();
                userParametrized_maxVel_ = getMaxVel();
                standard_maxFT_ = getMaxFt();
                standard_maxVel_ = getMaxVel();
                currentStep_ = baseX;
                sub_legtrack_ = root_nh.subscribe("/leg_detection/people_msg_stamped", 1, &FTSAdaptiveForceController::legTrackCallback, this );
                baseForce_allParamsStored_ = false;
                baseForce_fts_offset_reset_ = false;
        /* Parametrization debug topics */
                // base x
                pub_base_x_currentDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/x/currentDistanceFromStart", 1);
                pub_base_x_averageDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/x/averageDist", 1);
                pub_base_x_currentRawForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/x/currentRawForce", 1);
                pub_base_x_virtSpring_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/x/virtualSpringForce", 1);
                pub_base_x_resForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/x/resultingForce", 1);
                // base y left
                pub_base_y_left_currentDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/left/currentDistanceFromStart", 1);
                pub_base_y_left_averageDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/left/averageDist", 1);
                pub_base_y_left_currentRawForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/left/currentRawForce", 1);
                pub_base_y_left_virtSpring_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/left/virtualSpringForce", 1);
                pub_base_y_left_resForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/left/resultingForce", 1);
                // base y right
                pub_base_y_right_currentDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/right/currentDistanceFromStart", 1);
                pub_base_y_right_averageDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/right/averageDist", 1);
                pub_base_y_right_currentRawForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/right/currentRawForce", 1);
                pub_base_y_right_virtSpring_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/right/virtualSpringForce", 1);
                pub_base_y_right_resForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/y/right/resultingForce", 1);
                // base rot left
                pub_base_rot_left_currentDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/left/currentDistanceFromStart", 1);
                pub_base_rot_left_averageDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/left/averageDist", 1);
                pub_base_rot_left_currentRawForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/left/currentRawForce", 1);
                pub_base_rot_left_virtSpring_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/left/virtualSpringForce", 1);
                pub_base_rot_left_resForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/left/resultingForce", 1);
                // base rot right
                pub_base_rot_right_currentDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/right/currentDistanceFromStart", 1);
                pub_base_rot_right_averageDist_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/right/averageDist", 1);
                pub_base_rot_right_currentRawForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/right/currentRawForce", 1);
                pub_base_rot_right_virtSpring_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/right/virtualSpringForce", 1);
                pub_base_rot_right_resForce_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/base/rot/right/resultingForce", 1);
                // leg tracking
                travelledDistance_ = 0.0;
                // adaption factors
                pub_adaptive_scale_x_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/adaptX/currentScale", 1);
                pub_adaptive_factor_min_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/adaptX/minVelFactor", 1);
                pub_adaptive_factor_max_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/adaptX/maxVelFactor", 1);
                pub_adaptive_distDiff_ = root_nh.advertise<std_msgs::Float64>("robotrainer_controllers/parametrization/adaptX/distanceDiffToLast", 1);
                //force-torque
                pub_force_raw_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/ft/raw", 1);
                pub_force_raw_lim_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/ft/raw_limited", 1);
                pub_force_adapt_lim_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/ft/adapted_limited", 1);
                pub_force_adapt_unscaled_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/ft/adapted_unscaled", 1);
                pub_force_adapt_base_scaled_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/ft/adapted_base_scaled", 1);
                
                pub_adaptive_scale_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/adaption/scale", 1);
                pub_adaptive_max_ft_ = root_nh.advertise<geometry_msgs::Vector3>("robotrainer_controllers/adaption/maxFT", 1);
        
        /* Blinky LED initialization */
                led_ac_ = new actionlib::SimpleActionClient<iirob_led::BlinkyAction>("/rosy_test/leds_rectangle/blinky", true);
                if (led_ac_->waitForServer(ros::Duration(2))) {
                    pub_input_force_for_led_ = root_nh.advertise<geometry_msgs::WrenchStamped>("/rosy_test/leds_rectangle/led_force", 1);
                    ROS_INFO("[ADAPT - INIT] LED actionclient registered");
                }
                else {
                    ROS_WARN("Action server for LED-Rectangle not started and it will not be used!");
                }
                
        { /* Initialize Blinky messages */
                blinkyStartGreen_.color.r = 0.0;
                blinkyStartGreen_.color.g = 0.5;
                blinkyStartGreen_.color.b = 0.0;
                blinkyStartGreen_.color.a = 1.0;
                blinkyStartGreen_.blinks = 1;
                blinkyStartGreen_.duration_on = 0.5;
                blinkyStartGreen_.duration_off = 0.0;
                blinkyStartGreen_.start_led = 1;
                blinkyStartGreen_.end_led = 384;
                blinkyStartGreen_.num_leds = 0;
                blinkyStartGreen_.fade_in = false;
                blinkyStartGreen_.fade_out = false;
                
                blinkyStart_x_ = blinkyStartGreen_;
                blinkyStart_x_.blinks = 100;
                blinkyStart_x_.duration_on = 0.25;
                blinkyStart_x_.duration_off = 0.05;
                blinkyStart_x_.start_led = 300;
                blinkyStart_x_.end_led = 384;
                
                blinkyStart_x_back_= blinkyStart_x_;
                blinkyStart_x_back_.start_led = 108;
                blinkyStart_x_back_.end_led = 192;
                
                blinkyStart_y_left_ = blinkyStart_x_;
                blinkyStart_y_left_.start_led = 0;
                blinkyStart_y_left_.end_led = 108;
                
                blinkyStart_y_right_ = blinkyStart_x_;
                blinkyStart_y_right_.start_led = 192;
                blinkyStart_y_right_.end_led = 300;
                
                blinkyStart_rot_left_ = blinkyStart_x_;
                blinkyStart_rot_left_.start_led = 361;
                blinkyStart_rot_left_.end_led = 27;
                
                blinkyStart_rot_right_ = blinkyStart_x_;
                blinkyStart_rot_right_.start_led = 279;
                blinkyStart_rot_right_.end_led = 327;
                
                blinkyWalkForward_ = blinkyStartGreen_;
                blinkyWalkForward_.blinks = 50;
                blinkyWalkForward_.duration_on = 0.6;
                blinkyWalkForward_.duration_off = 0.15;
                blinkyWalkForward_.start_led = 300;
                blinkyWalkForward_.end_led = 384;
                
                blinkyWalkBackwards_ = blinkyWalkForward_;
                blinkyWalkBackwards_.start_led = 108;
                blinkyWalkBackwards_.end_led = 192;
                
                blinkyPhaseYellow_.color.r = 0.5;
                blinkyPhaseYellow_.color.g = 0.5;
                blinkyPhaseYellow_.color.b = 0.0;
                blinkyPhaseYellow_.color.a = 1.0;
                blinkyPhaseYellow_.blinks = 2;
                blinkyPhaseYellow_.duration_on = 0.20;
                blinkyPhaseYellow_.duration_off = 0.05;
                blinkyPhaseYellow_.start_led = 1;
                blinkyPhaseYellow_.end_led = 384;
                blinkyPhaseYellow_.num_leds = 0;
                blinkyPhaseYellow_.fade_in = false;
                blinkyPhaseYellow_.fade_out = false;
                
                blinkyFinishedRed_.color.r =0.6;
                blinkyFinishedRed_.color.g = 0.0;
                blinkyFinishedRed_.color.b = 0.0;
                blinkyFinishedRed_.color.a = 1.0;
                blinkyFinishedRed_.blinks = 1;
                blinkyFinishedRed_.duration_on = 0.5;
                blinkyFinishedRed_.duration_off = 0.0;
                blinkyFinishedRed_.start_led = 1;
                blinkyFinishedRed_.end_led = 384;
                blinkyFinishedRed_.num_leds = 0;
                blinkyFinishedRed_.fade_in = false;
                blinkyFinishedRed_.fade_out = false;
                
                blinkyAlmostFinishedRed_ = blinkyFinishedRed_;
                blinkyAlmostFinishedRed_.blinks = 10;
                blinkyAlmostFinishedRed_.duration_on = 0.15;
                blinkyAlmostFinishedRed_.duration_off = 0.05;
                
                blinkyStepAway_ = blinkyFinishedRed_;
                blinkyStepAway_.color.r = 0.9;
                blinkyStepAway_.blinks = 100;
                blinkyStepAway_.duration_on = 0.1;
                blinkyStepAway_.duration_off = 0.05;
                
                blinkyRobotAutomaticMovement_ = blinkyAlmostFinishedRed_;
                blinkyRobotAutomaticMovement_.color.r =0.8;
                blinkyRobotAutomaticMovement_.blinks = 200;
                
                blinkyRobotAutomaticMovement_x_ = blinkyRobotAutomaticMovement_;
                blinkyRobotAutomaticMovement_x_.start_led = 108;
                blinkyRobotAutomaticMovement_x_.end_led = 192;
                
                blinkyRobotAutomaticMovement_y_left_ = blinkyRobotAutomaticMovement_;
                blinkyRobotAutomaticMovement_y_left_.start_led = 192;
                blinkyRobotAutomaticMovement_y_left_.end_led = 300;
                
                blinkyRobotAutomaticMovement_y_right_ = blinkyRobotAutomaticMovement_;
                blinkyRobotAutomaticMovement_y_right_.start_led = 0;
                blinkyRobotAutomaticMovement_y_right_.end_led = 108;
                
                blinkyRobotAutomaticMovement_rot_left_ = blinkyRobotAutomaticMovement_;
                blinkyRobotAutomaticMovement_rot_left_.start_led = 279;
                blinkyRobotAutomaticMovement_rot_left_.end_led = 327;
                
                blinkyRobotAutomaticMovement_rot_right_ = blinkyRobotAutomaticMovement_;
                blinkyRobotAutomaticMovement_rot_right_.start_led = 361;
                blinkyRobotAutomaticMovement_rot_right_.end_led = 27;
                
                blinkyFreeMovementBlue_.color.r =0.11;
                blinkyFreeMovementBlue_.color.g = 0.56;
                blinkyFreeMovementBlue_.color.b = 1.0;
                blinkyFreeMovementBlue_.color.a = 0.4;
                blinkyFreeMovementBlue_.blinks = 1;
                blinkyFreeMovementBlue_.duration_on = 0.7;
                blinkyFreeMovementBlue_.duration_off = 0.0;
                blinkyFreeMovementBlue_.start_led = 1;
                blinkyFreeMovementBlue_.end_led = 384;
                blinkyFreeMovementBlue_.num_leds = 0;
                blinkyFreeMovementBlue_.fade_in = false;
                blinkyFreeMovementBlue_.fade_out = false;
        }
        setLEDPhase(unlocked); //Letting the robot blink blue to know initialization finished
        
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
        ROS_WARN_COND(!running_, "Base Controller is not running at the moment!");
        std::array<double, 3> vel_percent = getOldVelocityPercent();
        std::array<double, 3> fts_input_raw = FTSBaseController::getFTSInput(current_loop_time_);
        pub_force_raw_.publish(convertToMessage(fts_input_raw));
//         pub_force_raw_lim_.publish(convertToMessage(FTSBaseController::getScaledLimitedFTSInput(fts_input_raw)));
        
        
        if (parametrization_active_) {
                ROS_WARN_THROTTLE(10.0, "[PARAMETRIZATION STARTED]");
                
                if (switchStepRequested_) {
                        if (!userIsGripping()) {
                                switchParametrizationStep();
                        } else {
                                setLEDPhase(stepAwayFromRobot);
                                fts_input_raw = zeroForce_;
                        }
                } else if (!stepInitialized_) {
                        initParametrizationStep();
                }
                
                //step should be initialized, wait for user input to activate the step
                if ( stepInitialized_ && !stepActivated_) {
                        if ( (current_loop_time_ - step_finished_time_).toSec() < 1.0 ) { //wait until unlocking new phase
                                setLEDPhase(phaseFinished);
                        } else {
                                resetTravelledDistance();
                                switch (currentStep_) {
                                        case baseX: case baseYLeft: case baseYRight: case baseRotLeft: case baseRotRight:
                                                resetBaseForceTest();
                                                setLEDPhase(waitForInput);
                                                stepActivated_ = true;
                                                break;
                                        case recordFeetDistance: case adaptX:
                                                if (userIsGripping() ) {
                                                        setLEDPhase(walkForward);
                                                        stepActivated_ = true;
                                                } else {
                                                        setLEDPhase(waitForInput);
                                                }
                                                break;   
                                        case finished:
                                                stepActivated_ = true;
                                                setLEDPhase(unlocked);
                                                break;
                                }
                        }
                //step is initialized and activated, so perform a loop of the corresponding function
                } else if (stepInitialized_ && stepActivated_) {
                        updateTravelledDistance();
                        switch (currentStep_) {
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
                                        fts_input_raw = recordBaseFeetDistance( fts_input_raw );
                                        break;
                                case adaptX:
                                        ROS_WARN_ONCE("[PARAM ADAPTIVE X] - STARTED");
                                        parametrizeAdaptForceX();
                                        break;
                                case finished:
                                        ROS_WARN("[PARAM FINISHED!] - Parametrization has finished, setting robot to active!");
                                        switchParametrizationStep();
                                        parametrization_active_ = false;
                                        setActiveDimensions( all_active_ );
                                        //adaption_is_active_ = true;
                                        break;
                                default:
                                        ROS_WARN("Invalid parametrization step set, thus finishing parametrization now");
                                        currentStep_ = finished;
                        }
                } else {
                        fts_input_raw = zeroForce_; //prevent robot from moving when nobody is gripping it
                }
        } else { // robot in normal use
                setActiveDimensions( all_active_);
                                
                if (adaption_is_active_) { adaptForceScale(); }
        }
        
        sendLEDOutput(); // LED output to robot (if LEDPhase has changed or is set to showForce)
        pub_force_adapt_unscaled_.publish(convertToMessage(fts_input_raw));
        std::array<double,3> scaledLimitedFTSInput = FTSBaseController::getScaledLimitedFTSInput(fts_input_raw);
        pub_force_adapt_lim_.publish(convertToMessage(scaledLimitedFTSInput));
        
//         std::array<double,3> baseScaledInput;
//         for (int i = 0; i < 3; i ++) {
//                 baseScaledInput[i] = scaledLimitedFTSInput[i] * base_max_ft_[i];
//         }
//         pub_force_adapt_base_scaled_.publish(convertToMessage(baseScaledInput));
        
        
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
       
}

/**
 * \brief This function initializes the parameters for the respective parametrization step, which is only executed after initialization is marked true
 */
void FTSAdaptiveForceController::initParametrizationStep() {
                
        //initialize variables
        switch(currentStep_) {
                case baseX:
                        resetBaseForceTest();
                        resetTravelledDistance();
                        baseForce_springConstant_ = springConstant_x_;
                        baseForce_minimumForce_ = baseForce_minimumForce_x_;
                        baseForce_testCompleted_ = false;
                        ROS_DEBUG_THROTTLE(1, "[BASE X] - INIT - springConstant: %d, minForceNeeded: %.2f", 
                                                baseForce_springConstant_, baseForce_minimumForce_ );
                        break;
                case baseYLeft:
                        resetBaseForceTest();
                        resetTravelledDistance();
                        baseForce_springConstant_ = springConstant_y_;
                        baseForce_minimumForce_ = baseForce_minimumForce_y_;
                        baseForce_testCompleted_ = false;
                        ROS_DEBUG_THROTTLE(1, "[BASE Y LEFT] - INIT - springConstant: %d, minForceNeeded: %.2f", 
                                                baseForce_springConstant_, baseForce_minimumForce_ );
                        break;
                case baseYRight:
                        resetBaseForceTest();
                        resetTravelledDistance();
                        baseForce_springConstant_ = springConstant_y_;
                        baseForce_minimumForce_ = baseForce_minimumForce_y_;
                        baseForce_testCompleted_ = false;
                        ROS_DEBUG_THROTTLE(1, "[BASE Y RIGHT] - INIT - springConstant: %d, minForceNeeded: %.2f", 
                                                baseForce_springConstant_, baseForce_minimumForce_ );
                        break;
                case baseRotLeft:
                        resetBaseForceTest();
                        resetTravelledDistance();
                        baseForce_springConstant_ = springConstant_rot_;
                        baseForce_minimumForce_ = baseForce_minimumTorque_;
                        baseForce_testCompleted_ = false;
                        ROS_DEBUG_THROTTLE(1, "[BASE ROT LEFT] - INIT - springConstant: %d, minForceNeeded: %.2f", 
                                                baseForce_springConstant_, baseForce_minimumForce_ );
                        break;        
                case baseRotRight:
                        resetBaseForceTest();
                        resetTravelledDistance();
                        baseForce_springConstant_ = springConstant_rot_;
                        baseForce_minimumForce_ = baseForce_minimumTorque_;
                        baseForce_testCompleted_ = false;
                        ROS_DEBUG_THROTTLE(1, "[BASE ROT RIGHT] - INIT - springConstant: %d, minForceNeeded: %.2f", 
                                                baseForce_springConstant_, baseForce_minimumForce_ );
                        break;
                case recordFeetDistance:
                        footDistanceSum_ = 0.0;
                        storedFootDistances_ = 0;
                        fwd_footDistanceRecorded_ = false;
                        bwd_footDistanceRecorded_ = false;
                        resetTravelledDistance();
                        debug_legTrackingStarted_ = current_loop_time_;
                        break;
                case adaptX:
                        adaptX_startTime_ = current_loop_time_;
                        adaptX_movingLegDistList_.clear();
                        adaptX_movingLegDistList_.push_back(leg_dist_avg_x_); //reset stored distances
                        adaptX_movementSpeedList_.clear();
                        adaptX_movementSpeedList_.push_back(getOldVelocityPercent()[0]);
                        adaptX_unchangedCtr_ = 0;
                        adaptX_testIntervallDuration_ = 1.0;
                        adaption_is_active_ = true;
                        use_smooth_transition_ = true;
                        legtrack_updated_ = false;
                        force_scale_minvel_[0] = 1.5;
                        force_scale_minvel_[1] = 1.5;
                        force_scale_minvel_[2] = 1.0;
                        force_scale_maxvel_[0] = 0.35;
                        force_scale_maxvel_[1] = 0.35;
                        force_scale_maxvel_[2] = 0.35;
                        // sendDebugTopicsParamAdaptX(1.0, 0.0);  //ACTIVATE FOR DEBUG ONLY
                        ROS_DEBUG_THROTTLE(1, "[ADAPT X] - INIT - X-Adaptive initialized !");
                        break;
                case finished:
                        //adaption_is_active_ = true;
                        //use_smooth_transition_ = true;
                        parametrization_active_ = false;
                        currentStep_ = baseX; //reset for next time
                        ROS_INFO("[ADAPTION HAS FINISHED!] ALL DIMENSIONS ACTIVE NOW!");
                        
                        break;
                default:
                        ROS_WARN("TRIED TO INITIALIZE AN INVALID CASE...");
        }
        
        //enable right dimension only and set starting led
        switch(currentStep_) {
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
        stepInitialized_ = true;
        return;        
}

/**
 * \brief This step is the first parametrization step. It is used to parametrize the base force in the current direction which the user can handle safely. This is done by adding a virtual spring with predefined spring constant in front of the robot, against which the user should push and hold the platform at an acceptable counterforce given by himself. For this reason, only the to-be-tested direction of the robot is set active, and upon gripping the robot (e.g. giving its FTS-sensor inputs) the current position is stored as 'home'. For each step the travelled distance from this home-point is calculated by integrating the current velocity, and the respective virtual spring force is calculated. It is then added against the user input in the same direction to steer the robot, to give the user the impression of a counterforce becoming lineary stronger the more you push the platform forward. At the same time, a rolling average of the travelled distance is calculated and compared against the current distance. If the user keeps holding the robot around a margin of the same distance over the course of the rolling average, and the input force exceeds a minimum force requirement (to ensure a reasonable resulting force), the average force during the rolling average is stored as the new baseline force, from which the velocity will be derived. The parametrization step is then marked finished, and the next step triggered.
 */
std::array<double, 3> FTSAdaptiveForceController::baseForceTest( std::array<double, 3> fts_input_raw) {
        
        if (userIsGripping()) {
                last_gripped_time_ = current_loop_time_;
        }
        
        double secondsSinceLastGrip = (current_loop_time_ - last_gripped_time_).toSec();
        
        if (baseForce_testCompleted_) { //return robot to start new phase
                ledForceInput_ = 0.0;
                if ( userIsGripping()) {
                        setLEDPhase(stepAwayFromRobot);
                        last_gripped_time_ = current_loop_time_;
                        ROS_INFO_THROTTLE(5, "[BASE] - COMPLETED, USER STILL GRIPPING... Step away from Robot in order to reset its position.");
                        return zeroForce_;
                } else if (!baseForce_fts_offset_reset_) {
                        if (secondsSinceLastGrip > 0.1) {
                                recalculateFTSOffsets();
                                baseForce_fts_offset_reset_ = true;
                                secondsSinceLastGrip = 0.0;
                        }
                        
                } else if (travelledDistance_ > 0.0) {//push robot back to starting point
                        if (secondsSinceLastGrip > 1.0 ) {
                                setLEDPhase(robotInAutomaticMovement);
                                ROS_WARN_THROTTLE(2, "[BASE COMPLETED] - Pushing robot back (CURR_DIST: %.3f)", travelledDistance_);
                                ROS_DEBUG_THROTTLE(5, "[BASE] - COMPLETED, RETURNING - Phase completed, pushing robot back to start.");
                                return returnRobotToStartpoint();
                        } else {
                                return zeroForce_;
                        }
                } else { //robot has returned
                        ROS_INFO("[BASE TEST] - COMPLETED AND RETURNED! - Phase completed and robot returned to starting point. Starting new Phase");
                        recalculateFTSOffsets();
                        baseForce_testCompleted_ = false;
                        baseForce_fts_offset_reset_ = false;
                        setLEDPhase(phaseFinished);
                        switchParametrizationStep(); // to trigger the next step
                        return zeroForce_;
                }
        }
        
        // test not completed yet
        double currentVirtualSpringForce = travelledDistance_ * baseForce_springConstant_;
        double userInput = 0.0;
        switch(currentStep_) {
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
                        if (secondsSinceLastGrip < 1.0) {
                                setLEDPhase(stepAwayFromRobot);
                                return zeroForce_;
                        } else {
                                setLEDPhase(robotInAutomaticMovement);
                                ledForceInput_ = 0.0;
                                ROS_DEBUG_THROTTLE(5, "[BASE] - ABANDONED IN PHASE! - Slowly returning robot...");
                                return returnRobotToStartpoint();
                        }
                        ROS_DEBUG_THROTTLE(5, "[BASE] - ABANDONED IN PHASE! - Slowly returning robot...");
                        return returnRobotToStartpoint();
                } else {
                        resetBaseForceTest();
                        setLEDPhase(waitForInput);
                        ROS_INFO_THROTTLE(5, "[BASE] - WAIT FOR START- Grab the Robot and move in the shown direction to start recording !");
                        return zeroForce_;
                }   
        } else {
                 if ( (userInput < 0.0) && (travelledDistance_ < 0.01)) {
                         ROS_WARN_THROTTLE(0.5, "WRONG DIRECTION !");
                        setLEDPhase(waitForInput);
                        return zeroForce_;
                } else { //only case not returning 
                        ledForceInput_ = userInput;
                        setLEDPhase(showForce);
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
        
       // sendDebugTopicsParamBase(travelledDistance_, averageDistance, userInput, currentVirtualSpringForce, effectiveForce); // ACTIVATE FOR DEBUG
        
        //Check phase completion and return spring-modified force input
        if (baseForce_stableForceCounter_ < 120) {
                switch(currentStep_) {
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
                switch (currentStep_) {
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
                switch (currentStep_) {
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
        
        if (legtrack_updated_ && userIsGripping() && (std::fabs(lastStepDistance) > 0.002) ) { //update stored distances
                footDistanceSum_ += leg_dist_avg_x_;
                storedFootDistances_++;
                footDistance_lastTravelledDist_ = travelledDistance_;
        }
        
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
                setLEDPhase(walkBackwards);
        } else if (fwd_footDistanceRecorded_ && !bwd_footDistanceRecorded_ ) {
                if (travelledDistance_ < 0.5) {
                        setLEDPhase(almostReturned);
                        if (!userIsGripping() || travelledDistance_ < 0.01) {
                                bwd_medianFootDistance_ = footDistanceSum_ / storedFootDistances_;
                                bwd_footDistanceRecorded_ = true;
                                ROS_WARN("[PARAM FOOT DIST] - Backwards median dist. recorded as %.3f m. from center.", bwd_medianFootDistance_);
                                return returnRobotToStartpoint();
                        } else {
                                fts_input_raw = (fts_input_raw[0] > 0.0) ? zeroForce_ : fts_input_raw;
                        }
                }
        } else if (fwd_footDistanceRecorded_ && bwd_footDistanceRecorded_) {
                if (travelledDistance_ < 0.005) {
                        switchParametrizationStep();
                        return zeroForce_;
                } else {
                        return returnRobotToStartpoint();
                }
        }
        return fts_input_raw;
}

/**
 * \brief This function is used to parametrize the adaption parameters for the x-direction by using information about the averaged foot distance. Using the baseline-values 'fwd_medianFootDistance_' and 'bwd_medianFootDistance_' for forward and backwards direction, the difference towards these values is used as information about the current robot characteristics towards the user. If the robot is further away from the user than baseline, the robot reacts too fast. In this case, the reactiveness is reduced. In contrary, if the robot reacts too stiff and slow, the distance of the user towards it will decrease, and the parameters are tweaked towards a more adaptive behavior.
 */
void FTSAdaptiveForceController::parametrizeAdaptForceX() {
        
        double secondsInPhase = (current_loop_time_ - adaptX_startTime_).toSec();
        double secondsSinceLastParametrization = (current_loop_time_ - adaptX_lastTestTime_).toSec();
        
        
        if (legtrack_updated_) {
                
                adaptX_movingLegDistList_.push_back(leg_dist_avg_x_);
                adaptX_movementSpeedList_.push_back( std::fabs(getOldVelocityPercent()[0]) );
                
                if (adaptX_movingLegDistList_.size() < 50) {
                        return; // wait until 50 recordings are made for a stable legDistance
                } else {
                        adaptX_movingLegDistList_.pop_front();
                        adaptX_movementSpeedList_.pop_front();
                }
                        
                if (secondsSinceLastParametrization > adaptX_testIntervallDuration_) { // to let the user recognize an adaption and the robot gain new insight on the parameter changes with last adaption parametrization
                        double newAvgDist = std::accumulate(adaptX_movingLegDistList_.begin(), adaptX_movingLegDistList_.end(), 0.0) / adaptX_movingLegDistList_.size();
                        double avgSpeed = std::fmin( std::accumulate(adaptX_movementSpeedList_.begin(), adaptX_movementSpeedList_.end(), 0.0) / adaptX_movementSpeedList_.size(), 1.0);
                        double robotDistDiff = robotIsMovingForward() ? (newAvgDist - fwd_medianFootDistance_) : (newAvgDist - bwd_medianFootDistance_);
                        double percentualOfMaxDiff = (std::fabs(robotDistDiff) < minLegDistance_) ? 0.0 : std::fmin( std::fabs(robotDistDiff) / maxLegDistance_, 1.0);
                        double avgScale = force_scale_minvel_[0] + ( force_scale_maxvel_[0] - force_scale_minvel_[0] ) * avgSpeed;
                        
                        ROS_INFO("[PARAM AdaptX] [DEBUG] RobotDistDiff: %.2f, maxDiffFactor: %.2f, avgSpeedFactor: %.2f, avgScale: %.2f (timeInPhase: %.2f)", 
                                 robotDistDiff, percentualOfMaxDiff, avgSpeed, avgScale, secondsInPhase);
                        
                        if (percentualOfMaxDiff < 0.01) { // no need to adapt on this step
                                adaptX_unchangedCtr_++;
                                ROS_INFO("[PARAM AdaptX - NO CHANGE] Difference in last measurement less than minimal leg distance, thus not changing parameters now. (%d steps total)", adaptX_unchangedCtr_);
                                if (adaptX_unchangedCtr_ == 5) {
                                        //declare phase finished
                                        ROS_INFO("[PARAM AdaptX] Last five measurements were in range, which concludes parametrization of adaptive features\n Enabling velocity-adaption using force_scale_minvel: %.2f and force_scale_maxvel: %.2f as parameters now.", force_scale_minvel_[0], force_scale_maxvel_[0]);
                                        force_scale_maxvel_[1] = force_scale_maxvel_[0];
                                        force_scale_minvel_[1] = force_scale_minvel_[0];
                                        switchParametrizationStep();
                                        setLEDPhase(phaseFinished);
                                        return;
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
                                setLEDPhase(showAdapted);
                        }//adaption needed
                        adaptX_lastTestTime_ = current_loop_time_;
                        //sendDebugTopicsParamAdaptX(avgScale, robotDistDiff); // ACTIVATE FOR DEBUG ONLY
                }//adaptX_testInterval step
                
        }
        adaptForceScale();
}

/**
 * \brief Adapts the force scales depending on the velocity
 * 
 * This function is for adaptive force inputs. It adapts the maximal needed force depending on the current velocity and the scaling factors defined by the user.
 * The goal is to have more static behavior when accelerating while allowing for easier keeping high velocities.
 * 
 */
void FTSAdaptiveForceController::adaptForceScale() {
        
        if (adaption_is_active_) {
                std::array<double, 3> curr_scale = scaleBetweenValues(force_scale_minvel_, force_scale_maxvel_);
                
                std::array<double, 3> adaptive_max_ft;
                for (int i = 0; i < 3; i++) {
                        adaptive_max_ft[i] = curr_scale[i] * base_max_ft_[i];
                }
                setMaxFt(adaptive_max_ft);
                pub_adaptive_scale_.publish(convertToMessage(curr_scale));
                pub_adaptive_max_ft_.publish(convertToMessage(adaptive_max_ft));
        } else {
                ROS_WARN("[ADAPT_Force:OFF]");
                resetToBaseValues();
        }
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
void FTSAdaptiveForceController::resetToBaseValues() {
        
        setMaxFt(base_max_ft_);
}

/**
 * \brief Returns the current scaling factor based on current velocity and the input values for min and max scale factors
 * This function returns for each dimension of the robot the current adaptive scale factor between the input 'minScaleFactor' and 'maxScaleFactor' based on the current percentual velocity of the robot. 
 * If the toggle 'use_smooth_transition_' is switched on, the scale is calculated on a different function which puts a higher change-rate towards the lower-end of the velocity (where the robot is normally pushed more often)
 * 
 */
std::array<double, 3> FTSAdaptiveForceController::scaleBetweenValues( std::array<double, 3> minScaleFactor, std::array<double, 3> maxScaleFactor ) {
        
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
                switch (currentStep_) {
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

std::array<double, 3> FTSAdaptiveForceController::returnRobotToStartpoint() {
        
        double scale = (travelledDistance_ > 0.1) ? 1.0 : 0.5;
        switch (currentStep_) {
                case baseX: case recordFeetDistance:
                        return {-1.0 *  returnForce_x_ * scale, 0.0, 0.0};
                case baseYLeft: 
                        return {0.0, -1.0 * returnForce_y_ * scale, 0.0};
                case baseYRight:
                        return {0.0, returnForce_y_ * scale, 0.0};
                case baseRotLeft:
                        scale = std::fmin(1.0, (travelledDistance_ / 0.05) );
                        return {0.0, 0.0, -1.0 * returnForce_rot_ * scale};
                case baseRotRight:
                        scale = std::fmin(1.0, (travelledDistance_ / 0.05) );
                        return {0.0, 0.0, returnForce_rot_ * scale};
        }
}

/**
 * \brief Resets the FTSAdaptiveForceController by first resetting the base controller, then storing the base values and finally adapting them by the configured dynamics
 * 
 */
void FTSAdaptiveForceController::resetController() {
        
        ROS_DEBUG("[FTS_ADAPT:] Reset");
        resetToBaseValues();
        FTSBaseController::resetController();
}

/**
 * \brief This function resets the parameters for the baseForceTest (when the user abandons the robot mid-test)
 * 
 */
void FTSAdaptiveForceController::resetBaseForceTest() {
        
        adaption_is_active_ = false; //should be off here anyways, this is just for additional safety
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
        switch(currentStep_) {
                case baseX:
                        currentStep_ = baseYLeft;
                        ROS_INFO("[PARAM STEP - baseYLeft activated! ]");
                        break;
                case baseYLeft:
                        currentStep_ = baseYRight;
                        ROS_INFO("[PARAM STEP - baseYRight activated! ]");
                        break;
                case baseYRight:
                        currentStep_ = baseRotLeft;
                        ROS_INFO("[PARAM STEP - baseRotLeft activated! ]");
                        break;
                case baseRotLeft:
                        currentStep_ = baseRotRight;
                        ROS_INFO("[PARAM STEP - baseRotRight activated! ]");
                        break;
                case baseRotRight:
                        currentStep_ = finished;
                        ROS_INFO("[PARAM STEP - Base force parametrization finished! ]");
                        break;
                case recordFeetDistance:
                        currentStep_ = adaptX;
                        ROS_WARN("[PARAM STEP - Feet Distance Recording finished! ]");
                        break;
                case adaptX:
                        currentStep_ = finished;
                        ROS_WARN("[PARAM STEP - Finalizing parametrization now!! ]");
                        break;
                case finished:
                        currentStep_ = baseX;
                        ROS_WARN("[PARAM STEP - Parametrization finished!! Activating all degrees of freedom and resetting parametrization! ]");
                        setLEDPhase(unlocked);
                        parametrization_active_ = false;
                default: 
                        ROS_WARN("[SWITCH STEP] No Valid Step defined!");
                        return;
        }
        resetTravelledDistance();
        stepInitialized_ = false;
        stepActivated_ = false;
        switchStepRequested_ = false;
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
        switch (currentStep_) { // disable currently unavailable directions from showing
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
        pub_input_force_for_led_.publish(real_wrench_msg);
}

/**
 * \brief Sets the LED phase (used for LED feedback messages)
 */
void FTSAdaptiveForceController::setLEDPhase(led_phase_ requestPhase) {
        
        if (currentLEDPhase_ != requestPhase) { // to prevent from same message being shown multiple times
                led_ac_->cancelGoalsAtAndBeforeTime(current_loop_time_);
                currentLEDPhase_ = requestPhase;
                sendLEDGoal_ = true;
                ROS_DEBUG("LED PHASE HAS CHANGED!");
        }
}

/**
 * \brief sends the LED output which is defined by the currently active led_phase_ enum
 */
void FTSAdaptiveForceController::sendLEDOutput() {
        
       if (currentLEDPhase_ == showForce) {
               
               sendLEDForceTopics();
       } else if (sendLEDGoal_) {
                sendLEDGoal_ = false; //to only send it once the phase has changed
                switch (currentLEDPhase_) {
                        case unlocked:
                                led_ac_->sendGoal(blinkyFreeMovementBlue_);
                                return;
                        case waitForInput:
                                switch (currentStep_) {
                                        case baseX:
                                                led_ac_->sendGoal(blinkyStart_x_);
                                                return;
                                        case baseYLeft:
                                                led_ac_->sendGoal(blinkyStart_y_left_);
                                                return;
                                        case baseYRight:
                                                led_ac_->sendGoal(blinkyStart_y_right_);
                                                return;
                                        case baseRotLeft:
                                                led_ac_->sendGoal(blinkyStart_rot_left_);
                                                return;
                                        case baseRotRight:
                                                led_ac_->sendGoal(blinkyStart_rot_right_);
                                                return;
                                        case recordFeetDistance:
                                                if (!fwd_footDistanceRecorded_) {
                                                        led_ac_->sendGoal(blinkyStart_x_);
                                                } else {
                                                        led_ac_->sendGoal(blinkyStart_x_back_);
                                                }
                                                return;
                                        case adaptX:
                                                led_ac_->sendGoal(blinkyStartGreen_);
                                                return;
                                }
                        case walkForward:
                                led_ac_->sendGoal(blinkyWalkForward_);
                                return;
                        case walkBackwards:
                                led_ac_->sendGoal(blinkyWalkBackwards_);
                                return;
                        case showAdapted:
                                if (currentStep_ == adaptX) {
                                        led_ac_->sendGoal(blinkyPhaseYellow_);
                                }
                                return;
                        case stepAwayFromRobot:
                                led_ac_->sendGoal(blinkyStepAway_);
                                return;
                        case robotInAutomaticMovement:
                                switch (currentStep_) {
                                        case baseX: 
                                                led_ac_->sendGoal(blinkyRobotAutomaticMovement_x_);
                                                return;
                                        case baseYLeft:
                                                led_ac_->sendGoal(blinkyRobotAutomaticMovement_y_left_);
                                                return;
                                        case baseYRight:
                                                led_ac_->sendGoal(blinkyRobotAutomaticMovement_y_right_);
                                                return;
                                        case baseRotLeft:
                                                led_ac_->sendGoal(blinkyRobotAutomaticMovement_rot_left_);
                                                return;
                                        case baseRotRight:
                                                led_ac_->sendGoal(blinkyRobotAutomaticMovement_rot_right_);
                                                return;
                                        case recordFeetDistance: case adaptX:
                                                led_ac_->sendGoal(blinkyRobotAutomaticMovement_);
                                                return;
                                }           
                        case almostReturned:
                                led_ac_->sendGoal(blinkyAlmostFinishedRed_);
                                return;          
                        case phaseFinished:
                                led_ac_->sendGoal(blinkyFinishedRed_);
                                return;
                }
       }
}

/**
 * \brief debug function which pushes debug messages regarding the process of 'parametrizeBaseX', e.g. current distance to startpoint or springforce and resulting force
 */
void FTSAdaptiveForceController::sendDebugTopicsParamBase( double travelledDist, double averageDist, double raw_input, double currentVirtSpringForce, double effectiveForce ) {
        
        std_msgs::Float64 rawInput_msg, virtSpring_msg, resForce_msg,distCurr_msg, distAvg_msg; //debug topic messages
        rawInput_msg.data = raw_input;
        virtSpring_msg.data = currentVirtSpringForce;
        resForce_msg.data = effectiveForce;
        distCurr_msg.data = travelledDist;
        distAvg_msg.data = averageDist;         
        
        switch(currentStep_) {
                case baseX:
                        pub_base_x_currentRawForce_.publish(rawInput_msg);
                        pub_base_x_virtSpring_.publish(virtSpring_msg);
                        pub_base_x_resForce_.publish(resForce_msg);
                        pub_base_x_currentDist_.publish(distCurr_msg);
                        pub_base_x_averageDist_.publish(distAvg_msg);
                        break;
                case baseYLeft:
                        pub_base_y_left_currentRawForce_.publish(rawInput_msg);
                        pub_base_y_left_virtSpring_.publish(virtSpring_msg);
                        pub_base_y_left_resForce_.publish(resForce_msg);
                        pub_base_y_left_currentDist_.publish(distCurr_msg);
                        pub_base_y_left_averageDist_.publish(distAvg_msg);
                        break;
                case baseYRight:
                        pub_base_y_right_currentRawForce_.publish(rawInput_msg);
                        pub_base_y_right_virtSpring_.publish(virtSpring_msg);
                        pub_base_y_right_resForce_.publish(resForce_msg);
                        pub_base_y_right_currentDist_.publish(distCurr_msg);
                        pub_base_y_right_averageDist_.publish(distAvg_msg);
                        break;
                case baseRotLeft:
                        pub_base_rot_left_currentRawForce_.publish(rawInput_msg);
                        pub_base_rot_left_virtSpring_.publish(virtSpring_msg);
                        pub_base_rot_left_resForce_.publish(resForce_msg);
                        pub_base_rot_left_currentDist_.publish(distCurr_msg);
                        pub_base_rot_left_averageDist_.publish(distAvg_msg);
                        break;
                case baseRotRight:
                        pub_base_rot_right_currentRawForce_.publish(rawInput_msg);
                        pub_base_rot_right_virtSpring_.publish(virtSpring_msg);
                        pub_base_rot_right_resForce_.publish(resForce_msg);
                        pub_base_rot_right_currentDist_.publish(distCurr_msg);
                        pub_base_rot_right_averageDist_.publish(distAvg_msg);
                        break;
        }
}

/**
 * \brief Sends debug topics for paramerization
 */
void FTSAdaptiveForceController::sendDebugTopicsParamAdaptX( double currentScale, double robotDistDiff ) {
        
        std_msgs::Float64 currentScale_msg, minvelScale_msg, maxvelScale_msg, distDiff_msg;
        currentScale_msg.data = currentScale;
        minvelScale_msg.data = force_scale_minvel_[0];
        maxvelScale_msg.data = force_scale_maxvel_[0];
        distDiff_msg.data = robotDistDiff;
        pub_adaptive_scale_x_.publish(currentScale_msg);
        pub_adaptive_factor_min_.publish(minvelScale_msg);
        pub_adaptive_factor_max_.publish(maxvelScale_msg);
        pub_adaptive_distDiff_.publish(distDiff_msg);
}


//CALLBACK FUNCTIONS
/**
 * \brief Dynamic reconfigure Callback function for the controller
 */
void FTSAdaptiveForceController::reconfigureCallback(robotrainer_controllers::FTSAdaptiveForceControllerConfig &config, uint32_t level) {
        
        use_passive_behavior_ctrlr_ = config.use_passive_behavior_ctrlr;
        ROS_INFO_COND(use_passive_behavior_ctrlr_, "[ADAPT] - Enabling passive behavior control!");
        adaption_is_active_ = config.use_velocity_adaptive_force;
        
        //switch between standard ft/velocity values and user-parametrized values
        
        int used_ft_type = config.max_ft_values_used;
        if (used_ft_type == 1) {
                if (baseForce_allParamsStored_) {
                        useUserParametrizedValues(true);
                } else {
                        ROS_WARN("Please use ParametrizeBase before attempting to use the user-defined values!");
                }
        } else {
                useUserParametrizedValues(false);
        }
        
        //parametrization reconfigure part
        if (config.enable_parametrization_changes) {
                config.groups.user_parametrization_functionalities.state = true;
                
                bool activateForceParametrization = config.activate_force_parametrization;
                bool activateScaleParametrization = config.activate_adaptive_scale_parametrization;
                if (!parametrization_active_ && activateForceParametrization) {
                        ROS_INFO("[ADAPT - Parametrization] - Activating User Force Parametrization.");
                        parametrization_active_ = true;
                        currentStep_ = baseX;
                        switchStepRequested_ = false;
                        stepInitialized_ = false;
                } else if (!parametrization_active_ && activateScaleParametrization) {
                        ROS_INFO("[ADAPT - Parametrization]  - Activating Adaptive Scale parametrization.");
                        parametrization_active_ = true;
                        currentStep_ = recordFeetDistance;
                        switchStepRequested_ = false;
                        stepInitialized_ = false;
                } else if ( (parametrization_active_ && !activateForceParametrization) || (parametrization_active_ && !activateScaleParametrization) ) {
                        setActiveDimensions( all_active_);
                        parametrization_active_ = false;
                        currentStep_ = baseX;
                        stepInitialized_ = false;
                        setLEDPhase(unlocked);
                }
                
                if (!parametrization_active_) { //enable change of parametrization parameters
                        ROS_INFO("[ADAPT - Parametrization]: Updating parametrization parameters.");
                        baseForce_movingAverageTimeframe_ = config.movingAverageTimeframe;
                        baseForce_holdingDistance_ = config.holdingDistance;
                        minLegDistance_ = config.minLegDistance;
                        maxLegDistance_ = config.maxLegDistance;
                        adaptive_parametrization_adaptionRate_ = config.adaptionRate;
                } else {
                        ROS_INFO("[ADAPT - Parametrization]: Unable to update parametrization parameters because it is currently active!");
                }
                
        } else {
                config.groups.user_parametrization_functionalities.state = false;
        }
        
        //velocity adaptive feature reconfigure part
        if (config.enable_velocity_adaption_changes) {
                config.groups.velocity_adaptive_feature_parameters.state = true;
                
                ROS_INFO("[ADAPT]: Changing parameters according to dynamic reconfigure inputs");
                stopController();
                
                if (adaption_is_active_) {
                        ROS_INFO("[ADAPT_Force: ON]");
                        force_scale_minvel_[0] = config.force_scale_minvel_x;
                        force_scale_minvel_[1] = config.force_scale_minvel_y;
                        force_scale_minvel_[2] = config.force_scale_minvel_rot;
                        force_scale_maxvel_[0] = config.force_scale_maxvel_x;
                        force_scale_maxvel_[1] = config.force_scale_maxvel_y;
                        force_scale_maxvel_[2] = config.force_scale_maxvel_rot;
                        use_smooth_transition_ = config.use_smooth_transition;
                        transition_rate_ = config.transition_rate;
                        ROS_INFO_COND(use_smooth_transition_ && adaption_is_active_, "[ADAPT]: Using Smooth Transition");
                }
                
                resetController();
                
        } else {
                config.groups.velocity_adaptive_feature_parameters.state = false;
        }
        
}

/**
 * \brief Switch between userParametrized and standard values (initially used for quick switching during the evaluation, but leaving this function permanently active as it could be useful in general)
 */
void FTSAdaptiveForceController::useUserParametrizedValues(bool enable) {
        
        if (enable && baseForce_allParamsStored_) {
                ROS_INFO("Using userParametrized Force/Velocity values: [%.2f, %.2f, %.2f]", userParametrized_maxFT_[0], userParametrized_maxFT_[1], userParametrized_maxFT_[2]);
                setMaxFt(userParametrized_maxFT_);
                setMaxVel(userParametrized_maxVel_);
        } else {
                ROS_INFO("Using standard Force/Velocity values: [%.2f, %.2f, %.2f]", standard_maxFT_[0], standard_maxFT_[1], standard_maxFT_[2]);
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
        legtrack_updated_ = true;
}
        
}
PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::FTSAdaptiveForceController, controller_interface::ControllerBase)
