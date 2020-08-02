#include "robotrainer_controllers/passive_behavior_controller.h"


namespace robotrainer_controllers {
        
PassiveBehaviorControllerBase::PassiveBehaviorControllerBase() {}
bool PassiveBehaviorControllerBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {
        
        ROS_INFO("[PASS BHVR INIT] Init passiveBehaviorCtrlBase");
        //initalize the FTSBaseController
        if (!base_initialized_) { //init base controller if yet unitialized
                ROS_INFO("[PASS BHVR INIT] - Initializing FTSBaseController first...");
                robotrainer_controllers::FTSBaseController::init(robot_hw, root_nh, controller_nh);
        }
        
        initPublishersAndParameters(root_nh, controller_nh);
        //init parameters
        resetValues();
        return base_initialized_;
}

void PassiveBehaviorControllerBase::initInChain(ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {
        
        ROS_INFO("[PASS BHVR INIT AS CHAINED MODE]");
        initPublishersAndParameters(root_nh, controller_nh);
        usedMethod_ = slidingMD;
}

void PassiveBehaviorControllerBase::setValuesFromBaseController(std::array<double, 3> gain, std::array<double, 3> time_const) {
        
        gain_ = gain;
        time_const_ = time_const;
        ROS_INFO("[PASS BHVR INIT CHAINED] - Setting base ctrlr Values to gain:[%.2f, %.2f, %.2f] and timeConst:[%.2f, %.2f, %.2f]",
                 gain_[0], gain_[1], gain_[2], time_const_[0], time_const_[1], time_const_[2]);
}

void PassiveBehaviorControllerBase::initPublishersAndParameters(ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {

        ros::NodeHandle passive_behavior_base_nh(controller_nh, "PassiveBehaviorControllerBase");
        //init default values
        passive_behavior_base_nh.param<int>("integral_window_size", integral_window_size_, 32);
        passive_behavior_base_nh.param<double>("robot_virt_mass", robot_virt_mass_, 80.0);
        passive_behavior_base_nh.param<double>("system_const", system_const_, 1.2);
        passive_behavior_base_nh.param<double>("adapt_inputFoce_Xval_scale", adapt_input_force_scale_[0], 0.1);
        passive_behavior_base_nh.param<double>("adapt_inputFoce_Yval_scale", adapt_input_force_scale_[1], 0.1);
        passive_behavior_base_nh.param<double>("adapt_inputFoce_Tval_scale", adapt_input_force_scale_[2], 0.1);
        passive_behavior_base_nh.param<double>("min_adaption_factor", min_adaption_factor_, 0.01);
        passive_behavior_base_nh.param<double>("max_adaption_factor", max_adaption_factor_, 1.0);
        passive_behavior_base_nh.param<double>("eps", eps_, 0.001);
        passive_behavior_base_nh.param<double>("scale_down_time", scale_down_time_, 0.4);
        passive_behavior_base_nh.param<double>("scale_up_time", scale_down_time_, 0.7);
        lastloopTime_ = ros::Time::now();

        //debug initPublishersAndParameters
        pub_sliding_integral_ = new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "passive_SlidingIntegralVal",1);
        pub_adaption_factor_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "passive_AdaptionFactor",1);
        pub_real_input_force_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "passive_RealInputforce",1);
        pub_sliding_integral_3d_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "passive_SlidingIntegralVal3d",1);
        pub_chuy_integral_3d_ = new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(controller_nh, "passive_ChuyIntegral3d", 1);

        //get pointer for dynamic reconfigure
        p_dysrv_ = new dynamic_reconfigure::Server<robotrainer_controllers::PassiveBehaviorControllerConfig>(passive_behavior_base_nh);
        dynamic_reconfigure::Server<robotrainer_controllers::PassiveBehaviorControllerConfig>::CallbackType dycb =boost::bind(&PassiveBehaviorControllerBase::reconfigureCallback, this, _1, _2);
        p_dysrv_->setCallback(dycb);
}

void PassiveBehaviorControllerBase::starting(const ros::Time& time) {

        FTSBaseController::starting(time);
}

void PassiveBehaviorControllerBase::resetValues() {
        
        ROS_DEBUG("[PASS BHV] Reset Values!");
        chuy_MD_ptr_->resetIntegralValues();
        integral_ptr_->resetIntegral();
        integral_MD_ptr_->resetIntegral();
        
        scale_down_ = false;
        scale_up_ = false;
        for (int i = 0; i < 3; i++) {
            scale_down_MD_[i] = false;
            scale_up_MD_[i] = false;
            adaption_factor_[i] = 1.0;
            adaption_factor_old_[i] = 1.0;
            ignoringImpassiveBehavior_[i] = false;
            changedDirection_[i] = false;
            stableDuration_[i] = 0.0;
            resetToStability_Countdown_[i] = 0.0;
            instableDuration_[i] = 0.0;
            sameDirectionCtr_[i] = 0.0;
            deactivateAllDimensions_ = false;
        }
        fresh_resetted_ = true;
        resetSlidingVelocities();
        setDesiredMass();
        setDesiredAdaptionFactor_MD();
        
}

void PassiveBehaviorControllerBase::resetSlidingVelocities() {
        
        for (int i = 0; i < 3; i++) {
                slidingDirections_[i].clear();
                slidingDirections_[i].assign(directionWindowSize_, 0);
        }
}

void PassiveBehaviorControllerBase::setDesiredMass() {
        
        for(auto i = 0;  i < dim ; ++i) {
                desired_mass_[i] = time_const_[i] / gain_[i];
                ROS_DEBUG("[passive_behavior_base_ctrl.cpp] desired_mass_[%i]: %.2f",i, desired_mass_[i]);
        }
}

void PassiveBehaviorControllerBase::setDesiredAdaptionFactor_MD() {
        
        for(auto i = 0; i < dim; ++i) {
                desired_adaption_factor_[i] = 1/(((system_const_*robot_virt_mass_)/(desired_mass_[i])) - system_const_);
                ROS_DEBUG("[passive_behavior_base_ctrl.cpp] desired_adaption_factor_[%i]: %.2f", i, desired_adaption_factor_[i]);
        }
}

bool PassiveBehaviorControllerBase::detectedNonPassiveBehavior() {
        if(integral_ptr_->getIntegralValue() < 0.0) {
                return true;
        } else {
                return false;
        }
}

bool PassiveBehaviorControllerBase::detectedNonPassiveBehavior_MD() {
        
        std::array<double, 3> integral_values = {{0.0, 0.0, 0.0}};
        switch (usedMethod_) {
                case chuy2007:
                        integral_values = chuy_MD_ptr_->getIntegralValues();
                        break;
                case slidingMD:
                        integral_values = integral_MD_ptr_->getIntegralValues();
                        break;
        }

        if(integral_values[0] < 0.0 || integral_values[1] < 0.0 || integral_values[2] < 0.0) {
                return true;
        } else {
                return false;
        }
}

void PassiveBehaviorControllerBase::updateIntegrals(const std::array<double, dim>& force, const std::array<double, dim>& velocity, const ros::Time& time) {
        
    //direction change detection
    for (auto i = 0; i < dim; i++) {
        if (!control_dimensions_[i]) { continue; }
        int currentDirection = (velocity[i] > 0.0) ? 1 : ((velocity[i] < 0.0) ? -1 : 0);
        if (currentDirection * slidingDirections_[i].back() == 1) {
                sameDirectionCtr_[i]++;
        } else {
                sameDirectionCtr_[i] = 0;
        }
        slidingDirections_[i].push_back(currentDirection);
        //move window
        int oldestDirection = slidingDirections_[i].front();
        slidingDirections_[i].pop_front();                
        changedDirection_[i] = changedDirection_[i] || ((oldestDirection * currentDirection ) == -1);
        if (changedDirection_[i] and sameDirectionCtr_[i] == 0)
        {
            ROS_WARN("[DIM %d] - Multiple Direcation Changes Detected!", i);
            multipleDirectionChange_[i] = true;
        }
        if (sameDirectionCtr_[i] > (2 * directionWindowSize_)) {
                changedDirection_[i] = false;
                sameDirectionCtr_[i] = 2 * directionWindowSize_;
        }
    }
    

    old_velocity_ = velocity;
    std::array<double, dim> integral_values;

    switch (usedMethod_) {                
        case chuy2007:
        {
            integral_values = chuy_MD_ptr_->calculateIntegralMultiDim(force, velocity, time);
            //publish values
            if (pub_chuy_integral_3d_->trylock()) {
                pub_chuy_integral_3d_->msg_.x = integral_values[0];
                pub_chuy_integral_3d_->msg_.y = integral_values[1];
                pub_chuy_integral_3d_->msg_.z = integral_values[2];
                pub_chuy_integral_3d_->unlockAndPublish();
            }
            break;
        }
        case slidingSD:
        {
            integral_ptr_->calculateIntegral(force, velocity, time);
            //publish values
            if (pub_sliding_integral_->trylock()) {
                pub_sliding_integral_->msg_.data = integral_ptr_->getIntegralValue();
                pub_sliding_integral_->unlockAndPublish();
            }
            break;
        }

        case slidingMD:
        {

            integral_values = integral_MD_ptr_->calculateIntegralMultiDim(force, velocity, time);
            //publish values
            if (pub_sliding_integral_3d_->trylock()) {
                pub_sliding_integral_3d_->msg_.x = integral_values[0];
                pub_sliding_integral_3d_->msg_.y = integral_values[1];
                pub_sliding_integral_3d_->msg_.z = integral_values[2];
                pub_sliding_integral_3d_->unlockAndPublish();
            }
            //FOR RECORDING PURPOSE
            if (pub_sliding_integral_->trylock()) {
                pub_sliding_integral_->msg_.data = sumArray(integral_MD_ptr_->getIntegralValues());
                pub_sliding_integral_->unlockAndPublish();
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

double PassiveBehaviorControllerBase::scaleDownLinear_MD(const double& desired_adaption_factor, const double& current_adaption_factor, const ros::Time& scale_down_start_time, const double& start_value) {
        
        double adaption_factor;
        double linFactor =  fmin(1.0, (((ros::Time::now() - scale_down_start_time).toSec())/scale_down_time_));
        if(current_adaption_factor > desired_adaption_factor) {
                double adaptionDiff = start_value - desired_adaption_factor;
                //scale
                adaption_factor = start_value - adaptionDiff*linFactor;
                //scale between valid values
                adaption_factor = scale(adaption_factor, min_adaption_factor_, max_adaption_factor_);
        } else {
                adaption_factor = desired_adaption_factor;
        }
        return adaption_factor;
}

double PassiveBehaviorControllerBase::scaleUpLinear_MD(const double& current_adaption_factor, const ros::Time& scale_up_start_time, const double& start_value) {
       
        double adaption_factor;
        double linFactor =  fmin(1.0, (((ros::Time::now() - scale_up_start_time).toSec())/scale_up_time_));
        if(current_adaption_factor < max_adaption_factor_) {
                double adaptionDiff = max_adaption_factor_ - start_value;
                adaption_factor = start_value + adaptionDiff* linFactor;
                //scale between valid values
                adaption_factor = scale(adaption_factor, min_adaption_factor_, max_adaption_factor_);
        } else {
                adaption_factor = max_adaption_factor_;
        }
        return adaption_factor;
}

std::array<double, 3>PassiveBehaviorControllerBase::adaptInputForce(const std::array<double, dim>& adaption_factor, const std::array<double, dim> &input_force) {
        
        std::array<double, dim> adapted_input;
        for(auto i = 0; i < dim; ++i) {
                adapted_input[i] = adaption_factor[i] * input_force[i];
        }
        return adapted_input;
}

std::array<double, 3> PassiveBehaviorControllerBase::calcCurrentAdaptionFactor_time_MD(const ros::Time& time, std::array<double, dim> current_power) {
        
        std::array<double, 3> integral_values = {{0.0, 0.0, 0.0}};
        switch (usedMethod_) {
                case chuy2007:
                        integral_values = chuy_MD_ptr_->getIntegralValues();
                        break;
                case slidingMD:
                        integral_values = integral_MD_ptr_->getIntegralValues();
                        break;
        }
        std::array<double, dim> adaption_factor;
        
        ROS_DEBUG("CurrentDirection: [%d, %d, %d] - ChangedDirection: [%d, %d, %d] - CurrentlyInstable[%d,%d,%d] - StableDurations: [%.3f, %.3f, %.3f], InstableDurations: [%.3f, %.3f, %.3f]", 
                        slidingDirections_[0].back(), slidingDirections_[1].back(), slidingDirections_[2].back(),
                        changedDirection_[0], changedDirection_[1], changedDirection_[2], 
                        (integral_values[0] < 0.0), (integral_values[1] < 0.0), (integral_values[2] < 0.0), 
                        stableDuration_[0], stableDuration_[1], stableDuration_[2],
                        instableDuration_[0], instableDuration_[1], instableDuration_[2] );
        
        double timeDelta = (time - lastloopTime_).toSec();
        
        deactivateAllDimensions_ = (deactivateAllOnExtremeInstability_ && (resetToStability_Countdown_[0] + resetToStability_Countdown_[1] + resetToStability_Countdown_[2]) > 0.001);
        
        for(int i = 0; i < dim; i++) {
                if (!control_dimensions_[i]) { 
                        if (deactivateAllDimensions_) {
                                adaption_factor[i] = 0.0;
                                adaption_factor_old_[i] = 0.0;
                        } else {
                                //adaption factor is less than max, so adaption factor gets scaled back to max
                                if(adaption_factor_old_[i] < max_adaption_factor_) {
                                        // check if start of scaling up
                                        if(!scale_up_MD_[i]) {
                                                //set parameters for scaling up
                                                scale_down_MD_[i] = false;
                                                scale_up_MD_[i] = true;
                                                scale_up_time_begin_MD_[i] = time;
                                                scale_up_start_val_[i] = adaption_factor_old_[i];
                                                adaption_factor_old_[i] = adaption_factor_old_[i] < max_adaption_factor_ ? adaption_factor_old_[i] : max_adaption_factor_;
                                        }
                                        adaption_factor[i] = scaleUpLinear_MD(adaption_factor_old_[i],scale_up_time_begin_MD_[i],scale_up_start_val_[i]);
                                        // robot is in stable state and current input force is used
                                } else {
                                        scale_down_MD_[i] = false;
                                        scale_up_MD_[i] = false;
                                        adaption_factor[i] = max_adaption_factor_;
                                }
                        }
                        continue;
                }
                
                //robot is unstable
                if ( integral_values[i] < instable_threshold_value_[i] ) {
                         
                        //if (conditionalInstabilityDetection_ && changedDirection_[i] && (stableDuration_[i] > minimumStableDuration_) ) {
                        if (conditionalInstabilityDetection_ && changedDirection_[i] && !multipleDirectionChange_[i]) {
                            ROS_WARN("[DIM %d] - Flagging ignore on first Instability!", i);
                            scale_down_MD_[i] = false;
                            scale_up_MD_[i] = false;
                            adaption_factor[i] = max_adaption_factor_;
                            changedDirection_[i] = false;
                            ignoringImpassiveBehavior_[i] = true; // flag currently ignoring impassivity
                         }
                         
                         if (conditionalInstabilityDetection_ && changedDirection_[i] && multipleDirectionChange_[i]) {
                            ignoringImpassiveBehavior_[i] = false;
                         }
                         
                         stableDuration_[i] = 0.0;
                         
                         //ignore first instability 
                         if ( ignoringImpassiveBehavior_[i] ) {
                                 ROS_INFO("[DIM %d] - IGNORING INSTABILITY (current integral value: %.3f) ", i, integral_values[i]);
                                 scale_down_MD_[i] = false;
                                 scale_up_MD_[i] = false;
                                 adaption_factor[i] = max_adaption_factor_;
                        // counter instability 
                        } else {
                                
                                instableDuration_[i] += timeDelta;
                                
                                //check if start of scaling down time
                                if(!scale_down_MD_[i]) { 
                                        //set parameters for scaling down for normal instability
                                        scale_up_MD_[i] = false;
                                        scale_down_MD_[i] = true;
                                        scale_down_time_begin_MD_[i] = time;
                                        scale_down_start_val_[i] = adaption_factor_old_[i];
                                }
                                
                                //check if robot was instable for too long
                                if (conditionalInstabilityDetection_ && (instableDuration_[i] >= extremelyInstableDuration_)) {
                                        ROS_WARN_THROTTLE(0.2, "[DIM %d] - ROBOT CURRENTLY EXTREMELY INSTABLE ! DEACTIVATING DIMENSION UNTIL USER RELEASES ROBOT OR DIMENSION IS LONG ENOUGH STABLE", i);
                                        instableDuration_[i] = extremelyInstableDuration_;
                                        resetToStability_Countdown_[i] = resetCooldownAfterInstability_;
                                        adaption_factor[i] = 0.0;
                                        if (deactivateAllOnExtremeInstability_) {
                                                deactivateAllDimensions_ = true;
                                                return {{0.0, 0.0, 0.0}};
                                        }
                                } else {
                                        //scaling down
                                        adaption_factor[i] = scaleDownLinear_MD(desired_adaption_factor_[i], adaption_factor_old_[i], scale_down_time_begin_MD_[i], scale_down_start_val_[i]);
                                }
                        }
                //robot is stable
                } else {
                        
                        //check if robot was previously extremely instable, and only scale up after the countdown has finished
                        if (resetToStability_Countdown_[i] > timeDelta) {
                                resetToStability_Countdown_[i] -= timeDelta;
                                ROS_WARN_THROTTLE(0.2, "[DIM %d] - Robot was previously extremely unstable, currently [%.2f sec.] cooldown remaining before activating this dimension again!", i, resetToStability_Countdown_[i]);
                                continue; // break this loop to ignore stability while on cooldown
                        } else if (resetToStability_Countdown_[i] > 0.0) {
                                ROS_WARN_THROTTLE(0.2, "[DIM %d] - Robot was previously extremely unstable but will be reactivated after the cooldown phase!", i);
                                adaption_factor_old_[i] = min_adaption_factor_;
                                resetToStability_Countdown_[i] = 0.0;
                               
                        }
                        
                        //check if impassive deceleration was previously ignored, and if so start cool-down timer
                        if (ignoringImpassiveBehavior_[i]) {
                                ROS_INFO("[DIM %d] - Robot stable after ignored instability, starting to count stability timer up again!", i);
                                ignoringImpassiveBehavior_[i] = false;
                        } else {
                                stableDuration_[i] += timeDelta;
                        }
                        
                        //check if robot was stable for long enough to reset instableDuration
                        if (stableDuration_[i] > minimumStableDuration_) {
                            multipleDirectionChange_[i] = false;
                            instableDuration_[i] = 0.0;
                            stableDuration_[i] = minimumStableDuration_ + 0.1;
                        }                        
                        
                        //adaption factor is less than max, so adaption factor gets scaled back to max
                        if(adaption_factor_old_[i] < max_adaption_factor_) {
                                // check if start of scaling up
                                if(!scale_up_MD_[i]) {
                                        //set parameters for scaling up
                                        scale_down_MD_[i] = false;
                                        scale_up_MD_[i] = true;
                                        scale_up_time_begin_MD_[i] = time;
                                        scale_up_start_val_[i] = adaption_factor_old_[i];
                                        adaption_factor_old_[i] = adaption_factor_old_[i] < max_adaption_factor_ ? adaption_factor_old_[i] : max_adaption_factor_;
                                }
                                adaption_factor[i] = scaleUpLinear_MD(adaption_factor_old_[i],scale_up_time_begin_MD_[i],scale_up_start_val_[i]);
                        // robot is in stable state and current input force is used
                        } else {
                                scale_down_MD_[i] = false;
                                scale_up_MD_[i] = false;
                                adaption_factor[i] = max_adaption_factor_;
                        }
                }
                
        }
        
        return adaption_factor;
}


std::array<double, 3> PassiveBehaviorControllerBase::calcCurrentAdaptionFactor_time(const ros::Time& time) {
        double integralValue = integral_ptr_->getIntegralValue();
        std::array<double, dim> adaption_factor;
        //Adapt scaled input if necessary
        //if robot is in unstable state, scale down
        if(integralValue < 0.0) {
                //check if start of scaling down time
                if(!scale_down_) {
                        //set parameters for scaling down
                        scale_up_ = false;
                        scale_down_ = true;
                        scale_down_time_begin_ = time;
                        // determine in which dimension robot is unstable
                        for(auto i = 0; i < dim; ++i) {
                                //TODO could cause problems because of setting max_adaption_factor_
                                scale_down_start_val_[i] = adaption_factor_old_[i];
                                adaption_factor[i] = scaleDownLinear_MD(desired_adaption_factor_[i], adaption_factor_old_[i], scale_down_time_begin_, scale_down_start_val_[i]);
                        }
                } else {
                        for(auto i = 0; i < dim; ++i) {
                                adaption_factor[i] = scaleDownLinear_MD(desired_adaption_factor_[i], adaption_factor_old_[i], scale_down_time_begin_, scale_down_start_val_[i]);
                        }
                }
                //robot is not unstable but adaptionfactor is less than max, so adaption factor gets scaled back to max
        } else if( (adaption_factor_old_[0] < max_adaption_factor_) || (adaption_factor_old_[1] < max_adaption_factor_) || (adaption_factor_old_[2] < max_adaption_factor_)) {
                // check if start of scaling up
                if(!scale_up_) {
                        //set parametes for scaling up
                        scale_down_=false;
                        scale_up_ = true;
                        scale_up_time_begin_ = time;
                        for(auto i = 0; i < dim; ++i) {
                                scale_up_start_val_[i] = adaption_factor_old_[i];
                                adaption_factor_old_[i] = adaption_factor_old_[i] < max_adaption_factor_ ? adaption_factor_old_[i] : max_adaption_factor_;
                                adaption_factor[i] = scaleUpLinear_MD(adaption_factor_old_[i],scale_up_time_begin_, scale_up_start_val_[i]);
                        }
                } else {
                        for(auto i = 0; i < dim; ++i) {
                                adaption_factor[i] = scaleUpLinear_MD(adaption_factor_old_[i],scale_up_time_begin_,scale_up_start_val_[i]);
                        }
                }
                // robot is in stable state and current input force is used
        } else {
                scale_down_ = false;
                scale_up_ = false;
                for(auto i = 0; i < DIM; ++i) {
                        adaption_factor[i] = max_adaption_factor_;
                }
        }
        return adaption_factor;
}

void PassiveBehaviorControllerBase::update(const ros::Time& time, const ros::Duration& period) {
        //get input
        std::array<double, 3> fts_input_raw = FTSBaseController::getFTSInput(time);
        std::array<double, 3> behaviorAdaptedForce = updateWithInputs(time, period, FTSBaseController::getScaledLimitedFTSInput(fts_input_raw), FTSBaseController::getTimeSinceReleasingRobot(time) );
        
        FTSBaseController::setForceInput(behaviorAdaptedForce);
        FTSBaseController::update(time, period); //send to BaseController

        //calculate Energy of system
        updateIntegrals(fts_input_raw, FTSBaseController::getOldVelocity(), time);
}

std::array<double, 3> PassiveBehaviorControllerBase::updateWithInputs( const ros::Time& time, const ros::Duration& period, std::array<double, 3> scaled_fts_input, double timeSinceRelease) {
        
        if (pub_real_input_force_->trylock()) {
            pub_real_input_force_->msg_.x = scaled_fts_input[0];
            pub_real_input_force_->msg_.y = scaled_fts_input[1];
            pub_real_input_force_->msg_.z = scaled_fts_input[2];
            pub_real_input_force_->unlockAndPublish();
        }
        
        std::array<double,3> behaviorAdaptedInput = scaled_fts_input;
        
        //RESET ROBOT IF STANDING STILL FOR TOO LONG
        if (timeSinceRelease == 0.0) {
                fresh_resetted_ = false;
        } else if (!fresh_resetted_) {
                if ( timeSinceRelease > 2.0) {
                        ROS_WARN("[PASS BHV] Robot standing for longer period, resetting controller!");
                        resetValues();
                }
        }
        
        if (detect_main_dimension_) {
                ROS_INFO_THROTTLE(1.0, "Detecting main dimension");
                if ( std::fabs(old_velocity_[1]) > std::fabs(old_velocity_[0]) ) {
                        control_dimensions_[0] = false;
                        control_dimensions_[1] = true;
                } else {
                        control_dimensions_[0] = true;
                        control_dimensions_[1] = false;
                }
        }
        
        for(auto i = 0; i < dim; ++i) {
                current_power_[i] = scaled_fts_input[i] * old_velocity_[i];
        }
        
        switch (usedMethod_) {
                case slidingSD: 
                        adaption_factor_old_ = calcCurrentAdaptionFactor_time(time);
                        behaviorAdaptedInput = adaptInputForce(adaption_factor_old_, scaled_fts_input);
                        break;
                case slidingMD: case chuy2007:
                        adaption_factor_old_ = calcCurrentAdaptionFactor_time_MD(time, current_power_);
                        behaviorAdaptedInput = adaptInputForce(adaption_factor_old_, scaled_fts_input);
                        break;
                default:
                        ROS_WARN_ONCE("[Passive Behavior] - Update called but no integral style active");
        }
        
        if (pub_adaption_factor_->trylock()) {
            pub_adaption_factor_->msg_.x = adaption_factor_old_[0];
            pub_adaption_factor_->msg_.y = adaption_factor_old_[1];
            pub_adaption_factor_->msg_.z = adaption_factor_old_[2];
            pub_adaption_factor_->unlockAndPublish();
        }
        
        lastloopTime_ = time;
        
        return behaviorAdaptedInput;
}

// Function to update values to call from inherited classes
void PassiveBehaviorControllerBase::updateBaseControllerValues( const ros::Time& time, std::array<double, 3> oldForce, std::array<double, 3> oldVelocity) {
        
        updateIntegrals(oldForce, oldVelocity, time);
}


void PassiveBehaviorControllerBase::stopping(const ros::Time& /*time*/) {

}

void PassiveBehaviorControllerBase::resetController() {
        resetValues();
        FTSBaseController::resetController();
}

void PassiveBehaviorControllerBase::reconfigureCallback(robotrainer_controllers::PassiveBehaviorControllerConfig &config, uint32_t level) {
        stopController();

        ROS_INFO("Dyn reconfigure passiveBehaviorCtrlBase");
        
        integral_window_size_ = config.integral_window_size;
        adapt_input_force_scale_[0] = config.adapt_inputForce_Xval_scale;
        adapt_input_force_scale_[1] = config.adapt_inputForce_Yval_scale;
        adapt_input_force_scale_[2] = config.adapt_inputForce_Tval_scale;
        min_adaption_factor_ = config.min_adaption;
        max_adaption_factor_ = config.max_adaption;
        system_const_ = config.system_const;
        robot_virt_mass_ = config.robot_virt_mass;
        eps_ = config.eps;
        scale_down_time_ = config.scale_down_time;
        scale_up_time_= config.scale_up_time;
        enable_integral_resetting_ = config.integral_use_reset;
        
        instable_threshold_value_[0] = config.instable_threshold_x;
        instable_threshold_value_[1] = config.instable_threshold_y;
        instable_threshold_value_[2] = config.instable_threshold_z;
        
        minimumStableDuration_ = config.minimum_stable_duration;
        extremelyInstableDuration_ = config.extremely_instable_duration;
        deactivateAllOnExtremeInstability_ = config.deactivate_all_dimensions_on_extreme_instability;
        resetCooldownAfterInstability_ = config.reset_cooldown_after_instability;
        directionWindowSize_ = config.direction_window_size;
       
        
        int methodToUse = config.detection_method;
        
                
        if (methodToUse == 1) {
                usedMethod_ = chuy2007;
                ROS_INFO("USING: Chuy2007Integral");
                conditionalInstabilityDetection_ = false;
                chuy_MD_ptr_.reset(new robotrainer_helper_types::ChuyIntegralMultiDim{});
        } else if (methodToUse == 2) {
                usedMethod_ = slidingSD;
                conditionalInstabilityDetection_ =false;
                integral_ptr_.reset(new robotrainer_helper_types::SlidingIntegral{integral_window_size_, enable_integral_resetting_});
                ROS_INFO("USING: SlidingIntegral_SingleDimension");
        } else if (methodToUse == 3) {
                usedMethod_ = slidingMD;
                integral_MD_ptr_.reset(new robotrainer_helper_types::SlidingIntegralMultiDim{integral_window_size_, enable_integral_resetting_});
                 conditionalInstabilityDetection_ = config.enableConditionalInstabilityDetection;
                 ROS_INFO_COND(conditionalInstabilityDetection_, "USING: SlidingIntegral_MultiDimension WITH conditionalDetection of instabilities");
                 ROS_INFO_COND(!conditionalInstabilityDetection_, "USING: Standard SlidingIntegral_MultiDimension");
        } else {
                usedMethod_ = none;
                conditionalInstabilityDetection_ = false;
                ROS_INFO("USING NO INTEGRAL - TURNING OFF!");
        }
        
        int controlDimensions = config.control_dimensions;
        if (controlDimensions == 0) {
                ROS_INFO("PassiveBehaviorControl active for [X] DIMENSION!");
                control_dimensions_ = {{true, false, false}};
                detect_main_dimension_ = false;
        } else if (controlDimensions == 1) {
                ROS_INFO("PassiveBehaviorControl active for [Y] DIMENSION!");
                control_dimensions_ = {{false, true, false}};
                detect_main_dimension_ = false;
        } else if (controlDimensions == 2) {
                ROS_INFO("PassiveBehaviorControl active for [X+Y] DIMENSIONS (NO TORQUE)!");
               control_dimensions_ = {{ true, true, false}};
               detect_main_dimension_ = false;
        } else if (controlDimensions == 3) {
                ROS_INFO("PassiveBehaviorControl active for DOMINANT DIMENSION (Switching between X and Y based on main direction)");
                control_dimensions_ = {{true, false, false}};
                detect_main_dimension_ = true;
        } else {
                ROS_INFO("PassiveBehaviorControl active for [ALL] DIMENSIONS!");
                control_dimensions_ = {{true, true, true}};
                detect_main_dimension_ = false;
        }
        
        
        resetController();

}

void PassiveBehaviorControllerBase::reset() {

}

}//namespace

PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::PassiveBehaviorControllerBase, controller_interface::ControllerBase)
