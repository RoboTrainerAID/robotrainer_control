#ifndef ROBOTRAINER_CONTROLLERS_FTS_ADAPTIVE_FORCE_CONTROLLER_H
#define ROBOTRAINER_CONTROLLERS_FTS_ADAPTIVE_FORCE_CONTROLLER_H


#include <robotrainer_controllers/fts_controller.h>
#include <robotrainer_controllers/passive_behavior_controller.h>

#include <leg_tracker/PersonMsg.h>
#include <leg_tracker/LegMsg.h>

#include <std_msgs/Float64.h>
#include <robotrainer_controllers/FTSAdaptiveForceControllerConfig.h>
#include <actionlib/client/simple_action_client.h>
#include <iirob_led/BlinkyAction.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>



namespace robotrainer_controllers {
        
        class FTSAdaptiveForceController : public FTSController {
                
        public:
                FTSAdaptiveForceController();
                virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
                virtual void update(const ros::Time& time, const ros::Duration& period);
                void reconfigureCallback(robotrainer_controllers::FTSAdaptiveForceControllerConfig &config, uint32_t level);
                ~FTSAdaptiveForceController() { delete chain_ptr_;};
                
        private:
                dynamic_reconfigure::Server<robotrainer_controllers::FTSAdaptiveForceControllerConfig>* fts_adaptive_dysrv_;                
                
                
                //enable/disable parameter changes on dynamic reconfigure service
                bool enableParameterChanges_;
                
                //loop time for various things
                ros::Time current_loop_time_;
                
                //for parametrization
                bool parametrization_active_;
                bool stepInitialized_; //set step to initialized
                bool stepActivated_; //set step to activated after intialization and user-gripping
                enum parametrizationStepTag_{ baseX, baseYLeft, baseYRight, baseRotLeft, baseRotRight, recordFeetDistance, adaptX, finished };
                parametrizationStepTag_ currentStep_;
                ros::Time last_gripped_time_;
                bool switchStepRequested_;
                ros::Time step_finished_time_;
                
                
                //to store average forces
                int averageForcesCount_;
                std::array<double, 3> averageForcesSum_;
                
                //to store last leg distances
                double leg_dist_avg_x_;
                //if legtracking was updated;
                bool legtrack_updated_;                
                
                int springConstant_x_;
                int springConstant_y_;
                int springConstant_rot_;
                
                double baseForce_minimumForce_x_;
                double baseForce_minimumForce_y_;
                double baseForce_minimumTorque_;
                
                
                // baseForceTest variables
                ros::Time baseForce_startingTime_;
                int baseForce_springConstant_;
                double baseForce_movingAverageTimeframe_;
                double baseForce_minimumForce_;
                
                std::list<double>  baseForce_travelledDistanceList_;
                double baseForce_holdingDistance_;
                
                int baseForce_stableForceCounter_;
                std::vector<double> baseForce_storeRawInput_;
                
                bool baseForce_allParamsStored_;
                //user derived force and velocity values (coming from parametrization step)
                std::array<double, 3> userParametrized_maxFT_;
                std::array<double, 3> userParametrized_maxVel_;
                
                //standard force and velocity values (to be able to reset)
                std::array<double, 3> standard_maxFT_;
                std::array<double, 3> standard_maxVel_;
                
                bool baseForce_testCompleted_;                
                bool baseForce_fts_offset_reset_;
                
                double travelledDistance_;
                ros::Time lastDistanceUpdate_;
                
                // for feet distance recording
                double footDistanceSum_;
                int storedFootDistances_;
                bool fwd_footDistanceRecorded_;
                bool bwd_footDistanceRecorded_;
                double fwd_medianFootDistance_;
                double bwd_medianFootDistance_;
                double footDistance_lastTravelledDist_;
                
                
                //second step
                ros::Time adaptX_startTime_;
                ros::Time adaptX_lastTestTime_;
                
                std::list<double>  adaptX_movingLegDistList_;
                std::list<double> adaptX_movementSpeedList_;
                double adaptX_testIntervallDuration_; // time in seconds after which a check for good parameters is conducted
                int adaptX_unchangedCtr_;
                double minLegDistance_;
                double maxLegDistance_;
                double adaptive_parametrization_adaptionRate_; // value from 1.0 to 3.0, how fast big changes affect the scale change (1.0 meaning linear rate)
                
                
                double returnForce_x_;
                double returnForce_y_;
                double returnForce_rot_;
                
                //passive_behavior_control
                bool use_passive_behavior_ctrlr_;
                PassiveBehaviorControllerBase *passive_behavior_ctrl_;
                
                // base x
                ros::Publisher pub_base_x_virtSpring_;
                ros::Publisher pub_base_x_currentDist_;
                ros::Publisher pub_base_x_averageDist_;
                ros::Publisher pub_base_x_currentRawForce_;
                ros::Publisher pub_base_x_resForce_;
                //base y left
                ros::Publisher pub_base_y_left_virtSpring_;
                ros::Publisher pub_base_y_left_currentDist_;
                ros::Publisher pub_base_y_left_averageDist_;
                ros::Publisher pub_base_y_left_currentRawForce_;
                ros::Publisher pub_base_y_left_resForce_;
                //base y right
                ros::Publisher pub_base_y_right_virtSpring_;
                ros::Publisher pub_base_y_right_currentDist_;
                ros::Publisher pub_base_y_right_averageDist_;
                ros::Publisher pub_base_y_right_currentRawForce_;
                ros::Publisher pub_base_y_right_resForce_;
                //base rot left
                ros::Publisher pub_base_rot_left_virtSpring_;
                ros::Publisher pub_base_rot_left_currentDist_;
                ros::Publisher pub_base_rot_left_averageDist_;
                ros::Publisher pub_base_rot_left_currentRawForce_;
                ros::Publisher pub_base_rot_left_resForce_;
                //base rot right
                ros::Publisher pub_base_rot_right_virtSpring_;
                ros::Publisher pub_base_rot_right_currentDist_;
                ros::Publisher pub_base_rot_right_averageDist_;
                ros::Publisher pub_base_rot_right_currentRawForce_;
                ros::Publisher pub_base_rot_right_resForce_;
                
                // leg tracking                
                ros::Publisher pub_legtrack_x_1_;
                ros::Publisher pub_legtrack_x_2_;
                ros::Publisher pub_legtrack_x_avg_;
                ros::Publisher pub_legtrack_y_1_;
                ros::Publisher pub_legtrack_y_2_;
                ros::Publisher pub_legtrack_y_avg_;
                ros::Publisher pub_legtrack_travelled_;
                ros::Publisher pub_legtrack_distDiff_;
                ros::Publisher pub_legtrack_timestamp_;
                ros::Subscriber sub_legtrack_;
                bool debug_enableLegTracking_;
                ros::Time debug_legTrackingStarted_;
                
                // adaptive X
                ros::Publisher pub_adaptive_scale_x_;
                ros::Publisher pub_adaptive_factor_min_;
                ros::Publisher pub_adaptive_factor_max_;
                ros::Publisher pub_adaptive_distDiff_;
                
                //general debug
                ros::Publisher pub_force_raw_;
                ros::Publisher pub_force_raw_lim_;
                ros::Publisher pub_force_adapt_lim_;
                ros::Publisher pub_force_adapt_unscaled_;
                ros::Publisher pub_force_adapt_base_scaled_;
                
                ros::Publisher pub_adaptive_scale_;
                ros::Publisher pub_adaptive_max_ft_;
                
                bool adaption_is_active_;
                std::array<double, 3> base_max_ft_;
                bool use_smooth_transition_; //for interpolation cutoff at lower than 100% velocity
                double transition_rate_;
                std::array<double, 3> force_scale_minvel_;
                std::array<double, 3> force_scale_maxvel_;
                
                // LED
                actionlib::SimpleActionClient<iirob_led::BlinkyAction> *led_ac_;
                ros::Publisher pub_input_force_for_led_;
                enum led_phase_{ unlocked, waitForInput, walkForward, walkBackwards, almostReturned, phaseFinished, stepAwayFromRobot, robotInAutomaticMovement, showForce,  showAdapted };
                led_phase_ currentLEDPhase_;

                //general LED goals:
                iirob_led::BlinkyGoal blinkyStartGreen_;
                iirob_led::BlinkyGoal blinkyPhaseYellow_;
                iirob_led::BlinkyGoal blinkyFinishedRed_;
                iirob_led::BlinkyGoal blinkyFreeMovementBlue_;
                iirob_led::BlinkyGoal blinkyStepAway_;
                // dimension dependant goals:
                iirob_led::BlinkyGoal blinkyStart_x_;
                iirob_led::BlinkyGoal blinkyStart_x_back_;
                iirob_led::BlinkyGoal blinkyStart_y_left_;
                iirob_led::BlinkyGoal blinkyStart_y_right_;
                iirob_led::BlinkyGoal blinkyStart_rot_left_;
                iirob_led::BlinkyGoal blinkyStart_rot_right_;
                //legDistance and adaptX goals:
                iirob_led::BlinkyGoal blinkyWalkForward_;
                iirob_led::BlinkyGoal blinkyWalkBackwards_;
                iirob_led::BlinkyGoal blinkyAlmostFinishedRed_;
                //Automatic movement
                iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_;
                iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_x_;
                iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_y_left_;
                iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_y_right_;
                iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_rot_left_;
                iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_rot_right_;
                
                //LED-Related
                bool sendLEDGoal_;
                double ledForceInput_;
                
                //functions
                void initParametrizationStep();
                void switchParametrizationStep();
                void updateTravelledDistance();
                void resetTravelledDistance();
                
                std::array<double, 3> baseForceTest(std::array<double, 3> fts_input_raw); // virtual spring test for various directions
                void resetBaseForceTest();
                
                std::array<double, 3> recordBaseFeetDistance(std::array<double, 3> fts_input_raw);
                void parametrizeAdaptForceX();
                
                std::array<double, 3> returnRobotToStartpoint();
                
                void adaptForceScale();
                void setBaseValues();
                void resetToBaseValues();
                
                void legTrackCallback(const leg_tracker::PersonMsg::ConstPtr& msg);
                
                std::array<double, 3> scaleBetweenValues(std::array<double, 3> minScaleFactor, std::array<double, 3> maxScaleFactor);
                
                void sendLEDForceTopics();
                void setLEDPhase(led_phase_ requestedPhase);
                void sendLEDOutput();
                void sendDebugTopicsParamBase(double travelledDist, double averageDist, double raw_input, double currentVirtSpringForce, double effectiveForce);
                void sendDebugTopicsParamAdaptX(double currentScale, double robotDistDiff);
                
                void resetController(void);
                
                void useUserParametrizedValues(bool enable);
        };
}
#endif
