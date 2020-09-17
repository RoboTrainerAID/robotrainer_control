#ifndef ROBOTRAINER_CONTROLLERS_FTS_ADAPTIVE_FORCE_CONTROLLER_H
#define ROBOTRAINER_CONTROLLERS_FTS_ADAPTIVE_FORCE_CONTROLLER_H

#include <numeric>

#include <robotrainer_controllers/fts_controller.h>
#include <robotrainer_controllers/passive_behavior_controller.h>

#include <leg_tracker/PersonMsg.h>
#include <leg_tracker/LegMsg.h>

#include <std_msgs/Float64.h>
#include <robotrainer_controllers/FTSAdaptiveForceControllerConfig.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>

namespace robotrainer_controllers {

class FTSAdaptiveForceController : public FTSController {

public:
    FTSAdaptiveForceController() = default;

    ~FTSAdaptiveForceController() = default;
    
    virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);

    virtual void update(const ros::Time& time, const ros::Duration& period);
    void reconfigureCallback(robotrainer_controllers::FTSAdaptiveForceControllerConfig &config, uint32_t level);


protected:
    // Getters and setters to protect concurency of multiple threads
    bool getLegTrackUpdated();
    bool getSwitchStepRequested();

    void setLegTrackUpdated(const bool value);
    void setSwitchStepRequested(const bool value);

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

    bool autonomously_returning_ = false;
    double autonomously_needed_time_sec_;
    ros::Time autonomously_start_time_;
    ros::Duration autonomously_traveled_time_;
    // not constant, just for reserving memory on initializaton
    double autonomously_returning_velocity;


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

    // publishers for parametrization
    realtime_tools::RealtimePublisher<geometry_msgs::Twist> *pub_base_virtSpring_, *pub_base_currentDist_, *pub_base_averageDist_, *pub_base_currentRawForce_, *pub_base_resForce_;

    // leg tracking
    ros::Subscriber sub_legtrack_;
    bool debug_enableLegTracking_;
    ros::Time debug_legTrackingStarted_;

    // adaptive X
    realtime_tools::RealtimePublisher<std_msgs::Float64> *pub_adaptive_scale_x_, *pub_adaptive_factor_min_, *pub_adaptive_factor_max_, *pub_adaptive_distDiff_;

    //general debug
    realtime_tools::RealtimePublisher<geometry_msgs::Vector3> *pub_force_adapt_limited_input_, *pub_adaptive_scale_, *pub_adaptive_max_ft_;

    bool adaption_is_active_;
    std::array<double, 3> base_max_ft_;
    bool use_smooth_transition_; //for interpolation cutoff at lower than 100% velocity
    double transition_rate_;
    std::array<double, 3> force_scale_minvel_;
    std::array<double, 3> force_scale_maxvel_;

    boost::mutex leg_track_update_mutex_;
    boost::shared_mutex switch_step_update_mutex_;

    //functions
    void initParametrizationStep();
    void switchParametrizationStep();
    void updateTravelledDistance();
    void resetTravelledDistance();


    std::array<double, 3> baseForceTest(std::array<double, 3> fts_input_raw); // virtual spring test for various directions
    void resetBaseForceTest();

    std::array<double, 3> recordBaseFeetDistance(std::array<double, 3> fts_input_raw);
    void parametrizeAdaptForceX();

    void returnRobotAutonomouslyToStartPosition(bool finished_successfully);

    void adaptForceScale();
    void setBaseValues();
    void resetToBaseValues();

    void legTrackCallback(const leg_tracker::PersonMsg::ConstPtr& msg);

    std::array<double, 3> scaleBetweenValues(std::array<double, 3> minScaleFactor, std::array<double, 3> maxScaleFactor);

    void forceInputToLed(const geometry_msgs::WrenchStamped force_input);
    void sendLEDForceTopics();
    bool setLEDPhase(controller_led_phases requestedPhase);
    void sendDebugTopicsParamBase(double travelledDist, double averageDist, double raw_input, double currentVirtSpringForce, double effectiveForce);
    void sendDebugTopicsParamAdaptX(double currentScale, double robotDistDiff);

    void resetController(void);

    void useUserParametrizedValues(bool enable);
};

}  // robotrainer_controllers
#endif
