
#ifndef ROBOTRAINER_CONTROLLERS_FTS_BASE_CONTROLLER_H
#define ROBOTRAINER_CONTROLLERS_FTS_BASE_CONTROLLER_H

#include <memory>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <iirob_led/BlinkyAction.h>
#include "robotrainer_controllers/fts_controllers_led_defines.hpp"

#include "robotrainer_controllers/fts_wheelControllerBase.h"
#include "robotrainer_controllers/fts_GeomController.h"
#include "robotrainer_controllers/FTSBaseControllerConfig.h"

#include "robotrainer_modalities/VirtualForcesConfig.h"
#include "robotrainer_modalities/modality_base.h"
#include "robotrainer_modalities/virtual_forces.h"
#include "robotrainer_modalities/modalities_controller_base.h"

#include "robotrainer_helper_types/Integral.h"
#include "robotrainer_helper_types/wrench_twist_datatype.h"

#include <ros/ros.h>
#include <filters/filter_base.h>
#include <filters/filter_chain.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
//FIXME: remove this tf stuff?
#include <tf/transform_datatypes.h>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <force_torque_sensor/force_torque_sensor_handle.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <cob_omni_drive_controller/SteerCtrlConfig.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "robotrainer_controllers/undercarriage_drive_mode_service.h"
#include <realtime_tools/realtime_publisher.h>

// TODO(GLOBAL):  use smart pointers

namespace robotrainer_controllers {
    
enum class drive_mode_type : std::uint8_t
{
    NONE = 0,
    OMNIDIRECTIONAL = 1, 
    DIFFERENTIAL = 2,
    DIFFERENTIAL_MATRICES = 21,
    ACKERMANN = 4,
    ACKERMANN_SMOOTHER = 41,
};

// template<typename T>
// std::array<T> std::array<T>::operator+ (T value) {
//     for (T element : this) {
//         element += value;
//     }
// }

// template<typename T>
// std::array<T> operator= (T value) {
//     for (T element : this
// }

// template<typename T, uint SIZE>
// class AxisValues
// {
// private:
//     std::array<T, SIZE> values_;
// public:
//     AxisValues() = default;
//     AxisValues(std::array<T, SIZE> input)
//     {
//         values_ = input;
//     }
//     explicit AxisValues(uint size): values_(size) {};
// 
//     T& operator[](const std::size_t index)
//     {
//         return values_[index];
//     }
// 
//     AxisValues& operator=(const T value)
//     {
//         for (uint i=0; i < values_.size(); i++) {
//             values_[i] = value;
//         }
//     }
// 
//     AxisValues& operator=(const std::array<T, SIZE> value)
//     {
//         values_ = value;
//     }
// 
//    AxisValues& operator=(const AxisValues<T, SIZE> value)
//     {
//         values_ = std::array<T, SIZE>(value);
//     }
// 
//     AxisValues& operator+(const T value)
//     {
//         for (uint i=0; i < values_.size(); i++) {
//             values_[i] += value;
//         }
//     }
// 
//     AxisValues& operator+(const std::array<T, SIZE> & value)
//     {
//         for (uint i=0; i < values_.size(); i++) {
//             values_[i] += value[i];
//         }
//     }
// 
//     AxisValues& operator*(const T value)
//     {
//         for (uint i=0; i < values_.size(); i++) {
//             values_[i] *= value;
//             ROS_FATAL("In multiplicaiotn %f", values_[i]);
//         }
//     }
// 
// //     AxisValues<T, SIZE>& operator*(const int value)
// //     {
// //         for (uint i=0; i < values_.size(); i++) {
// //             values_[i] *= value;
// //         }
// //     }
// //
// //     AxisValues<double, SIZE>& operator*(const double value)
// //     {
// //         AxisValues<double, SIZE> result;
// //         for (uint i=0; i < values_.size(); i++) {
// //             result[i] = values_[i] * value;
// //         }
// //         return result;
// //     }
// 
//     AxisValues& operator*(const std::array<T, SIZE> & value)
//     {
//         for (uint i=0; i < values_.size(); i++) {
//             values_[i] *= value[i];
//         }
//     }
// 
//     operator std::string() const
//     {
//         std::string string;
//         string = "AxisValues: ";
// 
//         for (uint i=0; i < values_.size(); i++) {
//             string += std::string(i) + ": " + std::string((T)values_[i]) + "; ";
//         }
//         return string;
//     }
// 
//     operator std::array<T, SIZE>() const
//     {
//         return values_;
//     }
// 
//     uint size()
//     {
//         return values_.size();
//     }
// };


class FTSBaseController : public WheelControllerBase<GeomController<UndercarriageCtrl> > {

public:
    FTSBaseController() = default;
    
    ~FTSBaseController() { delete chain_ptr_;};
    
    virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
    
    virtual void starting(const ros::Time& time);
    
    virtual void update(const ros::Time& time, const ros::Duration& period);
    
    virtual void stopping(const ros::Time& /*time*/);

    class PosCtrl {
        public:
                PosCtrl() : updated(false) {}
                void try_configure(UndercarriageCtrl &ctrl) {
                        boost::recursive_mutex::scoped_try_lock lock(mutex);
                        if(lock && updated){
                                ctrl.configure(pos_ctrl_params);
                                updated = false;
                        }
                }
                void init(const wheel_params_t &params, const ros::NodeHandle &nh){
                        boost::recursive_mutex::scoped_lock lock(mutex);
                        pos_ctrl_params.resize(params.size());
                        reconfigure_server_axes_.clear();
                        reconfigure_server_.reset( new dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig>(mutex,ros::NodeHandle(nh, "default/steer_ctrl")));
                        reconfigure_server_->setCallback(boost::bind(&PosCtrl::setForAll, this, _1, _2)); // this writes the default into pos_ctrl_params
                        {
                                cob_omni_drive_controller::SteerCtrlConfig config;
                                copy(config, params.front().pos_ctrl);
                                reconfigure_server_->setConfigDefault(config);
                        }

                        for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                                boost::shared_ptr<dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > dr(new dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig>(mutex, ros::NodeHandle(nh, params[i].geom.steer_name)));
                                cob_omni_drive_controller::SteerCtrlConfig config;
                                copy(config, params[i].pos_ctrl);
                                dr->setConfigDefault(config);
                                dr->updateConfig(config);
                                dr->setCallback(boost::bind(&PosCtrl::setForOne, this, i, _1, _2));  // this writes the joint-specific config into pos_ctrl_params
                                reconfigure_server_axes_.push_back(dr);
                        }
                }
        private:
                static void copy(PosCtrlParams &params, const cob_omni_drive_controller::SteerCtrlConfig &config){
                        params.dSpring = config.spring;
                        params.dDamp = config.damp;
                        params.dVirtM = config.virt_mass;
                        params.dDPhiMax = config.d_phi_max;
                        params.dDDPhiMax = config.dd_phi_max;
                }
                static void copy(cob_omni_drive_controller::SteerCtrlConfig &config, const PosCtrlParams &params){
                        config.spring = params.dSpring;
                        config.damp = params.dDamp;
                        config.virt_mass = params.dVirtM;
                        config.d_phi_max = params.dDPhiMax;
                        config.dd_phi_max = params.dDDPhiMax;
                }
                // these function don't get locked
                void setForAll(cob_omni_drive_controller::SteerCtrlConfig &config, uint32_t /*level*/) {
//                                         ROS_INFO("configure all steers: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
                        for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                                copy(pos_ctrl_params[i], config);
                                if(!reconfigure_server_axes_.empty()){
                                reconfigure_server_axes_[i]->setConfigDefault(config);
                                reconfigure_server_axes_[i]->updateConfig(config);
                                }
                        }
                        updated = true;
                }
                void setForOne(size_t i, cob_omni_drive_controller::SteerCtrlConfig &config, uint32_t /*level*/) {
//                                         ROS_INFO("configure steer %d: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", (int)i, config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
                        copy(pos_ctrl_params[i], config);
                        updated = true;
                }
                std::vector<PosCtrlParams> pos_ctrl_params;
                boost::recursive_mutex mutex; // dynamic_reconfigure::Server calls the callback from the setCallback function
                bool updated;
                boost::scoped_ptr< dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > reconfigure_server_;
                std::vector<boost::shared_ptr< dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > > reconfigure_server_axes_;
    } pos_ctrl_;

protected:
    hardware_interface::ForceTorqueSensorHandle hw_fts_;

    wheel_params_t wheel_params_;
    ros::NodeHandle ctrl_nh_, wheel_ctrl_nh_;

    bool base_initialized_;

    bool enableCounterForce_;
    //Controller active modi
    std::array<bool, 3> all_inactive_;
    std::array<bool, 3> x_active_;
    std::array<bool, 3> y_active_;
    std::array<bool, 3> rot_active_ ;
    std::array<bool, 3> x_y_active_;
    std::array<bool, 3> x_rot_active_;
    std::array<bool, 3> y_rot_active_;
    std::array<bool, 3> all_active_;

    //zero force vector to prevent the robot from moving in certain situations
    std::array<double, 3> zeroForce_;

    ros::ServiceServer use_twist_input_srv_;
    ros::ServiceServer update_kinematics_service_srv_;

        /*____For Modalities____*/

    //modalitites parameters:
    enum modality_type{none, base_modalities, controller_modalities};
    modality_type modalities_used_;
    const std::vector<std::string> modality_type_names_ = {"none", "base", "controller"};

    ros::ServiceServer configure_modalities_srv_;

    // Modalities
    bool modalities_loaded_;
    std::shared_ptr<pluginlib::ClassLoader<
      robotrainer_modalities::ModalityBase<geometry_msgs::Twist>>> modalities_loader_;
    std::shared_ptr<pluginlib::ClassLoader<
      robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>>> modality_controllers_loader_;
    //pointers to base_modalities
    boost::shared_ptr<robotrainer_modalities::ModalityBase<geometry_msgs::Twist>> force_modality_ptr_;
    boost::shared_ptr<robotrainer_modalities::ModalityBase<geometry_msgs::Twist>> walls_modality_ptr_;
    boost::shared_ptr<robotrainer_modalities::ModalityBase<geometry_msgs::Twist>> pathtracking_modality_ptr_;
    boost::shared_ptr<robotrainer_modalities::ModalityBase<geometry_msgs::Twist>> area_modality_ptr_;

    //pointers to controller_modalities
    boost::shared_ptr<robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>> force_controller_modality_ptr_;
    boost::shared_ptr<robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>> walls_controller_modality_ptr_;
    boost::shared_ptr<robotrainer_modalities::ModalitiesControllerBase<robotrainer_helper_types::wrench_twist>> pathtracking_controller_modality_ptr_;

    // Modality Controllers
    bool modalities_configured_;


    filters::FilterChain<geometry_msgs::Twist>* chain_ptr_ = new filters::FilterChain<geometry_msgs::Twist>("geometry_msgs::Twist"); //Node template type and argument must match
    bool modalities_chain_configured_;
    //debug
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> 
        pub_platform_hw_velocity_,
        pub_admittance_velocity_,
        pub_final_velocity_;

    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>>
        pub_force_input_raw_,
        pub_force_input_scaled_limited_,
        pub_resulting_force_after_counterforce_,
        pub_resulting_force_after_cor_,
        pub_input_force_force_for_gui_;

    // LEDS
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>>
        pub_input_force_for_led_;
    std::shared_ptr<actionlib::SimpleActionClient<iirob_led::BlinkyAction>> led_ac_;

    controller_led_phases led_phase_, currentLEDPhase_ = controller_led_phases::UNDEFINED;
    LedBlinkyGoals blinky_goals_;
    bool sendLEDGoal_;
    double ledForceInput_;
    iirob_led::BlinkyGoal ledGoalToSend_;

    //debug LED
    bool debug_led_on_;

    // TODO: These here should be private... and moved to the global CA implementation
    //counter force modality
    std::array<double,3> staticCounterForce_ = {{0.0, 0.0, 0.0}};
    //adaptable center of gravity modality
    double cor_x_;
    double cor_y_;
    bool adapt_center_of_rotation_;
    //counterforce area subscriber
    ros::Subscriber counterforce_area_sub_;
    std::array<double,3> areaCounterForce_ = {{0.0, 0.0, 0.0}};
    double counterforce_distance_to_center_;
    double begin_scaledown_at_this_dist_;
    bool apply_areal_counterforce_;

    //modalities functions

    // TODO(denis): optimize this many functions here
    bool loadModalityInstances();
    bool loadControllerModalityInstances();
    bool loadBaseModalityInstances();

    bool configureModalities();
    bool configureBaseModalities();
    bool configureControllerModalities();
    bool setUseTwistInputCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool updateWheelParamsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool configureModalitiesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    std::array<double, 3> applyCounterforce( std::array<double, 3> scaledInputForce, std::array<double, 3> scaledCounterforce );
    std::array<double, 3> applyGlobalCounterforce( std::array<double, 3> scaledInputForce );
    std::array<double, 3> applyAreaCounterforce(std::array<double, 3> scaledInputForce);
    std::array<double, 3> adaptCenterOfRotation(std::array<double, 3> fts_input_raw);

    std::array<double, 3> applyModalities(const std::array<double, 3> & base_vel, 
                                          const std::array<double, 3> & force_input);

    void counterforce_area_callback( const std_msgs::Float64::ConstPtr& msg );

    //dynamic reconfigure
    dynamic_reconfigure::Server<robotrainer_controllers ::FTSBaseControllerConfig>* base_dysrv_;
    ros::ServiceClient dysrv_callback_service_;
    bool base_reconfigured_flag_;

    // Basic parameters
    bool no_hw_output_;
    bool use_twist_input_;
    // TODO: rename this to use proper casing
    double controllerUpdateRate_;
    std::string controllerFrameId_;

    // Input parameters
    bool yReversed_, rotReversed_;
    double backwardsMaxForceScale_, backwardsMaxVelScale_;
    double reversedMaxForceScale_, reversedMaxVelScale_;

    std::array<bool, 3> use_controller_;
    std::array<double, 3> min_ft_;
    std::array<double, 3> max_ft_;
    std::array<double, 3> max_vel_;
    std::array<double, 3> default_min_ft_;
    std::array<double, 3> default_max_ft_;
    std::array<double, 3> default_max_vel_;

    //admittanceParameters
    std::array<double, 3> gain_;
    std::array<double, 3> time_const_;

    // Controller parameters
    std::array<double, 3> a1_; //in order [x], [y], [torque]
    std::array<double, 3> b1_; //in order [x], [y], [torque]

    //input for used for controlling in update()
    std::array<double,3> force_input_;

    std::array<double, 3> force_old_;
    std::array<double, 3> velocity_old_;

    // real state of the platform
    PlatformState platform_state_;


    double delta_vel_;
    
    geometry_msgs::WrenchStamped forceInputForLED_;
    
    void forceInputToLed(const geometry_msgs::WrenchStamped force_input);
    void sendLEDForceTopics();
    bool setLEDPhase(controller_led_phases requestPhase);
    void sendLEDOutput();

    //controller functions
    void stopController(void);
    void restartController(void);
    void resetController(void);
    void resetControllerNew(void);
    void restartControllerAndOrientWheels(std::array<double,3>);
    void discretizeController();
    void discretizeWithNewParameters( std::array<double,3> time_const_for_control, std::array<double,3> gain_for_control);
    void discretizeWithNewMassDamping( std::array<double,3> virtual_mass, std::array<double,3> virtual_damping);


    void reconfigureCallback(robotrainer_controllers::FTSBaseControllerConfig &config, uint32_t level);
    
    double calculatevirtualdamping(double maxforce, double maxvelocity, double gain);
    double calculatevirtualmass(double timeconstant, double damping);

    //getter functions
    std::array<double, 3> getScaledLimitedFTSInput(std::array<double, 3> rawFTSInput);
    std::array<double, 3> getFTSInput(const ros::Time& time);
//     std::array<double, 3> getFTSInputIfUserGrips( const ros::Time& time );
    double getTimeSinceReleasingRobot(const ros::Time& time);
    std::array<double, 3> getOldForce();
    std::array<double, 3> getOldForcePercent();
    std::array<double, 3> getOldVelocity();
    std::array<double, 3> getOldVelocityPercent();
    std::array<double, 3> getVelocity();
    std::array<double, 3> getTimeConst();
    std::array<double, 3> getGain();
    std::array<double, 3> getDamping();
    std::array<double, 3> getMass();
    std::array<double, 3> getMaxFt();
    std::array<double, 3> getMaxVel();
    bool robotIsMovingForward(void);

    bool recalculateFTSOffsets(void);

    //setter functions
    void setForceInput(std::array<double,3> force_input_for_control);
    void setMaxFt(std::array<double,3> max_ft_for_control);
    void setMaxFtAxis(uint axis, double max_ft_for_control);
    void setMaxVel(std::array<double,3> max_vel_for_control);
    void setMaxVelAxis(uint axis, double max_vel_for_control);
    void setActiveDimensions(std::array<bool, 3> enable_controller_dimension);
    std::string setUseTwistInput(bool use_twist_input);

    //helper functions
    geometry_msgs::Vector3 convertToMessage(std::array<double, 3> input);
    void convertToWrenchAndPublish(
      const ros::Time & time,
      const std::array<double,3> & input_vec,
      std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> & wrench_pub);

    void convertToTwistAndPublish(
      const ros::Time & time,
      const std::array<double, 3> & input_vec,
      std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> & twist_pub);
    
    void untickDynamicReconfigureParam(std::string parameter_name);

    // Getters and setters to protect concurency of multiple threads
    bool userIsGripping(void);

    void setUserIsGripping(bool value);


protected:
    void protectedToggleControllerRunning(const bool value, const std::string locking_number);

    void updateRobotState();

    bool debug_;
    UndercarriageDriveMode * ucdm_;

    ucdm_cmd::Request drive_mode_request_;
    ucdm_cmd::Response drive_mode_response_;

    bool controller_started_;

private:
    boost::mutex controller_internal_states_mutex_;
    boost::mutex locking_mutex_;
    boost::shared_mutex running_mutex_, can_be_running_mutex_;
    boost::shared_mutex user_is_gripping_mutex_;

    // internal state variables
    bool internal_state_updated_ = false;
    double platfrom_velocity_ = 0;
    double platform_linear_vel_ = 0;
    bool platform_is_moving_ = false;

    // Update function variables
    bool running_;
    bool can_be_running_;

    diagnostic_updater::Updater diagnostic_;
    void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

    // Modalities control variables

    // Drive modes
    drive_mode_type drive_mode_used_;
    const std::map<drive_mode_type, std::string> drive_mode_type_names_ = 
    {
        {drive_mode_type::NONE, "none"}, 
        {drive_mode_type::OMNIDIRECTIONAL, "omnidirectional"}, 
        {drive_mode_type::DIFFERENTIAL, "differential"},
        {drive_mode_type::DIFFERENTIAL_MATRICES, "differential_matrices"},
        {drive_mode_type::ACKERMANN, "ackermann"},
        {drive_mode_type::ACKERMANN_SMOOTHER, "ackermann_smoother"},
    };

    // config variables
    int orient_wheels_;
    std::array<double, 3> orient_wheels_vel_;

    //to detect no user gripping
    int noGripTicks_;
    bool userIsGripping_;
    ros::Time timeSinceReleasing_;

    //for fts recalibration
    ros::ServiceClient fts_client_;

    // Controller running state control
    const std::string LOCKING_NONE = "--none--";
    std::string locking_string_ = LOCKING_NONE;
    void startController(void);

    // Protected change of controller internals
    void setOrientWheels(std::array<double,3> direction);
    bool unsafeRecalculateFTSOffsets();
    void internalSetNoHWOutput(bool no_hw_output);
    std::string internalSetUseTwistInput(bool use_twist_input);

    // Mutex-Protected getters and setters
    bool getRunning();
    bool getCanBeRunning();

    void setRunning(bool value);
    void setCanBeRunning(bool value);
    
    ros::Time test_begin_time_;
    bool test_started_ = false;
};

}  // namespce robotrainer_controllers
#endif  // ROBOTRAINER_CONTROLLERS_FTS_BASE_CONTROLLER_H
