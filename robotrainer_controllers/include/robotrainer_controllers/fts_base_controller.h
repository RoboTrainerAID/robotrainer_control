
#ifndef ROBOTRAINER_CONTROLLERS_FTS_BASE_CONTROLLER_H
#define ROBOTRAINER_CONTROLLERS_FTS_BASE_CONTROLLER_H

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
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>

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

namespace robotrainer_controllers {
        
        class FTSBaseController : public WheelControllerBase<GeomController<UndercarriageCtrl> > {
        
        public:
                FTSBaseController();
                virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
                virtual void starting(const ros::Time& time);
                virtual void update(const ros::Time& time, const ros::Duration& period);
                virtual void stopping(const ros::Time& /*time*/);
                ~FTSBaseController() { delete chain_ptr_;};

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
                                        ROS_INFO("configure all steers: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
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
                                        ROS_INFO("configure steer %d: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", (int)i, config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
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
                hardware_interface::ForceTorqueSensorHandle roboter_;

                bool running_;
                bool base_initialized_;
                
                bool enableCounterForce_;
                //Controller active modi
                std::array<bool, 3> all_inactive_;
                std::array<bool, 3> x_active_;
                std::array<bool, 3> y_active_;
                std::array<bool, 3> rot_active_ ;
                std::array<bool, 3>  x_y_active_;
                std::array<bool, 3> x_rot_active_;
                std::array<bool, 3> y_rot_active_;
                std::array<bool, 3> all_active_;
                
                //zero force vector to prevent the robot from moving in certain situations
                std::array<double, 3> zeroForce_;


                 /*____For Modalities____*/
                
                //modalitites parameters:
                enum modality_type{none, base_modalities, controller_modalities};
                modality_type modalities_used_;
                
                ros::ServiceServer service_server_; 
                
                
                // Modalities
                bool modalities_loaded_;
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
                ros::Publisher pub_wrench_lim_; //debug
                ros::Publisher pub_resulting_force_after_counterforce_;
                
                //counter force modality
                std::array<double,3> staticCounterForce_ = {{0.0, 0.0, 0.0}};
                //adaptable center of gravity modality
                double cog_x_;
                double cog_y_;
                bool adapt_center_of_gravity_;
                //counterforce area subscriber
                ros::Subscriber counterforce_area_sub_;
                std::array<double,3> areaCounterForce_ = {{0.0, 0.0, 0.0}};
                double counterforce_distance_to_center_;
                double begin_scaledown_at_this_dist_;
                bool apply_areal_counterforce_;
                
                //modalities functions
                
                bool createModalityInstances();
                bool createControllerModalityInstances();
                bool createBaseModalityInstances();
                
                bool configureModalities();
                bool configureBaseModalities();
                bool configureControllerModalities();
                bool configureModalitiesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
                std::array<double, 3> applyCounterforce( std::array<double, 3> scaledInputForce, std::array<double, 3> scaledCounterforce );
                std::array<double, 3> applyGlobalCounterforce( std::array<double, 3> scaledInputForce );
                std::array<double, 3> applyAreaCounterforce(std::array<double, 3> scaledInputForce);
                std::array<double, 3> adaptCenterOfGravity(std::array<double, 3> fts_input_raw);
                
                std::array<double, 3> applyModalities( std::array<double, 3> base_vel );
                
                void counterforce_area_callback( const std_msgs::Float64::ConstPtr& msg );
                
                
                
                //dynamic reconfigure
                dynamic_reconfigure::Server<robotrainer_controllers ::FTSBaseControllerConfig>* base_dysrv_;
                ros::ServiceClient dysrv_callback_service_;
                bool base_reconfigured_flag_;

                // Input parameters
                bool yReversed_, rotReversed_;
                double controllerUpdateRate_;
                double backwardsMaxForceScale_, backwardsMaxVelScale_;

                std::array<bool, 3> use_controller_;
                std::array<double, 3> min_ft_;
                std::array<double, 3> max_ft_;
                std::array<double, 3> max_vel_;

                //admittanceParameters
                std::array<double, 3> gain_;
                std::array<double, 3> time_const_;

                // Controller parameters
                std::array<double, 3> a1_; //in order [x], [y], [torque]
                std::array<double, 3> b1_; //in order [x], [y], [torque]

                ros::Publisher pub_cmd_; 

                boost::mutex mutex_;
                
                //iput for used for controlling in update()
                std::array<double,3> force_input_;

                std::array<double, 3> force_old_;
                std::array<double, 3> velocity_old_;
                
                
                double delta_vel_;
                
                bool simulate_;
                ros::Publisher pub_res_vel_;
                ros::Publisher pub_force_lim_;
                
                //to detect no user gripping
                int noGripTicks_;
                bool userIsGripping_;
                ros::Time timeSinceReleasing_;
                
                //for fts recalibration
                ros::ServiceClient fts_client_;
                
                
                //debug LED
                bool debug_led_on_;
                

                //controller functions
                void resetController(void);
                void stopController(void);               
                
                
                void discretizeController();
                void discretizeWithNewParameters( std::array<double,3> time_const_for_control, std::array<double,3> gain_for_control);

                
                void reconfigureCallback(robotrainer_controllers::FTSBaseControllerConfig &config, uint32_t level);
                
                //getter functions
                std::array<double, 3> getScaledLimitedFTSInput( std::array<double, 3> rawFTSInput );
                std::array<double, 3> getFTSInput( const ros::Time& time );
                std::array<double, 3> getFTSInputIfUserGrips( const ros::Time& time );
                double getTimeSinceReleasingRobot( const ros::Time& time );
                std::array<double, 3> getOldForce();
                std::array<double, 3> getOldForcePercent();
                std::array<double, 3> getOldVelocity();
                std::array<double, 3> getOldVelocityPercent();
                std::array<double, 3> getTimeConst();
                std::array<double, 3> getGain();
                std::array<double, 3> getMaxFt();
                std::array<double, 3> getMaxVel();
                bool userIsGripping();
                bool robotIsMovingForward();
                
                bool recalculateFTSOffsets();
                
                //setter functions
                void setForceInput(std::array<double,3> force_input_for_control);
                void setMaxVel(std::array<double,3> max_vel_for_control);
                void setMaxFt(std::array<double,3> max_ft_for_control);
                void setActiveDimensions(std::array<bool, 3> enable_controller_dimension);
                
                //helper functions
                geometry_msgs::Vector3 convertToMessage(std::array<double, 3> input);
                void untickDynamicReconfigureParam(std::string parameter_name);
        };

}
#endif
