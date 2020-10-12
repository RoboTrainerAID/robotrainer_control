#ifndef ROBOTRAINER_MODALITIES_VF_CONTROLLER_H
#define ROBOTRAINER_MODALITIES_VF_CONTROLLER_H

#include "robotrainer_modalities/modalities_controller_base.h"
#include "robotrainer_modalities/ModalitiesVFControllerParameters.h"
#include "robotrainer_modalities/ModalitiesVFControllerConfig.h"

#include "robotrainer_helper_types/Integral.h"
#include "robotrainer_helper_types/wrench_twist_datatype.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <cmath>
namespace robotrainer_modalities {
        
template <typename T> class ModalitiesVFController : public robotrainer_modalities::ModalitiesControllerBase<T>{
        
public:
        ModalitiesVFController();
        virtual bool update(const T& data_in, T& data_out);
        virtual bool configure();

        tf2::Vector3 getLastVelocity();
        tf2::Vector3 getLastscaledForce();

         ~ModalitiesVFController(){};

protected:
        ros::NodeHandle nh_;
//         ros::Publisher test_pub_;
//         ros::Publisher HF_HV_Energy_pub_;
//         ros::Publisher VF_MV_Energy_pub_;
//         
//         geometry_msgs::Vector3 test_msg_;
//         std_msgs::Float64 hf_hv_energy_msg_;
//         std_msgs::Float64 vf_mv_energy_msg_;
        
        boost::shared_ptr<ModalityBase<geometry_msgs::Twist>> vf_modalitie_ptr_;
        bool use_modalities_VF_controller_;
        bool limit_modalities_vel_norm_;
        
        robotrainer_helper_types::wrench_twist hf_mv_msg_;
        robotrainer_helper_types::wrench_twist vf_mv_msg_;
        
        boost::shared_ptr<robotrainer_helper_types::SlidingIntegral> integral_HF_HV_ptr_{new robotrainer_helper_types::SlidingIntegral{}};
        boost::shared_ptr<robotrainer_helper_types::SlidingIntegral> integral_HF_MV_ptr_{new robotrainer_helper_types::SlidingIntegral{}};
        boost::shared_ptr<robotrainer_helper_types::SlidingIntegral> integral_VF_MV_ptr_{new robotrainer_helper_types::SlidingIntegral{}};
        int integral_HF_MV_window_size_;
        int integral_VF_MV_window_size_;
        
        double max_abs_angle_;
        double forgetting_factor_;
        double vel_limit_factor_ = 1.0;
        double eps_;
//      bool human_let_robot_go_ = false;
        const double rad_to_deg_const_ = 180.0/M_PI;
                                
        //FUNCTIONS
//      bool humanEndangered(const robotrainer_helper_types::wrench_twist& hf_mv_msg, const robotrainer_helper_types::wrench_twist& vf_mv_msg);
        bool checkHumanLetRobotGo(const tf2::Vector3& human_input_force);
        bool humanEndangeredForce();
        bool humanEndangeredVelocity();
        void setForceLimitForModalitie(const tf2::Vector3& human_input_force);
        void unsetForceLimitForModalitie();
        double calcVelLimitationFactor(const tf2::Vector3& human_vel_lin, const tf2::Vector3& modalities_vel_lin);
        double scaleVelFactorBack(const double& vel_limit_factor_goal);
        
//      geometry_msgs::Twist limitVirtualForces(const robotrainer_helper_types::wrench_twist& hf_mv_msg, const robotrainer_helper_types::wrench_twist& vf_mv_msg);
geometry_msgs::Twist limitModalitiesVelocity(geometry_msgs::Twist input_twist);
                
private:
        const tf2::Vector3 null_vec_{0.0,0.0,0.0};
        const tf2::Vector3 minus_unit_x_vec_{-1.0,0.0,0.0};
        // Objects for param_handler and dynamic_reconfigure
        robotrainer_modalities::ModalitiesVFControllerParameters params_;
        dynamic_reconfigure::Server<robotrainer_modalities::ModalitiesVFControllerConfig> dynamic_reconfigure_server_;
        
        //FUNCTIONS
        void reconfigureRequest(const robotrainer_modalities::ModalitiesVFControllerConfig& config, uint32_t level);

};

}
#endif
