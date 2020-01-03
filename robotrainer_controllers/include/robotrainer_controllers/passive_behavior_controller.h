#ifndef _ROBOTRAINER_CONTROLLERS_PASSIVE_BEHAVIOR_CONTROLLER_BASE_H
#define _ROBOTRAINER_CONTROLLERS_PASSIVE_BEHAVIOR_CONTROLLER_BASE_H

#include "robotrainer_controllers/fts_base_controller.h"
#include "robotrainer_controllers/PassiveBehaviorControllerConfig.h"

#include "robotrainer_helper_types/Integral.h"

#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>

namespace robotrainer_controllers
{
class PassiveBehaviorControllerBase : public FTSBaseController {
public:
        PassiveBehaviorControllerBase();

        virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
        virtual void starting(const ros::Time& time);
        virtual void update(const ros::Time& time, const ros::Duration& period);
        
        virtual void stopping(const ros::Time& time);

        void reconfigureCallback(robotrainer_controllers::PassiveBehaviorControllerConfig &config, uint32_t level);
        void reset();
        
        //for use in chained mode (to process passive behavior from another controller)
        virtual void initInChain(ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
        void setValuesFromBaseController(std::array<double, 3> gain, std::array<double, 3> time_const);
        std::array<double, 3> updateWithInputs( const ros::Time& time, const ros::Duration& period, std::array<double, 3> ftsInput, double timeSinceRelease );
        // Function to update values to call from inherited classes
        void updateBaseControllerValues( const ros::Time& time, std::array<double, 3> oldForce, std::array<double, 3> oldVelocity );
    
private:
        dynamic_reconfigure::Server<robotrainer_controllers::PassiveBehaviorControllerConfig>* p_dysrv_;

        ros::Publisher SlidingIntegral_pub_;
        ros::Publisher Adaptionfactor_pub_;
        ros::Publisher RealInputforce_pub_;
        ros::Publisher SlidingIntegral3d_pub_;
        ros::Publisher ChuyIntegral3d_pub_;
        
        std_msgs::Float64 slidingIntegralVal_msg_;
        geometry_msgs::Vector3 adaptionFactor_msg_;
        geometry_msgs::Vector3 realInputforce_msg_;
        geometry_msgs::Vector3 slidingIntegral3d_msg_;

        boost::shared_ptr<robotrainer_helper_types::SlidingIntegral> integral_ptr_{ new robotrainer_helper_types::SlidingIntegral{}};
        boost::shared_ptr<robotrainer_helper_types::SlidingIntegralMultiDim> integral_MD_ptr_{new robotrainer_helper_types::SlidingIntegralMultiDim{}};
        boost::shared_ptr<robotrainer_helper_types::ChuyIntegralMultiDim> chuy_MD_ptr_{new robotrainer_helper_types::ChuyIntegralMultiDim{}};

        const static int dim = 3;
        std::array<bool, dim> control_dimensions_; //sets passive behavior control active for each dimension
        bool detect_main_dimension_;
        
        std::array<double, dim> current_power_;
        
        double min_adaption_factor_;
        double max_adaption_factor_;
        ros::Time scale_down_time_begin_;
        std::array<ros::Time, dim> scale_down_time_begin_MD_;
        ros::Time scale_up_time_begin_;
        std::array<ros::Time, dim> scale_up_time_begin_MD_;
        bool scale_down_;
        std::array<bool, dim> scale_down_MD_;
        bool scale_up_;
        std::array<bool, dim> scale_up_MD_;
        std::array<double, dim> scale_down_start_val_;
        std::array<double, dim> scale_up_start_val_;

        //Parameters which describe the observed robotstate
        std::array<double, dim> scaled_input_{ {0.0, 0.0, 0.0} };
        std::array<double, dim> old_force_{ {0.0, 0.0, 0.0} };
        std::array<double, dim> old_velocity_{ {0.0, 0.0, 0.0} };
        std::array<double, dim> desired_mass_;

        //reconfigureable params
        std::array<double, dim> desired_adaption_factor_;
        std::array<double, dim> adaption_factor_;
        std::array<double, dim> adaption_factor_old_;
        std::array<double, dim> adapt_input_force_scale_;

        enum detectionMethod{ none, chuy2007, slidingSD, slidingMD };
        detectionMethod usedMethod_;
        
        bool use_chuy_integral_;
        
        int integral_window_size_;
        double system_const_;
        double robot_virt_mass_;
        double eps_;
        double scale_down_time_;
        double scale_up_time_;
        
        //first instability detection
        ros::Time lastloopTime_;
        std::array<double, dim> stableDuration_;
        std::array<double, dim> instableDuration_;
        std::array<double, dim> resetToStability_Countdown_;
        std::array<bool, dim> ignoringImpassiveBehavior_;
        double minimumStableDuration_;
        double extremelyInstableDuration_;
        double resetCooldownAfterInstability_;
        
        bool deactivateAllOnExtremeInstability_;
        bool deactivateAllDimensions_;
        
        std::array<double, dim> instable_threshold_value_{ {0.0,0.0,0.0} };
        
        //same direction detection
        bool conditionalInstabilityDetection_;
        std::array<bool, dim> changedDirection_;
        std::array<bool, dim> multipleDirectionChange_;
        std::array<std::list<int>, dim> slidingDirections_; // 1 stands for positive velocity, 0 for no velocity and -1 for negative
        int directionWindowSize_;
        std::array<int, dim> sameDirectionCtr_;
        
        bool enable_integral_resetting_;
        
        bool fresh_resetted_;
        
        
        //Functions
        void initPublishersAndParameters(ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
        
        double derivate_first_order(const double& vel_old, const ros::Time& vel_old_time, const double& vel_new, const ros::Time& vel_time);

        void resetValues();
        void resetSlidingVelocities();
        void setDesiredMass();
        void setDesiredAdaptionFactor_MD();
        bool detectedNonPassiveBehavior();
        bool detectedNonPassiveBehavior_MD();
        void updateIntegrals(const std::array<double, dim>& force, const std::array<double,dim>& velocity, const ros::Time& time );
        std::array<double, 3> calcCurrentAdaptionFactor_time(const ros::Time& time);
        std::array<double, 3> calcCurrentAdaptionFactor_time_MD(const ros::Time& time, std::array<double, dim> current_power);
        std::array<double, 3> adaptInputForce(const std::array<double, dim>& adaption_factor, const std::array<double, dim> &input_force);
        double scaleUpLinear_MD(const double& current_adaption_factor, const ros::Time& scale_up_start_time, const double& start_value);
        double scaleDownLinear_MD(const double& desired_adaption_factor, const double& current_adaption_factor, const ros::Time& scale_down_start_time, const double& start_value);
        void resetController();

        template <typename T> struct SUM_ {
                SUM_() : sum{0} {}
                void operator() (T n) {
                        sum += n;
                }
                T sum;
        };

        template <typename T>  T scale(T toScale, const T min, const T max) {
                if(toScale > max ) {
                        return max;
                } else if( toScale < min) {
                        return min;
                } else {
                        return toScale;
                }
        }

        template <typename T> T sumArray(std::array<T, DIM> array) {
                SUM_<T> s = std::for_each(array.begin(), array.end(), SUM_<T>());
                return s.sum;
        }
};

}//namespace
#endif
