#ifndef ROBOTRAINER_CONTROLLERS_INTEGRAL_COMPARE_CTRL_H
#define ROBOTRAINER_CONTROLLERS_INTEGRAL_COMPARE_CTRL_H

#define DIM 3

#include "robotrainer_controllers/IntegralCompareCtrlConfig.h"
#include "robotrainer_controllers/fts_base_controller.h"

#include "robotrainer_helper_types/Integral.h"

#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>

namespace robotrainer_controllers {

class IntegralCompareCtrl : public FTSBaseController {
public:
    IntegralCompareCtrl();
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    ~IntegralCompareCtrl() {
        delete_pointers();
    };

    void reconfigureCallback(robotrainer_controllers::IntegralCompareCtrlConfig &config, uint32_t level);

private:
    dynamic_reconfigure::Server<robotrainer_controllers::IntegralCompareCtrlConfig>* p_dysrv_;


    ros::Publisher integral_raw_pub_;
    ros::Publisher integral_std_pub_;
    ros::Publisher integral_chuy_pub_;
    ros::Publisher integral_sliding_pub_;
    ros::Publisher integral_sliding_weight_pub_;
    //publisher for multidimensional integral calculation
    ros::Publisher integral_raw_MD_pub_;
    ros::Publisher integral_std_MD_pub_;
    ros::Publisher integral_chuy_MD_pub_;
    ros::Publisher integral_sliding_MD_pub_;
    ros::Publisher integral_sliding_weight_MD_pub_;

    ros::Publisher max_integral_val_pub_;
    ros::Publisher max_integral_val_3d_pub_;

    std_msgs::Float64 integral_raw_msg_;
    std_msgs::Float64 integral_std_msg_;
    std_msgs::Float64 integral_chuy_msg_;
    std_msgs::Float64 integral_sliding_msg_;
    std_msgs::Float64 integral_sliding_weight_msg_;
    // messages for multidimensional integral calculation
    geometry_msgs::Vector3 integral_raw_MD_msg_;
    geometry_msgs::Vector3 integral_std_MD_msg_;
    geometry_msgs::Vector3 integral_chuy_MD_msg_;
    geometry_msgs::Vector3 integral_sliding_MD_msg_;
    geometry_msgs::Vector3 integral_sliding_weight_MD_msg_;

    std_msgs::Float64 max_integral_val_msg_;
    geometry_msgs::Vector3 max_integral_val_3d_msg_;

    robotrainer_helper_types::RAWIntegral raw_integral_;
    robotrainer_helper_types::STDIntegral std_integral_;
    robotrainer_helper_types::ChuyIntegral chuy_integral_;
    boost::shared_ptr<robotrainer_helper_types::SlidingIntegral> sliding_integral_ptr_{ new robotrainer_helper_types::SlidingIntegral{}};
    boost::shared_ptr<robotrainer_helper_types::SlidingIntegralLineraWeigth> sliding_integral_weight_ptr_{new robotrainer_helper_types::SlidingIntegralLineraWeigth{}};

    robotrainer_helper_types::RAWIntegralMultiDim raw_integral_MD_;
    robotrainer_helper_types::STDIntegralMultiDim std_integral_MD_;
    robotrainer_helper_types::ChuyIntegralMultiDim chuy_integral_MD_;
    boost::shared_ptr<robotrainer_helper_types::SlidingIntegralMultiDim> sliding_integral_MD_ptr_{new robotrainer_helper_types::SlidingIntegralMultiDim{}};
    boost::shared_ptr<robotrainer_helper_types::SlidingIntegralLineraWeigthMultiDim> sliding_integral_weight_MD_ptr_{new robotrainer_helper_types::SlidingIntegralLineraWeigthMultiDim{}};

//  static int window_size_;
    bool use_raw_integral_ = false;
    bool use_std_integral_ = false;
    bool use_chuy_integral_ = false;
    bool use_sliding_integral_ = false;
    bool use_sliding_integral_linearWeight_ = false;

    bool use_raw_integral_3d_ = false;
    bool use_std_integral_3d_ = false;
    bool use_chuy_integral_3d_ = false;
    bool use_sliding_integral_3d_ = false;
    bool use_sliding_integral_linearWeight_3d_= false;

    double max_integral_val_;
    std::array<double, 3> max_integral_val_3d_;

    std::array<double, DIM> old_force_;
    std::array<double, DIM> old_velocity_;

    void resetIntegrals();
    void delete_pointers();

    template <typename T> struct SUM_ {
        SUM_() : sum{0} {}
        void operator() (T n) {
            sum += n;
        }
        T sum;
    };

    template <typename T> T sumArray(std::array<T, DIM> array)
    {
        SUM_<T> s = std::for_each(array.begin(), array.end(), SUM_<T>());
        return s.sum;
    }

//     std::array<double, DIM> mulTwoArrays(std::array<const std::array<double, DIM>& array1, const std::array<double, DIM>& array2)
// 		{
// 				std::array<double, DIM> ret_array;
// 				for(std::size_t i = 0; i < DIM ; ++i) {
// 					ret_array[i] = array1[i] + array2[i];
// 				}
// 				return ret_array;
// 		}

};

}
#endif
