#ifndef ROBOTRAINER_MODALITIES_MODALITY_BASE_H
#define ROBOTRAINER_MODALITIES_MODALITY_BASE_H

#include <geometry_msgs/Twist.h>
#include <filters/filter_base.h>
#include <tf2_ros/transform_listener.h>
#include <robotrainer_parameters/shared_params.h>

#include <realtime_tools/realtime_publisher.h>

/**
 * \brief namespace for the robotrainer modalities plugin.
 */
namespace robotrainer_modalities
{
/**
 * \brief The base class of the robotrainer modalities plugin.
*/
template <typename T> class ModalityBase : public filters::FilterBase<T> {
        
public:
        virtual bool update(const T& data_in, T& data_out) {ROS_INFO_ONCE("[modality_base.h]ModalityBase update");}
        virtual bool configure(){ROS_INFO_ONCE("[modality_base.h]ModalityBase config");}
        virtual tf2::Vector3 getLastVelocity() {ROS_INFO_ONCE("[modality_base.h] getLastVelocity Base"); }
        virtual tf2::Vector3 getLastscaledForce() {ROS_INFO_ONCE("[modality_base.h] getLastForce Base");  }
        virtual double getLastLimitVelocityFactor() {ROS_INFO_ONCE("[modality_base.h] getLastLimitVelocityFactor Base"); }
virtual ~ModalityBase(){};

        //HACK!!!!!
        //note if those functions are not implemented in derived classes, this can lead to undefined symbol in lib.so ERROR
        virtual void setForceLimit(tf2::Vector3 max_force) { }
        virtual void setUseLimit(bool use_limit){}
protected:
        ModalityBase();
            
        // Objects and methods for locating the robot
        tf2_ros::Buffer *p_tf_buffer_;
        tf2_ros::TransformListener *p_tf_listener_;
        uint num_transform_errors_;
        geometry_msgs::TransformStamped transform_map_robot_; ///< Describes the current robot location in the map frame.
        void updateRobotLocation();
        
        // Shared params
        robotrainer_parameters::SharedParams* sp_object_ptr_; ///< Object that manages the shared parameters
        robotrainer_parameters::SharedParamsParameters* ns_params_ptr_; ///< Commonly used parameters (namespace names)
};
    
/** 
 * \brief Constructor for every modality object.
*/
template <typename T> ModalityBase<T>::ModalityBase() { 
        
        // Initialize tf2 objects
        p_tf_buffer_ = new tf2_ros::Buffer;
        p_tf_listener_ = new tf2_ros::TransformListener(*p_tf_buffer_,true);
        num_transform_errors_ = 0;

        // Get common params
        sp_object_ptr_ = new robotrainer_parameters::SharedParams();
        ns_params_ptr_ = sp_object_ptr_->getParamsPtr();

        // Check if shared params exist with a default value.
        if(ns_params_ptr_->project_ns == "" 
                || ns_params_ptr_->scenario_ns == ""
                || ns_params_ptr_->force_ns == ""
                || ns_params_ptr_->config_ns == ""
                || ns_params_ptr_->data_ns == ""
                || ns_params_ptr_->arrow_ns == ""
                || ns_params_ptr_->area_ns == ""
                || ns_params_ptr_->margin_ns == ""
                || ns_params_ptr_->force_distance_function_ns == "")
        {
                ROS_ERROR("VirtualForces: Namespaces are not configured properly.");
        }
}
    
/**
* \brief Retrieve current location of the robot as transformation "/map" -> "/base_link".
* 
* Only if the transform fails a 100 times in a row an error log message will occur. That's because the map frame is not available immediately after init of robot.
*/
template <typename T> void ModalityBase<T>::updateRobotLocation() {
        
        try {
                transform_map_robot_ = p_tf_buffer_->lookupTransform("map", "base_link", ros::Time(0));
                num_transform_errors_ = 0;
        } catch(tf2::TransformException &ex) {
                if (num_transform_errors_%100 == 0){
                        ROS_ERROR("%s", ex.what());
                }
                num_transform_errors_++;
        }
}
    
    
}
#endif
