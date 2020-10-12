#ifndef ROBOTRAINER_MODALITIES_CONTROLLER_BASE_H
#define ROBOTRAINER_MODALITIES_CONTROLLER_BASE_H
#include "robotrainer_modalities/modality_base.h"

#include "robotrainer_helper_types/wrench_twist_datatype.h"

#include <filters/filter_base.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace robotrainer_modalities {
        
template <typename T> class ModalitiesControllerBase : public filters::FilterBase<T> {

public:
        ModalitiesControllerBase();
        virtual bool update(const T& data_in, T& data_out){ROS_INFO_ONCE("[modalities_controller_base.h]ModalityBase update");}
        virtual bool configure(){ROS_INFO_ONCE("[modalities_controller_base.h]ModalityBase configure");}
        virtual tf2::Vector3 getLastVelocity() {ROS_INFO_ONCE("[modalities_controller_base.h] getLastVela Base"); }
        virtual tf2::Vector3 getLastscaledForce() {ROS_INFO_ONCE("[modalities_controller_base.h]getLastForce Base");  }
        virtual ~ModalitiesControllerBase(){delete tf_buffer_ptr_; delete tf_Listener_ptr_;};

        geometry_msgs::TransformStamped getMaptoRobotTransform();
        geometry_msgs::TransformStamped getMaptoRobotTransform(const ros::Time & time);
protected:
                                
        tf2_ros::Buffer *tf_buffer_ptr_;
        tf2_ros::TransformListener *tf_Listener_ptr_; 
        
        bool modalitie_configured_;
        bool modalitie_loaded_;
//      virtual double calculateIntegrals(robotrainer_helper_types::wrench_twist input);
};


template <typename T>  ModalitiesControllerBase<T>::ModalitiesControllerBase() {   
        tf_buffer_ptr_ = new tf2_ros::Buffer;
        tf_Listener_ptr_ = new tf2_ros::TransformListener(*tf_buffer_ptr_, true);
        ROS_INFO_ONCE("[modalities_controller_base.h] constructor");
}
  
template <typename T> geometry_msgs::TransformStamped ModalitiesControllerBase<T>::getMaptoRobotTransform() {
        
        // TODO(denis): changes this to not be hard_coded frames
        geometry_msgs::TransformStamped transform_robot_map;
        try {
                transform_robot_map = tf_buffer_ptr_->lookupTransform("base_link", "map", ros::Time(0));
        } catch(tf2::TransformException &ex) { 
                throw ex;
        }
        return transform_robot_map;
}

template <typename T> geometry_msgs::TransformStamped ModalitiesControllerBase<T>::getMaptoRobotTransform(const ros::Time& time) {
      
        geometry_msgs::TransformStamped transform_robot_map;
        try {
                transform_robot_map = tf_buffer_ptr_->lookupTransform("base_link", "map", time);
        } catch(tf2::TransformException &ex) { 
                throw ex;
        }
        return transform_robot_map;
}

}//namespace
#endif
