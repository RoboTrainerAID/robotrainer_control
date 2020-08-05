/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <hardware_interface/joint_state_interface.h>

#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include <cob_base_controller_utils/OdometryTracker.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>

#include <std_srvs/Trigger.h>

#include "robotrainer_controllers/fts_GeomController.h"

namespace robotrainer_controllers
{

// this controller gets access to the JointStateInterface
class OdometryController : public OdomGeomController<hardware_interface::JointStateInterface, UndercarriageGeom>
{
public:
    OdometryController() {}

    virtual bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){
        ros::NodeHandle wheels_nh(root_nh, "wheel_controller");
        wheel_ctrl_nh_ = wheels_nh;
        
        wheel_params_t wheel_params;
        if(!cob_omni_drive_controller::parseWheelParams(wheel_params, wheel_ctrl_nh_) || !OdomGeomController::init(hw, wheel_params)) return false;

//         if(!GeomController::init(hw, controller_nh)) return false;

        double publish_rate;
        if (!controller_nh.getParam("publish_rate", publish_rate)){
            ROS_ERROR("Parameter 'publish_rate' not set");
            return false;
        }
        if(publish_rate <= 0){
            ROS_ERROR_STREAM("publish_rate must be positive.");
            return false;
        }

        const std::string frame_id = controller_nh.param("frame_id", std::string("odom"));
        const std::string child_frame_id = controller_nh.param("child_frame_id", std::string("base_footprint"));
        const double cov_pose = controller_nh.param("cov_pose", 0.1);
        const double cov_twist = controller_nh.param("cov_twist", 0.1);

        odom_tracker_.reset(new OdometryTracker(frame_id, child_frame_id, cov_pose, cov_twist));
        odom_ = odom_tracker_->getOdometry();

        topic_pub_odometry_ = controller_nh.advertise<nav_msgs::Odometry>("odometry", 1);

        bool broadcast_tf = true;
        controller_nh.getParam("broadcast_tf", broadcast_tf);

        if(broadcast_tf){
            odom_tf_.header.frame_id = frame_id;
            odom_tf_.child_frame_id = child_frame_id;
            tf_broadcast_odometry_.reset(new tf::TransformBroadcaster);
        }

        publish_timer_ = controller_nh.createTimer(ros::Duration(1/publish_rate), &OdometryController::publish, this);
        service_reset_ = controller_nh.advertiseService("reset_odometry", &OdometryController::srv_reset, this);
        
        // Kinematics services
        service_kinematic_update_ = controller_nh.advertiseService("update_kinematics", &OdometryController::updateWheelParamsCallback, this);

        return true;
    }
    virtual void starting(const ros::Time& time){
        if(time != stop_time_) odom_tracker_->init(time); // do not init odometry on restart
        reset_ = false;
    }

    virtual bool srv_reset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if(!isRunning()){
            res.message = "not running";
            res.success = false;
        }else{
            boost::mutex::scoped_lock lock(mutex_);
            reset_ = true;
            lock.unlock();
            res.success = true;
            ROS_INFO("Resetting odometry to zero.");
        }

        return true;
    }
    
    bool updateWheelParamsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    
        ROS_DEBUG("Called service to update kinematic configuration");
        res.success = true;
        boost::mutex::scoped_lock lock(mutex_geom_);
        wheel_params_t wheel_params;
        res.success &= cob_omni_drive_controller::parseWheelParams(wheel_params, wheel_ctrl_nh_);
        res.success &= OdomGeomController::update(wheel_params);
        lock.unlock();
        ROS_INFO("Kinematics sucessfully updated!");

    return true;
}

    virtual void update(const ros::Time& time, const ros::Duration& period){

        boost::mutex::scoped_try_lock lock_geom(mutex_geom_);
        if (lock_geom) {
            updateState();
            geom_->calcDirect(platform_state_);
        }

        odom_tracker_->track(time, period.toSec(), platform_state_.getVelX(), platform_state_.getVelY(), platform_state_.dRotRobRadS);

        boost::mutex::scoped_try_lock lock(mutex_);
        if(lock){
            if(reset_){
                odom_tracker_->init(time);
                reset_ = false;
            }
            odom_ =  odom_tracker_->getOdometry();
        }

    }
    virtual void stopping(const ros::Time& time) { stop_time_ = time; }

private:
    PlatformState platform_state_;
    
    ros::NodeHandle wheel_ctrl_nh_;

    ros::Publisher topic_pub_odometry_;                 // calculated (measured) velocity, rotation and pose (odometry-based) for the robot
    ros::ServiceServer service_reset_;                  // service to reset odometry to zero
    ros::ServiceServer service_kinematic_update_;

    boost::scoped_ptr<tf::TransformBroadcaster> tf_broadcast_odometry_;    // according transformation for the tf broadcaster
    boost::scoped_ptr<OdometryTracker> odom_tracker_;
    ros::Timer publish_timer_;
    nav_msgs::Odometry odom_;
    bool reset_;
    boost::mutex mutex_, mutex_geom_;
    geometry_msgs::TransformStamped odom_tf_;
    ros::Time stop_time_;


    void publish(const ros::TimerEvent&){
        if(!isRunning()) return;

        boost::mutex::scoped_lock lock(mutex_);

        topic_pub_odometry_.publish(odom_);

        if(tf_broadcast_odometry_){
            // compose and publish transform for tf package
            // compose header
            odom_tf_.header.stamp = odom_.header.stamp;
            // compose data container
            odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
            odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
            odom_tf_.transform.rotation = odom_.pose.pose.orientation;

            // publish the transform (for debugging, conflicts with robot-pose-ekf)
            tf_broadcast_odometry_->sendTransform(odom_tf_);
        }
    }
};

}

PLUGINLIB_EXPORT_CLASS(robotrainer_controllers::OdometryController, controller_interface::ControllerBase)
