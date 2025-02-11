#ifndef ROBOTRAINER_MODALITIES_VIRTUAL_AREAS_H
#define ROBOTRAINER_MODALITIES_VIRTUAL_AREAS_H

#include <robotrainer_modalities/modality_base.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotrainer_modalities/VirtualAreasParameters.h>
#include <robotrainer_modalities/VirtualAreasConfig.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace robotrainer_modalities
{
    
/**
 * \brief A class that contains the modality plugin for virtual areas.
 * 
 * Provides methods for loading a virtual area configuration (a set of virtual areas) from the parameter server and applying their effect to a a velocity wrench.
 */
template <typename T>
class VirtualAreas : public robotrainer_modalities::ModalityBase<T>
{
public:
    VirtualAreas();
    virtual bool update( const T & data_in, T& data_out);
    virtual bool configure();
    
private:
    // Objects for publishing messages
    // Debug purpose only.
    ros::NodeHandle nh_;
    ros::Publisher pub_velocity_in_;
    ros::Publisher pub_velocity_out_;
    ros::Publisher pub_rotation_in_;
    ros::Publisher pub_rotation_out_;
    ros::Publisher pub_counterforce_dist_;
    
    // Objects and methods for locating the robot
    using ModalityBase<T>::updateRobotLocation;
    using ModalityBase<T>::transform_map_robot_;
    
    // Shared params
    using ModalityBase<T>::ns_params_ptr_;
    
    // Objects for param_handler and dynamic_reconfigure
    robotrainer_modalities::VirtualAreasParameters params_;
    void reconfigureRequest(VirtualAreasConfig& config, uint32_t level);
    dynamic_reconfigure::Server<robotrainer_modalities::VirtualAreasConfig> dynamic_reconfigure_server_;

    // Maps and vectors to hold the virtual area configuration
    std::vector<tf2::Vector3> area_center_points_; ///< Contains the area center points.
    std::vector<double> area_radii_; ///< Contains the area radii.
    std::vector<std::map<std::string, bool>> area_effects_; ///< Contains the area effects
    std::vector<std::vector<std::string>> area_effects_names_; ///< Contains the area effect types
    std::vector<double> area_amplification_effects_; ///< Contains the amplification effects
    
    // Store current area and it's entry value
    tf2::Vector3 area_entry_velocity_; ///< Stores the linear input velocity when entering the area.
    tf2::Vector3 area_entry_rot_; ///< Stores the angular input velocity when entering the area.
    int current_area_;
};

/** 
 * \brief Constructor for VirtualAreas object.
 * 
 * * params_ and the dynamic_reconfigure_server get a node handle that is not in conflict with other functions.
 * * The pramas_ struct get all parameters from the parameter server.
 * * tf2-objects, dynamic reconfigure and publisher objects get initialized and configured.
 */
template <typename T>
VirtualAreas<T>::VirtualAreas()
: params_{ros::NodeHandle("/modalities/virtual_areas")}, dynamic_reconfigure_server_{ros::NodeHandle("/modalities/virtual_areas")}
{
    // Initialize params struct and the dynamic_reconfigure server with a node handle.
    // Get all parameter values from the parameter sever.
    params_.fromParamServer();
    
    // Dynamic reconfigure
    dynamic_reconfigure_server_.setCallback(boost::bind(&VirtualAreas<T>::reconfigureRequest, this, _1, _2));
       
    // Publish messages (debugging purpose)
    pub_velocity_in_ = nh_.advertise<geometry_msgs::Vector3>("virtual_areas/modalities_debug/velocity_in", 1000);
    pub_velocity_out_ = nh_.advertise<geometry_msgs::Vector3>("virtual_areas/modalities_debug/velocity_out", 1000);
    pub_rotation_in_ = nh_.advertise<geometry_msgs::Vector3>("virtual_areas/modalities_debug/rotation_in", 1000);
    pub_rotation_out_ = nh_.advertise<geometry_msgs::Vector3>("virtual_areas/modalities_debug/rotation_out", 1000);
    pub_counterforce_dist_ = nh_.advertise<std_msgs::Float64>("virtual_areas/counterforce/center_dist_percent", 1);
}

/**
 * \brief Update the effect of the virtual areas on the velocity twist at a timestep.
 * 
 * \param data_in input velocity as Twist
 * \param data_out output velocity as Twist
 */
template <typename T>
bool VirtualAreas<T>::update(const T& data_in, T& data_out) {
        
        // Initialize the output velocity with the input velocity
        data_out = data_in;
        
        // If there are no areas configured, return
        if (area_center_points_.size() < 1) {
                return true;
        }
        
        tf2::Vector3 velocity_in, rot_in;
        tf2::fromMsg(data_in.linear, velocity_in);
        tf2::fromMsg(data_in.angular, rot_in);
        
        // Update the current robot location
        updateRobotLocation();
        tf2::Vector3 robot_location;
        tf2::fromMsg(transform_map_robot_.transform.translation, robot_location);
        
        // Which is the nearest area center point?
        int j = 0;
        tf2::Vector3 to_center_j = area_center_points_[j] - robot_location;
        bool inside_area = false;
        for (int i = 0; i < area_center_points_.size(); i++) {
                tf2::Vector3 to_center_i = area_center_points_[i] - robot_location;
                if (to_center_i.length() < area_radii_[i] && to_center_i.length() <= to_center_j.length()) {
                inside_area = true;
                j = i;
                to_center_j = to_center_i;
                }
        }
        

        
        
        // If the robot is in no area, return
        if (!inside_area) {
                current_area_ = -1;
                return true;
        }
        
        // If the robot just entered the area, store current velocity_in
        if (j != current_area_) {
                area_entry_velocity_ = velocity_in;
                area_entry_rot_ = rot_in;
                current_area_ = j;
        }
        
        // Initialize output velocity and rotation
        tf2::Vector3 velocity_out = velocity_in;
        tf2::Vector3 rot_out = rot_in;
        
        
        // apply area effects names
        for (int k=0; k < area_effects_names_[j].size(); k++) {   
                
                /* TODO Possibly change data type to enum in the future, and only allow combinations of different effect groups to be active together 
                                (e.g. Keep_value_group + Adjust_speed_group or Invert_dimension + Adjust_speed) */
                bool found_one = false;
                
                std::string current_effect = area_effects_names_[j][k];
                if (current_effect == "keep_direction") {
                        velocity_out = tf2::tf2Dot(velocity_out, area_entry_velocity_) / pow(area_entry_velocity_.length(), 2) * area_entry_velocity_;
                        found_one = true;
                }
                if (current_effect == "keep_rotation") {
                        rot_out.setZero();
                        found_one = true;
                }
                if (current_effect == "invert_direction") {
                        velocity_out = tf2::tf2Dot(velocity_out, area_entry_velocity_) / pow(area_entry_velocity_.length(), 2) * area_entry_velocity_ * 2 - velocity_in;
                        found_one = true;
                }
                if (current_effect == "invert_y") {
                        velocity_out.setY(velocity_out.getY() * (-1));
                        found_one = true;
                        ROS_INFO("[Virtual Area] - In Invert y direction!");
                }
                if (current_effect == "invert_rotation") {
                        rot_out.setZ(rot_out.getZ() * (-1));
                        found_one = true;
                }
                if (current_effect == "double_speed") {
                        velocity_out = velocity_out * 2;
                        rot_out = rot_out * 2;
                        found_one = true;
                }
                if (current_effect == "half_speed") {
                        velocity_out = velocity_out / 2;
                        rot_out = rot_out / 2;
                        found_one = true;
                }
                if (current_effect == "apply_counterforce") {
                        
                        std_msgs::Float64 center_dist_percent;
                        double dist_percent =  (to_center_j.length() / area_radii_[j]);
                        ROS_DEBUG("[Virtual Area] - In Counterforce (distance to center: %.2f percent)", dist_percent);
                        center_dist_percent.data = dist_percent;
                        pub_counterforce_dist_.publish(center_dist_percent);
                        found_one = true;
                }
                
                if (!found_one) {
                        ROS_WARN_STREAM("VirtualAreas: Effect '" <<  current_effect << "' not implemented! To use it implement it! The Area is deactivated!");
                }
        }
        
        // set output
        data_out.linear = tf2::toMsg(velocity_out);
        data_out.angular = tf2::toMsg(rot_out);
        
        // debug
        pub_velocity_in_.publish(data_in.linear);   
        pub_velocity_out_.publish(data_out.linear);
        pub_rotation_in_.publish(data_in.angular);   
        pub_rotation_out_.publish(data_out.angular);    
        
        return true;
}

/**
 * \brief Callback function for dynamic reconfigure.
 */
template <typename T>
void VirtualAreas<T>::reconfigureRequest(VirtualAreasConfig& config, uint32_t level)
{   
    // update params hold by the params_-struct
    params_.fromConfig(config);
};

/**
 * \brief Retrieve the virtual area configuration from the parameter server.
 */
template <typename T>
bool VirtualAreas<T>::configure() {
    
    // Delete all existing parameters to receive the new ones
    area_center_points_.clear();
    area_radii_.clear();
    area_effects_names_.clear();
    area_effects_.clear();
    area_amplification_effects_.clear();
  
    // Namespaces
    std::string path_areas = "/" + ns_params_ptr_->project_ns + "/" + ns_params_ptr_->scenario_ns + "/" + ns_params_ptr_->area_ns;
    
    // Load the list of area names
    std::vector<std::string> area_names;
    ros::param::get( path_areas + "/" + ns_params_ptr_->config_ns + "/" + ns_params_ptr_->area_ns + "_names", area_names);
    if (area_names.size() < 1)
    {
        ROS_WARN_STREAM("VirtualAreas: No valid area configuration found in namespace " << path_areas << ".\n VirtualAreas-Modality will not be used!");
    }
    else {
        ROS_INFO_STREAM("VirtualAreas: Found " << area_names.size() << " area(s) in namespace " << path_areas << ".");  
        
        // Retrieve and store all area data based on the area names
        for(int i = 0; i < area_names.size(); i++){
        
            std::string path_data_area_i = path_areas + "/" + ns_params_ptr_->data_ns + "/" +  area_names[i];
            
            // center point of the area
            geometry_msgs::Point area_center_msg;
            ros::param::get(path_data_area_i + "/" + ns_params_ptr_->area_ns + "/x", area_center_msg.x);
            ros::param::get(path_data_area_i + "/" + ns_params_ptr_->area_ns + "/y", area_center_msg.y);
            ros::param::get(path_data_area_i + "/" + ns_params_ptr_->area_ns + "/z", area_center_msg.z);
            tf2::Vector3 area_center;
            tf2::fromMsg(area_center_msg, area_center);
            area_center_points_.push_back(area_center);
            
            // calculate area radius from edge point
            geometry_msgs::Point area_edge_msg;
            ros::param::get(path_data_area_i + "/" + ns_params_ptr_->margin_ns + "/x", area_edge_msg.x);
            ros::param::get(path_data_area_i + "/" + ns_params_ptr_->margin_ns + "/y", area_edge_msg.y);
            ros::param::get(path_data_area_i + "/" + ns_params_ptr_->margin_ns + "/z", area_edge_msg.z);
            tf2::Vector3 area_edge;
            tf2::fromMsg(area_edge_msg, area_edge);
            area_radii_.push_back(tf2::tf2Distance(area_center, area_edge));
            
            // area effects
            std::vector<std::string> area_effects;
            ros::param::get(path_data_area_i + "/" + "area_functions", area_effects);
            area_effects_names_.push_back(area_effects);
            
            std::map<std::string, bool> effects;
            double amplification_effect;
            ros::param::get(path_data_area_i + "/" + "invert_direction", effects["invert_direction"]);
            ros::param::get(path_data_area_i + "/" + "invert_rotation", effects["invert_rotation"]);
            ros::param::get(path_data_area_i + "/" + "invert_y", effects["invert_y"]);
            ros::param::get(path_data_area_i + "/" + "keep_direction", effects["keep_direction"]);
            ros::param::get(path_data_area_i + "/" + "keep_rotation", effects["keep_rotation"]);
            ros::param::get(path_data_area_i + "/" + "amplification", amplification_effect);
            ros::param::get(path_data_area_i + "/" + "apply_counterforce", effects["apply_counterforce"]);
            area_effects_.push_back(effects);
            area_amplification_effects_.push_back(amplification_effect);
        }
        
        // Initialize current area
        current_area_ = -1;
    }
    
    return true;
}

};

#endif
