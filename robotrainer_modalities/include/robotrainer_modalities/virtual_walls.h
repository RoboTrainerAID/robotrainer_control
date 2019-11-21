#ifndef ROBOTRAINER_MODALITIES_VIRTUAL_WALLS_H
#define ROBOTRAINER_MODALITIES_VIRTUAL_WALLS_H

#include <robotrainer_modalities/modality_base.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotrainer_modalities/VirtualWallsParameters.h>
#include <robotrainer_modalities/VirtualWallsConfig.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>

namespace robotrainer_modalities
{
    
/**
 * \brief A class that contains the modality plugin for virtual walls.
 * 
 * Provides methods for loading a virtual wall configuration (a set of virtual walls) from the parameter server and applying their effect to a a velocity wrench.
 */
template <typename T>
class VirtualWalls : public robotrainer_modalities::ModalityBase<T>
{
public:
    VirtualWalls();
    virtual bool update( const T & data_in, T& data_out);
    virtual bool configure();
		tf2::Vector3 getLastscaledForce() {return last_scaled_virtual_force_;}
		double getLastLimitVelocityFactor(){return last_limit_vel_factor_;}
    
private:
    // Objects for publishing messages
    // Debug purpose only.
    ros::NodeHandle nh_;
    ros::Publisher pub_velocity_in_;
    ros::Publisher pub_velocity_out_;
    ros::Publisher pub_resulting_velocity_;
    ros::Publisher pub_resulting_force_;
    ros::Publisher pub_position_;
    
    // Objects and methods for locating the robot
    using ModalityBase<T>::updateRobotLocation;
    using ModalityBase<T>::transform_map_robot_;
    
    // Shared params
    using ModalityBase<T>::ns_params_ptr_;
    
    // Objects for param_handler and dynamic_reconfigure
    robotrainer_modalities::VirtualWallsParameters params_;
    void reconfigureRequest(VirtualWallsConfig& config, uint32_t level);
    dynamic_reconfigure::Server<robotrainer_modalities::VirtualWallsConfig> dynamic_reconfigure_server_;

    // Maps and vectors to hold the virtual wall configuration
    std::vector<tf2::Vector3> wall_base_points_; ///< Vector that holds the base points of the walls
    std::vector<tf2::Vector3> wall_end_points_; ///< Vector that holds the end points of the walls
    std::vector<double> effective_radii_; ///< Vector that holds the effective radius of each wall
    std::vector<std::string> force_distance_functions_; ///< Contains the  force(distance)-function to be applied.
    
    // Storages for the values of the previous timestep
    tf2::Vector3 previous_resulting_velocity_;
    tf2::Vector3 previous_wall_force_;
    
    // Params for the mass damping system
    double mds_f_; ///< Precalculated parameter for the mass damping system equation.
    double mds_v_; ///< Precalculated parameter for the mass damping system equation.
		double max_effective_radius_;
		bool limit_human_velocity_;
		double min_allowed_distance_to_wall_;
// 		const double rad_to_deg_const_ = 180.0/M_PI;
		double limit_human_vel_at_min_;
		double last_limit_vel_factor_;
		tf2::Vector3 last_scaled_virtual_force_;
};

/** 
 * \brief Constructor for VirtualWalls object.
 * 
 * * params_ and the dynamic_reconfigure_server get a node handle that is not in conflict with other functions.
 * * The pramas_ struct get all parameters from the parameter server.
 * * tf2-objects, dynamic reconfigure and publisher objects get initialized and configured.
 */
template <typename T>
VirtualWalls<T>::VirtualWalls()
: params_{ros::NodeHandle("/modalities/virtual_walls")}, dynamic_reconfigure_server_{ros::NodeHandle("/modalities/virtual_walls")}
{
    // Initialize params struct and the dynamic_reconfigure server with a node handle.
    // Get all parameter values from the parameter sever.
    params_.fromParamServer();
    
    // Dynamic reconfigure
    dynamic_reconfigure_server_.setCallback(boost::bind(&VirtualWalls<T>::reconfigureRequest, this, _1, _2));
    
    // Publish messages (debugging purpose)
    pub_velocity_in_ = nh_.advertise<geometry_msgs::Vector3>("virtual_walls/modalities_debug/velocity_in", 1000);
    pub_velocity_out_ = nh_.advertise<geometry_msgs::Vector3>("virtual_walls/modalities_debug/velocity_out", 1000);
    pub_resulting_velocity_ = nh_.advertise<geometry_msgs::Vector3>("virtual_walls/modalities_debug/resulting_velocity", 1000);
    pub_resulting_force_ = nh_.advertise<geometry_msgs::Vector3>("virtual_walls/modalities_debug/resulting_force", 1000);
    pub_position_ = nh_.advertise<geometry_msgs::Vector3>("virtual_walls/modalities_debug/position", 1000);
}

/**
 * \brief Update the effect of the virtual walls on the velocity twist at a timestep.
 * 
 * \param data_in input velocity as Twist
 * \param data_out output velocity as Twist
 */
template <typename T>
bool VirtualWalls<T>::update(const T& data_in, T& data_out)
{
//         ROS_INFO_THROTTLE(1, "[virutal_walls.h] update");
    // Initialize the output velocity with the input velocity
    data_out = data_in;
    
    // If there are no walls configured, return
    if (wall_base_points_.size() < 1)
    {
        return true;
    }
    
    // Update the current robot location
    updateRobotLocation();
    tf2::Vector3 robot_location;
    tf2::fromMsg(transform_map_robot_.transform.translation, robot_location);
    pub_position_.publish(tf2::toMsg(robot_location));
    
    // For all walls, calculate the resulting forces
    tf2::Vector3 resulting_force;
    resulting_force.setZero();
		double min_distance_to_wall = 0.0;
		double min_effective_radius = max_effective_radius_;
		double current_distance_to_wall;
		last_scaled_virtual_force_.setZero();
    for(int i = 0; i < wall_base_points_.size(); i++)
    {
        tf2::Vector3 to_base = wall_base_points_[i] - robot_location;
        tf2::Vector3 to_end = wall_end_points_[i] - robot_location;
        tf2::Vector3 base_to_end = wall_end_points_[i] - wall_base_points_[i];
        tf2::Vector3 help_vector = base_to_end / pow(base_to_end.length() , 2);
        tf2::Vector3 to_base_parallel = tf2::tf2Dot(to_base, base_to_end) * help_vector;
        tf2::Vector3 to_base_vertical = to_base - to_base_parallel;
        tf2::Vector3 to_end_parallel = tf2::tf2Dot(to_end, base_to_end) * help_vector;
        
        // Check if robot is robot is at the edges of the wall. If so, the distance to wall is defined by the distance to the end or base point. If not, the distance is defined as the vertical distance to the wall.
        tf2::Vector3 wall_to_robot;
        wall_to_robot.setZero();
        if (to_base_parallel.length() > base_to_end.length() || to_end_parallel.length() > base_to_end.length())
        {
// 						ROS_INFO("[virutal_walls.h] parallel");
            wall_to_robot = to_base.length() < to_end.length() ? to_base * (-1): to_end * (-1);
        }
        else {
// 						ROS_INFO("[virutal_walls.h] not parallel");
            wall_to_robot = to_base_vertical * (-1);
        }
        double dist_wall_robot = wall_to_robot.length();
        current_distance_to_wall = dist_wall_robot;
// 				ROS_INFO("[virutal_walls.h] distance to Wall = %.2f", dist_wall_robot);
// 				ROS_INFO("[virutal_walls.h] effectiv radius = %.2f", effective_radii_[i]);
        // If robot is out of the effective area of the wall, continue with the next wall.
        if (dist_wall_robot > effective_radii_[i])
        {
// 						ROS_INFO("[virutal_walls.h] outside of effective_radius");
            continue;
        }
				
				if(effective_radii_[i] < min_effective_radius){
					min_effective_radius = effective_radii_[i];
				}	
        if(dist_wall_robot < min_distance_to_wall)
				{
					min_distance_to_wall = dist_wall_robot;
				}
        ROS_INFO("[virutal_walls.h] INSIDE");
        // Calculate the distance factor
        double distance_factor = 0; // [0...1]
        if (force_distance_functions_[i] == "trapezoidal") {
            double trapezoid_gradient;
            trapezoid_gradient = 1 / ((params_.trapezoid_max_at_percent_radius - 1) * effective_radii_[i]);
            distance_factor = trapezoid_gradient * dist_wall_robot - trapezoid_gradient * effective_radii_[i];
            if (distance_factor > 1){
                distance_factor = 1;
            }
        }
        else if (force_distance_functions_[i] == "trigonometrical") {
            distance_factor = cos(3.141592653589793238463 * dist_wall_robot / effective_radii_[i]) * 0.5 + 0.5;
        }
        else if (force_distance_functions_[i] == "gaussian") {
            // general normal distribution fuction, mu=0, maximum normalized to 1
            // 4*sigma=effective_radii_[i] --> distance_factor at effective_radii_[i] = 0.0003
            distance_factor = exp(-0.5 * pow(dist_wall_robot * 4 / effective_radii_[i], 2));
        }
        else {
            ROS_WARN_STREAM("VirtualForces: Force distance function of type '" << force_distance_functions_[i] << " not implemented! Force is deactivated!");
        }        
//         switch (force_distance_functions_[i]){  
//             case 0: // trapezoidal
//             double trapezoid_gradient;
//             trapezoid_gradient = 1 / ((params_.trapezoid_max_at_percent_radius - 1) * effective_radii_[i]);
//             distance_factor = trapezoid_gradient * dist_wall_robot - trapezoid_gradient * effective_radii_[i];
//             if (distance_factor > 1){
//                 distance_factor = 1;
//             }
//             break;  
//             case 1: // trigonometrical
//             distance_factor = cos(3.141592653589793238463 * dist_wall_robot / effective_radii_[i]) * 0.5 + 0.5;
//             break;
//             case 2: // gaussian
//             // general normal distribution fuction, mu=0, maximum normalized to 1
//             // 4*sigma=effective_radii_[i] --> distance_factor at effective_radii_[i] = 0.0003
//             distance_factor = exp(-0.5 * pow(dist_wall_robot * 4 / effective_radii_[i], 2));
//             break;  
//         }
        
        // Add the force of the wall to the resulting force (sum of all wall forces)
//         ROS_INFO("Resulting force %f, Distance factor %f, max_force %f, wall_to_robot %f, dist_wall_robot %f", resulting_force[i], distance_factor, params_.max_force, wall_to_robot[i], dist_wall_robot);
        if (dist_wall_robot != 0) {
            resulting_force += distance_factor * params_.max_force * wall_to_robot / dist_wall_robot;
						last_scaled_virtual_force_ += resulting_force;
        }
    }
    
    // Transform the force vector from map frame to base_link frame
    tf2::Quaternion rotation_q;
    tf2::fromMsg(transform_map_robot_.transform.rotation, rotation_q);
    rotation_q = rotation_q.inverse();
    resulting_force = tf2::quatRotate(rotation_q, resulting_force);
//     ROS_INFO("[virutal_walls.h] resulting_force= {x:%.2f}, {y:%.2f}, {z:%.2f}", resulting_force.getX(), resulting_force.getY(), resulting_force.getZ() );
    // Translate the virtual force to a virtual velocity by adopting a mass damping system.
    tf2::Vector3 resulting_velocity = previous_resulting_velocity_ * mds_v_ + previous_wall_force_ * mds_f_;
    
    // Save the force and velocity values for the next timestep
    previous_resulting_velocity_ = resulting_velocity;
    previous_wall_force_ = resulting_force;
    
    // Add the virtual velocity to the output velocity
		if(limit_human_velocity_){
			tf2::Vector3 human_vel(data_in.linear.x, data_in.linear.y, data_in.linear.z);
			double limit_factor = 1.0;
			
// 			double angle = (tf2::tf2Angle(human_vel, resulting_velocity) * rad_to_deg_const_);
			
			//HACK
			if((current_distance_to_wall < min_effective_radius)) {
				double length_to_min_distance = min_effective_radius - min_allowed_distance_to_wall_;
				double distance_to_min_distance = (current_distance_to_wall >= min_allowed_distance_to_wall_) ?  (current_distance_to_wall - min_allowed_distance_to_wall_) : 0.0;
// 				ROS_INFO("min_effective_radius,[%f]", min_effective_radius);
// 				ROS_INFO("length_to_min_distance,[%f]", length_to_min_distance);
// 				ROS_INFO("current_distance_to_wall,[%f]", current_distance_to_wall);
// 				ROS_INFO("current distance to min distance,[%f]", distance_to_min_distance);
				if(length_to_min_distance >= distance_to_min_distance){
					double dist_factor = ((length_to_min_distance - distance_to_min_distance) / length_to_min_distance);
					limit_factor = 1.0 -((1.0 - limit_human_vel_at_min_) * dist_factor);
					
					if(limit_factor > 1.0 || limit_factor < 0.0) {
						limit_factor = 1.0;
					}
// 					ROS_INFO("dist_factor,[%f]", dist_factor);
// 					ROS_INFO("limit_factor,[%f]", limit_factor);
				}else{
					ROS_WARN("[VirtualWalls.h] Something went wrong!");
				}
			}
				
// 					ROS_INFO("ANGLE %f", angle );
// 			
			
// 			if((current_distance_to_wall < min_effective_radius) && (human_vel.length() > (human_vel+resulting_velocity).length())){
// 				double length_to_min_distance = min_effective_radius - min_allowed_distance_to_wall_;
// 				double distance_to_min_distance = (current_distance_to_wall >= min_allowed_distance_to_wall_) ?  (current_distance_to_wall - min_allowed_distance_to_wall_) : 0.0;
// 				ROS_INFO("min_effective_radius,[%f]", min_effective_radius);
// 				ROS_INFO("length_to_min_distance,[%f]", length_to_min_distance);
// 				ROS_INFO("current_distance_to_wall,[%f]", current_distance_to_wall);
// 				ROS_INFO("current distance to min distance,[%f]", distance_to_min_distance);
// 				if(length_to_min_distance >= distance_to_min_distance){
// 					limit_factor = 1.0-((length_to_min_distance - distance_to_min_distance) / length_to_min_distance);
// 					ROS_INFO("limit_factor,[%f]", limit_factor);
// 				}else{
// 					ROS_WARN("Something went wrong!");
// 				}
// 			} else {
// 				if((current_distance_to_wall < min_effective_radius)){
// 					ROS_INFO("##########DRIVE_AWAY##############");
// 					ROS_INFO("limit_factor,[%f]", limit_factor);
// 				}
// 			}
				
			last_limit_vel_factor_ = limit_factor;
			data_out.linear.x = limit_factor * data_out.linear.x + resulting_velocity.getX();
			data_out.linear.y = limit_factor * data_out.linear.y + resulting_velocity.getY();
			data_out.linear.z = limit_factor * data_out.linear.z + resulting_velocity.getZ();

		}else {
			data_out.linear.x = data_out.linear.x + resulting_velocity.getX();
			data_out.linear.y = data_out.linear.y + resulting_velocity.getY();
			data_out.linear.z = data_out.linear.z + resulting_velocity.getZ();
		}
    // debug
    pub_resulting_force_.publish(tf2::toMsg(resulting_force * 0.01)); //Scaled down by 100 to better fit in a plot with the other values.
    pub_resulting_velocity_.publish(tf2::toMsg(resulting_velocity));
    pub_velocity_in_.publish(data_in.linear);   
    pub_velocity_out_.publish(data_out.linear);
    
    return true;
}

/**
 * \brief Callback function for dynamic reconfigure.
 */
template <typename T>
void VirtualWalls<T>::reconfigureRequest(VirtualWallsConfig& config, uint32_t level)
{   
    // update params hold by the params_-struct
    params_.fromConfig(config);
    
    // recalculate the params of the mass damping system
    double K = params_.max_velocity / params_.max_force;
    double factor = 1 - exp( -1 / (params_.time_const_T * params_.controller_update_rate));
    mds_f_ = K * factor;
    mds_v_ = 1 - factor;
		limit_human_velocity_ = params_.limit_human_velocity;
		min_allowed_distance_to_wall_ = params_.min_distance_to_wall;
		limit_human_vel_at_min_ = params_.limit_human_vel_at_min;
};

/**
 * \brief Retrieve the virtual wall configuration from the parameter server.
 */
template <typename T>
bool VirtualWalls<T>::configure() {
    
    wall_base_points_.clear();
    
    // Initialize mass-damping-system
    previous_wall_force_.setZero();
    previous_resulting_velocity_.setZero();
    
    // Namespaces
    std::string path_walls = "/" + ns_params_ptr_->project_ns + "/" + ns_params_ptr_->scenario_ns + "/" + ns_params_ptr_->wall_ns;  
    
    // Load the list of wall names
    std::vector<std::string> wall_names;
    ros::param::get(path_walls + "/" + ns_params_ptr_->config_ns + "/" + ns_params_ptr_->wall_ns + "_names", wall_names);
    if (wall_names.size() < 1)
    {
        ROS_WARN_STREAM("VirtualWalls: No valid wall configuration found in namespace " <<  path_walls << ". \n VirtualWalls-Modality will not be used!");
    }
    else {
        ROS_INFO_STREAM("VirtualWalls: Found " << wall_names.size() << " wall(s) in namespace "<< path_walls << ".");  
        
        // Retrieve and store all wall data based on the wall names
				max_effective_radius_ = 0.0;
        for(int i = 0; i < wall_names.size(); i++){
            
            std::string path_data_wall_i = path_walls + "/" + ns_params_ptr_->data_ns + "/" + wall_names[i];
        
            // base point (L)
            geometry_msgs::Point base_point_msg;
            ros::param::get(path_data_wall_i + "/L" + "/x", base_point_msg.x);
            ros::param::get(path_data_wall_i + "/L" + "/y", base_point_msg.y);
            ros::param::get(path_data_wall_i + "/L" + "/z", base_point_msg.z);
            tf2::Vector3 base_point;
            tf2::fromMsg(base_point_msg, base_point);
						ROS_INFO("[virutal_walls.h] Base_points= {x:%.2f}, {y:%.2f}, {z:%.2f}", base_point.getX(), base_point.getY(), base_point.getZ() );
            wall_base_points_.push_back(base_point);
            
            // end point (R)
            geometry_msgs::Point end_point_msg;
            ros::param::get(path_data_wall_i + "/R" + "/x", end_point_msg.x);
            ros::param::get(path_data_wall_i + "/R" + "/y", end_point_msg.y);
            ros::param::get(path_data_wall_i + "/R" + "/z", end_point_msg.z);
            tf2::Vector3 end_point;
            tf2::fromMsg(end_point_msg, end_point);
            wall_end_points_.push_back(end_point);
            
            // effective radius
            double effective_radius;
            ros::param::get(path_data_wall_i + "/" + ns_params_ptr_->area_ns + "/" + ns_params_ptr_->area_ns, effective_radius);
                if( effective_radius > max_effective_radius_)
                {
                        max_effective_radius_ = effective_radius;
                }
	          effective_radii_.push_back(effective_radius);
            
            // distance function
            std::string force_distance_function;
            ros::param::get(path_data_wall_i + "/" + ns_params_ptr_->force_distance_function_ns, force_distance_function);
            force_distance_functions_.push_back(force_distance_function);
        }
    }
    
    return true;
}

};

#endif
