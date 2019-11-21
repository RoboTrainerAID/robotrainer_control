#ifndef ROBOTRAINER_MODALITIES_VIRTUAL_FORCES_H
#define ROBOTRAINER_MODALITIES_VIRTUAL_FORCES_H

#include <robotrainer_modalities/modality_base.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotrainer_modalities/VirtualForcesParameters.h>
#include <robotrainer_modalities/VirtualForcesConfig.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>

namespace robotrainer_modalities {
    
/**
 * \brief A class that contains the modality plugin for virtual forces.
 * 
 * Provides methods for loading a virtual forces configuration (a set of virtual forces) from the parameter server and applying their effect to a a velocity wrench.
 */
template <typename T>
class VirtualForces : public robotrainer_modalities::ModalityBase<T> {
        
public:
        VirtualForces();
        virtual bool update( const T & data_in, T& data_out);
        virtual bool configure();
        tf2::Vector3 getLastVelocity() {
                return resulting_velocity_linear_last_;
        }
        tf2::Vector3 getLastscaledForce() {
                return last_scaled_virtual_force_;
        }
        
        //HACK
        void setForceLimit(tf2::Vector3 max_force) {
                this->force_limit_ = max_force;
        }
        void setUseLimit(bool use_limit){
                this->use_limit_ = use_limit;
        }
    
private:
    
        bool use_limit_ = false;
        tf2::Vector3 force_limit_{0.0,0.0,0.0};
                        
        // Objects for publishing messages
        // Debug purpose only. Only for the first virtual force.
        ros::NodeHandle nh_;
        ros::Publisher pub_position_;
        ros::Publisher pub_velocity_in_;
        ros::Publisher pub_velocity_out_;
        ros::Publisher pub_resulting_velocity_;
        ros::Publisher pub_resulting_force_;
        //     realtime_tools::RealtimePublisher realtime_pub_resulting_force_;
        
        // Objects and methods for locating the robot
        using ModalityBase<T>::updateRobotLocation;
        using ModalityBase<T>::transform_map_robot_;
        
        // Shared params
        using ModalityBase<T>::ns_params_ptr_;
        
        // Objects for param_handler and dynamic_reconfigure
        robotrainer_modalities::VirtualForcesParameters params_;
        void reconfigureRequest(const robotrainer_modalities::VirtualForcesConfig& config, uint32_t level);
        dynamic_reconfigure::Server<robotrainer_modalities::VirtualForcesConfig> dynamic_reconfigure_server_;
        
        // Params for the mass damping system
        double mds_f_; ///< Precalculated parameter for the mass damping system equation.
        double mds_v_; ///< Precalculated parameter for the mass damping system equation.
        
        bool use_admitance_velocity_;
        double damping_inverse_;

        // Vector-containers to hold the virtual force configuration
        std::vector<tf2::Vector3> forces_; ///< Contains the force vectors.
        std::vector<tf2::Vector3> area_center_points_; ///< Contains the area center points.
        std::vector<double> area_radii_; ///< Contains the area radii.
        std::vector<std::string> force_distance_functions_; ///< Contains the  force(distance)-function to be applied.
        
        // Storages for the values of the previous timestep
        std::vector<tf2::Vector3> prev_virt_forces_; ///< Stores the force values of the previous timestep.
        std::vector<tf2::Vector3> prev_virt_velocities_linear_; ///< Stores the velocity values of the previous timestep.
        std::vector<bool> prev_affected_by_; ///< Stores by which virtual force the robot has been affected in the previous timestep.
        std::vector<tf2::Vector3> velocities_on_entry_; ///< Stores the velocity of the robot when entering a force area.
        
        //
        tf2::Vector3 resulting_velocity_linear_last_;
        tf2::Vector3 last_scaled_virtual_force_;

};

/** 
 * \brief Constructor for VirtualForces object.
 * 
 * * params_ and the dynamic_reconfigure_server get a node handle that is not in conflict with other functions.
 * * The pramas_ struct get all parameters from the parameter server.
 * * tf2-objects, dynamic reconfigure and publisher objects get initialized and configured.
 */
template <typename T> VirtualForces<T>::VirtualForces() : params_{ros::NodeHandle("/modalities/virtual_forces")}, dynamic_reconfigure_server_{ros::NodeHandle("/modalities/virtual_forces")} {
        // Initialize params struct and the dynamic_reconfigure server with a node handle.
        // Get all parameter values from the parameter server.
        params_.fromParamServer();
        
        // Dynamic reconfigure
        dynamic_reconfigure_server_.setCallback(boost::bind(&VirtualForces<T>::reconfigureRequest, this, _1, _2));
        
        // Publish messages (debugging purpose)
        pub_position_ = nh_.advertise<geometry_msgs::Vector3>("virtual_forces/virtual_forces/modalities_debug/position", 1000);
        pub_velocity_in_ = nh_.advertise<geometry_msgs::Vector3>("virtual_forces/modalities_debug/velocity_in", 1000);
        pub_velocity_out_ = nh_.advertise<geometry_msgs::Vector3>("virtual_forces/modalities_debug/velocity_out", 1000);
        pub_resulting_velocity_ = nh_.advertise<geometry_msgs::Vector3>("virtual_forces/modalities_debug/resulting_velocity", 1000);
        // pub_resulting_force_ = nh_.advertise<geometry_msgs::WrenchStamped>("virtual_forces/modalities_debug/resulting_force", 1000);
        pub_resulting_force_ = nh_.advertise<geometry_msgs::Vector3>("virtual_forces/modalities_debug/resulting_force", 1000);
}

/**
 * \brief Update the effect of the virtual forces on the velocity twist at a timestep.
 * 
 * 1. This calcultes (for the current timestep) the virtual force based on its location and the force configuration stored in
 * the corresponing vector container objects. The robot is affeced by a virtual force if it is within in effective area. Furthermore the
 * virtual force is scaled depending on the robots distance to its area center according to a fuction that controlls a distance
 * factor. This distance function can be configured to look like a trapezoid, a trigonometrical function or gaussian function.
 * 
 * 2. (Optional) When a user moves through a force area with a high velocity he will get less deviated by the virtual force than he 
 * would get if he moved slower. This can be compensated by multiplying the virtual force with a factor that depends from a reference
 * velocity. Note that the virtual force should not get reduced at velocities slower that the reference velocity as this would not
 * be the intended training behaviour.
 * The compensation can be activated and configured using dynamic reconfigure.
 * 
 * 3. Each scaled force is then indiviually translated to a virtual velocity in adoption of on a mass-damping-system. 
 * The mass-damping-system is a specialized PT1 element. It recusivly calculates the sampled virtual velocity value of the current 
 * timestep depending on the previous virtual velocity and the previous virtual force:
 * > Continuous mass-damping-system: \n
 * > m * a(t) + d * v(t) = F(t) \n \n
 * > Continuous mass-damping-system in the form of a generic PT1 element: \n
 * > T * a(t) + v(t) = K * F(t) \n \n
 * > The PT1 elemnt parameters result in: \n
 * > K = 1 / d = max_velocity / max_force\n
 * > T = m / d
 * > Time discrete PT1 element (recursive equation): \n
 * > v(t_2) = v(t_1) * (1-C) + F(t_1) * K * C with C = 1 - exp(-1 / (r * T))
 * > > K : PT1 element gain
 * > > T : PT1 element time constant 
 * > > v : (virtual) velocity [m/s] \n
 * > > a : (virtual) acceleration [m/s^2]
 * > > F : (virutal) force [N] \n
 * > > d : damping intensity [kg/s] \n
 * > > m : (virtual) mass [kg] \n
 * > > r : controller update rate [1/s]
 * > > t : continuous time
 * > > t_1 : previous timestep
 * > > t_2 : current timestep
 * The parameters of the PT1 element equation (K and T) are convenient to configure the robots behaviour,* of a PT1 element. In terms 
 * of a mass-damping-system, K is proportional to the maximum virtual * velocity. From a standstill the robot would accelerate 
 * within 3*T to 0.95*K.
 * The maximum (virtual) velocity and force as well as T can be dynamically reconfigured.
 * 
 * 4. Lastly each virtual velocity is superposed with the input velocity to get the  output velocity. Note that the virtual velocity 
 * is independent of the input velocity and the previous output velocity.
 * 
 * Before applying this in a timestep, the configure() method needs to be called once in order to load the force configuration from 
 * the parameter server into the vector container objects.
 * 
 * \param data_in input velocity as Twist
 * \param data_out output velocity as Twist
 */
template <typename T> bool VirtualForces<T>::update(const T& data_in, T& data_out) {
        // Initialize the output velocity with the input velocity
        data_out = data_in;
    
        // Update the current robot location in stored in transform_map_robot_
        updateRobotLocation();
        
        // Debug
        bool publish = true;
        
        // needed for velocity compensation
        std::vector<bool> on_entry;
        
        if (forces_.size() < 1) {
                return true;
        }
    
        resulting_velocity_linear_last_.setZero();
        last_scaled_virtual_force_.setZero();
    
        // For all forces found in the configuration, add its effect to the output velocity:
        for(int i = 0; i < forces_.size(); i++) {
                // Get the force area
                double area_radius = area_radii_[i];
                tf2::Vector3 area_center = area_center_points_[i];
                
                // Get the current position of the robot     
                tf2::Vector3 current_position;
                tf2::fromMsg(transform_map_robot_.transform.translation, current_position);
                
                // Calculate distance of robot to the center of the force area
                double distance_to_center = tf2::tf2Distance(area_center, current_position);
                
                // Check if the robot is inside of the force area
                bool inside_area = distance_to_center < area_radius;
                
                // Init on_entry
                on_entry.push_back(false);
                
                // If the robot is outside of the area radius and the inertia of the virtual force is small (or zero), continue with the next virtual force from the configuration.
                if (!inside_area && prev_virt_velocities_linear_[i].length() < 0.001)
                {
                        prev_virt_velocities_linear_[i].setZero();
                        pub_position_.publish(tf2::toMsg(current_position));
                        continue;
                }
                
                // Get the virtual force value in the center of the area.
                tf2::Vector3 center_force = forces_[i];
                
                // Logging, if the robot moved form the outside of a force area to the inside
        
                if (inside_area) {
                        ROS_WARN_STREAM_COND(prev_affected_by_[i] == false , "Within effective area of " << i << ". Virtual Force: [" << center_force.getX() << ", " << center_force.getY() << ", " << center_force.getZ() <<"]. Area radius:" << area_radius);
                        ROS_INFO_COND(prev_affected_by_[i] == false ,"Forces influence START [x:%.2f],[y:%.2f],                [z:%.2f]", current_position.getX(), current_position.getY(), current_position.getZ());
                        if (prev_affected_by_[i] == false){
                                on_entry[i] = true;
                        }
                        prev_affected_by_[i]  = true;
                } else {
                        ROS_WARN_STREAM_COND(prev_affected_by_[i] == true , "Out of the effective area of " << i << ".");
                        ROS_INFO_COND(prev_affected_by_[i] == true, "Forces influence STOP [x:%.2f],[y:%.2f],                [z:%.2f]",  current_position.getX(), current_position.getY(), current_position.getZ());
                        prev_affected_by_[i]  = false;
                }
                
                // Initialize the resulting force vector with zero and only calculate it, if the robot is inside of the force area
                tf2::Vector3 resulting_force;
                resulting_force.setZero();
                if (inside_area) {
                        // Scale down the virtual force depending on the robots distance from the center of the force area and preselected function for the distance factor. 
                        double distance_factor = 0; // [0...1]
                        if (force_distance_functions_[i] == "trapezoidal") {
                                double trapezoid_gradient;
                                trapezoid_gradient = 1 / ((params_.trapezoid_max_at_percent_radius - 1) * area_radius);
                                distance_factor = trapezoid_gradient * distance_to_center - trapezoid_gradient * area_radius;
                                if (distance_factor > 1){
                                        distance_factor = 1;
                                }
                        } else if (force_distance_functions_[i] == "trigonometrical") {
                                distance_factor = cos(3.141592653589793238463 * distance_to_center / area_radius) * 0.5 + 0.5;
                        } else if (force_distance_functions_[i] == "gaussian") {
                                // general normal distribution fuction, mu=0, maximum normalized to 1
                                // 4*sigma=area_radius --> distance_factor at area edge = 0.0003
                                distance_factor = exp(-0.5 * pow(distance_to_center * 4 / area_radius, 2));
                        } else {
                                ROS_WARN_STREAM_THROTTLE(1, "VirtualForces: Force distance function of type '" << force_distance_functions_[i] << "' not implemented! Force is deactivated!");
                        }
                
                        tf2::Vector3 scaled_force = distance_factor * center_force;
                        
                        // Transform the force vector from map frame to base_link frame
                        tf2::Quaternion rotation_q;
                        tf2::fromMsg(transform_map_robot_.transform.rotation, rotation_q);
                        rotation_q = rotation_q.inverse();
                        resulting_force = tf2::quatRotate(rotation_q, scaled_force);
                        if(use_limit_){
                                ROS_INFO("[virtual_forces.h] use limit");
                                if(resulting_force.length() >= force_limit_.length()) {
                                        ROS_INFO("[virtual_forces.h] limit Force");
                                        resulting_force /= resulting_force.length();
                                        resulting_force *= force_limit_.length();
                                }
                        }
                
                        // Compensate velocities higher than the reference velocity
                        if (params_.compensate_velocity && !on_entry[i]) {
                                tf2::Vector3 velocity_out;
                                tf2::fromMsg(data_out.linear, velocity_out);
                                
                                //projection of current input velocity on the velocity on entry of the area.
                                tf2::Vector3 actual_velocity = tf2::tf2Dot(velocity_out, velocities_on_entry_[i]) / pow(velocities_on_entry_[i].length(), 2) * velocities_on_entry_[i];
                                double abs_actual_velocity = actual_velocity.length();
                                if (abs_actual_velocity > params_.compensation_reference_velocity){
                                double param_actual = 2 * area_radius / (params_.time_const_T * abs_actual_velocity);
                                double param_ref = 2 * area_radius / (params_.time_const_T * params_.compensation_reference_velocity);
                                double compensation_factor = ( param_ref + exp(- param_ref) - 1 ) / ( param_actual + exp(- param_actual) -1 );
                                resulting_force = resulting_force * compensation_factor;
                                }
                        }
                }
                
                tf2::Vector3 resulting_velocity_linear;
                // Translate the virtual force to a virtual velocity by adopting a mass damping system.
                if(use_admitance_velocity_) {
                        resulting_velocity_linear = prev_virt_forces_[i] * damping_inverse_;
                } else {
                        resulting_velocity_linear = prev_virt_velocities_linear_[i] * mds_v_ + prev_virt_forces_[i] * mds_f_;
                }

                //#########
                resulting_velocity_linear_last_ += resulting_velocity_linear;
                last_scaled_virtual_force_ += resulting_force;
                
                // Save the force and velocity values for the next timestep
                prev_virt_forces_[i] = resulting_force;
                prev_virt_velocities_linear_[i] = resulting_velocity_linear;
                // ROS_INFO("CURRENT RESULTING Force: [x=%.4f],[y=%.4f],[z=%.4f]",resulting_force.getX(),resulting_force.getY(),resulting_force.getZ());
                
                // Add the virtual velocity to the output velocity
                data_out.linear.x = data_out.linear.x + resulting_velocity_linear.getX();
                data_out.linear.y = data_out.linear.y + resulting_velocity_linear.getY();
                data_out.linear.z = data_out.linear.z + resulting_velocity_linear.getZ();
                
                // debug messages (only when affected) (only for first virtual force)
                if (publish) {
                // geometry_msgs::WrenchStamped resulting_force_wrench;

                // resulting_force_wrench.header.frame_id = "map";
                // resulting_force_wrench.wrench.force = tf2::toMsg(resulting_force);
                /* pub_resulting_force_.publish(resulting_force_wrench)*/; //Scaled down by 100 to better fit in a plot with the other values.

                pub_resulting_force_.publish(	tf2::toMsg(resulting_force));
                pub_resulting_velocity_.publish(tf2::toMsg(resulting_velocity_linear));
                // publish = false; //make sure that only the first virtual force is published	
                }
        }
        
        // Save output velocity when entering a force area
        for(int i = 0; i < forces_.size(); i++) {
                if (on_entry[i]) {
                        tf2::Vector3 velocity_out;
                        tf2::fromMsg(data_out.linear, velocity_out);
                        velocities_on_entry_[i] = velocity_out;
                }
        }
                        
        // ROS_INFO("##RESULTING FORCE##: [x=%.4f],[y=%.4f],[z=%.4f]", last_scaled_virtual_force_.getX(), last_scaled_virtual_force_.getY(), last_scaled_virtual_force_.getZ());
        // debug
        pub_velocity_in_.publish(data_in.linear);   
        pub_velocity_out_.publish(data_out.linear);
        
        return true;
}

/**
 * \brief Callback function for dynamic reconfigure.
 */
template <typename T> void VirtualForces<T>::reconfigureRequest(const robotrainer_modalities::VirtualForcesConfig& config, uint32_t level) {   
        // update params hold by the params_-struct
        params_.fromConfig(config);
        
        // recalculate the params of the mass damping system
        double K = params_.max_velocity / params_.max_force;
        double factor = 1 - exp( -1 / (params_.time_const_T * params_.controller_update_rate));
        mds_f_ = K * factor;
        mds_v_ = 1 - factor;
        // B_ = 1/K;
        use_admitance_velocity_ = params_.use_admitance_velocity;
        damping_inverse_  = K;

        // Logging
        ROS_DEBUG_STREAM("VirtualForces: Dynamic reconfigure."
                << "\n    max_force:            " << params_.max_force
                << "\n    max_velocity          " << params_.max_velocity
                << "\n    controller_update_rate: " << params_.controller_update_rate
                << "\n    time_const_T: " << params_.time_const_T
        );
};


/**
 * \brief Retrieve the virtual force configuration from the parameter server.
 * 
 * The list of force names is loaded from the parameter server and stored in vector force_names.
 * 
 * Based on the force names the force data can be found on the parameter server as well:
 * * the base points and the end points of arrows that defines a force in direction, magnitude and location (location not relevant here)
 * * the center points of the effective areas of the forces
 * * the edge point of the effective areas of the forces
 * 
 * The data is then adapted and stored in vector containers accessible by consistent index.
 * * the force vectors in forces_
 * * the area center point in area_center_points_
 * * the area radius in area_radii_
 * 
 * Also in this method the vector containers that hold the virtual velocities and forces of the previous timesteps and others get initialized with zeros.
 */
template <typename T> bool VirtualForces<T>::configure() {
        ROS_INFO("VirtualForces config");
        forces_.clear();
        
        // Namespaces
        std::string path_forces = "/" + ns_params_ptr_->project_ns + "/" + ns_params_ptr_->scenario_ns + "/" + ns_params_ptr_->force_ns;
        
        // Load the list of force names
        std::vector<std::string> force_names;
        ros::param::get(path_forces + "/" + ns_params_ptr_->config_ns + "/" + ns_params_ptr_->force_ns + "_names", force_names);
        if (force_names.size() < 1) {
                ROS_WARN_STREAM("VirtualForces: No force configuration found in namespace " << path_forces << ".\n VirtualForces-Modality will not be used!");
        } else {
                ROS_INFO_STREAM("VirtualForces: Found " << force_names.size() << " virtual force(s) in namespace "<< path_forces << ".");
                
                // Retrieve and store all force related data based on the force names
                for(int i = 0; i < force_names.size(); i++) {
                        std::string path_data_force_i = path_forces + "/" + ns_params_ptr_->data_ns + "/" + force_names[i];
                        
                        // force data
                        geometry_msgs::Vector3 force_msg;
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->arrow_ns + "/x", force_msg.x);
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->arrow_ns + "/y", force_msg.y);
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->arrow_ns + "/z", force_msg.z);
                        tf2::Vector3 force;
                        tf2::fromMsg(force_msg, force);
                        forces_.push_back(force);
                        // ROS_INFO_STREAM("Force:" << i << " ||[x: " << force.getX() << "] [y: " << force.getY() <<"] [z: " << force.getZ() << "]");
                        
                        // center point of the effective area of the force
                        geometry_msgs::Point area_center_msg;
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->area_ns + "/x", area_center_msg.x);
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->area_ns + "/y", area_center_msg.y);
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->area_ns + "/z", area_center_msg.z);
                        tf2::Vector3 area_center;
                        tf2::fromMsg(area_center_msg, area_center);
                        area_center_points_.push_back(area_center);
                        // if(area_center){
                        //      ROS_INFO_STREAM("Area:" << i << " ||[x: " << area_center.getX() << "] [y: " << area_center.getY() <<"] [z: " << area_center.getZ() << "]");
                        // }else{
                        //       ROS_INFO("No area_center");
                        // }
                        // calculate area radius from edge point
                        geometry_msgs::Point area_edge_msg;
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->margin_ns + "/x", area_edge_msg.x);
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->margin_ns + "/y", area_edge_msg.y);
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->margin_ns + "/z", area_edge_msg.z);
                        tf2::Vector3 area_edge;
                        tf2::fromMsg(area_edge_msg, area_edge);
                        area_radii_.push_back(tf2::tf2Distance(area_center, area_edge));
                        // if(area_edge){
                        //      ROS_INFO_STREAM("Area_edge:" << i << " ||[x: " << area_edge.getX() << "] [y: " << area_edge.getY() <<"] [z: " << area_edge.getZ() << "]");
                        // }else{
                        //      ROS_INFO("No area_edge");
                        // }
                        // force area configuration
                        std::string force_distance_function;
                        ros::param::get(path_data_force_i + "/" + ns_params_ptr_->arrow_ns + "/force_distance_function", force_distance_function);
                        force_distance_functions_.push_back(force_distance_function);
                        // ROS_INFO_STREAM("Distancefunction:" << force_distance_function);
                        
                        // Initialize mass-damping-system and other virtual force related vars
                        tf2::Vector3 zero;
                        zero.setZero();
                        prev_virt_forces_.push_back(zero);
                        prev_virt_velocities_linear_.push_back(zero);
                        prev_affected_by_.push_back(false);
                        velocities_on_entry_.push_back(zero);
                }
        }
        return true;
}

};

#endif
