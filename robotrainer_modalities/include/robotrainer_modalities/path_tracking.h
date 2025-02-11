#ifndef ROBOTRAINER_MODALITIES_PATH_TRACKING_H
#define ROBOTRAINER_MODALITIES_PATH_TRACKING_H

#include <robotrainer_modalities/modality_base.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotrainer_modalities/PathTrackingParameters.h>
#include <robotrainer_modalities/PathTrackingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <cmath>

namespace robotrainer_modalities
{
    
/**
 * \brief A class that contains the modality plugin for path tracking
 * 
 * Provides methods for loading several path sections (lists of points in a specific
 * order) from the parameter sever and using it to prevent the user from getting to far
 * away from the current path section by manipulating an output velocity wrench.
 */
template <typename T>
class PathTracking : public robotrainer_modalities::ModalityBase<T>
{
public:
    PathTracking();
    virtual bool update( const T & data_in, T& data_out);
    virtual bool configure();
    
private:   
    double getDistancePointLine(double px, double py, double l1x, double l1y, double l2x, double l2y);
    
    // Objects and methods for locating the robot
    using ModalityBase<T>::updateRobotLocation;
    using ModalityBase<T>::transform_map_robot_;
    
    // Shared params
    using ModalityBase<T>::ns_params_ptr_;
    
    // Objects for param_handler and dynamic_reconfigure
    robotrainer_modalities::PathTrackingParameters params_;
    void reconfigureRequest(PathTrackingConfig& config, uint32_t level);
    dynamic_reconfigure::Server<robotrainer_modalities::PathTrackingConfig> dynamic_reconfigure_server_;
    
    // Params for the mass damping system
    double mds_f_; ///< Precalculated parameter for the mass damping system equation.
    double mds_v_; ///< Precalculated parameter for the mass damping system equation.
    
    // Storages for the values of the previous timestep
    tf2::Vector3 previous_force_; ///< Stores the force value of the previous timestep.
    tf2::Vector3 previous_velocity_linar_; ///< Stores the velocity value of the previous timestep.
    
    // Vector-containers to hold the path tracking sections
    std::vector<std::vector<tf2::Vector3>> path_sections_; ///< Stores the sections of the path, where pathtracking should be enabled. A path section consists of path points.
    std::vector<double> max_deviations_; ///< Stores the maximum deviation the path segments
    std::vector<std::string> force_distance_functions_; ///< Contains the  force(distance)-function to be applied.
    
    int pos_prev_segment_; ///< Postion of previous nearest path point in path_sections_[current_section_]
    int current_section_; ///< The current path section, where tracking is active
    
    // Objects for publishing messages
    // Debug purpose only
    ros::NodeHandle nh_;
    ros::Publisher pub_position_;
    ros::Publisher pub_abs_attraction_force_;
    ros::Publisher pub_velocity_in_;
    ros::Publisher pub_velocity_out_;
    ros::Publisher pub_resulting_velocity_;
    ros::Publisher pub_resulting_force_;
    ros::Publisher pub_distance_;
    
};

/** 
 * \brief Constructor for PathTracking object.
 * 
 * * params_ and the dynamic_reconfigure_server get a node handle that is not in conflict with other functions.
 * * The pramas_ struct get all parameters from the parameter server.
 * * tf2-objects, dynamic reconfigure and publisher objects get initialized and configured.
 */
template <typename T>
PathTracking<T>::PathTracking()
: params_{ros::NodeHandle("/modalities/path_tracking")}, dynamic_reconfigure_server_{ros::NodeHandle("/modalities/path_tracking")}
{
    // Initialize params struct and the dynamic_reconfigure server with a node handle.
    // Get all parameter values from the parameter sever.
    params_.fromParamServer();
    
    // Dynamic reconfigure
    dynamic_reconfigure_server_.setCallback(boost::bind(&PathTracking<T>::reconfigureRequest, this, _1, _2));
    
    // Publish messages (debugging purpose)
    pub_position_ = nh_.advertise<geometry_msgs::Vector3>("path_tracking/modalities_debug/position", 1000);
    pub_abs_attraction_force_ = nh_.advertise<std_msgs::Float64>("path_tracking/modalities_debug/abs_attraction_force", 1000);
    pub_velocity_in_ = nh_.advertise<geometry_msgs::Vector3>("path_tracking/modalities_debug/velocity_in", 1000);
    pub_velocity_out_ = nh_.advertise<geometry_msgs::Vector3>("path_tracking/modalities_debug/velocity_out", 1000);
    pub_resulting_velocity_ = nh_.advertise<geometry_msgs::Vector3>("path_tracking/modalities_debug/resulting_velocity", 1000);
    pub_resulting_force_ = nh_.advertise<geometry_msgs::Vector3>("path_tracking/modalities_debug/resulting_force", 1000);
    pub_distance_ = nh_.advertise<std_msgs::Float64>("path_tracking/modalities_debug/distance", 1000);
}

/**
 * \brief Help function to calculate the distance of a point p from a straigth line defined by two points l1 and l2
 * 
 * Reference: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
 * 
 * \param px x coordinate of the point p
 * \param py y coordinate of the point p
 * \param l1x x coordinate of the point l1
 * \param l1y y coordinate of the point l1
 * \param l2x x coordinate of the point l2
 * \param l2y y coordinate of the point l2
 */
template <typename T>
double PathTracking<T>::getDistancePointLine(double px, double py, double l1x, double l1y, double l2x, double l2y)
{
    return std::abs((l2y - l1y) * px - (l2x - l1x) * py + l2x * l1y - l2y * l1x) / sqrt(pow(l2y - l1y, 2) + pow(l2x - l1x, 2));
}


/**
 * \brief Update the effect of the path tracking on the velocity twist at a timestep.
 * 
 * Segment: A (path-) segment is the line connecting two path points. 
 * 
 * \param data_in input velocity as Twist
 * \param data_out output velocity as Twist
 */
template <typename T>
bool PathTracking<T>::update(const T& data_in, T& data_out)
{
    // Initialize the output velocity with the input velocity
    data_out = data_in;
    
    // If there is no path section defined, return.
    if (path_sections_.size() < 1)
    {
        return true;
    }
    
    // Update the current robot location stored in transform_map_robot_
    updateRobotLocation();
    tf2::Vector3 robot_location;
    tf2::fromMsg(transform_map_robot_.transform.translation, robot_location);
    pub_position_.publish(tf2::toMsg(robot_location)); // debug
    
    // If the robot is not already in tracking mode (=on a path section): Is robot near the start point of a section?
    if (current_section_ == -1)
    {
        // Check all sections
        for(int i = 0; i < path_sections_.size(); i++)
        {
            double dist_to_start = tf2::tf2Distance(path_sections_[i][0], robot_location);
            //TODO: Add this as parameter for path tracking. Distance from point for the path tracking.
            if (dist_to_start <= 0.3)
            {
                ROS_WARN_ONCE("PathTracking: Robot near the start point of a pathtracking section. Pathtracking started.");
                current_section_ = i;
                pos_prev_segment_= 0;
                break;
            }
        }
        
        // Still not in tracking mode? return 
        if (current_section_ == -1)
        {
            return true;
        }
    }
    
    // **Robot is in tracking mode on a path section**
    
    // If at the end of the section: end tracking mode and return.
    if (pos_prev_segment_ >= (int)path_sections_[current_section_].size() - 6) // neglect last 5 points
    {
        ROS_WARN("PathTracking: Reached the end point of the pathtracking section. Leave tracking mode.");
        current_section_ = -1;
        pos_prev_segment_ = 0;
        previous_force_.setZero();
        previous_velocity_linar_.setZero();
        return true;
    }
    
    // Calculate distance of the robot to the previous nearest point of the current section
    double dist_to_prev_segment = tf2::tf2Distance(path_sections_[current_section_][pos_prev_segment_], robot_location);
    
    // Direction of the force
    // Calculate the sum of the weighted directions of all consecutive points that are within a distance of max_deviation*1.2 from the robot
    // The weight of a direction of a point drops from 1 linear with its distance from the robot (zero at max_deviation*1.2)
    tf2::Vector3 direction;
    direction.setZero();
    for (int fb=0; fb <= 1; fb++) //forwards 0/backwards 1
    {
        int j = (fb==0) ? pos_prev_segment_ : pos_prev_segment_ - 1; // start search from a segment point of the previous timestep (forwards) or -1  backwards.
        bool consecutive = true;
        while (consecutive == true)
        {
            ROS_WARN_STREAM_THROTTLE(1, "Path_sections " << path_sections_[current_section_].size());
            if (j < 0 || j >= (int)path_sections_[current_section_].size())
            {
                break;
            }
            tf2::Vector3 rob_to_j = path_sections_[current_section_][j] - robot_location;
            double dist_to_j = rob_to_j.length();
            if (dist_to_j < max_deviations_[current_section_] * 1.2)
            {
                double potential_j = (-1 / (max_deviations_[current_section_] * 1.2)) * dist_to_j + 1;
                tf2::Vector3 potential_vector_j = rob_to_j / dist_to_j * potential_j;
                direction += potential_vector_j;
                j = (fb == 0) ? j + 1 : j - 1;
            }
            else
            {
                consecutive = false;
            }
        }
    }
    ROS_WARN_STREAM_THROTTLE(1, "Direction " << direction.length());
    if (direction.length() != 0)
    {
        direction = direction / direction.length(); //normalize distance
    }
    // Find path segment that is cut by the line through the robot in the direction of the force
    // From a start point (a point from the path segment of the previous timestep) start to search forwards
    // Calculate the distance of the point j to the line and of the point k = j+1 to the line.
    // Compare the distances: If the second distance is smaller, set increase j and k by one and contine forwards. If the first distance is smaller, continue backwards.
    // Continue comparing the distances until the first distance is smaller, then the closest point a to the line is found. The second closest point is either a-1 or a+1.
    double pos_a;
    double pos_b;
    bool backwards = false;
    bool change_search_direction = true;
    tf2::Vector3 point_on_line = robot_location + direction;
    int j = pos_prev_segment_;   // Start-point of the search
    double dist_j_line = getDistancePointLine(path_sections_[current_section_][j].getX(), path_sections_[current_section_][j].getY(), robot_location.getX(), robot_location.getY(), point_on_line.getX(), point_on_line.getY());
    bool found = false;
    while (!found)
    {
        int k = backwards ? j - 1 : j + 1;
        if (k >= (int)path_sections_[current_section_].size())
        {
            backwards = true;
            change_search_direction = false;
            continue;
        }
        if (k < 0)
        {
            pos_a = 0;
            pos_b = 1;
            break;
        }
        double dist_k_line = getDistancePointLine(path_sections_[current_section_][k].getX(), path_sections_[current_section_][k].getY(), robot_location.getX(), robot_location.getY(), point_on_line.getX(), point_on_line.getY());
        if (dist_k_line <= dist_j_line)
        {
            j = k; 
            dist_j_line = dist_k_line;
            change_search_direction = false; // the distance got smaller so we are searching in the right direction
        }
        else if (!change_search_direction)
        {
            found = true;
            pos_a = j;
            int i = backwards ? j + 1 : j - 1;
            double dist_i_line;
            if ( i < 0 || i >= (int)path_sections_[current_section_].size())
            {
                dist_i_line = 1000000; // definitly bigger than dist_k_line
            }
            else
            {
                dist_i_line = getDistancePointLine(path_sections_[current_section_][i].getX(), path_sections_[current_section_][i].getY(), robot_location.getX(), robot_location.getY(), point_on_line.getX(), point_on_line.getY());
            }
            pos_b = (dist_k_line <= dist_i_line) ? k : i;
        }
        else
        {
            backwards = true;
            change_search_direction = false; // already changed search direction
        }
    }
    
    // Intersection of path segment and line
    // Reference: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    double x1 = path_sections_[current_section_][pos_a].getX();
    double y1 = path_sections_[current_section_][pos_a].getY();
    double x2 = path_sections_[current_section_][pos_b].getX();
    double y2 = path_sections_[current_section_][pos_b].getY();
    double x3 = robot_location.getX();
    double y3 = robot_location.getY();
    double x4 = point_on_line.getX();
    double y4 = point_on_line.getY();
    tf2::Vector3 intersection;
    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    intersection.setX(((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator);
    intersection.setY(((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator);
    intersection.setZ(0);
    
    // Strenght of the force
    double distance = tf2::tf2Distance(robot_location, intersection);
    
    double px = x3;
    double py = y3;
    double l1x = x1;
    double l1y = y1;
    double l2x = x2;
    double l2y = y2;
    
    distance = std::fabs((l2y - l1y) * px - (l2x - l1x) * py + l2x * l1y - l2y * l1x) / sqrt(pow(l2y - l1y, 2) + pow(l2x - l1x, 2));
    
    //Debug: workaround if distance gets NaN, skip this iteration
    if (std::isnan(distance))
    {
        ROS_WARN_STREAM_THROTTLE(1, "(Debug) NaN value skipped. distance:" << distance << " denominator:" << denominator << " pos_a:" << pos_a << " pos_b:" << pos_b << " pos_prev_segment_:" << pos_prev_segment_ << " direction_x:" << direction.getX() << " direction_y:" << direction.getY());
        data_out.linear = tf2::toMsg(previous_velocity_linar_);
        return true;
    }
    
    // Only save new start point if it is surely within the potenial radius.
    // Why? When the curve radius is smaller that the potential radius it can happen, that the the direction vector points to a path segment
    // that is not within the potential radius. In the next iteration the direction would result in zero if the algorithm would start searching
    // from a point on that path segment.
    if (distance < max_deviations_[current_section_])
    {
        pos_prev_segment_ = pos_a;
    }
    else
    {
        ROS_WARN_ONCE("PathTracking: Curve radius of the path is smaller that max_deviation! Please reduce max_deviation parameter to prevent possible unwanted behaviour.");
    }
    
    // Generate attraction_force based on a distance function
    double abs_attraction_force = 0;
    if (force_distance_functions_[current_section_] == "linear" or force_distance_functions_[current_section_] == "trapezoidal") {
        double d0; // distance to path at which is tolerated without any attraction_force.
        d0 = 0.2;
        abs_attraction_force = (params_.max_force / (max_deviations_[current_section_] - d0)) * (distance - d0);
        if (abs_attraction_force < 0.0) {
            abs_attraction_force = 0.0;
        }
    }
    else if (force_distance_functions_[current_section_] == "quadratic" or force_distance_functions_[current_section_] == "trigonometrical") {
        abs_attraction_force = params_.max_force / pow(max_deviations_[current_section_], 2) * pow(distance, 2);
    }
    else {
        ROS_WARN_STREAM("PathTracking: Force distance function of type '" << force_distance_functions_[current_section_] << " not implemented! PathTracking is deactivated!");
    }
    
    //TODO: remove this when works
//     double abs_attraction_force;
//     switch (force_distance_functions_[current_section_]){  
//         case 0: // linear
//             double d0; // distance to path at which is tolerated without any attraction_force.
//             d0 = 0.2;
//             abs_attraction_force = (params_.max_force / (max_deviations_[current_section_] - d0)) * (distance - d0);
//             if (abs_attraction_force < 0.0) {
//                 abs_attraction_force = 0.0;
//             }
//         break;  
//         case 1: // quadratic
//             abs_attraction_force = params_.max_force / pow(max_deviations_[current_section_], 2) * pow(distance, 2);
//         break;  
//     }
    
    tf2::Vector3 attraction_force = abs_attraction_force * direction;
    ROS_INFO_STREAM_THROTTLE(1, "PathTracking: nearest segment: " << pos_a << "-" << pos_b << " | dist_to_seg: " << distance << "m | abs_attraction_force: " << abs_attraction_force);
    
    // Transform the force vector from map frame to base_link frame
    tf2::Quaternion rotation_q;
    tf2::fromMsg(transform_map_robot_.transform.rotation, rotation_q);
    rotation_q = rotation_q.inverse();
    ROS_INFO_STREAM_THROTTLE(1, "PathTracking: quaternion x: " << rotation_q.x() << " y:" << rotation_q.y() << " z:" << rotation_q.z() << " w: " << rotation_q.w());
    tf2::Vector3 resulting_attraction_force = tf2::quatRotate(rotation_q, attraction_force);
    
    // Translate the force to a velocity by adopting a mass damping system.    
    tf2::Vector3 resulting_velocity_linear = previous_velocity_linar_ * mds_v_ + previous_force_ * mds_f_;
    
    // Save the values for the next timestep
    previous_force_ = resulting_attraction_force;
    previous_velocity_linar_ = resulting_velocity_linear;
    
    // Add the virtual velocity to the output velocity
    data_out.linear.x = data_out.linear.x + resulting_velocity_linear.getX();
    data_out.linear.y = data_out.linear.y + resulting_velocity_linear.getY();
    data_out.linear.z = data_out.linear.z + resulting_velocity_linear.getZ();
    
    // debug
    std_msgs::Float64 abs_attraction_force_msg;
    abs_attraction_force_msg.data = abs_attraction_force / 100;
    pub_abs_attraction_force_.publish(abs_attraction_force_msg);
    std_msgs::Float64 distance_msg;
    distance_msg.data = distance;
    pub_distance_.publish(distance_msg);
    pub_resulting_force_.publish(tf2::toMsg(resulting_attraction_force / 100));
    pub_resulting_velocity_.publish(tf2::toMsg(resulting_velocity_linear));
    pub_velocity_in_.publish(data_in.linear);   
    pub_velocity_out_.publish(data_out.linear);
    
    return true;
}

/**
 * \brief Callback function for dynamic reconfigure.
 */
template <typename T>
void PathTracking<T>::reconfigureRequest(PathTrackingConfig& config, uint32_t level)
{    
    // update params hold by the params_-struct
    params_.fromConfig(config);
    
    // recalculate the params of the mass damping system
    double K = params_.max_velocity / params_.max_force;
    double factor = 1 - exp( -1 / (params_.time_const_T * params_.controller_update_rate));
    mds_f_ = K * factor;
    mds_v_ = 1 - factor;
    
    // Logging
    ROS_DEBUG_STREAM("PathTracking: Dynamic reconfigure."
        << "\n    max_force:            " << params_.max_force
        << "\n    max_velocity          " << params_.max_velocity
        << "\n    controller_update_rate: " << params_.controller_update_rate
    );
};

/**
 * \brief Retrieve the path configuration from the parameter server.
 * 
 * A path consists of path points in a specific order, meeting the following requirements:
 * * The distance of two pathpoints needs to be consistent and much smaller than max_deviation!
 * * (The radius of a curve of the path should be a little bigger than max_deviation)
 * Otherwise the behaviour of the path tracking might not be as expected
 * 
 * A path section is a subset of the path. It is defined by a start point and a end point. It is possible to define multiple sections on one path, but they shouldn't overlap. Pathtracking will be enabled on all sections (not on the entire path).
 */
template <typename T>
bool PathTracking<T>::configure() {
        
    // Delete all existing parameters to receive the new ones
    path_sections_.clear();
    force_distance_functions_.clear();
    max_deviations_.clear();
  
    // Namespaces
    std::string path_path = "/" + ns_params_ptr_->project_ns + "/" + ns_params_ptr_->scenario_ns + "/" + ns_params_ptr_->path_ns;
    std::string path_section = "/" + ns_params_ptr_->project_ns + "/" + ns_params_ptr_->scenario_ns + "/" + ns_params_ptr_->section_ns;
    
    // Retrieve pathtracking section names
    std::vector<std::string> section_names;
    ros::param::get(path_section + "/" + ns_params_ptr_->config_ns + "/" + ns_params_ptr_->section_ns + "_names", section_names);
    if (section_names.size() < 1)
    {
        ROS_WARN_STREAM("PathTracking: No pathtracking sections found in namespace " << path_section << "\n PathTracking-Modality will not be used!");
        return true;
    }
    ROS_INFO_STREAM("PathTracking: Found " << section_names.size() << " pathtracking section(s) in namespace "<< path_section << ".");
    
    // Retrieve path point names
    std::vector<std::string> point_names;
    ros::param::get(path_path + "/points", point_names);
    if (point_names.size() < 2)
    {
        ROS_WARN_STREAM("PathTracking: No valid path configuration found in namespace " << path_path << ".\n PathTracking-Modality will not be used!");
        return true;
    }
    ROS_INFO_STREAM("PathTracking: Found a path configuration in namespace " << path_path << " with " << point_names.size() << " points.");
    
    // Retrieve and store all pathtracking section related data based on the section names
    for(int i = 0; i < section_names.size(); i++)
    {
        std::string path_data_section_i = path_section + "/" + ns_params_ptr_->data_ns + "/" + section_names[i];
        
        // name of start and end point of section
        std::string start, end;
        ros::param::get(path_data_section_i + "/start", start);
        ros::param::get(path_data_section_i + "/end", end);
        
        // section points
        std::vector<tf2::Vector3> section;
        bool in_section = false;
        for(int i = 0; i < point_names.size(); i++)
        {
            if(in_section || point_names[i] == start || point_names[i] == end)
            {
                in_section = true;
                double x, y, z; 
                ros::param::get(path_path + "/" + point_names[i] + "/x", x);
                ros::param::get(path_path + "/" + point_names[i] + "/y", y);
                ros::param::get(path_path + "/" + point_names[i] + "/z", z);
                tf2::Vector3 point(x,y,z);
                section.push_back(point);
            }
            if(point_names[i] == end){
                break;
            }
        }
        if (section.size() < 2)
        {
            ROS_WARN("PathTracking: Found a pathtracking section which consists of less than 2 points. This section will be skipped.");
        }
        else
        {
            path_sections_.push_back(section);
        }
        
        // force distance function
        std::string force_distance_function;
        ros::param::get(path_data_section_i + "/" + ns_params_ptr_->force_distance_function_ns, force_distance_function);
        force_distance_functions_.push_back(force_distance_function);
        
        // max deviation
        int max_deviation;
        ros::param::get(path_data_section_i + "/max_deviation", max_deviation);
        max_deviations_.push_back(max_deviation);
    }
    
    // Initialize previous segment point and current section
    current_section_ = -1;
    pos_prev_segment_ = 0;
    
    // Initialize mass-damping-system
    previous_force_.setZero();
    previous_velocity_linar_.setZero();
    
    return true;
}

};

#endif
