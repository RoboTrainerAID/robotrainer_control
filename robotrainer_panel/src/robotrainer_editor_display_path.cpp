/**
 * This file is meant to hold the necessary functions to hold an organize the path of a RobotrainerEditor
 * experiment.
 * It is meant to be include by the RobotrainerEditor Panel and used through its UI.
 * The functions concerning any objects except the pure path shall be implemented in their own files.
 *
 * This file is a manipulated version of a legacy "robotrainer_editor_display_path"
 */

#include <cstdio>
#include <ctime>
#include <locale>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/param.h>

#include <fstream>
#include <cmath>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>


using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std;
using namespace boost;

#include <robotrainer_panel/robotrainer_editor_display_path.h>
#include <cstdlib>

RobotrainerEditorDisplayPath::RobotrainerEditorDisplayPath() {}

RobotrainerEditorDisplayPath::RobotrainerEditorDisplayPath(robotrainer_panel::robotrainer_editorParameters* params_,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* path_update_publisher_) {

    params = params_;

    //the lambda function do_this is required even for functions within this class because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    do_this = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->doNothing(feedback);
    };

    path_menu_handler = new interactive_markers::MenuHandler;

    setMetaParams(server_, path_update_publisher_);
}

void RobotrainerEditorDisplayPath::setMetaParams(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* path_update_publisher_) {
  
    point_menu_handler = new interactive_markers::MenuHandler;
    std::string add = "add";
    //if (isMarkerInPivotPoints(feedback)) {
    //    add = "remove";
    //}
    point_menu_handler->insert ( add + "point as pivot", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->addSelectedPointAsPivot ( feedback );
    } );
    point_menu_handler->insert ( "delete path from this point on", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->deletePathFromSelectedPoint ( feedback );
    } );
    server = server_;
    path_update_publisher = path_update_publisher_;
}

std::string RobotrainerEditorDisplayPath::getPathName() {
    return path_name;
}

int RobotrainerEditorDisplayPath::getPathLength() {
    return path_points.size();
}

int RobotrainerEditorDisplayPath::getCount() {
    return getPathLength();
}

std::string RobotrainerEditorDisplayPath::getNs() {
    return params->path_ns;
}

std::vector<std::string> RobotrainerEditorDisplayPath::getCubePointNames() {
    return cube_point_names;
}

std::map<std::string, geometry_msgs::Point> RobotrainerEditorDisplayPath::getCubePointMap() {
    return cube_point_map;
}

void RobotrainerEditorDisplayPath::doNothing(const InteractiveMarkerFeedbackConstPtr &feedback) {
  
}

/**
 * delete the whole path
 */
void RobotrainerEditorDisplayPath::deletePath() {
    ostringstream oss;

    // delete segments
    for (int i = 0; i < (path_points.size()-1); i++) {
        oss.str("");
        oss << params->segment_ns << "_" << i;
        server->erase(oss.str());
    }
    ros::param::del( params->path_ns);

    //update changes in the interactive_marker_server
    server->applyChanges();

    //clear the list of points
    path_points.clear();

    //clear the parameter server
    ros::param::del( params->path_ns);
}

void RobotrainerEditorDisplayPath::createPath() {
  
    //there is no path to be created between no path_points
    if (path_points.size() == 0) {
        ROS_ERROR("points list is empty (robotrainer_editor_display_path::createPath). is there a syntax error in rosparam calls?");
        return;
    }

    std::ostringstream oss;
    Point prev; //is empty for first run (according to robotrainer_editor_development doesnt cause a problem
    int cube_id = 0; //id for interactive path_points, gets incremented only for interactive path_points
    double margin = 0.01;
    
    //used to check if next point should be interactive or not, taking curves into account
    double distance_to_last_interactive = params->path_step_rate; //ensure first point is interactive , but do not risk overflow

    for (unsigned int i = 0; i < (path_points.size() - 1); i++) {

        //more convenient than calling the element of the vector each time
        Point curr = path_points[i];

        //there is no edge towards the first point, so it can be skipped
        if (i > 0) {
            //create a marker of type "arrow", beeing an "edge" of the path "graph"
            Marker path_marker;
            path_marker.type = Marker::ARROW;
            path_marker.color.r = 1.0;
            path_marker.color.g = 0.0;
            path_marker.color.b = 0.0;
            path_marker.color.a = 1.0;
            path_marker.scale.x = 0.1 * params->base_scale;
            path_marker.scale.y = 0.0001 * params->base_scale;
            path_marker.scale.z = 0.0001 * params->base_scale;

            //special behaviour for first point after an interactive one
            if ((distance_to_last_interactive) == 0) {
                double x_diff = curr.x - prev.x;
                double y_diff = curr.y - prev.y;
                double length = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
                Point newP;
                newP.x = prev.x + margin/length * x_diff;
                newP.y = prev.y + margin/length * y_diff;
                path_marker.points.push_back(newP);
            } else {
                path_marker.points.push_back(prev); //? see below
            }

            distance_to_last_interactive += vectorLength(pointDiff(prev, curr));  //distance is sum of disjunct distance of points since last interactive point (to take curves into account)
            
            //special behaviour for interactive points
            if (distance_to_last_interactive < params->path_step_rate) {
                double x_diff = curr.x - prev.x;
                double y_diff = curr.y - prev.y;
                double length = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
                Point newP;
                newP.x = prev.x + (length-margin)/length * x_diff;
                newP.y = prev.y + (length-margin)/length * y_diff;
                path_marker.points.push_back(newP);
            } else {
                path_marker.points.push_back(curr); //? see below
            }

            InteractiveMarkerControl path_control;
            path_control.interaction_mode = InteractiveMarkerControl::NONE;
            path_control.always_visible = true;
            path_control.markers.push_back(path_marker);

            InteractiveMarker path_int_marker;
            path_int_marker.header.frame_id = params->frame_id;
            oss.str("");
            oss << "path_" << i;
            path_int_marker.name = oss.str();
            path_int_marker.controls.push_back(path_control);

            server->insert(path_int_marker);
            path_menu_handler->apply(*server, oss.str());
            server->applyChanges();

        }
        prev = curr; //needs to happen after distance_to_last_interactive comaprison, but before "path step rate" check, to make shure it happens at all!

        //actual creation of the interactive marker @ path step rate
        //if ((i %  params->path_step_rate) != 0) continue; //used when path_step rate was still measurde in "amount of points" instead of "centimeters"
        
        //create an interactive point at approximately equal mdistances
        if (distance_to_last_interactive < params->path_step_rate) continue;
        
        distance_to_last_interactive = 0.0;

        oss.str("");
        oss << params->cube_ns << "_" << cube_id;
        cube_point_map[oss.str()] = curr;

        // create dragable box
        InteractiveMarker inter_marker;
        inter_marker.header.frame_id = params->frame_id;
        inter_marker.pose.position = curr;
        inter_marker.scale = params->base_scale;
        inter_marker.name = oss.str();
        cube_point_names.push_back(oss.str());

        InteractiveMarkerControl control;
        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 1.0;
        control.orientation.z = 0.0;
        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        control.always_visible = true;
        control.orientation = normalizeQuaternion(control.orientation); 

        Marker cube;
        cube.type = Marker::CUBE;
        cube.scale.x = cube.scale.y = cube.scale.z = inter_marker.scale * 0.1;
        cube.color.r = cube.color.g = cube.color.b = 1.0;
        cube.color.a = 1.0;
        cube.pose.orientation = normalizeQuaternion(cube.pose.orientation); 
        control.markers.push_back(cube);
        inter_marker.controls.push_back(control);

        //insert the cube-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
        //the lambda function do_this is required even for functions within this class because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
        //for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
        std::function<void(const InteractiveMarkerFeedbackConstPtr &)> server_function = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
            this->interactionFunction(feedback);
        };
        server->insert(inter_marker, server_function);
        point_menu_handler->apply(*server, oss.str());
        server->applyChanges();

        cube_id++;
    }
    setDoNothing();
}


void RobotrainerEditorDisplayPath::setInteractionFunction(std::function<void(const InteractiveMarkerFeedbackConstPtr &feedback)> f) {
    do_this = f;
}

//allow all interactive markers to move plane for this function to work properly
//this happens specificly in this functon and not in'RobotrainerEditorDisplayPath::setInteractionFunction' because it might not be necessary for all actions in the future.
void RobotrainerEditorDisplayPath::setDoSomething() {setDoSomething(do_this);}
void RobotrainerEditorDisplayPath::setDoSomething(std::function<void(const InteractiveMarkerFeedbackConstPtr &feedback)> f) {
    for(int i = 0; i < cube_point_names.size(); i++) {
        InteractiveMarker inter_marker;
        if (!(server->get(cube_point_names[i], inter_marker))) {
            ROS_ERROR("A cube_point_names element was not found in the interactive marker server! (enableArrowEditing(), robotrainer_editor_panel.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

        server->insert(inter_marker);
    }
    server->applyChanges();
    setInteractionFunction(f);
}

void RobotrainerEditorDisplayPath::setDoNothing() {

    //this loop makes the points unmovable as long as they have no interaction function set
    for(int i = 0; i < cube_point_names.size(); i++) {
        InteractiveMarker inter_marker;
        if (!(server->get(cube_point_names[i], inter_marker))) {
            continue;
        }

        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        server->insert(inter_marker);
    }
    server->applyChanges();

    //the lambda function "nothing" is required because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    do_this = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->doNothing(feedback);
    };

}

void RobotrainerEditorDisplayPath::interactionFunction(const InteractiveMarkerFeedbackConstPtr &feedback) {
    (do_this)(feedback);
}

// loads path from yaml to parameter server, shall be called by loadFile, shall call deletePath
void RobotrainerEditorDisplayPath::loadPath(std::string filename) {

    //call to bash to load file
    std::ostringstream sstr;
    //sstr << "bash -i -c 'rosparam load " << filename << " /" << params->path_ns << "'"; //crashed when using "bash -i -c" for second time
    sstr << "rosparam load " << filename << " /" << params->editor_ns << "/" << params->path_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);

    file_name = filename;
}




// read path positions and create resulting path from yaml-file
void RobotrainerEditorDisplayPath::loadFile(std::string filename) {
    loadPath(filename);
    //create topic to publish information about path points
    vector<string> topics;
    string path_point_topic = string("/") +  filename;
    topics.push_back(path_point_topic);
    
    loadFile();
    
    //publish loaded filename
    std_msgs::String msg;
    msg.data = filename;
    path_update_publisher->publish(msg);
}
    
void RobotrainerEditorDisplayPath::loadFile() {
    
    // Get path name if is is not existing then create new one
    if (!ros::param::get(params->editor_ns + "/" + params->path_ns + "/path_name", path_name)) {
      std::time_t t = std::time(nullptr);
      char mbstr[100];
      std::strftime(mbstr, sizeof(mbstr), "%Y%m%d%H%M", std::localtime(&t));
      path_name = "path_";
      path_name.append(mbstr);
      ros::param::set(params->editor_ns + "/" + params->path_ns + "/path_name", path_name);
    }
    
    if (!ros::param::get(params->editor_ns + "/" + params->path_ns + "/pivot_points", pivot_points)) {

      ros::param::set(params->editor_ns + "/" + params->path_ns + "/pivot_points", pivot_points);
    }
    

    //clear the points lists, they shall be regenerated
    if (!path_points.empty()) path_points.clear();
    if (!path_point_names.empty()) path_point_names.clear();

    //load point names from parameterServer
    ros::param::get(params->editor_ns + "/" + params->path_ns + "/points", path_point_names);

    //load all points from parameterServer
    for(int i = 0; i < path_point_names.size(); i++) {
        Point point;
        ros::param::get(params->editor_ns + "/" + params->path_ns + "/" + path_point_names[i] + "/x", point.x);
        ros::param::get(params->editor_ns + "/" + params->path_ns + "/" + path_point_names[i] + "/y", point.y);
        ros::param::get(params->editor_ns + "/" + params->path_ns + "/" + path_point_names[i] + "/z", point.z);
        path_points.push_back(point);
    }

    //create path according to loaded points
    createPath();
}

void RobotrainerEditorDisplayPath::addPoint(Point point) {
  
    int index = path_point_names.size();
    
    Point point_diff;
    if (index){ //if index > 0, might be 0 with the point we intend to record is the first one!
        Point point_diff = pointDiff(path_points[index - 1], point); //order is important! the other way arround, path would be built in oposite direction!

        double point_dist = vectorLength(point_diff);
        
        if (point_dist > params->record_point_rate) {
            int splits = (int) (point_dist/(params->record_point_rate));
            if (splits != 1){ //should allways be true, but if it would and i wouldn't catch it, might end in endless recursion
                for (int i = 1; i <= splits; i++){
                  double scale = (double) i / splits;
                  //addPoint(pointAdd(point, pointScale(point_diff, scale)));
                  addPoint(pointAdd(path_points[index - 1], pointScale(point_diff, scale)));
                }
                return;
            }
        }
    }
  
    std::ostringstream point_name_stream;
    point_name_stream << "point" << index;
    std::string point_name = point_name_stream.str();
    
    while(std::find(path_point_names.begin(), path_point_names.end(), point_name) != path_point_names.end()) {
        index++;
        point_name_stream.clear();
        point_name_stream << "point" << index;
        point_name = point_name_stream.str();
    }
    
    ROS_INFO_STREAM("point '" << point_name << "' added: " << point.x << " | " << point.y);
    
    path_point_names.push_back(point_name);
    path_points.push_back(point);
}

std::map<std::string, geometry_msgs::Point> RobotrainerEditorDisplayPath::deletePathFromPoint(std::string name){
  
    int index = std::find(cube_point_names.begin(), cube_point_names.end(), name) - cube_point_names.begin();
    
    if (index == (cube_point_names.end() - cube_point_names.begin())) {
        ROS_ERROR("could not delete tail of path (RobotrainerEditorDisplayPath::deletePathFromPoint), for the (cube-)point-name read from the interactive marker has not been found in the cube_point_names list!");
        return cube_point_map;
    }
    
    Point point = cube_point_map[name];
    
    for (int i = index; i < cube_point_names.size(); i++){
        cube_point_map.erase(cube_point_names[i]);
    }
    
    cube_point_names.erase (cube_point_names.begin() + index, cube_point_names.end());
    
    for (index = 0; index < (path_points.size() - 1); index++){ //adapt index to all points, not only interactive cubes
        if (pointEqual(path_points[index], point)) break;
    }
    
    path_point_names.erase (path_point_names.begin() + index, path_point_names.end());
    path_points.erase (path_points.begin() + index, path_points.end());
    
    server->clear();
    reloadSession(); //reload interactive_marker- and param-server state
    return cube_point_map;
}

std::map<std::string, geometry_msgs::Point> RobotrainerEditorDisplayPath::deletePathFromSelectedPoint(const InteractiveMarkerFeedbackConstPtr &feedback){
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        return deletePathFromPoint ( feedback->marker_name );
    } else {
        ROS_ERROR("tried to crop a path from a non 'InteractiveMarkerFeedback::MENU_SELECT' feedback event!");
        return cube_point_map;
    }
    
}

std::map<std::string, geometry_msgs::Point> RobotrainerEditorDisplayPath::addPointAsPivot(std::string name){
    int index = std::find(cube_point_names.begin(), cube_point_names.end(), name) - cube_point_names.begin();
    if (index == (cube_point_names.end() - cube_point_names.begin())) {
        ROS_ERROR("could not add point as pivot (RobotrainerEditorDisplayPath::addPointAsPivot), for the (cube-)point-name read from the interactive marker has not been found in the cube_point_names list!");
        return cube_point_map;
    }
    
    Point point = cube_point_map[name];
    for (index = 0; index < (path_points.size() - 1); index++){ //adapt index to all points, not only interactive cubes
        if (pointEqual(path_points[index], point)) break;
    }
    for(int i = 0; i < pivot_points.size(); i++) {
        if (pivot_points[i].compare(path_point_names[index]) == 0) {
           pivot_points.erase(pivot_points.begin() + i);
           ros::param::set(params->editor_ns + "/" + params->path_ns + "/pivot_points", pivot_points);
           return cube_point_map;
        }
    } 
    pivot_points.push_back(path_point_names[index]);
    ros::param::set(params->editor_ns + "/" + params->path_ns + "/pivot_points", pivot_points);
    return cube_point_map;
}

std::map<std::string, geometry_msgs::Point> RobotrainerEditorDisplayPath::addSelectedPointAsPivot(const InteractiveMarkerFeedbackConstPtr &feedback){
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        return addPointAsPivot ( feedback->marker_name );
    } else {
        ROS_ERROR("tried to add point as pivot from a non 'InteractiveMarkerFeedback::MENU_SELECT' feedback event!");
        return cube_point_map;
    }
}

void RobotrainerEditorDisplayPath::reloadSession() {

    //upload all points to parameterServer
    for(int i = 0; i < path_points.size(); i++) {
        Point point = path_points[i];
        std::ostringstream oss;
        oss << "point" << i;
        ros::param::set(params->editor_ns + "/" + params->path_ns + "/" + path_point_names[i] + "/x", point.x);
        ros::param::set(params->editor_ns + "/" + params->path_ns + "/" + path_point_names[i] + "/y", point.y);
        ros::param::set(params->editor_ns + "/" + params->path_ns + "/" + path_point_names[i] + "/z", point.z);
    }

    //upload point names to parameterServer
    ros::param::set(params->editor_ns + "/" + params->path_ns + "/points", path_point_names);
    
    //upload path name to parameterServer
    ros::param::set(params->editor_ns + "/" + params->path_ns + "/path_name", path_name);
    
     ros::param::set(params->editor_ns + "/" + params->path_ns + "/pivot_points", pivot_points);
    //create path on interactive marekr server
    createPath();
}

void RobotrainerEditorDisplayPath::resetServer() {
    server->clear();
    server->applyChanges();
}

// save current path config to file and update menu entries
void RobotrainerEditorDisplayPath::saveFile(std::string filename) {
    saveFileTool(filename, params->editor_ns + "/" + params->path_ns);
}

// bool RobotrainerEditorDisplayPath::isMarkerInPivotPoints(const InteractiveMarkerFeedbackConstPtr &feedback) {
//     if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
//     
//         int index = std::find(cube_point_names.begin(), cube_point_names.end(), feedback->marker_name) - cube_point_names.begin();
//     if (index == (cube_point_names.end() - cube_point_names.begin())) {
//         ROS_ERROR("could not add point as pivot (RobotrainerEditorDisplayPath::addPointAsPivot), for the (cube-)point-name read from the interactive marker has not been found in the cube_point_names list!");
//         return false;
//     }
//     
//     Point point = cube_point_map[feedback->marker_name];
//     for (index = 0; index < (path_points.size() - 1); index++){ //adapt index to all points, not only interactive cubes
//         if (pointEqual(path_points[index], point)) break;
//     }
//     for(int i = 0; i < pivot_points.size(); i++) {
//         if (pivot_points[i].compare(path_point_names[index]) == 0) {
//           return true;
//         }
//     }
//     return false;
//     } else {
//         return false;
//     }
// }
