/**
 * This file is meant to hold the necessary functions to hold an organize the path of a RobotrainerEditor
 * experiment.
 * It is meant to be include by the RobotrainerEditor Panel and used through its UI.
 * The functions concerning any objects except the pure path shall be implemented in their own files.
 * 
 * This file is a manipulated version of a legacy "robotrainer_editor_display_path"
 */

#ifndef ROBOTRAINER_EDITOR_DISPLAY_PATH_H
#define ROBOTRAINER_EDITOR_DISPLAY_PATH_H

#include <robotrainer_panel/robotrainer_editorParameters.h>
#include <robotrainer_panel/robotrainer_editor_tool.h>

class RobotrainerEditorDisplayPath : public RobotrainerEditorTool {
  
robotrainer_panel::robotrainer_editorParameters* params;

//maps and vectors to hold enough data to define the current configuration
std::vector<Point> path_points;
std::vector<std::string> path_point_names;
std::map<int, std::string> menu_id_map;
std::vector<std::string> cube_point_names;
std::map<std::string, geometry_msgs::Point> cube_point_map;

// this holds the function that shall be called when an interactive Marker is clicked
std::function<void(const InteractiveMarkerFeedbackConstPtr &)> do_this;

//adresses to important shared objects
//a server that handles the display of the interactive Markers in rviz
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler* path_menu_handler;
interactive_markers::MenuHandler* point_menu_handler;
ros::Publisher* path_update_publisher;
//ros::Subscriber checkpoint_subscriber;
ros::NodeHandle nh;

public:
  
RobotrainerEditorDisplayPath();
  
RobotrainerEditorDisplayPath(robotrainer_panel::robotrainer_editorParameters* params_,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* path_update_publisher_);

/**
 * Set the attributes that handle data and communication. These are pointers, the same handler shall be used in multiple contextes throughout the project.
 */
void setMetaParams(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* path_update_publisher_) ;

int getPathLength();

int getCount();

std::string getNs();
        
std::vector<std::string> getCubePointNames();

std::map<std::string, geometry_msgs::Point> getCubePointMap();

//void addCheckpoint(std::string name, Point point);

//void receiveCheckpoint(const PointConstPtr& point_msg);

void doNothing(const InteractiveMarkerFeedbackConstPtr &feedback);

void deletePath();

//create path segments connecting path points -- So "forms" the path form allready loaded/created points
void createPath();

//set the function to be called when interacting with an interactive point
void setInteractionFunction(std::function<void(const InteractiveMarkerFeedbackConstPtr &feedback)>);

//enable editing of interactive path points
void setDoSomething();
void setDoSomething(std::function<void(const InteractiveMarkerFeedbackConstPtr &feedback)> f);

//set all interactive markers to "doNothing"
void setDoNothing();

//the function the interactive_marker_server shall be linked to
void interactionFunction(const InteractiveMarkerFeedbackConstPtr &feedback);

//loads path from yaml to parameter server, shall be called by loadPathPoints, shall call deletePath
void loadPath(std::string filename);

//read path positions and create resulting path yaml-file loaded to parameter server
void loadFile(std::string filename);

void loadFile();

//add a single point to the path, meant for use in path recording
void addPoint(Point point);

std::map<std::string, geometry_msgs::Point> deletePathFromPoint(std::string name);

std::map<std::string, geometry_msgs::Point> deletePathFromSelectedPoint(const InteractiveMarkerFeedbackConstPtr &feedback);

//save all data in parameter server to persistent yaml file
void saveFile(std::string filename);

/**
* In case the Parameter-Server has been overloaded with different data (or in case there is doubt that the Parameter-Server and object data are in sync).
*/
void reloadSession();

void resetServer();
};

#endif
