/**
 * This file is meant to hold the necessary functions to hold an organize the force-fields of a RobotrainerEditor
 * experiment.
 * It is meant to be include by the RobotrainerEditor Panel and used through its UI.
 * The functions concerning any other objects, including the pure path shall be implemented in their own
 * files.
 * 
 * This file is a manipulated version of a legacy "robotrainer_editor_display_path"
 */

#ifndef ROBOTRAINER_EDITOR_FORCE_H
#define ROBOTRAINER_EDITOR_FORCE_H

#include <robotrainer_panel/robotrainer_editorParameters.h>
#include <robotrainer_panel/robotrainer_editor_tool.h>
#include <QInputDialog>
#include <QWidget>

class RobotrainerEditorForce : public RobotrainerEditorTool {
  
//rosparam handler
robotrainer_panel::robotrainer_editorParameters* params;

//if true, arrows can be created by moving path interactive markers
bool create_when_move;

//used to be recognized in the loaded sessions
std::string file_name;
//used to be mapped to the propper display path this belongs to
std::string display_path_file_name;

//maps and vectors to hold enough data to define the current configuration
std::vector<std::string> force_names;
std::map<std::string, geometry_msgs::Point> arrow_map;
std::map<std::string, geometry_msgs::Point> cube_point_map;
std::map<std::string, geometry_msgs::Point> force_point_map;
std::map<std::string, geometry_msgs::Point> margin_map;


//adresses to important shared objects
//a server that handles the display of the interactive Markers in rviz
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::map<std::string, interactive_markers::MenuHandler*> arrow_menu_handler;
std::map<std::string, interactive_markers::MenuHandler*> force_menu_handler;
ros::Publisher* arrow_publisher;

std::map<std::string, std::string> force_distance_function;

QWidget* parent;

public:
  
RobotrainerEditorForce();
  
RobotrainerEditorForce(robotrainer_panel::robotrainer_editorParameters* params_,
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
  ros::Publisher* arrow_publisher_,
  std::string display_path_file_name_);

/**
 * Set the attributes that handle data and communication. These are pointers, the same handler shall be used in multiple contextes throughout the project.
 */
void setMetaParams(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
    ros::Publisher* arrow_publisher_);

void setCreateWhenMove(bool create_when_move_);

std::string getDisplayPathFileName();

int getArrowCount();

int getCount();

std::string getNs();

void setCubePointMap(std::map<std::string, geometry_msgs::Point> cube_point_map_);

// adjust the force circle and send appropriate message
void moveBoxRadius(const InteractiveMarkerFeedbackConstPtr &feedback);

// used to pass an adress of the function
std::function<void(const InteractiveMarkerFeedbackConstPtr &)> returnMoveBoxArrow();

//check if Display File Name from loaded File matches the one in the attribute
bool checkDisplayFileName(std::string filename);

void loadFile(std::string filename);
void loadFile();

/**
 * Persistantly write the configuration of this object to a yaml file.
 */
void saveFile(std::string filename);

/**
 * Delete the Arrow on the server and param server, but not in the object (for session switch)
 */
void deleteArrowFromServer(std::string arrow_name);

/**
 * Delete an Arrow and all related objects that become redundant with its exctinction.
 */
void deleteArrow(std::string arrow_name);

// load arrows of selected bag-file
void deleteSelectedArrow(const InteractiveMarkerFeedbackConstPtr &feedback);
void deleteSelectedForce(const InteractiveMarkerFeedbackConstPtr &feedback);

//used when the path is croped and the forces residing on the removed part of the path also need to be removed
void clearArrowsFromCropedPath(std::map<std::string, geometry_msgs::Point> cube_point_map_);
void clearArrowsFromCropedPath(std::vector<std::string> cube_point_names);

// resets all related data on the server (for session switching)
void resetServer();

// resets ALL related data on server and object!
void reset();

// creates adjustable force around arrow
void createForce(std::string arrow_name, geometry_msgs::Point pos, geometry_msgs::Point cube_pos); //const Point& pos, const Point& cube_pos);

// create arrow at given points with given name
void createArrow(Point base, Point end, std::string name, std::string force_distance_function_index);

// create arrow that points from the cube's initial position to its new position
void moveBoxArrow(const InteractiveMarkerFeedbackConstPtr &feedback);
    
//activate/deactive modification in UI
void setDoSomething();
void setDoNothing();

/**
* In case the Parameter-Server has been overloaded with different data (or in case there is doubt that the Parameter-Server and object data are in sync).
*/
void reloadSession();

void updateModalities(std::string name);

void setForceDistanceFunction(const InteractiveMarkerFeedbackConstPtr &feedback);

void setupMenuHandler(std::string name);

void setupArrowMenuHandler(std::string name);

void setupForceMenuHandler(std::string name);

};

#endif
