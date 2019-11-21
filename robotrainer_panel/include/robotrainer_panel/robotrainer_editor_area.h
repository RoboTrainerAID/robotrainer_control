#ifndef ROBOTRAINER_EDITOR_AREA
#define ROBOTRAINER_EDITOR_AREA

#include <robotrainer_panel/robotrainer_editorParameters.h>
#include <robotrainer_panel/robotrainer_editor_tool.h>
#include <QInputDialog>
#include <QWidget>

class RobotrainerEditorArea : public RobotrainerEditorTool {
  
  robotrainer_panel::robotrainer_editorParameters* params;

  //used to be recognized in the loaded sessions
  std::string file_name;
  
  std::vector<std::string> area_names;
  
  //map that holds all areas by holding their start and end point
  std::map<std::string, geometry_msgs::Point> area_map;
  std::map<std::string, geometry_msgs::Point> edge_map;
  
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  std::map <std::string, interactive_markers::MenuHandler*> area_menu_handler;
  ros::Publisher* area_publisher;
  
  //modification settings, implemented in "robotrainer modalities" tool
  std::map<std::string, std::vector<std::string>> area_functions;

  
public:
  
  RobotrainerEditorArea();
  
  RobotrainerEditorArea(robotrainer_panel::robotrainer_editorParameters* params_,
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
    ros::Publisher* area_publisher_);
  
  int getAreaCount();
  
  int getCount();
  
  std::string getNs();
  
  void moveBoxRadius(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  void createAreaWithoutAddingName(geometry_msgs::Point center, geometry_msgs::Point edge, std::string name, std::vector<std::string> area_functions_);
  
  void createArea(geometry_msgs::Point center, geometry_msgs::Point edge, std::string name);
  
  void moveArea(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  void loadFile(std::string filename);
  
  void loadFile();
  
  void saveFile(std::string filename);
  
  void deleteAreaFromServer(std::string arrow_name);
  
  void deleteArea(std::string arrow_name);

  // load arrows of selected bag-file
  void deleteSelectedArea(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  void toggleAreaFunction(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  // resets all related data on the server (for session switching)
  void resetServer();
  
  //activate/deactive modification in UI
  void setDoSomething();
  void setDoNothing();
  
  /**
  * In case the Parameter-Server has been overloaded with different data (or in case there is doubt that the Parameter-Server and object data are in sync).
  */
  void reloadSession();
  
  void updateModalities(std::string name);
  
  void setupAreaMenuHandler(std::string name);
  
};

#endif
