#ifndef ROBOTRAINER_EDITOR_HEIKA
#define ROBOTRAINER_EDITOR_HEIKA

#include <robotrainer_panel/robotrainer_editorParameters.h>
#include <robotrainer_panel/robotrainer_editor_tool.h>
#include <QInputDialog>
#include <QWidget>

class RobotrainerEditorWall : public RobotrainerEditorTool {

    robotrainer_panel::robotrainer_editorParameters* params;

    //used to be recognized in the loaded sessions
    std::string file_name;

    std::vector<std::string> wall_names;

    //map that holds all walls by holding their start and end point
    std::map<std::string, std::array<geometry_msgs::Point, 2> > wall_map;

    std::map<std::string, geometry_msgs::Point> aoe_map;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    std::map<std::string, interactive_markers::MenuHandler*> wall_menu_handler;
    std::map<std::string, interactive_markers::MenuHandler*> area_menu_handler;
    ros::Publisher* wall_publisher;

    std::map<std::string, std::string> force_distance_function;

public:

    RobotrainerEditorWall();

    RobotrainerEditorWall(robotrainer_panel::robotrainer_editorParameters* params_,
                          boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
                          ros::Publisher* wall_publisher_);

    int getWallCount();

    int getCount();

    std::string getNs();

    void createWallWithoutAddingName(Point base, Point end, std::string name, std::string force_distance_function_index);

    void createWall(Point base, Point end, std::string name);

    void moveWall(const InteractiveMarkerFeedbackConstPtr &feedback, bool is_left);

    void moveWallLeft(const InteractiveMarkerFeedbackConstPtr &feedback);

    void moveWallRight(const InteractiveMarkerFeedbackConstPtr &feedback);

    void scaleAreaOfEffect(const InteractiveMarkerFeedbackConstPtr &feedback);

    void updateAreaOfEffect(std::string wall_name);

    void loadFile(std::string filename);

    void loadFile();

    void saveFile(std::string filename);

    void deleteWallFromServer(std::string arrow_name);

    void deleteWall(std::string arrow_name);

    // load arrows of selected bag-file
    void deleteSelectedWall(const InteractiveMarkerFeedbackConstPtr &feedback);

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

    void setForceDistanceFunction(const InteractiveMarkerFeedbackConstPtr &feedback);

    void setupMenuHandler(std::string name);

    void setupWallMenuHandler(std::string name);

    void setupAreaMenuHandler(std::string name);

};

#endif
