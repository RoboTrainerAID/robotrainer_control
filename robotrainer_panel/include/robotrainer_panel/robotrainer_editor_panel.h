/**
 * An RViz plugin to edit RobotrainerEditor Projects with a nice UI.
 *
 * This File could no have been created without the inspiration from the "rviz_plugin_tutorial::teleop_panel".
 * Though all remaining matches are basic rviz/Qt contents, we'd like to give credit to it's creators:
 * http://docs.ros.org/lunar/api/rviz_plugin_tutorials/html/panel_plugin_tutorial.html
 */

#ifndef ROBOTRAINER_EDITOR_HEIKA_PANEL_H
#define ROBOTRAINER_EDITOR_HEIKA_PANEL_H

#ifndef Q_MOC_RUN
#include <stdio.h>
#include <cstdio>
#include <ros/ros.h>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QItemSelectionModel>
#include <QComboBox>
#include <QMessageBox>
#include <QFileDialog>

#include <rviz/panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <robotrainer_panel/robotrainer_editor_tool.h>
#include <robotrainer_panel/robotrainer_editor_display_path.h>
#include <robotrainer_panel/robotrainer_editor_force.h>
#include <robotrainer_panel/robotrainer_editor_wall.h>
#include <robotrainer_panel/robotrainer_editor_area.h>
#include <robotrainer_panel/robotrainer_editor_section.h>
#include <robotrainer_panel/robotrainer_editorParameters.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <fstream>
#include <cmath>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <ctime>

#include <robotrainer_data_service/FileList.h>

#endif

class QLineEdit;

namespace rviz
{
class IntProperty;
}

namespace robotrainer_editor_panel
{

class RobotrainerEditorPanel: public rviz::Panel
{

    Q_OBJECT
public:

    RobotrainerEditorPanel( QWidget *parent_ = 0, ros::NodeHandle params_node_handle = ros::NodeHandle("params"));
    
    QWidget* parent;

    virtual void load( const rviz::Config& config);
    virtual void save( rviz::Config config ) const;

////Session Actions
    
    //name
    std::string findName(std::string prefix, std::vector<std::string> in_use);
    template <typename T> std::string nameSession(std::string name, std::string prefix, std::string display_name, std::vector<std::string>* names, T* tool);
    
    //save
    template <typename T> void saveSession(T* tool, std::vector<std::string>* names, std::map<std::string, T*>* sessions, QComboBox* combobox);
    void saveForceSession();
    void savePathSession();
    void saveWallSession();
    void saveAreaSession();
    void saveSectionSession();
    
    //reset
    void reset(RobotrainerEditorDisplayPath* d);
    void reset(RobotrainerEditorForce* f);
    void reset(RobotrainerEditorArea* a);
    void reset(RobotrainerEditorWall* w);
    void reset(RobotrainerEditorSection* s);

    //load
    template <typename T> void loadSession(T** tool, std::string name, std::vector<std::string>* names, std::map<std::string, T*>* sessions, QComboBox* combobox);
    void loadPathSession(std::string name);
    void loadForceSession(std::string name);
    void loadWallSession(std::string name);
    void loadAreaSession(std::string name);
    void loadSectionSession(std::string name);

////File Actions
    template <typename T> void saveFile(T** tool);
    template <typename T> bool checkNotYetLoaded(std::string file_name, std::map<std::string, T*>* sessions);
    template <typename T> bool checkNotYetLoaded(QString file_name, std::map<std::string, T*>* sessions);
    template <typename T> bool checkPathComaptible(std::string file_name, T* tool);
    template <typename T> bool checkPathComaptible(QString file_name, T* tool);
    template <typename T> void loadFile(T** tool, std::vector<std::string>* names, std::map<std::string, T*>* sessions, QComboBox* combobox);
    
public Q_SLOTS:
  
    //functions to create GUI elements
    std::string getParamType(std::string s);
    void fillTable(QTableWidget** table, std::vector<std::array<std::string, 3>>* content);
    void setupTables();
    void createTableLayout(QTableWidget** table, std::string text);
    QVBoxLayout* createActionLayout(QPushButton** button, const char* slot, QString text);
    QVBoxLayout* createSessionLayout(QComboBox** combo_box, const char* slot, QString text);
    QVBoxLayout* createLoadSaveLayout(const char* load_slot, const char* save_slot, QString text);
    QVBoxLayout* createDataServiceLayout(const char* slot, QString text);
    void setupLayout();
    
    //Actions
    void setActive(); //enable current session to be used for experiments
    void toggleMode(); //mode dedicated for observing experiments - all editing is disabled
    //----
    void enableEditing();
    void enableArrowCreation();
    void enableSectionCreation();
    void createWall();
    void createArea();
    void setDoNothing();

    void recordSinglePoint();

    void clearSession();
    void loadPathSession(QString name);
    void loadForceSession(QString name);
    void loadWallSession(QString name);
    void loadAreaSession(QString name);
    void loadSectionSession(QString name);
    void setupSession(std::string file_name);
    void sessionFromActive();

    void resetPathDependencies();
    void resetRobotrainerEditorForce();
    void resetRobotrainerEditorSection();

    //File Actions
    void saveScenarioFile(std::string filename);
    void saveScenarioFile();
    void loadScenarioFile(std::string filename);
    void loadScenarioFile();
    //--------------------------------------------
    void savePathFile();
    void saveForceFile();
    void saveWallFile();
    void saveAreaFile();
    void saveSectionFile();
    void loadPathFile();
    void loadForceFile();
    void loadWallFile();
    void loadAreaFile();
    void loadSectionFile();
    // links to equaly named function in robotrainer_editor_display_path
    //void receiveCheckpoint(RobotrainerEditorDisplayPath* robotrainer_editor_display_path_, const PointConstPtr& point_msg);
    
    void dataServiceSaveEditor();
    void dataServiceSave(std::string ns, std::string filename);
    void dataServiceLoadEditor();
    void dataServiceLoadEditor(std::string filename);
    void dataServiceLoadEditor(QString filename);
    void dataServiceLoadActive();
    std::string dataServiceLoad(std::string ns);
    std::string dataServiceLoad(std::string ns, std::string filename);
    void dataServiceFetchFileList();

protected Q_SLOTS:

protected:

    //rosparam handler
    robotrainer_panel::robotrainer_editorParameters* params;

    RobotrainerEditorDisplayPath* robotrainer_editor_display_path; //the "current" robotrainer_editor_display_path object
    RobotrainerEditorForce* robotrainer_editor_force; //the "current" robotrainer_editor_force object
    RobotrainerEditorWall* robotrainer_editor_wall;
    RobotrainerEditorArea* robotrainer_editor_area;
    RobotrainerEditorSection* robotrainer_editor_section;

    //menu handler for interactive points along the path, needs to be declared here to handle sideeffects //what sideeffects?
    //interactive_markers::MenuHandler* point_menu_handler;

    std::vector<std::string> path_names;
    std::map<std::string, RobotrainerEditorDisplayPath*> robotrainer_editor_display_path_sessions;
    
    std::map<std::string, std::vector<string>> force_names;
    std::map<std::string, RobotrainerEditorForce*> robotrainer_editor_force_sessions;

    std::vector<std::string> wall_names;
    std::map<std::string, RobotrainerEditorWall*> robotrainer_editor_wall_sessions;

    std::vector<std::string> area_names;
    std::map<std::string, RobotrainerEditorArea*> robotrainer_editor_area_sessions;

    std::map<std::string, std::vector<string>> section_names;
    std::map<std::string, RobotrainerEditorSection*> robotrainer_editor_section_sessions;

    //a server that handles the display of the interactive Markers in rviz
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

    ros::Publisher path_update_publisher;

    ros::Publisher arrow_publisher;

    ros::Publisher wall_publisher;

    ros::Publisher area_publisher;

    ros::Publisher section_publisher;

    ros::Publisher record_info_publisher;

    tf2_ros::Buffer *p_tfBuffer;
    tf2_ros::TransformListener *p_tfListener;

    //QT objects
    QTabWidget* tab_widget;

    //tables
    QListWidget* tables; //list of all tables
    QTableWidget* modifier_table; //table to show and edit parameters that concern scaling/ (size/amount/rate modifiers)
    QTableWidget* ns_table; //table for namespaces for parameter server
    QTableWidget* topic_table; //table for topics for publishing/subscribing

    bool record = false; //flag for record mode
    QPushButton* toggle_mode_button;
    //enable current session to be used for experiments
    QPushButton* set_active_button;
    //edit-mode buttons
    QPushButton* editing_button;
    QPushButton* create_force_button;
    QPushButton* create_wall_button;
    QPushButton* create_area_button;
    QPushButton* create_section_button;
    //record-mode buttons
    QPushButton* record_single_point_button;

    QVBoxLayout* session_layout;
    QPushButton* session_from_active_button;
    QPushButton* clear_session_button;
    QComboBox* robotrainer_editor_display_path_sessions_combobox;
    QComboBox* robotrainer_editor_force_sessions_combobox;
    QComboBox* robotrainer_editor_wall_sessions_combobox;
    QComboBox* robotrainer_editor_area_sessions_combobox;
    QComboBox* robotrainer_editor_section_sessions_combobox;

    QPushButton* cancel_button;
    QLabel* current_action_label;
    
    QString yaml_file_directory;

    ros::NodeHandle nh;
    
    //the files that the data_service has (may in the future be the content of the database)
    std::vector<std::string> service_files;
    QComboBox* service_file_combobox;
};
} //end namespace robotrainer_editor_panel

#endif
