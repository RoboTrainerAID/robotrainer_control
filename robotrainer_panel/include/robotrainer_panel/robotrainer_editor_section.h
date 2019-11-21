#ifndef ROBOTRAINER_EDITOR_SECTION
#define ROBOTRAINER_EDITOR_SECTION

#include <robotrainer_panel/robotrainer_editorParameters.h>
#include <robotrainer_panel/robotrainer_editor_tool.h>
#include <QInputDialog>
#include <QWidget>

#define MIN_MAX_DEVIATION 0.3 //minum value for maximum deviation modifier
#define MAX_MAX_DEVIATION 10 //maximum value for maximum deviation modifier
#define DECIMAL_PRECISION 2 //decimal precision for maximum deviation modifier

class RobotrainerEditorSection : public RobotrainerEditorTool {
  
  robotrainer_panel::robotrainer_editorParameters* params;

  //used to be recognized in the loaded sessions
  std::string file_name;
  std::string display_path_file_name;
  
  int path_length;
  std::vector<std::string> section_names;
  std::vector<std::string> section_start;
  std::vector<std::string> section_end;
  
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  
  //modification settings, implemented in "robotrainer modalities" tool
  std::map<std::string, double> max_deviation;
  interactive_markers::MenuHandler* path_menu_handler[2];
  std::map<std::string, interactive_markers::MenuHandler*>section_menu_handler;
  ros::Publisher* section_publisher;

  std::map<std::string, std::string> force_distance_function;

  QWidget* parent;
  
public:
  
  RobotrainerEditorSection();
  
  RobotrainerEditorSection(robotrainer_panel::robotrainer_editorParameters* params_,
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
    int path_length_,
    ros::Publisher* section_publisher_,
    std::string display_path_file_name_,
    QWidget* parent_);
  
  int getSectionCount();
  
  int getCount();
  
  std::string getNs();
  
  int getIterator(std::string str);
  
  int getIterator(std::string str, std::string pattern);
  
  std::string replaceIterator(std::string str, int i);
  
  std::string getSectionName(std::string path_segment_name);
  
  int isInSection(int path_segment);
  
  void setColor(std::string name, float r, float g); //blue is irrelevant
  
  void setMenuHandler(std::string name, int handler_id); //set a certain menu handler for a certain piece of path
  
  void setupSection(int section_number);
  void setupSection(std::string name);
  
  void setupSection(std::string name, int start, int end);
  
  void setup();
  
  void setupPath();
  
  void resetPath();
  
  //void addName(std::string name); //will diynamically decide if this is a start or an end
  
  void startSection(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  void startSection(std::string name); //perform the necessary steps to identify a pathsement as section
  
  void createSection(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  //start name will be found at end of corresponding list
  void createSection(std::string end_name);
  
  void createSectionWithoutAddingName(int current);
  
  void deleteSelectedSection(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  void deleteSection(std::string name);
  
  void loadFile(std::string filename);
  
  void loadFile();
  
  void saveFile(std::string filename);
  
  // resets all related data on the server (for session switching)
  void resetServer();
  
  //activate/deactive modification in UI
  void setDoSomething();
  void setDoNothing();
  void setDo(int interaction_mode);
  
  /**
  * In case the Parameter-Server has been overloaded with different data (or in case there is doubt that the Parameter-Server and object data are in sync).
  */
  void reloadSession();
  
  void setForceDistanceFunction(const InteractiveMarkerFeedbackConstPtr &feedback);
  //void setMaxDeviation(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  void updateModalities(std::string name);
  
};

#endif
