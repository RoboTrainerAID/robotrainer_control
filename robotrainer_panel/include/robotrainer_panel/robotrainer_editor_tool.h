 /**
  * A class to be inherited by all kinds of tools in robotrainerPanel.
  */
 
#ifndef ROBOTRAINER_EDITOR_TOOL
#define ROBOTRAINER_EDITOR_TOOL

#include <cstdio>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "rviz/validate_quaternions.h"

#include <fstream>
#include <cmath>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <robotrainer_panel/robotrainer_editorParameters.h>

#include <std_msgs/String.h>

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std;
using namespace boost;

class RobotrainerEditorTool {
    
protected:
    
    //rosparam handler
    robotrainer_panel::robotrainer_editorParameters* params;
    
    //used to be recognized in the loaded sessions
    std::string file_name;
    
public:
    
    RobotrainerEditorTool();
    
    std::string getFileName();
    
    void setFileName(std::string name);
    
    virtual std::string getNs();
    
    virtual std::string getDisplayPathFileName();
    
    virtual int getCount();
    
    virtual void setDoNothing();
    
    virtual void clear();
    
    virtual void reloadSession();
    
    virtual void resetServer();
    
    std::string makeFileName(std::string filename);
    
    virtual void saveFile(std::string filename);
    
    void saveFileTool(std::string filename, std::string full_ns);
    
    virtual void loadFile(std::string filename);
    
    virtual void loadFile();
    
    static geometry_msgs::Quaternion normalizeQuaternion(geometry_msgs::Quaternion quaternion_msg);
    static geometry_msgs:: Point pointDiff(geometry_msgs::Point a, geometry_msgs::Point b);
    static geometry_msgs:: Point pointAdd(geometry_msgs::Point a, geometry_msgs::Point b);
    static geometry_msgs:: Point pointScale(geometry_msgs::Point a, double scale);
    static geometry_msgs:: Point pointCenter(geometry_msgs::Point a, geometry_msgs::Point b);
    static double vectorLength(geometry_msgs::Point a);
    static geometry_msgs::Point vectorToPoint(geometry_msgs::Vector3 a);
    static bool pointEqual(geometry_msgs::Point a, geometry_msgs::Point b);
    static double scalarMult(geometry_msgs::Point a, geometry_msgs::Point b);
    static double distanceFromLine(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point c);
    
};

#endif
