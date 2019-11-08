#include <ros/param.h>

#include <robotrainer_panel/robotrainer_editor_tool.h>

#include <cstdlib>
#include <boost/concept_check.hpp>

RobotrainerEditorTool::RobotrainerEditorTool() {}

std::string RobotrainerEditorTool::getFileName()
{
    return file_name;
}

void RobotrainerEditorTool::setFileName(std::string name) {
    file_name = name;
}

int RobotrainerEditorTool::getCount() {
    return 0;
}

std::string RobotrainerEditorTool::getDisplayPathFileName() {
    return ""; //only needed for certain types and generic methods
}

std::string RobotrainerEditorTool::getNs() {
    return "tool";
}

void RobotrainerEditorTool::setDoNothing() {
    ROS_ERROR("tried to call setDoNothing on non-specified tool type! (robotrainer_editor_tool.cpp/setDoNothing");
}

void RobotrainerEditorTool::clear() {
    ros::param::del(getNs());
}

void RobotrainerEditorTool::reloadSession() {
    ROS_ERROR("tried to reload Session of non-specified tool type! (robotrainer_editor_tool.cpp/reloadSession");
}

void RobotrainerEditorTool::resetServer() {
    ROS_ERROR("tried to reset Server for non-specified tool type! (robotrainer_editor_tool.cpp/resetServer");
}

//if filename doesnt allready contain ".namespace.yaml", add it
std::string RobotrainerEditorTool::makeFileName(std::string filename) {
    
    std::string yaml = ".yaml";
    std::string ending = "." + getNs() + yaml;
    if (filename.length() <= ending.length() || 0 != filename.compare (filename.length() - ending.length(), ending.length(), ending)) {
        if (filename.length() > yaml.length() && 0 == filename.compare (filename.length() - yaml.length(), yaml.length(), yaml))
            filename = filename.substr(0, filename.size() - yaml.size());
        filename += ending;
    }
    return filename;
}

void RobotrainerEditorTool::saveFile(std::string s) {
    ROS_ERROR("tried to save file '%s' for non-specified tool type! (robotrainer_editor_tool.cpp/saveFile", s.c_str());
}

void RobotrainerEditorTool::saveFileTool(std::string filename, std::string full_ns) {
    filename = makeFileName(filename);
    ROS_ERROR("got it");
    std::ostringstream sstr;
    sstr << "rosparam dump " << filename << " /" << full_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);
}

void RobotrainerEditorTool::loadFile(std::string s) {
    ROS_ERROR("tried to load file '%s'  for non-specified tool type! (robotrainer_editor_tool.cpp/loadFile", s.c_str());
}

void RobotrainerEditorTool::loadFile() {
    ROS_ERROR("tried to load file for non-specified tool type! (robotrainer_editor_tool.cpp/loadFile");
}

geometry_msgs::Quaternion RobotrainerEditorTool::normalizeQuaternion(geometry_msgs::Quaternion quaternion_msg)
{
    double x = quaternion_msg.x, y = quaternion_msg.y, z = quaternion_msg.z, w = quaternion_msg.w;

    if ( 0.0 == x && 0.0 == y && 0.0 == z && 0.0 == w )
    {
        w = 1.0;
        quaternion_msg.w = w;
    }
    else
    {
        double norm2 = w * w + x * x + y * y + z * z;
        norm2 = std::sqrt( norm2 );
        double invnorm = 1.0 / norm2;
        w *= invnorm;
        x *= invnorm;
        y *= invnorm;
        z *= invnorm;
        quaternion_msg.w = w;
        quaternion_msg.x = x;
        quaternion_msg.y = y;
        quaternion_msg.z = z;
    }
    return quaternion_msg;
}

geometry_msgs:: Point RobotrainerEditorTool::pointDiff(geometry_msgs::Point a, geometry_msgs::Point b) {
    geometry_msgs::Point c;
    c.x = (b.x - a.x);
    c.y = (b.y - a.y);
    c.z = (b.z - a.z);
    
    //ROS_ERROR("b = (%f, %f, %f) - a = (%f, %f, %f) == c= (%f, %f, %f)", a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);

    return c;
}

geometry_msgs:: Point RobotrainerEditorTool::pointAdd(geometry_msgs::Point a, geometry_msgs::Point b) {
    geometry_msgs::Point c;
    c.x = (b.x + a.x);
    c.y = (b.y + a.y);
    c.z = (b.z + a.z);
    //ROS_ERROR("a = (%f, %f, %f) + b = (%f, %f, %f) == c= (%f, %f, %f)", a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);

    return c;
}

geometry_msgs:: Point RobotrainerEditorTool::pointScale(geometry_msgs::Point a, double scale) {
    a.x *= scale;
    a.y *= scale;
    a.z *= scale;
    return a;
}

geometry_msgs:: Point RobotrainerEditorTool::pointCenter(geometry_msgs::Point a, geometry_msgs::Point b) {
    geometry_msgs::Point c;
    c = pointDiff(a, b);
    c.x /= 2;
    c.y /= 2;
    c.z /= 2;
    c = pointAdd(a, c);
    //ROS_ERROR("the center between a = (%f, %f, %f) and b = (%f, %f, %f) is c= (%f, %f, %f)", a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);

    return c;
}

double RobotrainerEditorTool::vectorLength(geometry_msgs::Point a) {
    double scalar_mult = scalarMult(a,a);
    double result = std::sqrt(scalarMult(a,a));
    //ROS_ERROR("the square root of %f is %f", scalar_mult, result);
    return result;
}

geometry_msgs::Point RobotrainerEditorTool::vectorToPoint(geometry_msgs::Vector3 a) {
    geometry_msgs::Point p;
    p.x = a.x;
    p.y = a.y;
    p.z = a.z;
    return p;
}

bool RobotrainerEditorTool::pointEqual(geometry_msgs::Point a, geometry_msgs::Point b) {
    return a.x == b.x && a.y == b.y && a.z == b.z; //ignore z, should never be different for us, if it was that would be potentially malicious anyway

}

double RobotrainerEditorTool::scalarMult(geometry_msgs::Point a, geometry_msgs::Point b) {
    double result = a.x*b.x + a.y*b.y + a.z*b.z;
    //ROS_ERROR("scalar mult of a = (%f, %f, %f) and b = (%f, %f, %f) is %f", a.x, a.y, a.z, b.x, b.y, b.z, result);
    return result;
}

/* given up. (at least in the context of walls) this results in the calculation of he distance to some line, but not the one intended. (could be due to some interactive marker black magic)
double RobotrainerEditorTool::distanceFromLine(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point c) {
    geometry_msgs::Point d = pointDiff(a, b);
    double result = scalarMult(pointDiff(a, c), d)/vectorLength(d);
    if (result > 0) return result;
    else return -1 * result;
}
*/
