#include <robotrainer_panel/robotrainer_editor_area.h>
#include <rviz/display.h>
RobotrainerEditorArea::RobotrainerEditorArea() {}

RobotrainerEditorArea::RobotrainerEditorArea(robotrainer_panel::robotrainer_editorParameters* params_,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* area_publisher_) {

    params = params_;
    server = server_;
    area_publisher = area_publisher_;
}

int RobotrainerEditorArea::getAreaCount() {
    return area_names.size();
}

int RobotrainerEditorArea::getCount() {
    return getAreaCount();
}

std::string RobotrainerEditorArea::getNs() {
    return params->area_ns;
}

// adjust the area circle and send appropriate message
void RobotrainerEditorArea::moveBoxRadius(const InteractiveMarkerFeedbackConstPtr &feedback) {
    // the function should be called the event of an interactive marker beeing moved. only then the that code is required to run.
    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {

        //read the names of the marker involved, this will be needed to identify the objects to change
        string cube_name = feedback->marker_name;

        //read the position of the marker, as the changes to do to the area depend on it
        Point pos = feedback->pose.position;

        //identify the area by matching the names and namespaces its name depended on at creation
        string area_name =  cube_name.substr(0, cube_name.rfind("/")); //" + params->area_ns;" not needed, inserted into interactive marker server with name of whole namesapce of the area in param server

        //use the name we just matched to retrieve the interactive marker from the interactive_marker_server
        InteractiveMarker area_int_marker;
        server->get(area_name, area_int_marker);

        //update the area depending on the new position of the interactive_marker cube
        Point center_pos = area_int_marker.pose.position;
        double new_area = sqrt(pow(center_pos.x - pos.x, 2) + pow(center_pos.y - pos.y, 2));
        area_int_marker.controls[0].markers[0].scale.x = new_area * 2;
        area_int_marker.controls[0].markers[0].scale.y = new_area * 2;

        edge_map[cube_name] =  pos;

        //the interactive_marker object holds the name (which is how we retrieved it in the first place), so we can simply reupload it to replace the outdated one
        server->insert(area_int_marker);
        server->applyChanges();

        //save all data relevant to reproduce the current state to the parameter server
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/x", pos.x);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/y", pos.y);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/z", pos.z);
        //area doesnt need to be saved, as it is calculatable from the cube
        
    }
}

//this is important to load files without infinetely readding area names
void RobotrainerEditorArea::createAreaWithoutAddingName(Point center, Point edge, std::string name, std::vector<std::string> area_functions_) {

    //define the size of the circle by its force
    double radius = sqrt(pow(center.x - edge.x, 2) + pow(center.y - edge.y, 2));

    //create the visual object for the area
    Marker area_marker;
    area_marker.type = Marker::CYLINDER;
    area_marker.scale.x = radius * 2;
    area_marker.scale.y = radius * 2;
    area_marker.scale.z = 0.001;
    area_marker.color.r = 0.0;
    area_marker.color.g = 1.0;
    area_marker.color.b = 0.2;
    area_marker.color.a = 0.3;
    area_marker.pose.orientation = normalizeQuaternion(area_marker.pose.orientation);

    //define the way the area behaves when manipulated
    InteractiveMarkerControl area_control;
    area_control.orientation.w = 1.0;
    area_control.orientation.x = 0.0;
    area_control.orientation.y = 1.0;
    area_control.orientation.z = 0.0;
    area_control.interaction_mode = InteractiveMarkerControl::NONE;
    area_control.always_visible = true;
    area_control.markers.push_back(area_marker);
    area_control.orientation = normalizeQuaternion(area_control.orientation);

    //create the interactive_marker for the area (holder of visual and control element)
    InteractiveMarker area_inter_marker;
    area_inter_marker.header.frame_id = params->frame_id;
    area_inter_marker.name = name;
    area_inter_marker.pose.position = center;
    area_inter_marker.controls.push_back(area_control);

    //insert the area-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
    //	the lambda function "moveArea" is required because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //	for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    std::function<void(const InteractiveMarkerFeedbackConstPtr &)> moveArea = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->moveArea(feedback);
    };
    //update the interactive_marker_server according to new changes
    server->insert(area_inter_marker, moveArea);

    //set some modification data to be editable by the context menu handler
    area_functions[name] = area_functions_;

    //setup menu handler for this area
    setupAreaMenuHandler(name);

    //map to hold all area locations
    area_map[name] =  center;

    //create a cube that is meant to adjust the circles size, this is the interactive object
    InteractiveMarker cube_inter_marker;
    cube_inter_marker.header.frame_id = params->frame_id;
    cube_inter_marker.pose.position = edge;
    cube_inter_marker.scale = params->base_scale;
    std::string cube_name = name + "/" + params->margin_ns;
    cube_inter_marker.name = cube_name;

    //this control object is meant to define the way that the cube can be manipulated
    InteractiveMarkerControl cube_control;
    cube_control.orientation.w = 1.0;
    cube_control.orientation.x = 0.0;
    cube_control.orientation.y = 1.0;
    cube_control.orientation.z = 0.0;
    cube_control.interaction_mode = InteractiveMarkerControl::NONE;
    cube_control.always_visible = true;

    //this is the visible cube
    Marker cube;
    cube.type = Marker::CUBE;
    cube.scale.x = cube.scale.y = cube.scale.z = cube_inter_marker.scale * 0.2;
    cube.color.r = 0.0;
    cube.color.g = 1.0;
    cube.color.b = 0.2;
    cube.color.a = 0.8;
    cube.pose.orientation = normalizeQuaternion(cube.pose.orientation);
    cube_control.markers.push_back(cube);
    cube_control.orientation = normalizeQuaternion(cube_control.orientation);
    cube_inter_marker.controls.push_back(cube_control);

    //insert the cube-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
    //	the lambda function "moveBox" is required because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //	for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    std::function<void(const InteractiveMarkerFeedbackConstPtr &)> moveBox = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->moveBoxRadius(feedback);
    };
    server->insert(cube_inter_marker, moveBox);
    server->applyChanges();

    //map that holds all force-scaling cubes and their positions locally
    edge_map[cube_name] =  edge;

    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + name + "/" + params->area_ns + "/x", center.x);
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + name + "/" + params->area_ns + "/y", center.y);
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + name + "/" + params->area_ns + "/z", center.z);
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + name + "/" + params->margin_ns + "/x", edge.x);
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + name + "/" + params->margin_ns + "/y", edge.y);
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + name + "/" + params->margin_ns + "/z", edge.z);

    updateModalities(name);
}

void RobotrainerEditorArea::createArea(Point center, Point edge, std::string name) {

    std::vector<std::string> baum;
    createAreaWithoutAddingName(center, edge, name, baum);

    area_names.push_back(name);
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->config_ns + "/" + params->area_ns + "_names", area_names);

}

void RobotrainerEditorArea::moveArea(const InteractiveMarkerFeedbackConstPtr &feedback) {

    //something is odd here, concerning what is the cube and what is the area, but is also handled accordingly and acts correctly  ("- * - = +" ?)

    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {

        string area_name = feedback->marker_name;

        //string area_name = cube_name.substr(0, cube_name.rfind("/") +1) + params->area_ns;

        Point end_pos = feedback->pose.position;

        Point center_pos;

        center_pos = area_map[area_name];

        //fetch teh area marker form the interactive marker server
        InteractiveMarker area_int_marker;
        if(!server->get(area_name, area_int_marker)) ROS_ERROR("couldn't fetch area_int_marker from server!");

        //move the correct cube arround
        area_int_marker.pose.position = end_pos;

        //update area marker in interactive marker server
        server->insert(area_int_marker);

        //adapt position of edge cube on server
        std::string edge_name = area_name + "/" + params->margin_ns;
        InteractiveMarker edge_inter_marker;
        if(!server->get(edge_name, edge_inter_marker)) ROS_ERROR("couldn't fetch area_edge_inter_marker from server!");
        Point new_edge_pos;
        new_edge_pos.x = edge_inter_marker.pose.position.x + (end_pos.x - center_pos.x);
        new_edge_pos.y = edge_inter_marker.pose.position.y + (end_pos.y - center_pos.y);
        new_edge_pos.z = edge_inter_marker.pose.position.z + (end_pos.z - center_pos.z);
        edge_inter_marker.pose.position = new_edge_pos;

        //update edge_cube marker in interactive marker server
        server->insert(edge_inter_marker);

        server->applyChanges();

        area_publisher->publish(area_int_marker);

        area_map[area_name] = end_pos;
        edge_map[edge_name] = new_edge_pos;
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/x", end_pos.x);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/y", end_pos.y);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/z", end_pos.z);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/x", new_edge_pos.x);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/y", new_edge_pos.y);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/z", new_edge_pos.z);
    }
    
}

void RobotrainerEditorArea::setDoSomething() {

    for(int i = 0; i < area_names.size(); i++) {
        InteractiveMarker inter_marker;
        std::string area_name = area_names[i];
        if (!(server->get(area_name, inter_marker))) {
            ROS_ERROR("An area element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_area.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

        server->insert(inter_marker);
    }

    for(int i = 0; i < area_names.size(); i++) {
        InteractiveMarker inter_marker;
        std::string edge_name = area_names[i] + "/" + params->margin_ns;
        if (!(server->get(edge_name, inter_marker))) {
            ROS_ERROR("An edge_cube element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_area.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

        server->insert(inter_marker);
    }
}

void RobotrainerEditorArea::setDoNothing() {

    for(int i = 0; i < area_names.size(); i++) {
        InteractiveMarker inter_marker;
        std::string area_name = area_names[i];
        if (!(server->get(area_name, inter_marker))) {
            ROS_ERROR("An area element was not found in the interactive marker server! (setDoNothing(), robotrainer_editor_area.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        server->insert(inter_marker);
    }

    for(int i = 0; i < area_names.size(); i++) {
        InteractiveMarker inter_marker;
        std::string edge_name = area_names[i] + "/" + params->margin_ns;
        if (!(server->get(edge_name, inter_marker))) {
            ROS_ERROR("An edge_cube element was not found in the interactive marker server! (setDoNothing(), robotrainer_editor_area.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        server->insert(inter_marker);
    }
}

void RobotrainerEditorArea::loadFile(std::string filename) {
    //run a bash function to load the file into the parameter server
    std::ostringstream sstr;
    sstr << "rosparam load " << filename << " /" << params->editor_ns << "/" << params->area_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);

    file_name = filename;

    loadFile();
}

void RobotrainerEditorArea::loadFile() {

    //create a topic to publish force areas on
    vector<string> topics;
    topics.push_back(params->area_topic);

    //load the list of area names
    ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->config_ns + "/" + params->area_ns + "_names", area_names);

    //retrieve all data based on the area names (all links to any data can be derived from the area name)
    for(int i = 0; i < area_names.size(); i++) {

        std::string area_name = area_names[i];

        //retrieve area data
        Point center;
        ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/x", center.x);
        ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/y", center.y);
        ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/z", center.z);

        Point edge;
        ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns +  "/" + area_name + "/" + params->margin_ns + "/x", edge.x);
        ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns +  "/" + area_name + "/" + params->margin_ns + "/y", edge.y);
        ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns +  "/" + area_name + "/" + params->margin_ns + "/z", edge.z);

        if (!ros::param::get(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns +  "/" + area_name + "/" + params->area_functions_ns, area_functions[area_name])) area_functions[area_name].clear();

        createAreaWithoutAddingName(center, edge, area_name, area_functions[area_name]);
    }

}

void RobotrainerEditorArea::reloadSession() {

    ros::param::del("/" + params->area_ns);

    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->config_ns + "/" + params->area_ns + "_names", area_names);

//reupload all data the parameter server centerd on the area names
    for(int i = 0; i < area_names.size(); i++) {

        std::string area_name = area_names[i];
        std::string edge_name = area_name + "/" + params->margin_ns;

        //retrieve area data
        Point center = area_map[area_name];
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/x", center.x);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/y", center.y);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->area_ns + "/z", center.z);

        Point edge = edge_map[edge_name];
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/x", edge.x);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/y", edge.y);
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name + "/" + params->margin_ns + "/z", edge.z);
        
        ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns +  "/" + area_name + "/" + params->area_functions_ns, area_functions[area_name]);

        createAreaWithoutAddingName(center, edge, area_name, area_functions[area_name]);
    }

}

void RobotrainerEditorArea::saveFile(std::string filename) {
    saveFileTool(filename, params->editor_ns + "/" + params->area_ns);
}

void RobotrainerEditorArea::deleteAreaFromServer(std::string area_name) {

    InteractiveMarker area_int_marker;
    //if the area can be retrieved from the server, clear it's controls so it can't be adressed anymore and publish that "unusable" area
    if (server->get(area_name, area_int_marker)) {
        area_int_marker.controls.clear();
        area_publisher->publish(area_int_marker);
    }
    //erase the area object from the interactive_marker_server
    server->erase(area_name);

    string cube_name = area_name + "/" + params->margin_ns;

    //erase the cubes at the ends of the area
    server->erase(cube_name);

    server->applyChanges();

}

void RobotrainerEditorArea::deleteArea(std::string area_name) {

    deleteAreaFromServer(area_name);

    //area_names.erase(area_name); worked when "area_names" was still a set, but needs to be vector for ros::param
    for(int i = 0; i < area_names.size(); i++) {
        if(area_names[i] == area_name) area_names.erase(area_names.begin()+i);
        else if(i = area_names.size() -1) std::system("Warning: area_name could not be removed from area_names as it has not been found!");
    }

    //update the list of area names in the parameter server
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->config_ns + "/" + params->area_ns + "_names", area_names);
    //delete the area-data from the parameter server
    ros::param::del(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns + "/" + area_name);
}

// load areas of selected bag-file
void RobotrainerEditorArea::deleteSelectedArea(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        deleteArea(feedback->marker_name);
    }
}

//in the parameters, a set of functions is saved. the area object only saves those activated, but the menu show all available
void RobotrainerEditorArea::toggleAreaFunction(const InteractiveMarkerFeedbackConstPtr &feedback){
  
    std::string name;
    std::string func_name;
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        name = feedback->marker_name;
        int menu_entry_id = feedback->menu_entry_id;
        func_name = params->area_functions[menu_entry_id - 2]; //cannot pass a string into this function, so this is the only way to retrieve the name of the selected functions // -1 because of "delete" menu enty, -2 because of "array started 0"
    }
    else return;

        //will be true if rmeove fails
    if(std::remove(area_functions[name].begin(), area_functions[name].end(), func_name) == area_functions[name].end())
        area_functions[name].push_back(func_name);

    updateModalities(name);
}

// resets all related data on the server (for session switching)
void RobotrainerEditorArea::resetServer() {
    for (auto &area_name : area_names) {
        deleteAreaFromServer(area_name);
    }
    //delete all data concerning this object from the parameter server
    ros::param::del(params->area_ns);
    server->applyChanges();
}

void RobotrainerEditorArea::updateModalities(std::string name) {
    ros::param::set(params->editor_ns + "/" + params->area_ns + "/" + params->data_ns +  "/" + name + "/" + params->area_functions_ns, area_functions[name]);
    
    setupAreaMenuHandler(name);
    //std::system("rosservice call /base/configure_modalities"); not anymore, only when "set active" in panel happens!
}

void RobotrainerEditorArea::setupAreaMenuHandler(std::string name) {

    interactive_markers::MenuHandler* menu_handler = new interactive_markers::MenuHandler;
    menu_handler->insert("delete area", [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->deleteSelectedArea(feedback);
    });

    for(std::string func_name : params->area_functions){
      bool is_on = (std::find(area_functions[name].begin(), area_functions[name].end(), func_name) != area_functions[name].end());
      std::ostringstream oss;
      oss << func_name << " " << is_on;
      menu_handler->insert(oss.str(), [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
          this->toggleAreaFunction ( feedback);
      });
    }

    area_menu_handler[name] = menu_handler;

    menu_handler->apply(*server, name);
    
    server->applyChanges();
}
