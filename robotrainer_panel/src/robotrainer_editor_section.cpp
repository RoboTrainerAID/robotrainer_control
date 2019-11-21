/**
 * This file is meant to define sections of the path, in which certain modalities trake effect.
 * Namely the behaviour of the robot when  building up distance to this part of the path.
 */


#include <ros/param.h>
#include <robotrainer_panel/robotrainer_editor_section.h>

RobotrainerEditorSection::RobotrainerEditorSection() {}

RobotrainerEditorSection::RobotrainerEditorSection(robotrainer_panel::robotrainer_editorParameters* params_,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        int path_length_,
        ros::Publisher* section_publisher_,
        std::string display_path_file_name_,
        QWidget* parent_)
{
    params = params_;
    server = server_;
    path_length = path_length_ - 1; //there is one less edge in the graph than there are points
    section_publisher = section_publisher_;
    display_path_file_name = display_path_file_name_,
    parent = parent_;

    path_menu_handler[0] = new interactive_markers::MenuHandler;
    path_menu_handler[0]->insert ( "start section here", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->startSection ( feedback );
    });
    path_menu_handler[1] = new interactive_markers::MenuHandler;
    path_menu_handler[1]->insert ( "end section here", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->createSection( feedback );
    });
    setup();
}

int RobotrainerEditorSection::getSectionCount() {
    return section_end.size();
}

int RobotrainerEditorSection::getCount() {
    return getSectionCount();
}

std::string RobotrainerEditorSection::getNs() {
    return params->section_ns;
}

int RobotrainerEditorSection::getIterator(std::string str) {
    return getIterator(str, params->path_ns + "_");
}

int RobotrainerEditorSection::getIterator(std::string str, std::string pattern) {
    try {
      return std::stoi(str.substr(str.rfind(pattern)+pattern.size()));
    }
    catch (const std::invalid_argument& e) {
            ROS_ERROR_STREAM("Invalid argument! cannot turn '" << str.substr(str.rfind(pattern)+pattern.size()) << "' into Integer! (robotrainer_editor_section/getIterator)");
      return -1;
    }
}

std::string RobotrainerEditorSection::getSectionName(std::string path_segment_name) {
    int section_id = isInSection(getIterator(path_segment_name));
    if (section_id != -1) return section_names[section_id];
    ROS_ERROR_STREAM("path segment '" << path_segment_name << "' is not part of a tracking section, but apperently should be!");
    return params->section_ns + "_none";
}

int RobotrainerEditorSection::isInSection(int path_segment) {
    for(int i = 0; i < section_end.size(); i++) {
        int start = getIterator(section_start[i]);
        int end = getIterator(section_end[i]);
        if (start <= path_segment && end >= path_segment) return i;
    }
    return -1;
}

std::string RobotrainerEditorSection::replaceIterator(std::string str, int i) {
    std::size_t found = str.rfind("path_")+5;
    if (found != std::string::npos) str.replace(found,  std::string::npos , std::to_string(i));
    else {
        ROS_WARN("could not find substring 'path' in %s, robotrainer_editor_section/replaceIterator", str.c_str());
        return "";
    }
    return str;
}

void RobotrainerEditorSection::setColor(std::string name, float r, float g) { //blue is irrelevant
    InteractiveMarker path_interactive_marker;
    if(!server->get(name, path_interactive_marker)) {
        ROS_WARN("could not find %s in interactive marker server. robotrainer_editor_section/setColor", name.c_str());
        return;
    }
    //make clear that something has changed
    path_interactive_marker.controls[0].markers[0].color.r = r;
    path_interactive_marker.controls[0].markers[0].color.g = g;
    server->insert ( path_interactive_marker );
}

void RobotrainerEditorSection::setMenuHandler(std::string path_segment_name, int handler_id) {
    InteractiveMarker path_interactive_marker;
    if(!server->get(path_segment_name, path_interactive_marker)) {
        ROS_WARN("could not find %s in interactive marker server. robotrainer_editor_section/setMenuHandler", path_segment_name.c_str());
        return;
    }
    if (handler_id != 2) path_menu_handler[handler_id]->apply (*server, path_segment_name);
    else {
        std::string section_name = getSectionName(path_segment_name);
        setupSection(section_name);
    }
}

void RobotrainerEditorSection::setupSection(int section_number) {
    std::ostringstream section_name;
    section_name << params->section_ns << "_" << section_number;
    setupSection(section_name.str().c_str());
}

void RobotrainerEditorSection::setupSection(std::string section_name) {
    int i = getIterator(section_name, params->section_ns + "_");
    int start = getIterator(section_start[i], params->path_ns + "_");
    int end = getIterator(section_end[i], params->path_ns + "_");
    setupSection(section_name, start, end);
}

//
void RobotrainerEditorSection::setupSection(std::string section_name, int start, int end) {
    //path segments that actually contain a section need custom menu handlers, so that they can show their current modifier states
    interactive_markers::MenuHandler* menu_handler = new interactive_markers::MenuHandler;
    menu_handler = new interactive_markers::MenuHandler;
    menu_handler->insert ( "delete this section", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->deleteSelectedSection( feedback );
    });
    if (max_deviation[section_name] < MIN_MAX_DEVIATION) max_deviation[section_name] = MIN_MAX_DEVIATION; //enforce minimum value
    /*
    std::ostringstream sstr;
    sstr << "set maximum deviation - currently: " << max_deviation[section_name];
    menu_handler->insert (sstr.str(), [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->setMaxDeviation ( feedback );
    });
    sstr.clear();
    sstr.str("");
    */
    std::string curr_func_name = force_distance_function[section_name];
    for(std::string func_name : params->force_distance_functions){
        bool is_this = (curr_func_name == func_name);
        std::ostringstream oss;
        oss << func_name << " " << is_this;
        menu_handler->insert ( oss.str(), [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
            this->setForceDistanceFunction (feedback);
        });
    }
    
    section_menu_handler[section_name] = menu_handler;
    
    for (int i = start; i <= end; i++) {
      std::ostringstream oss;
      oss.str("");
      oss << "path_" << i;
      setColor(oss.str(), 0.0, 1.0);
      menu_handler->apply (*server, oss.str());
    }
    server->applyChanges();
}

void RobotrainerEditorSection::setup() {
    setupPath();
    section_start.clear();
    section_end.clear();
}

void RobotrainerEditorSection::setupPath() {
    std::ostringstream oss;
    for (int i = 1; i < path_length; i++) {
        oss.str("");
        oss << "path_" << i;
        setMenuHandler(oss.str(), 0);
        setColor(oss.str(), 1.0, 0.0);
    }
    server->applyChanges();
}

void RobotrainerEditorSection::resetPath() {
    setup();
    ros::param::del (params->editor_ns + "/" + params->section_ns);
}

void RobotrainerEditorSection::startSection(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        startSection( feedback->marker_name.substr(0, feedback->marker_name.rfind ( "/" )));
    }
}

void RobotrainerEditorSection::startSection(std::string name) {

    section_start.push_back(name);

    //set all non-section path pieces not to expect a "section start" but a "section end"
    for (int j = 1; j < path_length; j++) {
        if (isInSection(j) != -1) continue;
        else {
            setMenuHandler(replaceIterator(name, j), 1);
        }
    }

    setColor(name, 0.0, 1.0);
    server->applyChanges();
}

void RobotrainerEditorSection::createSection(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        createSection( feedback->marker_name.substr(0, feedback->marker_name.rfind ( "/" )));
    }
}

void RobotrainerEditorSection::createSection(std::string end_name) {

    int current = section_start.size() - 1;
    std::string curr = std::to_string(current);
    std::string section_name = params->section_ns + "_" + curr;
    
    section_names.push_back(section_name);
    std::string start_name = section_start[current];
    section_end.push_back(end_name);

    createSectionWithoutAddingName(current);

    ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->config_ns + "/" + params->section_ns + "_names", section_names);
}

//this is used by "create section" after adding the name, or when reloading a session and the name is allready listed
//"current" needs to be passed, because it might mean something different coming from "reloadSession" thatn coming from "createSection"
void RobotrainerEditorSection::createSectionWithoutAddingName(int current) {
    
    std::string section_name = section_names[current];
    std::string start_name = section_start[current];
    std::string end_name = section_end[current];
    
    force_distance_function[section_name] = params->force_distance_functions[0];
  
    if(section_end.size() > section_start.size()) section_start.push_back(start_name); //start_name hadn't been added yet (probably being loaded from file)
    
    int start =  getIterator(start_name);
    int end =  getIterator(end_name);

    //selected an end behind the start, just turn it arround
    if(end < start) {
        std::string temp_string = section_start[current];
        int temp_int = start;
        section_start[current] = section_end[current];
        section_end[current] = temp_string;
        start = end;
        end = temp_int;
    }

    //make this a section
    setupSection(section_name, start, end);

    //set all non-section path pieces not to expect a "section end" but a "section start"
    for (int j = 1; j < path_length; j++) {
        bool is_in_section = false;

        for(int i = 0; i < section_end.size(); i++) {
            int start =getIterator(section_start[i]);
            int end = getIterator(section_end[i]);
            if (start <= j && end >= j){
              is_in_section = true;
              break;
            }
        }

        if (is_in_section) continue;
        else {
            setMenuHandler(replaceIterator(end_name, j) , 0);
        }
    }

    server->applyChanges();

    //save all data relevant to reproduce the current state to the parameter server
    ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + "start", "point" + std::to_string(start) );
    ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + "end", "point" + std::to_string(end + 1) );
    ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + params->force_distance_function_ns, force_distance_function[section_name]);
    ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + params->max_deviation_ns, max_deviation[section_name]);
    
    
}

void RobotrainerEditorSection::deleteSection(std::string name) {

    int id = getIterator(name);
    int i;
    int start = 0;
    int end = path_length;
    for(i = 0; i < section_end.size(); i++) {
        start = getIterator(section_start[i]);
        end = getIterator(section_end[i]);
        if (start <= id && end >= id) break;
    }

    for (int j = start; j <= end; j++) {
        std::string part_name = replaceIterator(name, j);
        setMenuHandler(part_name, 0);
        setColor(part_name, 1.0, 0.0);
    }
    section_start.erase(section_start.begin()+i);
    section_end.erase(section_end.begin()+i);

    std::string section_name = section_names[i];

    section_names.erase(section_names.begin()+i);

    server->applyChanges();

    //save all data relevant to reproduce the current state to the parameter server
    ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->config_ns + "/" + params->section_ns + "_names", section_names);
    ros::param::del (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name);
    
    
}

void RobotrainerEditorSection::loadFile(std::string filename) {

    //run a bash function to load the file into the parameter server
    std::ostringstream sstr;
    //sstr << "bash -i -c 'rosparam load " << filename << "'";
    sstr << "rosparam load " << filename << " /" << params->editor_ns << "/" << params->section_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system ( chr );

    file_name = filename;

    loadFile();
}

void RobotrainerEditorSection::loadFile() {

    //display path filename was set when object was created, now check if it fits the one of the loaded file

    std::string display_path_file_name_;

    ros::param::get (params->editor_ns + "/" +  params->section_ns + "/" + params->config_ns +  "/display_path_file_name", display_path_file_name_ );

    if ( display_path_file_name != display_path_file_name_ ) ROS_WARN ( "loaded section file doesnt seem to be made for the path file" );

    ros::param::get(params->editor_ns + "/" + params->section_ns + "/" + params->config_ns + "/" + params->section_ns + "_names", section_names);
    
    for(int i = 0; i < section_names.size(); i++) {

        std::string index = std::to_string(i);
        
        std::string section_name = params->section_ns + "_" + index;

        std::string start;
        ros::param::get (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + "start", start);
        start = "path_" + std::to_string(getIterator(start, "point"));

        std::string end;
        ros::param::get (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + "end", end);
        end = "path_" + std::to_string(getIterator(end, "point") - 1);

        //int force_distance_function_;
        ros::param::get (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + params->force_distance_function_ns, force_distance_function[section_name]);
        //force_distance_function[section_name] = force_distance_function;
        ros::param::get (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + params->max_deviation_ns, max_deviation[section_name]);
        
        section_start.push_back(start);
        section_end.push_back(end);
        createSectionWithoutAddingName(i);
    }
}

void RobotrainerEditorSection::saveFile(std::string filename) {
    saveFileTool(filename, params->editor_ns + "/" + params->section_ns);
}

void RobotrainerEditorSection::deleteSelectedSection(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        deleteSection( feedback->marker_name.substr(0, feedback->marker_name.rfind ( "/" )));
    }
}

// resets all related data on the server (for session switching)
void RobotrainerEditorSection::resetServer() {
    //resetPath();
}

//this function makes the path clickable
void RobotrainerEditorSection::setDoSomething() {
    setDo(InteractiveMarkerControl::BUTTON);
    //this loop makes shure all path segments have the right menu handlers. this is necessary becasue they might get lost when editing (i.e. croping) the path
    for(int i = 1; i < path_length; i++) {
        std::ostringstream path_segment_name;
        path_segment_name << params->path_ns << "_" << i;
        int j = isInSection(i);
        if (j == -1) setMenuHandler(path_segment_name.str().c_str(), 0);
        else {
          setupSection(j);
          i = getIterator(section_end[j]); //+1 will come automatically
        }
    }
}

//this function makes the path unclickable
void RobotrainerEditorSection::setDoNothing() {
    setDo(InteractiveMarkerControl::NONE);
}

void RobotrainerEditorSection::setDo(int interaction_mode) {
    for(int i = 0; i < path_length; i++) {
        InteractiveMarker inter_marker;
        std::ostringstream sstr;
        sstr << params->path_ns << "_" << i;
        if (!(server->get(sstr.str().c_str(), inter_marker))) {
            continue;
        }

        inter_marker.controls[0].interaction_mode = interaction_mode;

        server->insert(inter_marker);
    }
    server->applyChanges();
}

/**
* In case the Parameter-Server has been overloaded with different data (or in case there is doubt that the Parameter-Server and object data are in sync).
*/
void RobotrainerEditorSection::reloadSession() {

    setupPath();
  
    ros::param::del("/" + params->section_ns);

    ros::param::set(params->editor_ns + "/" + params->section_ns + "/" + params->config_ns + "/" + params->section_ns + "_names", section_names);

    for(int i = 0; i < section_names.size(); i++) {

        std::string index = std::to_string(i);
        std::string section_name = params->section_ns + "_" + index;

        ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + "start", "point" + std::to_string(getIterator(section_start[i])));
        ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + "end", "point" + std::to_string(getIterator(section_end[i]) + 1));

        createSectionWithoutAddingName(i);
        
        ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + params->force_distance_function_ns, force_distance_function[section_name]);
        ros::param::set (params->editor_ns + "/" + params->section_ns + "/" + params->data_ns + "/" + section_name + "/" + params->max_deviation_ns, max_deviation[section_name]);
    }

}

/*
void RobotrainerEditorSection::setMaxDeviation(const InteractiveMarkerFeedbackConstPtr &feedback) {

    std::string name;
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        name = feedback->marker_name;
    }
    else return;

    bool ok;
    std::string section_name = getSectionName(name);
    double max = max_deviation[section_name];

    double d = QInputDialog::getDouble(parent, "Please enter the max deviation modifier of your choice.",
                                       "Max Deviation:", max, MIN_MAX_DEVIATION, MAX_MAX_DEVIATION, DECIMAL_PRECISION, &ok);

    if (ok) max_deviation[section_name] = d;
    
    updateModalities(section_name);
}
*/

void RobotrainerEditorSection::setForceDistanceFunction(const InteractiveMarkerFeedbackConstPtr &feedback) {
  
    std::string name;
    std::string func_name;
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        name = feedback->marker_name;
        int menu_entry_id = feedback->menu_entry_id;
        func_name = params->force_distance_functions[menu_entry_id - 2]; //cannot pass a string into this function, so this is the only way to retrieve the name of the selected functions // -1 because of "delete" menu enty, -2 because of "array started 0"
    }
    else return;
    
    std::string section_name = getSectionName(name);
    
    force_distance_function[section_name] = func_name;

    updateModalities(section_name);
}

void RobotrainerEditorSection::updateModalities(std::string section_name) {

    ros::param::set(params->editor_ns + "/" + params->section_ns + "/" + params->data_ns +  "/" + section_name + "/" + params->max_deviation_ns, max_deviation[section_name]);
    ros::param::set(params->editor_ns + "/" + params->section_ns + "/" + params->data_ns +  "/" + section_name + "/" + params->force_distance_function_ns, force_distance_function[section_name]);
    
    setupSection(section_name);
    //std::system("rosservice call /base/configure_modalities"); not anymore, only when "set active" in panel happens!
}

