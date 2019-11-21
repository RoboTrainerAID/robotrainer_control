/**
 * This file is meant to hold the necessary functions to hold an organize the force force of a RobotrainerEditor
 * experiment.
 * It is meant to be include by the RobotrainerEditor Panel and used through its UI.
 * The functions concerning any other objects, including the pure path shall be implemented in their own
 * files.
 *
 * This file is a manipulated version of a legacy "robotrainer_editor_display_path"
 */
#include <ros/param.h>

#include <robotrainer_panel/robotrainer_editor_force.h>

#include <cstdlib>
#include <boost/concept_check.hpp>

RobotrainerEditorForce::RobotrainerEditorForce() {}

RobotrainerEditorForce::RobotrainerEditorForce ( robotrainer_panel::robotrainer_editorParameters* params_,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* arrow_publisher_,
        std::string display_path_file_name_)
{

    create_when_move = false;

    params = params_;
    setMetaParams ( server_, arrow_publisher_ );

    display_path_file_name = display_path_file_name_;

    ros::param::set ( params->editor_ns + "/" + params->force_ns +"/" + params->config_ns +  "/display_path_file_name", display_path_file_name );
    ros::param::set ( params->editor_ns + "/" + params->force_ns +"/" + params->config_ns +  "/newton_per_meter", params->newton_per_meter );
}

void RobotrainerEditorForce::setMetaParams ( boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* arrow_publisher_ )
{

    server = server_;
    arrow_publisher = arrow_publisher_;

}

void RobotrainerEditorForce::setCreateWhenMove ( bool create_when_move_ )
{
    create_when_move = create_when_move_;
}

std::string RobotrainerEditorForce::getDisplayPathFileName()
{
    return display_path_file_name;
}

int RobotrainerEditorForce::getArrowCount()
{
    return force_names.size();
}

int RobotrainerEditorForce::getCount() {
    return getArrowCount();
}

std::string RobotrainerEditorForce::getNs() {
    return params->force_ns;
}

void RobotrainerEditorForce::setCubePointMap ( std::map<std::string, geometry_msgs::Point> cube_point_map_ )
{
    cube_point_map = cube_point_map_;
}

// adjust the force circle and send appropriate message
void RobotrainerEditorForce::moveBoxRadius ( const InteractiveMarkerFeedbackConstPtr &feedback )
{
    // the function should be called the event of an interactive marker beeing moved. only then the that code is required to run.
    if ( feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE ) {

        //read the names of the marker involved, this will be needed to identify the objects to change
        string margin_name = feedback->marker_name;

        //read the position of the marker, as the changes to do to the force depend on it
        Point pos = feedback->pose.position;

        //identify the force area by matching the names and namespaces its name depended on at creation
        string area_name = margin_name.substr(0, margin_name.rfind ( "/" ) +1) + params->area_ns;

        //use the name we just matched to retrieve the interactive marker from the interactive_marker_server
        InteractiveMarker area_int_marker;
        server->get ( area_name, area_int_marker );

        //update the force depending on the new position of the interactive_marker cube
        Point base_pos = area_int_marker.pose.position;
        double new_radius = sqrt ( pow ( base_pos.x - pos.x, 2 ) + pow ( base_pos.y - pos.y, 2 ) );
        area_int_marker.controls[0].markers[0].scale.x = new_radius * 2;
        area_int_marker.controls[0].markers[0].scale.y = new_radius * 2;

        margin_map[margin_name] =  pos;

        //the interactive_marker object holds the name (which is how we retrieved it in the first place), so we can simply reupload it to replace the outdated one
        server->insert ( area_int_marker );
        server->applyChanges();


        //identify and retrieve the arrow by matching the names and namespaces its name depended on at creation, as we did for the force
        string arrow_name = area_name.substr(0, area_name.rfind ( "/" ) +1) + params->arrow_ns;
        InteractiveMarker arrow_int_marker;
        server->get ( arrow_name, arrow_int_marker );

        //update the information in the arrow
        //arrow_int_marker.controls[0].markers[0].points[0].z = new_radius;

        //publish the updated arrow object on the arrow publisher
        arrow_publisher->publish ( arrow_int_marker );

        //save all data relevant to reproduce the current state to the parameter server
        ros::param::set (params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + margin_name + "/x", pos.x );
        ros::param::set (params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + margin_name + "/y", pos.y );
        ros::param::set (params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + margin_name + "/z", pos.z );
        //force doesnt need to be saved, as it is calculatable from the cube
        
    }
}

void RobotrainerEditorForce::deleteArrowFromServer ( std::string arrow_name )
{
    InteractiveMarker arrow_int_marker;
    //if the arrow can be retrieved from the server, clear it's controls so it can't be adressed anymore and publish that "unusable" arrow
    if ( server->get ( arrow_name, arrow_int_marker ) ) {
        arrow_int_marker.controls.clear();
        arrow_publisher->publish ( arrow_int_marker );
    }
    
    //erase the arrow object from the interactive_marker_server
    server->erase ( arrow_name );
    
    //the arrow was based on a cube laying on the path. but from the moment it was used to create the arrow it didn't lie on the path anymore. that cube shall be returned to the path now.
    string force_name = arrow_name.substr(0, arrow_name.rfind ( "/" )); //no "+1" for this use
    
    string cube_name = params->cube_ns + force_name.substr(force_name.rfind("_"));
    
    Pose pose;
    pose.position = cube_point_map[cube_name]; //the cube_point_map holds the original location of the cube, on the path
    server->setPose ( cube_name, pose );

    //delete the circle and the circle-configuration cube that depended on the arrow we are deleting
    string area_name = arrow_name.substr(0, arrow_name.rfind ( "/" ) +1) + params->area_ns;
    server->erase ( area_name );
    string margin_name = arrow_name.substr(0, arrow_name.rfind ( "/" ) +1) + params->margin_ns;
    server->erase ( margin_name );
    string text_name = arrow_name.substr(0, arrow_name.rfind("/") +1) + "text";
    server->erase ( text_name );

    server->applyChanges();

}

void RobotrainerEditorForce::deleteArrow ( std::string arrow_name )
{    
    deleteArrowFromServer ( arrow_name );
    
    string force_name = arrow_name.substr(0, arrow_name.rfind("/"));

    //force_names.erase(arrow_name); worked when "force_names" was still a set, but needs to be vector for ros::param
    for ( int i = 0; i < force_names.size(); i++ ) {
        if ( force_names[i] == force_name ) {
            force_names.erase ( force_names.begin()+i );
        } else if ( i == force_names.size() -1 ) ROS_ERROR ( "Warning: robotrainer_editor_force - force_name could not be removed from force_names as it has not been found!" );
    }

    
    
    //update the list of arrow names in the parameter server
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->config_ns + "/"  + params->force_ns + "_names", force_names );
    //delete the arrow-data from the parameter server
    ros::param::del(params ->editor_ns + "/" +  params->force_ns + "/" + params->data_ns + "/" + force_name);
    
    
}

// delete arrow(+force) selected in GUI
void RobotrainerEditorForce::deleteSelectedArrow ( const InteractiveMarkerFeedbackConstPtr &feedback )
{
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        deleteArrow ( feedback->marker_name );
    }
}

// delete force(+arrow) selected in GUI
void RobotrainerEditorForce::deleteSelectedForce( const InteractiveMarkerFeedbackConstPtr &feedback )
{
    if ( feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT ) {
        deleteArrow ( feedback->marker_name.substr(0, feedback->marker_name.rfind ( "/" )) + "/arrow");
    }
}

void RobotrainerEditorForce:: clearArrowsFromCropedPath(std::map<std::string, geometry_msgs::Point> cube_point_map_) {
    cube_point_map = cube_point_map_;
    std::vector<std::string> cube_point_names;
    for(std::map<std::string, geometry_msgs::Point>::iterator it = cube_point_map.begin(); it != cube_point_map.end(); ++it) {
      cube_point_names.push_back(it->first);
    }
    clearArrowsFromCropedPath(cube_point_names);
}

//used when the path is croped and the forces residing on the removed part of the path also need to be removed
void RobotrainerEditorForce::clearArrowsFromCropedPath(std::vector<std::string> cube_point_names){
    for ( int i = 0; i < force_names.size(); i++ ){
        std::string cube_name = params->cube_ns + force_names[i].substr(force_names[i].rfind("_"));
        if (std::find(cube_point_names.begin(), cube_point_names.end(), cube_name) == cube_point_names.end()){ //this is true when there is no more cube_point corresponding to this force
            string arrow_name = force_names[i] + "/" + params->arrow_ns;
            deleteArrow(arrow_name);
        }
    }
            
}

void RobotrainerEditorForce::resetServer()
{
    for ( auto &force_name : force_names ) {
        deleteArrowFromServer ( force_name  + "/" + params->arrow_ns);
    }
    
    //delete all data concerning this object from the parameter server
    ros::param::del (params->editor_ns + "/" + params->force_ns );
    server->applyChanges();
}

// reset/ delete all created arrows and their related data (all circles, actually everything that this object holds)
void RobotrainerEditorForce::reset()
{
    for ( vector<string>::iterator it = force_names.begin(); it != force_names.end(); it++ ) {
        deleteArrow ( *it );
    }
    //delete all data concerning this object from the parameter server
    ros::param::del (params->editor_ns + "/" + params->force_ns );
}

// creates adjustable force around arrow
void RobotrainerEditorForce::createForce ( std::string arrow_name, geometry_msgs::Point pos, geometry_msgs::Point cube_pos ) //const Point& pos, const Point& cube_pos) {
{

    //create an interactive marker for the circle
    InteractiveMarker inter_marker;
    inter_marker.header.frame_id = params->frame_id;
    inter_marker.pose.position = pos;
    std::string area_name = arrow_name.substr(0, arrow_name.rfind ( "/" ) +1) + params->area_ns;
    inter_marker.name = area_name;
    force_point_map[area_name] =  pos; //still might insert allready existing objects (when loading session) BUT shouldn't cause problems because of the nature of maps

    //create the control object that defines the manipulation behaviour (none)
    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    //define the size of the circle by its force
    double radius = sqrt ( pow ( pos.x - cube_pos.x, 2 ) + pow ( pos.y - cube_pos.y, 2 ) );

    //create the visual circle
    Marker force_cylinder;
    force_cylinder.type = Marker::CYLINDER;
    force_cylinder.scale.x = radius * 2;
    force_cylinder.scale.y = radius * 2;
    force_cylinder.scale.z = 0.001;
    force_cylinder.color.r = 0.0;
    force_cylinder.color.g = 0.2;
    force_cylinder.color.b = 1.0;
    force_cylinder.color.a = 0.3;
    control.markers.push_back ( force_cylinder );
    inter_marker.controls.push_back ( control );

    //insert the created marker into the interactive_marker_server
    server->insert ( inter_marker );
    force_menu_handler[arrow_name]->apply ( *server, area_name );
    server->applyChanges();

    //create a cube that is meant to adjust the circles size, this is the interactive object
    InteractiveMarker cube_inter_marker;
    cube_inter_marker.header.frame_id = params->frame_id;
    cube_inter_marker.pose.position = cube_pos;
    cube_inter_marker.scale = params->base_scale;
    std::string margin_name = arrow_name.substr(0, arrow_name.rfind ( "/" ) +1 ) + params->margin_ns;
    cube_inter_marker.name = margin_name;

    //this control object is meant to define the way that the cube can be manipulated
    InteractiveMarkerControl cube_control;
    cube_control.orientation.w = 1.0;
    cube_control.orientation.x = 0.0;
    cube_control.orientation.y = 1.0;
    cube_control.orientation.z = 0.0;
    cube_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    cube_control.always_visible = true;

    //this is the visible cube
    Marker cube;
    cube.type = Marker::CUBE;
    cube.scale.x = cube.scale.y = cube.scale.z = cube_inter_marker.scale * 0.2;
    cube.color.r = 0.0;
    cube.color.g = 0.2;
    cube.color.b = 1.0;
    cube.color.a = 0.8;
    cube.pose.orientation = normalizeQuaternion(cube.pose.orientation);
    cube_control.markers.push_back ( cube );
    cube_control.orientation = normalizeQuaternion(cube_control.orientation);
    cube_inter_marker.controls.push_back ( cube_control );

    //insert the cube-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
    //	the lambda function "moveBox" is required because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //	for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    std::function<void ( const InteractiveMarkerFeedbackConstPtr & ) > moveBox = [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->moveBoxRadius ( feedback );
    };
    server->insert ( cube_inter_marker, moveBox );
    server->applyChanges();
    //map that holds all force-scaling cubes and their positions locally
    margin_map[margin_name] =  cube_pos; //still might insert allready existing objects (when loading session) BUT shouldn't cause problems because of the nature of maps

    //insert data relevant to restore the new configuration into the parameter server
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" +  params->data_ns + "/" + area_name + "/x", pos.x );
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" +  params->data_ns + "/" + area_name + "/y", pos.y );
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" +  params->data_ns + "/" + area_name + "/z", pos.z );
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" +  params->data_ns + "/" + margin_name + "/x", cube_pos.x );
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" +  params->data_ns + "/" + margin_name + "/y", cube_pos.y );
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" +  params->data_ns + "/" + margin_name + "/z", cube_pos.z );

    
}

// create arrow at given points with given name
void RobotrainerEditorForce::createArrow ( Point base, Point end, std::string name, std::string force_distance_function_index)
{
  
    force_distance_function[name] = force_distance_function_index;
    setupMenuHandler(name);
    
    double length = sqrt ( pow ( end.x-base.x, 2 ) + pow ( end.y-base.y, 2 ) );

    //create the visual object for the arrow
    Marker arrow_marker;
    arrow_marker.type = Marker::ARROW;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 0.2;
    arrow_marker.color.b = 1.0;
    arrow_marker.color.a = 1.0;
    arrow_marker.scale.x = 0.1 * params->base_scale;
    arrow_marker.scale.y = 0.2 * params->base_scale;
    arrow_marker.scale.z = 0.3 * length;
    arrow_marker.points.push_back ( base );
    arrow_marker.points.push_back ( end );

    //define the way the arrow behaves when manipulated
    InteractiveMarkerControl arrow_control;
    arrow_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    arrow_control.always_visible = true;
    arrow_control.markers.push_back ( arrow_marker );

    //create the interactive_marker for the arrow (holder of visual and control element)
    InteractiveMarker arrow_int_marker;
    arrow_int_marker.header.frame_id = params->frame_id;
    arrow_int_marker.name = name;
    arrow_int_marker.controls.push_back ( arrow_control );
    
    //save information about arrow as relative vector
    geometry_msgs::Point arrow_value;
    arrow_value = pointDiff(base, end);
    arrow_map[name] = arrow_value; //still might insert allready existing objects (when loading session) BUT shouldn't cause problems because of the nature of maps
    
    //update the interactive_marker_server and arrow_menu_handler according to new changes
    server->insert ( arrow_int_marker );
    arrow_menu_handler[name]->apply(*server, name );

    //update the interactive path cube so it is actually at the position of the end of the arrow
    string force_name = name.substr(0, name.rfind ( "/" )); //no "+1" for this use
    string cube_name = params->cube_ns + force_name.substr(force_name.rfind("_"));
    Pose pose;
    pose.position = end; //the cube_point_map holds the original location of the cube, on the path
    server->setPose ( cube_name, pose );
    
    //add a text below the cube, indicating how many newtons of force this arrow represents in his current configuration
    Marker text_marker;
    text_marker.type = Marker::TEXT_VIEW_FACING;
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.2;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.scale.z = 0.1 * params->base_scale;
    text_marker.pose = pose;
    text_marker.pose.position.y -= 1;
    std::ostringstream strs;
    strs << sqrt(arrow_value.x*arrow_value.x + arrow_value.y*arrow_value.y)*params->newton_per_meter;
    text_marker.text = strs.str() + "N";
    
    
    //controller that tells the text not to be movable
    InteractiveMarkerControl text_control;
    text_control.interaction_mode = InteractiveMarkerControl::NONE;
    text_control.always_visible = true;
    text_control.markers.push_back ( text_marker );

    //create the interactive_marker for the text (holder of visual and control element)
    InteractiveMarker text_int_marker;
    text_int_marker.header.frame_id = params->frame_id;
    text_int_marker.name = name.substr(0, name.rfind ( "/" ) +1) + "text";
    //let the text display the newton-force that the arrow represents
    text_int_marker.controls.push_back ( text_control );
    
    server->insert( text_int_marker );
    
    server->applyChanges();
    //force_names.push_back(name); shall NOT happen here because this function is also called when arrows are loaded from file, is loaded in "moveBoxArrow" where "createArrow" is triggered when manually creating an arrow instead

    arrow_int_marker.controls[0].markers[0].points[0].z = params->influence_radius;
    arrow_publisher->publish ( arrow_int_marker );

    //the parameter server only needs the data provided in the parameters of the function, as they are enough to reproduce the state!
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + name + "/x", arrow_value.x * params->newton_per_meter);
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + name + "/y", arrow_value.y * params->newton_per_meter );
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + name + "/z", arrow_value.z * params->newton_per_meter );
    
    updateModalities(name);
}

// create arrow that points from the cube's initial position to its new position
void RobotrainerEditorForce::moveBoxArrow ( const InteractiveMarkerFeedbackConstPtr &feedback )
{
    if ( feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE ) {

        string cube_name = feedback->marker_name;

        string force_name = params->force_ns + cube_name.substr(cube_name.rfind("_"));
        
        string arrow_name = force_name + "/" + params->arrow_ns;
        
        string text_name = force_name + "/text";

        if ( ! ( cube_point_map.count ( cube_name ) ) ) ROS_ERROR ( "unknown point in cube_point_map in robotrainer_editor_force!" );

        Point base_pos = cube_point_map[cube_name];

        Point end_pos = feedback->pose.position;

        InteractiveMarker arrow_int_marker;
        if ( !server->get ( arrow_name, arrow_int_marker ) ) {
            //if we are in edit, and not in create mode, no new arrows shall be created, and the cube shall be returned to his intitial position
            if ( !create_when_move ) {
                InteractiveMarker cube_int_marker;
                server->get ( cube_name, cube_int_marker );
                cube_int_marker.pose.position = base_pos;
                server->insert ( cube_int_marker );
                server->applyChanges();
                return;
            }
            //if otherwise, we are in create mode, allow creating a new arrow
            else {
                createArrow ( base_pos, end_pos, arrow_name, params->force_distance_functions[0]);
                force_names.push_back ( force_name ); //shall happen here because createArrow is also called when arrows are loaded from file
                ros::param::set (params->editor_ns + "/" +  params->force_ns + "/" + params->config_ns + "/" + params->force_ns + "_names", force_names );
                Point cube_pos;
                cube_pos.x = base_pos.x + params->influence_radius * sqrt ( 0.5 );
                cube_pos.y = base_pos.y + params->influence_radius * sqrt ( 0.5 );
                createForce ( arrow_name, base_pos, cube_pos );
            }
        } else {
            double length = sqrt ( pow ( end_pos.x-base_pos.x, 2 ) + pow ( end_pos.y-base_pos.y, 2 ) );

            arrow_int_marker.controls[0].markers[0].points[1] = end_pos;
            arrow_int_marker.controls[0].markers[0].scale.z = 0.3 * length;

            server->insert ( arrow_int_marker );
            arrow_menu_handler[arrow_name]->apply ( *server, arrow_name );

            arrow_publisher->publish ( arrow_int_marker );
        }

        geometry_msgs::Point arrow_value;
        arrow_value = pointDiff(base_pos, end_pos);
        arrow_map[arrow_name] = arrow_value; //still might insert allready existing objects (when loading session) BUT shouldn't cause problems because of the nature of maps
        
        InteractiveMarker text_int_marker;
        if (server->get(text_name, text_int_marker)) {
          std::ostringstream strs;
          strs << sqrt(arrow_value.x*arrow_value.x + arrow_value.y*arrow_value.y)*params->newton_per_meter;
          text_int_marker.controls[0].markers[0].text = strs.str() + "N";
          text_int_marker.controls[0].markers[0].pose.position = end_pos;
          text_int_marker.controls[0].markers[0].pose.position.y -= 0.2;
          server->insert ( text_int_marker );
        }
        else ROS_ERROR("Couldn't fetch force-marker text to display force in Newton!");
        
        server->applyChanges();
        
        //the parameter server only needs the data provided in the parameters of the function, as they are enough to reproduce the state!
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + arrow_name + "/x", arrow_value.x  * params->newton_per_meter);
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + arrow_name + "/y", arrow_value.y  * params->newton_per_meter);
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + arrow_name + "/z", arrow_value.z  * params->newton_per_meter);
    }
    
}

std::function<void ( const InteractiveMarkerFeedbackConstPtr & ) > RobotrainerEditorForce::returnMoveBoxArrow()
{
    return [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->moveBoxArrow ( feedback );
    };
}

void RobotrainerEditorForce::setDoSomething()
{
    for ( int i = 0; i < force_names.size(); i++ ) {
        InteractiveMarker inter_marker;
        std::string force_margin_name = force_names[i] + "/" + params->margin_ns;
        if ( ! ( server->get ( force_margin_name, inter_marker ) ) ) {
            ROS_ERROR ( "A force_cube element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_force.cpp)" );
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

        server->insert ( inter_marker );
    }
}

void RobotrainerEditorForce::setDoNothing()
{
    for ( int i = 0; i < force_names.size(); i++ ) {
        InteractiveMarker inter_marker;
        std::string force_margin_name = force_names[i]+ "/" + params->margin_ns;
        if ( ! ( server->get ( force_margin_name, inter_marker ) ) ) {
            ROS_ERROR ( "A force_cube element was not found in the interactive marker server! (setDoNothing(), robotrainer_editor_force.cpp)" );
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        server->insert ( inter_marker );
    }
    create_when_move = false;
}

//check if Display File Name from loaded File matches the one in the attribute
bool RobotrainerEditorForce::checkDisplayFileName ( std::string filename )
{

    bool val = true;

    //run a bash function to load the file into the parameter server
    std::ostringstream sstr;
    sstr << "rosparam load " << filename << " /temp/";
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system ( chr );

    std::string display_path_file_name_;

    ros::param::get ( "/temp/" + params->config_ns + "/display_path_file_name", display_path_file_name_ );
    
    ROS_ERROR("%s", display_path_file_name.c_str());
    ROS_ERROR("%s", display_path_file_name_.c_str());

    if ( display_path_file_name != display_path_file_name_ ) {
        //check if filename would match without path, in case the path was removed in the file or this instance
        /*
        std::string display_path_file_name2 = display_path_file_name.substr(display_path_file_name.find_last_of("\\/") + 1,display_path_file_name.size());
        display_path_file_name_ = display_path_file_name_.substr(display_path_file_name_.find_last_of("\\/") + 1,display_path_file_name_.size());
        if (display_path_file_name2 != display_path_file_name_)*/
        val = false;
    }

    ros::param::del ( "/temp" );

    return val;
}

// load path config of selected yaml-file
void RobotrainerEditorForce::loadFile ( std::string filename ){

    //run a bash function to load the file into the parameter server
    std::ostringstream sstr;
    //sstr << "bash -i -c 'rosparam load " << filename << "'";
    sstr << "rosparam load " << filename << " /" << params->editor_ns << "/" << params->force_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system ( chr );

    file_name = filename;

    loadFile();
}
    
// load path config of selected yaml-file
void RobotrainerEditorForce::loadFile(){    

    //display path filename was set when object was created, now check if it fits the one of the loaded file

    std::string display_path_file_name_;

    ros::param::get (params->editor_ns + "/" +  params->force_ns + "/" + params->config_ns +  "/display_path_file_name", display_path_file_name_ );

    if ( display_path_file_name != display_path_file_name_ ) ROS_WARN ( "loaded force file doesnt seem to be made for the path file" );

    double newton_per_meter_;
    
    ros::param::get (params->editor_ns + "/" +  params->force_ns + "/" + params->config_ns +  "/newton_per_meter", newton_per_meter_ );
    
    if ( params->newton_per_meter != newton_per_meter_) {
      ros::param::set(params->editor_ns + "/" +  params->force_ns + "/" + params->config_ns +  "/newton_per_meter", params->newton_per_meter );
      ROS_WARN ("loaded force file uses different newton_per_meter value than rosparam-handler default! adapted to rosparam-handler default on param server!");
    }
    
    //create a topic to publish force arrows on
    vector<string> topics;
    topics.push_back ( params->force_arrow_topic );

    //load the list of arrow names
    ros::param::get (params->editor_ns +"/" + params->force_ns + "/" + params->config_ns + "/" + params->force_ns + "_names", force_names );

    //retrieve all data based on the arrow names (all links to any data can be derived from the arrow name)
    for ( int i = 0; i < force_names.size(); i++ ) {

        std::string arrow_name = force_names[i] + "/" + params->arrow_ns;

        //retrieve arrow data
        Point end; //teknically not the end anymore, but a relative vector
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + arrow_name + "/x", end.x );
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + arrow_name + "/y", end.y );
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + arrow_name + "/z", end.z );
        end.x /= params->newton_per_meter;
        end.y /= params->newton_per_meter;
        end.z /= params->newton_per_meter;
        //arrow_map.insert(std::make_pair(arrow_name, end)); shall NOT happen here because it happens in createForce

        //retrieve force data
        std::string area_name = force_names[i] + "/" + params->area_ns;
        Point pos;
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + area_name + "/x", pos.x );
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + area_name + "/y", pos.y );
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + area_name + "/z", pos.z );
        //force_point_map.insert(std::make_pair(area_name, pos)); shall NOT happen here because it happens in createForce

        //NOTE cube_point_map/margin_map naming can be confusing: one is dedicated to the arrow, the other to the circle-force-calibration cube

        //retrieve force_cube data
        std::string margin_name = force_names[i] + "/" + params->margin_ns;
        Point cube_pos;
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + margin_name + "/x", cube_pos.x );
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + margin_name + "/y", cube_pos.y );
        ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + margin_name + "/z", cube_pos.z );
        //margin_map.insert(std::make_pair(margin_name, cube_pos)); shall NOT happen here because it happens in createForce
        
        if (!ros::param::get(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + arrow_name + "/" + params-> force_distance_function_ns, force_distance_function[arrow_name])) force_distance_function[arrow_name] = params->force_distance_functions[0];
        
        //create actual elements for use
        createArrow ( pos, pointAdd(pos, end), arrow_name, force_distance_function[arrow_name]);
        createForce ( arrow_name, pos, cube_pos );
    }
}


/**
 * In case the Parameter-Server has been overloaded with different data (or in case there is doubt that the Parameter-Server and object data are in sync).
 */
void RobotrainerEditorForce::reloadSession()
{
    //call to bash to delete all data of this namespace
    ros::param::del("/" + params->editor_ns + "/" + params->force_ns );
    
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->config_ns + "/" +  params->force_ns + "_names", force_names );
    
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/display_path_file_name", display_path_file_name );
    
    //reupload all data the parameter server based on the arrow names (all links to any data can be derived from the arrow name)
    for ( int i = 0; i < force_names.size(); i++ ) {

        std::string arrow_name = force_names[i] + "/" + params->arrow_ns;

        //retrieve arrow data
        Point end = arrow_map[arrow_name];  //teknically not the end anymore, but a relative vector
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + arrow_name + "/x", end.x );
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + arrow_name + "/y", end.y );
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + arrow_name + "/z", end.z );

        //retrieve force data
        std::string area_name = force_names[i] + "/" + params->area_ns;
        Point pos = force_point_map[area_name];
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + area_name + "/x", pos.x );
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + area_name + "/y", pos.y );
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + area_name + "/z", pos.z );

        //retrieve force_cube data
        std::string margin_name = force_names[i] + "/" +  params->margin_ns;
        Point cube_pos = margin_map[margin_name];
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + margin_name + "/x", cube_pos.x * params->newton_per_meter);
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + margin_name + "/y", cube_pos.y * params->newton_per_meter);
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/"  + margin_name + "/z", cube_pos.z * params->newton_per_meter);
        
        ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns + "/" + arrow_name + "/" + params-> force_distance_function_ns, force_distance_function[arrow_name]);

        //create actual elements for use
        createArrow ( pos, pointAdd(pos, end), arrow_name, force_distance_function[arrow_name]);
        createForce ( arrow_name, pos, cube_pos);
    }
}

// save current path config to file and update menu entries
void RobotrainerEditorForce::saveFile ( std::string filename )
{
    saveFileTool(filename, params->editor_ns + "/" + params->force_ns);
}

void RobotrainerEditorForce::setForceDistanceFunction(const InteractiveMarkerFeedbackConstPtr &feedback) {
  
    std::string name;
    std::string func_name;
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        name = feedback->marker_name;
        int menu_entry_id = feedback->menu_entry_id;
        func_name = params->force_distance_functions[menu_entry_id - 2]; //cannot pass a string into this function, so this is the only way to retrieve the name of the selected functions // -1 because of "delete" menu enty, -2 because of "array started 0"
    }
    else return;

    name = name.substr(0, name.rfind ( "/" ) +1) + params->arrow_ns;
    
    force_distance_function[name] = func_name;

    updateModalities(name);
}

void RobotrainerEditorForce::updateModalities(std::string name) {
  
    ros::param::set(params->editor_ns + "/" + params->force_ns + "/" + params->data_ns +  "/" + name + "/" + params->force_distance_function_ns, force_distance_function[name]);
    
    setupMenuHandler(name);
    //std::system("rosservice call /base/configure_modalities"); not anymore, only when "set active" in panel happens!
}

void RobotrainerEditorForce::setupMenuHandler(std::string name)
{
  
    setupArrowMenuHandler(name);
    setupForceMenuHandler(name);
    
    server->applyChanges();
}

void RobotrainerEditorForce::setupArrowMenuHandler(std::string name)
{
  
    arrow_menu_handler[name] = new interactive_markers::MenuHandler;
    arrow_menu_handler[name]->insert ( "delete arrow", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        //this->deleteSelectedArrow ( feedback );
    });
    
    std::string curr_func_name = force_distance_function[name];
    for(std::string func_name : params->force_distance_functions){
        bool is_this = (curr_func_name == func_name);
        std::ostringstream oss;
        oss << func_name << " " << is_this;
        arrow_menu_handler[name]->insert ( oss.str(), [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
            this->setForceDistanceFunction (feedback);
        });
    }
    
    arrow_menu_handler[name]->apply(*server, name);
}

void RobotrainerEditorForce::setupForceMenuHandler(std::string name)
{
  
    force_menu_handler[name] = new interactive_markers::MenuHandler;
    force_menu_handler[name]->insert ( "delete arrow", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->deleteSelectedArrow ( feedback );
    });
    
    std::string curr_func_name = force_distance_function[name];
    for(std::string func_name : params->force_distance_functions){
        bool is_this = (curr_func_name == func_name);
        std::ostringstream oss;
        oss << func_name << " " << is_this;
        force_menu_handler[name]->insert ( oss.str(), [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
            this->setForceDistanceFunction (feedback);
        });
    }
    
    std::string force_name = name.substr(0, name.rfind ( "/" ) +1) + params->area_ns;
    
    force_menu_handler[name]->apply(*server, force_name);
}
