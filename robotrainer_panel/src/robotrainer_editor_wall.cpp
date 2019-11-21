#include <robotrainer_panel/robotrainer_editor_wall.h>

RobotrainerEditorWall::RobotrainerEditorWall() {}

RobotrainerEditorWall::RobotrainerEditorWall(robotrainer_panel::robotrainer_editorParameters* params_,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_,
        ros::Publisher* wall_publisher_) {

    params = params_;
    server = server_;
    wall_publisher = wall_publisher_;
}

int RobotrainerEditorWall::getWallCount() {
    return wall_names.size();
}

int RobotrainerEditorWall::getCount() {
    return getWallCount();
}

std::string RobotrainerEditorWall::getNs() {
    return params->wall_ns;
}

//this is important to laod files without infinetely readding wall names
void RobotrainerEditorWall::createWallWithoutAddingName(Point base, Point end, std::string name, std::string force_distance_function_index) {

    force_distance_function[name] = force_distance_function_index;
    setupMenuHandler(name);
  
    //define length of wall (might not be necessary anymore)
    double length = sqrt(pow(end.x-base.x, 2) + pow(end.y-base.y, 2));

    //create the visual object for the wall
    Marker wall_marker;
    wall_marker.type = Marker::ARROW;//LINE_STRIP; //LINE STRIP not supported on RVIZ and NIVIDIA at the moment, see https://github.com/ros-visualization/rviz/issues/1192
    wall_marker.color.r = 1.0;
    wall_marker.color.g = 1.0;
    wall_marker.color.b = 0.0;
    wall_marker.color.a = 1.0;
    wall_marker.scale.x = 0.1 * params->base_scale;
    wall_marker.scale.y = 0.0001 * params->base_scale; //might be ignored, but shouldn't be malicious
    wall_marker.scale.z = 0.0001 * length;  //might be ignored, but shouldn't be malicious
    wall_marker.points.push_back(base);
    wall_marker.points.push_back(end);
    wall_marker.pose.orientation = normalizeQuaternion(wall_marker.pose.orientation);

    //define the way the wall behaves when manipulated
    InteractiveMarkerControl wall_control;
    wall_control.orientation.w = 1.0;
    wall_control.orientation.x = 0.0;
    wall_control.orientation.y = 1.0;
    wall_control.orientation.z = 0.0;
    wall_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    wall_control.always_visible = true;
    wall_control.markers.push_back(wall_marker);
    wall_control.orientation = normalizeQuaternion(wall_control.orientation);

    //create the interactive_marker for the wall (holder of visual and control_left element)
    InteractiveMarker wall_inter_marker;
    wall_inter_marker.header.frame_id = params->frame_id;
    wall_inter_marker.name = name;
    wall_inter_marker.controls.push_back(wall_control);

    //update the interactive_marker_server to new changes
    server->insert(wall_inter_marker);

    //areas have link to marker feedback function here, not necessary for wall because it cannot be moved

    //create the cube_lefts at the ends of the wall

    string cube_name_l =  name + "/" + params->cube_ns  + "L";

    // create dragable box (left)
    InteractiveMarker inter_marker_left;
    inter_marker_left.header.frame_id = params->frame_id;
    inter_marker_left.pose.position = base;
    inter_marker_left.scale = params->base_scale;
    inter_marker_left.name = cube_name_l;

    InteractiveMarkerControl control_left;
    control_left.orientation.w = 1.0;
    control_left.orientation.x = 0.0;
    control_left.orientation.y = 1.0;
    control_left.orientation.z = 0.0;
    control_left.interaction_mode = InteractiveMarkerControl::NONE;
    control_left.always_visible = true;

    Marker cube_left;
    cube_left.type = Marker::CUBE;
    cube_left.scale.x = cube_left.scale.y = cube_left.scale.z = inter_marker_left.scale * 0.1;
    cube_left.color.r = cube_left.color.g = 1.0;
    cube_left.color.b = 0.0;
    cube_left.color.a = 1.0;
    cube_left.pose.orientation = normalizeQuaternion(cube_left.pose.orientation);
    control_left.markers.push_back(cube_left);
    control_left.orientation = normalizeQuaternion(control_left.orientation);
    inter_marker_left.controls.push_back(control_left);

    //insert the cube_left-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
    //the lambda function do_this is required even for functions within this class because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    std::function<void(const InteractiveMarkerFeedbackConstPtr &)> server_function_l = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->moveWallLeft(feedback);
    };
    server->insert(inter_marker_left, server_function_l);

    string cube_name_r = name + "/" + params->cube_ns  + "R";

    // create dragable box (right)
    InteractiveMarker inter_marker_right;
    inter_marker_right.header.frame_id = params->frame_id;
    inter_marker_right.pose.position = end;
    inter_marker_right.scale = params->base_scale;
    inter_marker_right.name = cube_name_r;

    InteractiveMarkerControl control_right;
    control_right.orientation.w = 1.0;
    control_right.orientation.x = 0.0;
    control_right.orientation.y = 1.0;
    control_right.orientation.z = 0.0;
    control_right.interaction_mode = InteractiveMarkerControl::NONE;
    control_right.always_visible = true;

    Marker cube_right;
    cube_right.type = Marker::CUBE;
    cube_right.scale.x = cube_right.scale.y = cube_right.scale.z = inter_marker_right.scale * 0.1;
    cube_right.color.r = cube_right.color.g = 1.0;
    cube_right.color.b = 0.0;
    cube_right.color.a = 1.0;
    cube_right.pose.orientation = normalizeQuaternion(cube_right.pose.orientation);
    control_right.markers.push_back(cube_right);
    control_right.orientation = normalizeQuaternion(control_right.orientation);
    inter_marker_right.controls.push_back(control_right);

    //insert the cube_right-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
    //the lambda function do_this is required even for functions within this class because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
    //for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
    std::function<void(const InteractiveMarkerFeedbackConstPtr &)> server_function_r = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
        this->moveWallRight(feedback);
    };
    server->insert(inter_marker_right, server_function_r);

    server->applyChanges();

    wall_map[name] =  std::array<geometry_msgs::Point, 2> {base,end};
    aoe_map[name] = wall_marker.pose.position;
    aoe_map[name].y += 0.5;
    updateAreaOfEffect(name);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + name + "/L/x", base.x);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + name + "/L/y", base.y);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + name + "/L/z", base.z);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + name + "/R/x", end.x);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + name + "/R/y", end.y);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + name + "/R/z", end.z);
    setDoNothing();

    updateModalities(name);
}

void RobotrainerEditorWall::createWall(Point base, Point end, std::string name) {
  
    createWallWithoutAddingName(base, end, name, params->force_distance_functions[0]);

    wall_names.push_back(name);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->config_ns + "/" + params->wall_ns + "_names", wall_names);

}

//*BUG if the "area of effect" cube isn't set up in two steps, it will allways have an offset between its displayed and its theoretical position
void RobotrainerEditorWall::updateAreaOfEffect(std::string wall_name) {

    Marker wall_marker;
    InteractiveMarker wall_inter_marker;
    if (!server->get(wall_name, wall_inter_marker)) {
        ROS_ERROR("Interactive marker server doesn't know any Wall with this name!");
        return;
    }
    wall_marker = wall_inter_marker.controls[0].markers[0];
    Point base = wall_map[wall_name][0];
    Point end = wall_map[wall_name][1];

    //double aoe = distanceFromLine(base, end, aoe_map[wall_name]); //behaves orthogonal for some reason
    double aoe = vectorLength(pointDiff(pointCenter(base, end), aoe_map[wall_name]));

    InteractiveMarker aoe_cube_inter_marker;
    std::string cube_name = wall_name + "/" + params->area_ns + "/" + params->cube_ns;

    if (!server->get(cube_name, aoe_cube_inter_marker)) {
        //create the visual object for the wall-area of effect modification cube
        Marker aoe_cube_marker;
        aoe_cube_marker.type = Marker::CUBE;
        aoe_cube_marker.color.r = 1.0;
        aoe_cube_marker.color.g = 1.0;
        aoe_cube_marker.color.b = 0.0;
        aoe_cube_marker.color.a = 1.0;
        aoe_cube_marker.scale.x = aoe_cube_marker.scale.y = aoe_cube_marker.scale.z = params->base_scale * 0.1;
        //aoe_cube_marker.pose.position = aoe_map[wall_name]; //do this later to prevent BUG*

        //define the way the aoe behaves when manipulated
        InteractiveMarkerControl aoe_cube_control;
        aoe_cube_control.orientation.w = 1.0;
        aoe_cube_control.orientation.x = 0.0;
        aoe_cube_control.orientation.y = 1.0;
        aoe_cube_control.orientation.z = 0.0;
        aoe_cube_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        aoe_cube_control.always_visible = true;
        aoe_cube_control.markers.push_back(aoe_cube_marker);
        aoe_cube_control.orientation = normalizeQuaternion(aoe_cube_control.orientation);

        //create the interactive_marker for the aoe
        aoe_cube_inter_marker.header.frame_id = params->frame_id;
        aoe_cube_inter_marker.name = cube_name;
        aoe_cube_inter_marker.controls.push_back(aoe_cube_control);

        //insert the aoe_cube_inter_marker-marker into the interactive marker server, assosiated with the function that shall be called when you interact with (move) it!
        //the lambda function do_this is required even for functions within this class because the compiler adds an additional parameter "this" to all functions that are member of the object, but the server->insert function expects specificly a function that has a single parameter of a certain type.
        //for more detail see https://stackoverflow.com/questions/45103622/refer-to-a-function-that-is-member-of-a-class/45107147#45107147
        std::function<void(const InteractiveMarkerFeedbackConstPtr &)> server_function_area = [this] (const InteractiveMarkerFeedbackConstPtr &feedback) {
            this->scaleAreaOfEffect(feedback);
        };
        server->insert(aoe_cube_inter_marker, server_function_area);

        server->applyChanges(); //apply changes early, then perform new changes and apply them, to prevent BUG*
    } //else { //do this anyway, because we need to two-step create the point to prevent a bug
    Pose pose;
    pose.position = aoe_map[wall_name]; //the cube_point_map holds the original location of the cube, on the path
    server->setPose ( cube_name, pose );
    //}

    InteractiveMarker aoe_inter_marker;
    std::string aoe_name = wall_name + "/" + params->area_ns;

    if (!server->get(aoe_name, aoe_inter_marker)) {
        //create the visual object for the wall-area of effect
        Marker aoe_marker;
        aoe_marker.type = Marker::ARROW;//LINE_LIST;
        aoe_marker.color.r = 1.0;
        aoe_marker.color.g = 1.0;
        aoe_marker.color.b = 0.0;
        aoe_marker.color.a = 0.3;
        aoe_marker.scale.x = aoe;
        aoe_marker.scale.y = 0.001;
        aoe_marker.scale.z = 0.001;
        aoe_marker.points.push_back(base);
        aoe_marker.points.push_back(end);
        aoe_marker.pose.orientation = normalizeQuaternion(aoe_marker.pose.orientation);

        //define the way the aoe behaves when manipulated
        InteractiveMarkerControl aoe_control;
        aoe_control.orientation = wall_inter_marker.controls[0].orientation;
        aoe_control.interaction_mode = InteractiveMarkerControl::BUTTON;
        aoe_control.always_visible = true;
        aoe_control.markers.push_back(aoe_marker);
        aoe_control.orientation = normalizeQuaternion(aoe_control.orientation);

        //create the interactive_marker for the aoe
        aoe_inter_marker.header.frame_id = params->frame_id;
        aoe_inter_marker.name = aoe_name;
        aoe_inter_marker.controls.push_back(aoe_control);

    } else {
        aoe_inter_marker.controls[0].markers[0].scale.x = aoe;
        aoe_inter_marker.controls[0].markers[0].points[0] = base;
        aoe_inter_marker.controls[0].markers[0].points[1] = end;
    }

    //update the interactive_marker_server
    server->insert(aoe_inter_marker);

    InteractiveMarker aoe_inter_marker_l;
    std::string aoe_name_l = wall_name + "/" + params->area_ns + "_L";

    if (!server->get(aoe_name_l, aoe_inter_marker_l)) {
        //create the visual object for the wall-area of effect - left end
        Marker aoe_marker_l;
        aoe_marker_l.type = Marker::CYLINDER;
        aoe_marker_l.color.r = 1.0;
        aoe_marker_l.color.g = 1.0;
        aoe_marker_l.color.b = 0.0;
        aoe_marker_l.color.a = 0.3;
        aoe_marker_l.pose.position = base;
        aoe_marker_l.scale.x = aoe_marker_l.scale.y = aoe;
        aoe_marker_l.scale.z = 0.05 * params->base_scale;
        aoe_marker_l.pose.orientation = normalizeQuaternion(aoe_marker_l.pose.orientation);

        //define the way the aoe behaves when manipulated
        InteractiveMarkerControl aoe_control_l;
        aoe_control_l.orientation.w = 1.0;
        aoe_control_l.orientation.x = 0.0;
        aoe_control_l.orientation.y = 1.0;
        aoe_control_l.orientation.z = 0.0;
        aoe_control_l.interaction_mode = InteractiveMarkerControl::NONE;
        aoe_control_l.always_visible = true;
        aoe_control_l.markers.push_back(aoe_marker_l);
        aoe_control_l.orientation = normalizeQuaternion(aoe_control_l.orientation);

        //create the interactive_marker for the aoe
        aoe_inter_marker_l.header.frame_id = params->frame_id;
        aoe_inter_marker_l.name = aoe_name_l;
        aoe_inter_marker_l.controls.push_back(aoe_control_l);

    } else {
        aoe_inter_marker_l.controls[0].markers[0].scale.x = aoe_inter_marker_l.controls[0].markers[0].scale.y = aoe;
        aoe_inter_marker_l.controls[0].markers[0].pose.position = base;
    }

    //update the interactive_marker_server
    server->insert(aoe_inter_marker_l);

    InteractiveMarker aoe_inter_marker_r;
    std::string aoe_name_r = wall_name + "/" + params->area_ns + "_R";

    if (!server->get(aoe_name_r, aoe_inter_marker_r)) {
        //create the visual object for the wall-area of effect - left end
        Marker aoe_marker_r;
        aoe_marker_r.type = Marker::CYLINDER;
        aoe_marker_r.color.r = 1.0;
        aoe_marker_r.color.g = 1.0;
        aoe_marker_r.color.b = 0.0;
        aoe_marker_r.color.a = 0.3;
        aoe_marker_r.pose.position = end;
        aoe_marker_r.scale.x = aoe_marker_r.scale.y = aoe;
        aoe_marker_r.scale.z = 0.05 * params->base_scale;
        aoe_marker_r.pose.orientation = normalizeQuaternion(aoe_marker_r.pose.orientation);

        //define the way the aoe behaves when manipulated
        InteractiveMarkerControl aoe_control_r;
        aoe_control_r.orientation.w = 1.0;
        aoe_control_r.orientation.x = 0.0;
        aoe_control_r.orientation.y = 1.0;
        aoe_control_r.orientation.z = 0.0;
        aoe_control_r.interaction_mode = InteractiveMarkerControl::NONE;
        aoe_control_r.always_visible = true;
        aoe_control_r.markers.push_back(aoe_marker_r);
        aoe_control_r.orientation = normalizeQuaternion(aoe_control_r.orientation);

        //create the interactive_marker for the aoe
        aoe_inter_marker_r.header.frame_id = params->frame_id;
        aoe_inter_marker_r.name = aoe_name_r;
        aoe_inter_marker_r.controls.push_back(aoe_control_r);

    } else {
        aoe_inter_marker_r.controls[0].markers[0].scale.x = aoe_inter_marker_r.controls[0].markers[0].scale.y = aoe;
        aoe_inter_marker_r.controls[0].markers[0].pose.position = end;
    }

    //update the interactive_marker_server
    server->insert(aoe_inter_marker_r);

    server->applyChanges();

    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/x", aoe_map[wall_name].x);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/y", aoe_map[wall_name].y);
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/z", aoe_map[wall_name].z);

    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->area_ns, aoe);

}

void RobotrainerEditorWall::scaleAreaOfEffect(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type != InteractiveMarkerFeedback::POSE_UPDATE) return;

    string aoe_name = feedback->marker_name;
    string wall_name = aoe_name.substr(0, aoe_name.rfind("/"));
    wall_name = wall_name.substr(0, wall_name.rfind("/"));
    aoe_map[wall_name] = feedback->pose.position;

    updateAreaOfEffect(wall_name);
}

void RobotrainerEditorWall::moveWall(const InteractiveMarkerFeedbackConstPtr &feedback, bool is_left) {

    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {

        //read the names of the marker involved, this will be needed to identify the objects to change
        string cube_name = feedback->marker_name;

        //identify the wall by matching the names and namespaces its name depended on at creation
        string wall_name = cube_name.substr(0, cube_name.rfind("/"));

        Point end_pos;

        Point base_pos;

        if (is_left) {
            base_pos  = feedback->pose.position;
            end_pos = wall_map[wall_name][1];
        }
        else {
            base_pos = wall_map[wall_name][0];
            end_pos = feedback->pose.position;
        }

        //fetch teh wall marker form the interactive marker server
        InteractiveMarker wall_int_marker;
        if(!server->get(wall_name, wall_int_marker)) ROS_ERROR("couldn't fetch wall_int_marker from server!");

        //move cubes to correct position
        wall_int_marker.controls[0].markers[0].points[0] = base_pos;
        wall_int_marker.controls[0].markers[0].points[1] = end_pos;
        wall_int_marker.controls[0].orientation = normalizeQuaternion(wall_int_marker.controls[0].orientation);

        //update wall marker in interactive marker server
        server->insert(wall_int_marker);

        server->applyChanges();

        wall_publisher->publish(wall_int_marker);

        //Point aoe_diff = is_left ? pointDiff(wall_map[wall_name][0], base_pos) : pointDiff(wall_map[wall_name][1], end_pos);
        Point aoe_diff = pointDiff(pointCenter(wall_map[wall_name][0], wall_map[wall_name][1]), pointCenter(base_pos, end_pos));
        aoe_map[wall_name] = pointAdd(aoe_map[wall_name], aoe_diff);
        wall_map[wall_name] = std::array<geometry_msgs::Point, 2> {base_pos,end_pos};
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/x", base_pos.x);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/y", base_pos.y);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/z", base_pos.z);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/x", end_pos.x);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/y", end_pos.y);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/z", end_pos.z);

        updateAreaOfEffect(wall_name);
    }
}

void RobotrainerEditorWall::moveWallLeft(const InteractiveMarkerFeedbackConstPtr &feedback) {
    moveWall(feedback, true);
}

void RobotrainerEditorWall::moveWallRight(const InteractiveMarkerFeedbackConstPtr &feedback) {
    moveWall(feedback, false);
}

void RobotrainerEditorWall::setDoSomething() {

    for(int i = 0; i < wall_names.size(); i++) {
        InteractiveMarker inter_marker;
        string cube_name_l = wall_names[i] + "/" + params->cube_ns  + "L";
        if (!(server->get(cube_name_l, inter_marker))) {
            ROS_ERROR("An cube_l element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_wall.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

        inter_marker.controls[0].orientation = normalizeQuaternion(inter_marker.controls[0].orientation);

        server->insert(inter_marker);
    }

    for(int i = 0; i < wall_names.size(); i++) {
        InteractiveMarker inter_marker;
        string cube_name_r = wall_names[i] + "/" + params->cube_ns  + "R";
        if (!(server->get(cube_name_r, inter_marker))) {
            ROS_ERROR("An cube_r element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_wall.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

        inter_marker.controls[0].orientation = normalizeQuaternion(inter_marker.controls[0].orientation);

        server->insert(inter_marker);
    }

    for(int i = 0; i < wall_names.size(); i++) {
        InteractiveMarker inter_marker;
        string aoe_cube_name = wall_names[i] + "/" + params->area_ns + "/" + params->cube_ns;
        if (!(server->get(aoe_cube_name, inter_marker))) {
            ROS_ERROR("An aoe_cube element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_wall.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;//MOVE_AXIS;

        inter_marker.controls[0].orientation = normalizeQuaternion(inter_marker.controls[0].orientation);

        server->insert(inter_marker);
    }

}

void RobotrainerEditorWall::setDoNothing() {

    for(int i = 0; i < wall_names.size(); i++) {
        InteractiveMarker inter_marker;
        string cube_name_l = wall_names[i] + "/" + params->cube_ns  + "L";
        if (!(server->get(cube_name_l, inter_marker))) {
            ROS_ERROR("An cube_l element was not found in the interactive marker server! (setDoNothing(), robotrainer_editor_wall.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        server->insert(inter_marker);
    }

    for(int i = 0; i < wall_names.size(); i++) {
        InteractiveMarker inter_marker;
        string cube_name_r = wall_names[i] + "/" + params->cube_ns  + "R";
        if (!(server->get(cube_name_r, inter_marker))) {
            ROS_ERROR("An cube_r element was not found in the interactive marker server! (setDoNothing(), robotrainer_editor_wall.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        server->insert(inter_marker);
    }

    for(int i = 0; i < wall_names.size(); i++) {
        InteractiveMarker inter_marker;
        string aoe_cube_name = wall_names[i] + "/" + params->area_ns + "/" + params->cube_ns;
        if (!(server->get(aoe_cube_name, inter_marker))) {
            ROS_ERROR("An aoe_cube element was not found in the interactive marker server! (setDoSomething(), robotrainer_editor_wall.cpp)");
            continue;
        }
        inter_marker.controls[0].interaction_mode = InteractiveMarkerControl::NONE;

        inter_marker.controls[0].orientation = normalizeQuaternion(inter_marker.controls[0].orientation);

        server->insert(inter_marker);
    }

}

void RobotrainerEditorWall::loadFile(std::string filename) {
    //run a bash function to load the file into the parameter server
    std::ostringstream sstr;
    sstr << "rosparam load " << filename << " /" << params->editor_ns << "/" << params->wall_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);

    file_name = filename;

    loadFile();
}

void RobotrainerEditorWall::loadFile() {

    //create a topic to publish force walls on
    vector<string> topics;
    topics.push_back(params->wall_topic);

    //load the list of wall names
    ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->config_ns + "/" + params->wall_ns + "_names", wall_names);

    //retrieve all data based on the wall names (all links to any data can be derived from the wall name)
    for(int i = 0; i < wall_names.size(); i++) {

        std::string wall_name = wall_names[i];

        //retrieve wall data
        Point base;
        Point end;
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/x", base.x);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/y", base.y);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/z", base.z);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/x", end.x);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/y", end.y);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/z", end.z);
        //wall_map.insert(std::make_pair(wall_name, std::make_tuple(base,end))); WARNING APPERENTLY happens in createForce

        Point aoe;

        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/x", aoe.x);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/y", aoe.y);
        ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/z", aoe.z);
        
        if (!ros::param::get(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params-> force_distance_function_ns, force_distance_function[wall_name])) force_distance_function[wall_name] = params->force_distance_functions[0];
        
        createWallWithoutAddingName(base, end, wall_name, force_distance_function[wall_name]);

        aoe_map[wall_name] = aoe;
        updateAreaOfEffect(wall_name);
    }

}

void RobotrainerEditorWall::reloadSession() {

    ros::param::del("/" + params->editor_ns + "/" + params->wall_ns);

    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->config_ns + "/" + params->wall_ns + "_names", wall_names);

//reupload all data the parameter server based on the wall names
    for(int i = 0; i < wall_names.size(); i++) {

        std::string wall_name = wall_names[i];

        //retrieve wall data
        Point base = (wall_map[wall_name])[0];
        Point end = (wall_map[wall_name])[1];
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/x", base.x);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/y", base.y);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/L/z", base.z);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/x", end.x);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/y", end.y);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/R/z", end.z);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params-> force_distance_function_ns, force_distance_function[wall_name]);
        
        Point aoe = aoe_map[wall_name];

        createWallWithoutAddingName(base, end, wall_name, force_distance_function[wall_name]);

        aoe_map[wall_name] = aoe;

        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/x", aoe_map[wall_name].x);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/y", aoe_map[wall_name].y);
        ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name + "/" + params->area_ns + "/" + params->cube_ns + "/z", aoe_map[wall_name].z);

        updateAreaOfEffect(wall_name);
    }

}

void RobotrainerEditorWall::saveFile(std::string filename) {
    saveFileTool(filename, params->editor_ns + "/" + params->wall_ns);
}

void RobotrainerEditorWall::setForceDistanceFunction(const InteractiveMarkerFeedbackConstPtr &feedback) {
  
    std::string name;
    std::string func_name;
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        name = feedback->marker_name;
        int menu_entry_id = feedback->menu_entry_id;
        func_name = params->force_distance_functions[menu_entry_id - 2]; //cannot pass a string into this function, so this is the only way to retrieve the name of the selected functions // -1 because of "delete" menu enty, -2 because of "array started 0"
    }
    else return;

    name = name.substr(0, name.rfind ( "/" ) );

    force_distance_function[name] = func_name;

    updateModalities(name);
}

void RobotrainerEditorWall::updateModalities(std::string name) {
  
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns +  "/" + name + "/" + params->force_distance_function_ns, force_distance_function[name]);
    
    setupMenuHandler(name);
    //std::system("rosservice call /base/configure_modalities"); not anymore, only when "set active" in panel happens!
}

void RobotrainerEditorWall::setupMenuHandler(std::string name)
{
  
    setupWallMenuHandler(name);
    setupAreaMenuHandler(name);
    
    server->applyChanges();
}

void RobotrainerEditorWall::setupWallMenuHandler(std::string name)
{
  
    wall_menu_handler[name] = new interactive_markers::MenuHandler;
    wall_menu_handler[name]->insert ( "delete wall", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        //this->deleteSelectedWall ( feedback );
    });
    
    std::string curr_func_name = force_distance_function[name];
    for(std::string func_name : params->force_distance_functions){
        bool is_this = (curr_func_name == func_name);
        std::ostringstream oss;
        oss << func_name << " " << is_this;
        wall_menu_handler[name]->insert ( oss.str(), [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
            //this->setForceDistanceFunction ( feedback, str);
        });
    }
    
    wall_menu_handler[name]->apply(*server, name);
}

void RobotrainerEditorWall::setupAreaMenuHandler(std::string name)
{
  
    area_menu_handler[name] = new interactive_markers::MenuHandler;
    area_menu_handler[name]->insert ( "delete wall", [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
        this->deleteSelectedWall ( feedback );
    });
    
    std::string curr_func_name = force_distance_function[name];
    for(std::string func_name : params->force_distance_functions){
        bool is_this = (curr_func_name == func_name);
        std::ostringstream oss;
        oss << func_name << " " << is_this;
        area_menu_handler[name]->insert ( oss.str(), [this] ( const InteractiveMarkerFeedbackConstPtr &feedback ) {
            this->setForceDistanceFunction ( feedback);
        });
    }
    
    string area_name = name + "/" + params->area_ns;
    string area_name_l = name + "_L";
    string area_name_r = name + "_R";
    
    area_menu_handler[name]->apply(*server, area_name);
    area_menu_handler[name]->apply(*server, area_name_l);
    area_menu_handler[name]->apply(*server, area_name_r);
}

void RobotrainerEditorWall::deleteWallFromServer(std::string wall_name) {

    InteractiveMarker wall_int_marker;
    //if the wall can be retrieved from the server, clear it's controls so it can't be adressed anymore and publish that "unusable" wall
    if (server->get(wall_name, wall_int_marker)) {
        wall_int_marker.controls.clear();
        wall_publisher->publish(wall_int_marker);
    }
    //erase the wall object from the interactive_marker_server
    server->erase(wall_name);

    string cube_name_l =  wall_name + "/" + params->cube_ns  + "L";
    string cube_name_r =  wall_name + "/" + params->cube_ns  + "R";

    //erase the cubes at the ends of the wall
    server->erase(cube_name_l);
    server->erase(cube_name_r);

    string area_name = wall_name + "/" + params->area_ns;
    string area_cube_name = area_name + "/" + params->cube_ns;
    string area_name_l = area_name + "_L";
    string area_name_r = area_name + "_R";

    server->erase(area_name);
    server->erase(area_cube_name);
    server->erase(area_name_l);
    server->erase(area_name_r);

    server->applyChanges();

}

void RobotrainerEditorWall::deleteWall(std::string wall_name) {

    deleteWallFromServer(wall_name);

    //wall_names.erase(wall_name); worked when "wall_names" was still a set, but needs to be vector for ros::param
    for(int i = 0; i < wall_names.size(); i++) {
        if(wall_names[i] == wall_name) wall_names.erase(wall_names.begin()+i);
        else if(i = wall_names.size() -1) std::system("Warning: wall_name could not be removed from wall_names as it has not been found!");
    }

    //update the list of wall names in the parameter server
    ros::param::set(params->editor_ns + "/" + params->wall_ns + "/" + params->config_ns + "/" + params->wall_ns + "_names", wall_names);
    //delete the wall-data from the parameter server
    ros::param::del(params->editor_ns + "/" + params->wall_ns + "/" + params->data_ns + "/" + wall_name);

}

// load walls of selected bag-file
void RobotrainerEditorWall::deleteSelectedWall(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT) {
        std::string wall_name = feedback->marker_name;
        wall_name = wall_name.substr(0, wall_name.rfind("/")); //for the case that the area of effect of the wall is clicked. also works for the wall itself (when there is no "/" in the name)
        deleteWall(wall_name);
    }
}

// resets all related data on the server (for session switching)
void RobotrainerEditorWall::resetServer() {
    for (auto &wall_name : wall_names) {
        deleteWallFromServer(wall_name);
    }
    //delete all data concerning this object from the parameter server
    ros::param::del(params->editor_ns + "/" + params->wall_ns);
    server->applyChanges();
}
