/**
 * An RViz plugin to edit RobotrainerEditor Projects with a nice UI.
 *
 * This File could no have been created without the inspiration from the "rviz_plugin_tutorial::teleop_panel".
 * Though all remaining matches are basic rviz/Qt contents, we'd like to give credit to it's creators:
 * http://docs.ros.org/lunar/api/rviz_plugin_tutorials/html/panel_plugin_tutorial.html
 */
#include <robotrainer_panel/robotrainer_editor_panel.h>

#include <cstdio>
#include <ctime>
#include <locale>

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std;
using namespace boost;

namespace robotrainer_editor_panel
{

std::string RobotrainerEditorPanel::getParamType(std::string s) {
    bool isInt = true;
    std::string::const_iterator it = s.begin();
    while (it != s.end()) {
        if(!std::isdigit(*it)) {
            isInt = false;
            if(*it != '.' && *it != 'e' && *it != '-' && *it != '+') {
                return "string";
            }
        }
        it++;
    }

    if (isInt) return "int";
    else return "double";
}

void RobotrainerEditorPanel::fillTable(QTableWidget** table, std::vector<std::array<std::string, 3>>* content) {

    *table = new QTableWidget;
    (*table)->setRowCount(content->size());
    (*table)->setColumnCount(3);

    for (int i = 0; i < content->size(); i++) {
        for(int j = 0; j < 3; j++) {
            (*table)->setItem(i, j, new QTableWidgetItem((*content)[i][j].c_str()));
            QTableWidgetItem* item = (*table)->item(i,j);
            item->setFlags(item->flags() &  ~Qt::ItemIsEditable);
        }
    }
    (*table)->resizeColumnsToContents();
    (*table)->setMinimumHeight((*table)->rowHeight(0)*((*table)->rowCount()+1));
    (*table)->setMaximumHeight((*table)->rowHeight(0)*((*table)->rowCount()+1));
}

void RobotrainerEditorPanel::setupTables() {

    std::vector<std::array<std::string, 3>> modifiers;
    std::vector<std::array<std::string, 3>> namespaces;
    std::vector<std::array<std::string, 3>> topcis;

    std::stringstream params_str_str;
    params_str_str << *params;
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(params_str_str, line, '\n'))
    {
        lines.push_back(line);
    }

    //first two lines are useless
    for(int i = 2; i < lines.size(); i++) {
        std::array<std::string, 3> arr;
        std::string value = lines[i].substr(lines[i].rfind ( ":" ) + 1, lines[i].size());
        arr[2] = value;
        arr[0] = getParamType(value);
        arr[1] = lines[i].substr(lines[i].rfind ( "/" ) + 1, lines[i].rfind ( ":" ) - lines[i].rfind ( "/" ) - 1); //second param is relative position, not absolute position
        if(arr[0] == "string") {
            if(arr[1].rfind("topic") == - 1) namespaces.push_back(arr);
            else topcis.push_back(arr);
        }
        else modifiers.push_back(arr);
    }

    fillTable(&modifier_table, &modifiers);
    fillTable(&ns_table, &namespaces);
    fillTable(&topic_table, &topcis);

}

void RobotrainerEditorPanel::createTableLayout(QTableWidget** table, std::string text) {
    //add label as title for the table
    QListWidgetItem* table_label = new QListWidgetItem;
    tables->addItem(table_label);
    tables->setItemWidget(table_label, new QLabel(text.c_str()));
    //create a QListWidgetItem with equal size, QListWidget also controlls scroll behaviour
    QListWidgetItem* table_item = new QListWidgetItem;
    table_item->setSizeHint(* new QSize((*table)->minimumWidth(), (*table)->minimumHeight()));
    tables->addItem(table_item);
    tables->setItemWidget(table_item, (*table));
    //new line
    QListWidgetItem* table_newline = new QListWidgetItem;
    tables->addItem(table_newline);
    tables->setItemWidget(table_newline, new QLabel( "" ));
}

QVBoxLayout* RobotrainerEditorPanel::createActionLayout(QPushButton** button, const char* slot, QString text) {
    QVBoxLayout* layout = new QVBoxLayout;
    *button = new QPushButton;
    QObject::connect(*button, SIGNAL (clicked()), slot);
    (*button)->setText(text);
    layout->addWidget(*button);
    return layout;
}


QVBoxLayout* RobotrainerEditorPanel::createSessionLayout(QComboBox** combo_box, const char* slot, QString text) {
    QVBoxLayout* layout = new QVBoxLayout;
    QLabel* label = new QLabel(text);
    *combo_box = new QComboBox;
    (*combo_box)->addItem("None/New","");
    QObject::connect(*combo_box, SIGNAL(activated(QString)), slot);
    layout->addWidget(label);
    layout->addWidget(*combo_box);
    return layout;
}

QVBoxLayout* RobotrainerEditorPanel::createLoadSaveLayout(const char* load_slot, const char* save_slot, QString text) {

    QVBoxLayout* layout = new QVBoxLayout;

    QLabel* label = new QLabel(text);

    QPushButton* load_button = new QPushButton;
    QObject::connect(load_button, SIGNAL (clicked()), load_slot);
    load_button->setText("Load "+text+" File");

    QPushButton* save_button = new QPushButton;
    QObject::connect(save_button, SIGNAL (clicked()), save_slot);
    save_button->setText("Save "+text+" File");

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);

    layout->addWidget(label);
    layout->addWidget(load_button);
    layout->addWidget(save_button);
    layout->addWidget(line);

    return layout;
}

//load/save data via service. meant to handle databse stuff in the future
QVBoxLayout* RobotrainerEditorPanel::createDataServiceLayout(const char* slot, QString text) {

    QVBoxLayout* layout = new QVBoxLayout;

    QLabel* label = new QLabel(text);

    QPushButton* button = new QPushButton;
    QObject::connect(button, SIGNAL (clicked()), slot);
    button->setText(text);

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);

    layout->addWidget(label);
    layout->addWidget(button);
    layout->addWidget(line);

    return layout;
}

void RobotrainerEditorPanel::setupLayout() {

//TabWidget and Layout
    tab_widget = new QTabWidget;
    tab_widget->addTab(new QWidget(), "Params");
    tab_widget->addTab(new QWidget(), "Actions");
    tab_widget->addTab(new QWidget(), "Session");
    tab_widget->addTab(new QWidget(), "Load/Save");
    tab_widget->addTab(new QWidget(), "Data Service");

    QHBoxLayout* tab_layout = new QHBoxLayout;
    tab_layout->addWidget(tab_widget);

//Params Layout
    QVBoxLayout* params_layout = new QVBoxLayout();

    tables = new QListWidget;

    setupTables();

    createTableLayout(&modifier_table, "Modifiers:");
    createTableLayout(&ns_table, "Namespaces:");
    createTableLayout(&topic_table, "Topics:");

    //bring it all together
    params_layout->addWidget(tables);
    tab_widget->widget(0)->setLayout(params_layout);


//Actions Layout
    QVBoxLayout* actions_layout = new QVBoxLayout;

    actions_layout->addLayout(createActionLayout(&toggle_mode_button, SLOT (setActive()), "Set Session to Active"));
    QFrame* line2 = new QFrame();
    line2->setFrameShape(QFrame::HLine);
    line2->setFrameShadow(QFrame::Sunken);
    actions_layout->addWidget(line2);
    actions_layout->addLayout(createActionLayout(&toggle_mode_button, SLOT (toggleMode()), "Toggle Record Mode"));
    QFrame* line3 = new QFrame();
    line3->setFrameShape(QFrame::HLine);
    line3->setFrameShadow(QFrame::Sunken);
    actions_layout->addWidget(line3);
    actions_layout->addLayout(createActionLayout(&editing_button, SLOT (enableEditing()), "Enable Editing"));
    actions_layout->addLayout(createActionLayout(&create_force_button, SLOT (enableArrowCreation()), "Create/Move Force Arrows"));
    actions_layout->addLayout(createActionLayout(&create_wall_button, SLOT (createWall()), "Create Wall"));
    actions_layout->addLayout(createActionLayout(&create_area_button, SLOT (createArea()), "Create Area"));
    actions_layout->addLayout(createActionLayout(&create_section_button, SLOT(enableSectionCreation()), "Enable Section Creation"));
    QFrame* line4 = new QFrame();
    line4->setFrameShape(QFrame::HLine);
    line4->setFrameShadow(QFrame::Sunken);
    actions_layout->addWidget(line4);
    actions_layout->addLayout(createActionLayout(&record_single_point_button, SLOT (recordSinglePoint()), "Record Single Point"));
    record_single_point_button->setEnabled(false);

    tab_widget->widget(1)->setLayout(actions_layout);

//Session Layout
    session_layout = new QVBoxLayout;
    session_layout->addLayout(createActionLayout(&session_from_active_button, SLOT (sessionFromActive()), "Load Session from Active Scenario"));
    QFrame* line5 = new QFrame();
    line5->setFrameShape(QFrame::HLine);
    line5->setFrameShadow(QFrame::Sunken);
    session_layout->addWidget(line5);
    session_layout->addLayout(createActionLayout(&clear_session_button, SLOT (clearSession()), "Clear Session"));
    QFrame* line6 = new QFrame();
    line6->setFrameShape(QFrame::HLine);
    line6->setFrameShadow(QFrame::Sunken);
    session_layout->addWidget(line6);
    session_layout->addLayout(createSessionLayout(&robotrainer_editor_display_path_sessions_combobox, SLOT(loadPathSession(QString)), "Path Session"));
    session_layout->addLayout(createSessionLayout(&robotrainer_editor_force_sessions_combobox, SLOT (loadForceSession(QString)), "Force Session"));
    session_layout->addLayout(createSessionLayout(&robotrainer_editor_wall_sessions_combobox, SLOT (loadWallSession(QString)), "Wall Session"));
    session_layout->addLayout(createSessionLayout(&robotrainer_editor_area_sessions_combobox, SLOT (loadAreaSession(QString)), "Area Session"));
    session_layout->addLayout(createSessionLayout(&robotrainer_editor_section_sessions_combobox, SLOT (loadSectionSession(QString)), "Section Session"));

    tab_widget->widget(2)->setLayout(session_layout);

//Save/Load Layout
    QVBoxLayout* save_load_layout = new QVBoxLayout;

    save_load_layout->addLayout(createLoadSaveLayout(SLOT (loadScenarioFile()), SLOT (saveScenarioFile()), "Scenario"));
    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    save_load_layout->addWidget(line);
    save_load_layout->addLayout(createLoadSaveLayout(SLOT (loadPathFile()), SLOT (savePathFile()), "Path"));
    save_load_layout->addLayout(createLoadSaveLayout(SLOT (loadForceFile()), SLOT (saveForceFile()), "Force"));
    save_load_layout->addLayout(createLoadSaveLayout(SLOT (loadWallFile()), SLOT (saveWallFile()), "Wall"));
    save_load_layout->addLayout(createLoadSaveLayout(SLOT (loadAreaFile()), SLOT (saveAreaFile()), "Area"));
    save_load_layout->addLayout(createLoadSaveLayout(SLOT (loadSectionFile()), SLOT (saveSectionFile()), "Section"));


    tab_widget->widget(3)->setLayout(save_load_layout);

//Data Service Layout
    QVBoxLayout* data_service_layout = new QVBoxLayout;
    
    service_file_combobox = new QComboBox;
    QObject::connect(service_file_combobox, SIGNAL(activated(QString)), SLOT(dataServiceLoadEditor(QString)));
    data_service_layout->addLayout(createDataServiceLayout(SLOT(dataServiceFetchFileList()), "Fetch File List"));
    data_service_layout->addWidget(service_file_combobox);
    data_service_layout->addLayout(createDataServiceLayout(SLOT (dataServiceSaveEditor()), "Save Scenario"));
    data_service_layout->addLayout(createDataServiceLayout(SLOT (dataServiceLoadEditor()), "Load Scenario"));
    data_service_layout->addLayout(createDataServiceLayout(SLOT(dataServiceLoadActive()), "Load to Active"));
    dataServiceFetchFileList();
    
    tab_widget->widget(4)->setLayout(data_service_layout);
    
//add tablayout to layout
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( tab_layout);

//current action layout
    QHBoxLayout* current_action_layout = new QHBoxLayout;
    cancel_button = new QPushButton;
    current_action_label = new QLabel;
    QObject::connect(cancel_button, SIGNAL (clicked()), SLOT (setDoNothing()));
    cancel_button->setText("Cancel");
    current_action_layout->addWidget(current_action_label);
    current_action_layout->addWidget(cancel_button);
    cancel_button->setVisible(false);
    //label is empty and button not shown until an action is selected

    layout->addLayout(current_action_layout);

    setLayout( layout );

//workarround for a bug that occurs when session layout is loaded later
    tab_widget->setCurrentIndex(2);
    tab_widget->setCurrentIndex(0);
}


RobotrainerEditorPanel::RobotrainerEditorPanel( QWidget *parent_, ros::NodeHandle params_node_handle)
    : rviz::Panel( parent_ )
{
    params = new robotrainer_panel::robotrainer_editorParameters(params_node_handle);
    nh = params_node_handle;
    params->fromParamServer();
    
    server.reset(new interactive_markers::InteractiveMarkerServer(params->editor_ns,"",false));

    //initialize robotrainer_editor_display_path and robotrainer_editor_force objects
    robotrainer_editor_display_path = new RobotrainerEditorDisplayPath(params, server, &path_update_publisher);
    robotrainer_editor_force = new RobotrainerEditorForce(params, server, &arrow_publisher, robotrainer_editor_display_path->getFileName());
    robotrainer_editor_wall = new RobotrainerEditorWall(params, server, &wall_publisher);
    robotrainer_editor_area = new RobotrainerEditorArea(params, server, &area_publisher);
    robotrainer_editor_section = new RobotrainerEditorSection(params, server, robotrainer_editor_display_path->getPathLength(), &section_publisher, robotrainer_editor_display_path->getFileName(), this);

    path_update_publisher =  nh.advertise<std_msgs::String>(params->path_update_topic, 1);
    arrow_publisher = nh.advertise<InteractiveMarker>(params->arrow_update_topic, 1);
    wall_publisher = nh.advertise<InteractiveMarker>(params->wall_update_topic, 1);
    area_publisher = nh.advertise<InteractiveMarker>(params->area_update_topic, 1);

    p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);

    record_info_publisher = nh.advertise<geometry_msgs::Vector3>(params->record_info_topic, 1);
    
    //get home directory
    struct passwd *pw = getpwuid(getuid());
    std::string homedir(pw->pw_dir);
    //set default directory for saving/loading files
    yaml_file_directory = QString::fromStdString(homedir + "/" + params->workspace_ns + "/src/" + params->project_ns + "/" + params->yaml_ns + "/");
    
    setupLayout();
}

void RobotrainerEditorPanel::setActive() {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Set Session to Active", "You are about to send your current editor session to the robot for experiment use. Are you sure?", QMessageBox::Yes|QMessageBox::No);
    if (reply != QMessageBox::Yes) {
        return;
    } else {
        //clear active session (in case this doesnt overwrite all)
        ros::param::del(params->project_ns + "/" + params->scenario_ns);
        //save temporary scenario file
        std::ostringstream sstr;
        time_t now = time(0);
        sstr << "/tmp/temp_robotrainer_scenario_" << now;
        std::string file_name = sstr.str();
        saveScenarioFile(file_name);
        //call to bash to load file
        //sstr.clear();
        sstr.str("");
        sstr << "rosparam load " << file_name << ".yaml" << " /" << params->project_ns << "/" << params->scenario_ns;
        std::string str =  sstr.str();
        const char* chr = str.c_str();
        std::system(chr);
        // warn modalities about changed data
        std::system("rosservice call /base/configure_modalities");
        // warn deviation about changed data
        std::system("rosservice call /robotrainer_deviation/configure");
        // warn user performance calculation about new data
        std::system("rosservice call /robotrainer_performance/configure");
    }
}

void RobotrainerEditorPanel::toggleMode() {

    setDoNothing();

    if (!record) {
        editing_button->setEnabled(false);
        create_force_button->setEnabled(false);
        create_wall_button->setEnabled(false);
        create_area_button->setEnabled(false);
        create_section_button->setEnabled(false);
        record_single_point_button->setEnabled(true);
        record = true;
        toggle_mode_button->setText("Toggle Edit Mode");
    } else {
        editing_button->setEnabled(true);
        create_force_button->setEnabled(true);
        create_wall_button->setEnabled(true);
        create_area_button->setEnabled(true);
        create_section_button->setEnabled(true);
        record_single_point_button->setEnabled(false);
        record = false;
        toggle_mode_button->setText("Toggle Record Mode");
    }
}

void RobotrainerEditorPanel::enableEditing() {
    robotrainer_editor_force->setDoSomething();
    robotrainer_editor_force->setCreateWhenMove(false);
    robotrainer_editor_wall->setDoSomething();
    robotrainer_editor_area->setDoSomething();
    robotrainer_editor_section->setDoSomething();
    robotrainer_editor_display_path->setDoSomething(robotrainer_editor_force->returnMoveBoxArrow());

    current_action_label->setText("Editing is enabled!");
    current_action_label->repaint();
    cancel_button->show();
}

void RobotrainerEditorPanel::enableSectionCreation() {
    robotrainer_editor_force->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_force->setCreateWhenMove(false);
    robotrainer_editor_wall->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_area->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_display_path->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_section->setDoSomething();
    
    current_action_label->setText("Section creation is enabled!");
    current_action_label->repaint();
    cancel_button->show();
}

void RobotrainerEditorPanel::enableArrowCreation() {
    robotrainer_editor_force->setDoSomething();
    robotrainer_editor_force->setCreateWhenMove(true);
    robotrainer_editor_wall->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_area->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_section->setDoNothing(); //shall not be edited in this mode
    robotrainer_editor_display_path->setDoSomething(robotrainer_editor_force->returnMoveBoxArrow());

    current_action_label->setText("Force Creation and Editing is enabled!");
    current_action_label->repaint();
    cancel_button->show();
}

void RobotrainerEditorPanel::createWall() {
    std::ostringstream name;
    name << params->wall_ns << "_" << robotrainer_editor_wall->getWallCount();
    geometry_msgs::Point base;
    base.x = 1.f;
    base.y = 0.f;
    base.z = 0.f;
    geometry_msgs::Point end;
    end.x = -1.f;
    end.y = 0.f;
    end.z = 0.f;
    robotrainer_editor_wall->createWall(base, end, name.str());

    setDoNothing();
}

void RobotrainerEditorPanel::createArea() {
    std::ostringstream name;
    name << params->area_ns << "_" << robotrainer_editor_area->getAreaCount();
    geometry_msgs::Point center;
    center.x = 0.f;
    center.y = 0.f;
    center.z = 0.f;
    geometry_msgs::Point edge;
    edge.x = -1.f;
    edge.y = 0.f;
    edge.z = 0.f;
    robotrainer_editor_area->createArea(center, edge, name.str());

    setDoNothing();
}

void RobotrainerEditorPanel::setDoNothing() {

    robotrainer_editor_display_path->setDoNothing(); //set all interactive markers from path to do nothing
    robotrainer_editor_force->setDoNothing();
    robotrainer_editor_wall->setDoNothing();
    robotrainer_editor_area->setDoNothing();
    robotrainer_editor_section->setDoNothing();

    current_action_label->setText("");
    current_action_label->repaint();
    cancel_button->hide();
}

void RobotrainerEditorPanel::recordSinglePoint() {
    geometry_msgs::TransformStamped transform_ee_base_stamped;
    try {
        transform_ee_base_stamped = p_tfBuffer->lookupTransform("map", params->path_record_frame_id, ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException ex ) {
        ROS_ERROR("lookupTransform failed! Current point is not saved.");
        return;
    }

    geometry_msgs::Vector3 pickedPoint = transform_ee_base_stamped.transform.translation;
    pickedPoint.z = 0.0;
    ROS_INFO_STREAM("pickedPoint: " + std::to_string(pickedPoint.x) + " | " + std::to_string(pickedPoint.y));

    robotrainer_editor_display_path->addPoint(RobotrainerEditorTool::vectorToPoint(pickedPoint));
    robotrainer_editor_display_path->reloadSession();
    resetPathDependencies();
}

void RobotrainerEditorPanel::resetPathDependencies()
{
    resetRobotrainerEditorForce();
    resetRobotrainerEditorSection();
}


void RobotrainerEditorPanel::resetRobotrainerEditorForce() {
    saveForceSession();
    robotrainer_editor_force = new RobotrainerEditorForce(params, server, &arrow_publisher, robotrainer_editor_display_path->getFileName());
    robotrainer_editor_force->setCubePointMap(robotrainer_editor_display_path->getCubePointMap());
}

void RobotrainerEditorPanel::resetRobotrainerEditorSection() {
    saveSectionSession();
    robotrainer_editor_section = new RobotrainerEditorSection(params, server, robotrainer_editor_display_path->getPathLength(), &section_publisher, robotrainer_editor_display_path->getFileName(), this);
    robotrainer_editor_section->setup();
}

void RobotrainerEditorPanel::clearSession() {
    //in case you forgot
    savePathSession();
    //deactivate running actions as they won't be active for newly loaded markers
    setDoNothing();

    //delete all data on all related namespaces on param server
    ros::param::del(params->editor_ns);

    robotrainer_editor_display_path = new RobotrainerEditorDisplayPath(params, server, &path_update_publisher);
    resetPathDependencies();
    robotrainer_editor_wall = new RobotrainerEditorWall(params, server, &wall_publisher);
    robotrainer_editor_area = new RobotrainerEditorArea(params, server, &area_publisher);

    //delete all data on interactive marker server
    server->clear();
    robotrainer_editor_section->setup();
    server->applyChanges();
}

void RobotrainerEditorPanel::reset(RobotrainerEditorDisplayPath* d) {
    ROS_INFO_STREAM("resetting Path");
    clearSession();
}

void RobotrainerEditorPanel::reset(RobotrainerEditorForce* f) {
    ROS_INFO_STREAM("resetting Forces");
    resetRobotrainerEditorForce();
}

void RobotrainerEditorPanel::reset(RobotrainerEditorArea* a) {
    ROS_INFO_STREAM("resetting Areas");
    robotrainer_editor_area = new RobotrainerEditorArea(params, server, &area_publisher);
}

void RobotrainerEditorPanel::reset(RobotrainerEditorWall* w) {
    ROS_INFO_STREAM("resetting Walls");
    robotrainer_editor_wall = new RobotrainerEditorWall(params, server, &wall_publisher);
}

void RobotrainerEditorPanel::reset(RobotrainerEditorSection* s) {
    ROS_INFO_STREAM("resetting Sections");
    resetRobotrainerEditorSection();
}

//find a new name with geiven prefix that isn't allready in list of used names
std::string RobotrainerEditorPanel::findName(std::string prefix, std::vector<std::string> in_use) {
    std::string name = "";

    for(int i = 0; name == ""; i++) {
        std::ostringstream new_name;
        new_name << prefix << i;
        if(std::find(in_use.begin(), in_use.end(), new_name.str()) == in_use.end()) {
            name = new_name.str();
        }
    }
    return name;
}

template <typename T> std::string RobotrainerEditorPanel::nameSession(std::string name, std::string prefix, std::string display_name, std::vector<std::string>* names, T* tool) {
    if(name == "") {
        if (display_name != "") prefix = display_name + "_" + prefix + "_";
        name = findName(prefix, *names);
        tool->setFileName(name);
        ROS_INFO_STREAM("session had no name, automatically named it '"+ name +"'");
    }

    return name;
}

template <typename T> void RobotrainerEditorPanel::saveSession(T* tool, std::vector<std::string>* names, std::map<std::string, T*>* sessions, QComboBox* combobox) {
    BOOST_STATIC_ASSERT((std::is_base_of<RobotrainerEditorTool, T>::value));
    if (tool->getCount()) { //save session if it has any content worth saving
        std::string name = nameSession(tool->getFileName(), tool->getNs(), tool->getDisplayPathFileName(), names, tool);
        if(std::find(names->begin(), names->end(), name) == names->end()) {
            names->push_back(name);
            ROS_INFO_STREAM("session name '" + name + "' not found in "+ tool->getNs() + " vector, added");
        }
        sessions->insert(std::make_pair(name, tool));
        //cannot use "!"("not") because function returns "-1" and not "0" when not found (because else it would return an index, which can be 0)
        if((combobox->findText(QString::fromStdString(name))) == -1) {
            combobox->addItem(QString::fromStdString(name), QString::fromStdString(name));
            ROS_INFO_STREAM("session name '" + name + "' not found in "+ tool->getNs() + " combobox, added");
        }
    }
    ROS_INFO_STREAM("saved " + tool->getNs() + " session.");
}

void RobotrainerEditorPanel::savePathSession() {
    saveSession(robotrainer_editor_display_path, &path_names, &robotrainer_editor_display_path_sessions, robotrainer_editor_display_path_sessions_combobox);
    saveForceSession();
    saveWallSession();
    saveAreaSession();
    saveSectionSession();
}

void RobotrainerEditorPanel::saveForceSession() {
    saveSession(robotrainer_editor_force, &force_names[robotrainer_editor_display_path->getFileName()], &robotrainer_editor_force_sessions, robotrainer_editor_force_sessions_combobox);
}

void RobotrainerEditorPanel::saveWallSession() {
    saveSession(robotrainer_editor_wall, &wall_names, &robotrainer_editor_wall_sessions, robotrainer_editor_wall_sessions_combobox);
}

void RobotrainerEditorPanel::saveAreaSession() {
    saveSession(robotrainer_editor_area, &area_names, &robotrainer_editor_area_sessions, robotrainer_editor_area_sessions_combobox);
}

void RobotrainerEditorPanel::saveSectionSession() {
    saveSession(robotrainer_editor_section, &section_names[robotrainer_editor_display_path->getFileName()], &robotrainer_editor_section_sessions, robotrainer_editor_section_sessions_combobox);
}

template <typename T> void RobotrainerEditorPanel::loadSession(T** tool, std::string name, std::vector<std::string>* names, std::map<std::string, T*>* sessions, QComboBox* combobox) {

    //make sure 'tool' is a "RobotrainerEditorTool"
    BOOST_STATIC_ASSERT((std::is_base_of<RobotrainerEditorTool, T>::value));

    //save current sate as session
    saveSession((*tool), names, sessions, combobox);
    //savePathSession();

    //deactivate running actions as they won't be active for newly loaded markers
    setDoNothing();

    //select to have no session selected
    if(name == "" || name == "None/New") {
        if (!(*tool)->getCount()) return; //session is allready empty, no new empty session needed
        ROS_INFO_STREAM("new empty " + (*tool)->getNs() + " session");
        (*tool)->resetServer();
        reset((*tool));
        //tool->reloadSession();
        server->applyChanges();
        
        return;
    }
    if (!(sessions->count(name))) {
        ROS_ERROR("%s session '%s' not found in map!", (*tool)->getNs().c_str() , name.c_str());
        return;
    }

    (*tool)->resetServer();
    (*tool) = (*sessions)[name];
    (*tool)->reloadSession();
    
}

void RobotrainerEditorPanel::loadPathSession(std::string name) {
    clearSession();
    loadSession(&robotrainer_editor_display_path, name, &path_names, &robotrainer_editor_display_path_sessions, robotrainer_editor_display_path_sessions_combobox);
    resetPathDependencies();
}

void RobotrainerEditorPanel::loadPathSession(QString name) {
    std::string name_ = name.toUtf8().constData();
    loadPathSession(name_);
}

void RobotrainerEditorPanel::loadForceSession(std::string name) {
    loadSession(&robotrainer_editor_force, name, &force_names[robotrainer_editor_force->getDisplayPathFileName()], &robotrainer_editor_force_sessions, robotrainer_editor_force_sessions_combobox);
}

void RobotrainerEditorPanel::loadForceSession(QString name) {
    std::string name_ = name.toUtf8().constData();
    loadForceSession(name_);
}

void RobotrainerEditorPanel::loadWallSession(std::string name) {
    loadSession(&robotrainer_editor_wall, name, &wall_names, &robotrainer_editor_wall_sessions, robotrainer_editor_wall_sessions_combobox);
}

void RobotrainerEditorPanel::loadWallSession(QString name) {
    std::string name_ = name.toUtf8().constData();
    loadWallSession(name_);
}

void RobotrainerEditorPanel::loadAreaSession(std::string name) {
    loadSession(&robotrainer_editor_area, name, &area_names, &robotrainer_editor_area_sessions, robotrainer_editor_area_sessions_combobox);
}

void RobotrainerEditorPanel::loadAreaSession(QString name) {
    std::string name_ = name.toUtf8().constData();
    loadAreaSession(name_);
}

void RobotrainerEditorPanel::loadSectionSession(std::string name) {
    loadSession(&robotrainer_editor_section, name, &section_names[robotrainer_editor_section->getDisplayPathFileName()], &robotrainer_editor_section_sessions, robotrainer_editor_section_sessions_combobox);
}

void RobotrainerEditorPanel::loadSectionSession(QString name) {
    std::string name_ = name.toUtf8().constData();
    loadSectionSession(name_);
}

void RobotrainerEditorPanel::setupSession(std::string file_name) {
    robotrainer_editor_display_path->loadFile();
    resetPathDependencies(); //needed explicitly here because it depends on data that is only available when loaded by path
    robotrainer_editor_force->loadFile();
    robotrainer_editor_wall->loadFile();
    robotrainer_editor_area->loadFile();
    robotrainer_editor_section->loadFile();
    robotrainer_editor_display_path->setFileName(file_name);
    robotrainer_editor_force->setFileName(file_name);
    robotrainer_editor_wall->setFileName(file_name);
    robotrainer_editor_area->setFileName(file_name);
    robotrainer_editor_section->setFileName(file_name);
}

void RobotrainerEditorPanel::sessionFromActive() {
  
    clearSession();
    
    //save temporary scenario file
    std::ostringstream sstr;
    time_t now = time(0);
    sstr << "/tmp/temp_robotrainer_scenario_" << now;
    std::string file_name = sstr.str();
    sstr.str("");
    sstr << "rosparam dump " << file_name << ".yaml" << " /" << params->project_ns << "/" << params->scenario_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);
    
    //call to bash to load file
    sstr.str("");
    sstr << "rosparam load " << file_name << ".yaml" << " /" << params->editor_ns;
    str =  sstr.str();
    chr = str.c_str();
    std::system(chr);

    setupSession(file_name);
}

// save (whole!) current scenerio to file
void RobotrainerEditorPanel::saveScenarioFile(std::string filename) {
     std::string scenario_id;
    if (!ros::param::get(params->editor_ns + "/scenario_id", scenario_id)) {
      std::time_t t = std::time(nullptr);
      char mbstr[100];
      std::strftime(mbstr, sizeof(mbstr), "%Y%m%d%H%M", std::localtime(&t));
      scenario_id = "scenario_id";
      scenario_id.append(mbstr);
      ros::param::set(params->editor_ns + "/scenario_id", scenario_id);
    }
    
    //call to bash to save data into persistent yaml file
    std::ostringstream sstr;

    ros::param::set ( params->editor_ns + "/" + params->scenario_ns +"/", filename );
    
    //if filename doesnt allready contain ".yaml", add it
    std::string yaml = ".yaml";
    if (filename.length() <= yaml.length() || 0 != filename.compare (filename.length() - yaml.length(), yaml.length(), yaml)) {
        filename += yaml;
    }

    sstr << "rosparam dump " << filename << " /" << params->editor_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);
}

// save (whole!) current scenerio to file
void RobotrainerEditorPanel::saveScenarioFile() {
    QString file_name = QFileDialog::getSaveFileName(this,tr("Save YAML file"), yaml_file_directory, tr("Yaml Files (*.yaml)"));
    if(file_name.isEmpty()) return;
    else {
        saveScenarioFile(file_name.toUtf8().constData());
    }
}

void RobotrainerEditorPanel::loadScenarioFile(std::string filename) {

    //call to bash to load file
    std::ostringstream sstr;
    sstr << "rosparam load " << filename << " /" << params->editor_ns;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);
}

void RobotrainerEditorPanel::loadScenarioFile() {

    QString file_name = QFileDialog::getOpenFileName(this,tr("Save YAML file"), yaml_file_directory, tr("Yaml Files (*.yaml)"));
    if(file_name.isEmpty()) return;
    else {

        clearSession();

        std::string file_name_ = file_name.toUtf8().constData();
        loadScenarioFile(file_name_);

        setupSession(file_name_);
    }
}


template <typename T> void RobotrainerEditorPanel::saveFile(T** tool) {
  
    //make sure 'tool' is a "RobotrainerEditorTool"
    BOOST_STATIC_ASSERT((std::is_base_of<RobotrainerEditorTool, T>::value));

    QString file_name = QFileDialog::getSaveFileName(this,tr("Save YAML file"), yaml_file_directory, tr("Yaml Files (*.yaml)"));
    if(file_name.isEmpty()) return;
    else {
        (*tool)->saveFile(file_name.toUtf8().constData());
    }
}

void RobotrainerEditorPanel::savePathFile() { saveFile(&robotrainer_editor_display_path);}
void RobotrainerEditorPanel::saveForceFile() { saveFile(&robotrainer_editor_force);}
void RobotrainerEditorPanel::saveWallFile() { saveFile(&robotrainer_editor_wall);}
void RobotrainerEditorPanel::saveAreaFile() { saveFile(&robotrainer_editor_area);}
void RobotrainerEditorPanel::saveSectionFile() { saveFile(&robotrainer_editor_section);}

template <typename T> bool RobotrainerEditorPanel::checkNotYetLoaded(std::string file_name, std::map<std::string, T*>* sessions) {
  
      if(!sessions->count(file_name)) return true;

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "File allready loaded", "The name of the File you are trying to load fits the one of an open session. Loading it will overwrite that session, load it anyway?", QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        return true;
    } else {
        return false;
    }
}

template <typename T> bool RobotrainerEditorPanel::checkNotYetLoaded(QString file_name, std::map<std::string, T*>* sessions) {
    std::string file_name_ = file_name.toUtf8().constData();
    return checkNotYetLoaded(file_name_, sessions);
}

//check if a the display that a file is configured for matches the loaded display
template <typename T> bool RobotrainerEditorPanel::checkPathComaptible(std::string file_name, T* tool) {
    if(tool->getDisplayPathFileName() == file_name) return true;

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "File not matching Path", "The File you are trying to load doesn't seem to fit the active Display Path, load it anyway?",
                                  QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        return true;
    } else {
        return false;
    }
}

template <typename T> bool RobotrainerEditorPanel::checkPathComaptible(QString file_name, T* tool) {
    std::string file_name_ = file_name.toUtf8().constData();
    return checkPathComaptible(file_name_, tool);
}

template <typename T> void RobotrainerEditorPanel::loadFile(T** tool, std::vector<std::string>* names, std::map<std::string, T*>* sessions, QComboBox* combobox){
    QString file_name = QFileDialog::getOpenFileName(this,tr("Save YAML file"), yaml_file_directory, tr("Yaml Files (*.yaml)"));
    if(file_name.isEmpty()) return;
    else {
        saveSession(*tool, names, sessions, combobox);

        if(!checkNotYetLoaded(file_name, sessions)) return; //check if this would overwrite a session, and if the user wants to do it
        if((*tool)->getDisplayPathFileName() != "" && !checkPathComaptible(file_name, *tool)) return; //if this depends on path, check path comaptibility

        reset(*tool);

        (*tool)->loadFile(file_name.toUtf8().constData());
        if(((*tool)->getNs()).compare(params->path_ns) == 0) resetPathDependencies(); //more elegant way?
        
    }
}

void RobotrainerEditorPanel::loadPathFile()
{
    loadFile(&robotrainer_editor_display_path, &path_names, &robotrainer_editor_display_path_sessions, robotrainer_editor_display_path_sessions_combobox);
}

void RobotrainerEditorPanel::loadForceFile()
{
    loadFile(&robotrainer_editor_force, &force_names[robotrainer_editor_force->getDisplayPathFileName()], &robotrainer_editor_force_sessions, robotrainer_editor_force_sessions_combobox);
}

void RobotrainerEditorPanel::loadWallFile() {
    loadFile(&robotrainer_editor_wall, &wall_names, &robotrainer_editor_wall_sessions, robotrainer_editor_wall_sessions_combobox);
}

void RobotrainerEditorPanel::loadAreaFile() {
    loadFile(&robotrainer_editor_area, &area_names, &robotrainer_editor_area_sessions, robotrainer_editor_area_sessions_combobox);
}

void RobotrainerEditorPanel::loadSectionFile() {
  loadFile(&robotrainer_editor_section, &section_names[robotrainer_editor_section->getDisplayPathFileName()], &robotrainer_editor_section_sessions, robotrainer_editor_section_sessions_combobox);
}

void RobotrainerEditorPanel::save( rviz::Config config ) const
{
    rviz::Panel::save( config );

}

// Load all configuration data for this panel from the given Config object.
void RobotrainerEditorPanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );

}

void RobotrainerEditorPanel::dataServiceSaveEditor()
{
   std::string name;
    if(!ros::param::get(params->editor_ns + "/" + params->scenario_ns, name))
    {
        bool ok;
        QString s = QInputDialog::getText(this, "Please enter a name for this Scenario.",
                                       "Max Deviation:", QLineEdit::Normal, "MyScenario", &ok);

        if (ok) name = s.toUtf8().constData();
        else{
          ROS_ERROR("could not set scenario name!");
          return;
        }
        ros::param::set(params->editor_ns + "/" + params->scenario_ns, name);
    }
    dataServiceSave(params->editor_ns, name);
}

void RobotrainerEditorPanel::dataServiceSave(std::string ns, std::string file_name)
{
    std::ostringstream sstr;
    sstr << "rosservice call /save " << ns << " " << file_name;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);
}

void RobotrainerEditorPanel::dataServiceLoadEditor()
{
    savePathSession();
    setupSession(dataServiceLoad(params->editor_ns));
}

void RobotrainerEditorPanel::dataServiceLoadEditor(std::string file_name)
{
    savePathSession();
    setupSession(dataServiceLoad(params->editor_ns, file_name));
}

void RobotrainerEditorPanel::dataServiceLoadEditor(QString name) {
    std::string name_ = name.toUtf8().constData();
    dataServiceLoadEditor(name_);
}

void RobotrainerEditorPanel::dataServiceLoadActive()
{
    dataServiceLoad(params->project_ns + "/" + params->scenario_ns);
    std::system("rosservice call /base/configure_modalities");
}

std::string RobotrainerEditorPanel::dataServiceLoad(std::string ns)
{
    QString file_name = QFileDialog::getOpenFileName(this,tr("Save YAML file"), yaml_file_directory, tr("Yaml Files (*.yaml)"));
    if(file_name.isEmpty()) return "false";
    else
    {
        clearSession();

        std::string file_name_ = file_name.toUtf8().constData();
        loadScenarioFile(file_name_);

        setupSession(file_name_);
        
        return dataServiceLoad(ns, file_name_);
    }
}

std::string RobotrainerEditorPanel::dataServiceLoad(std::string ns, std::string file_name)
{
    std::ostringstream sstr;
    sstr << "rosservice call /load " << ns << " " << file_name;
    std::string str =  sstr.str();
    const char* chr = str.c_str();
    std::system(chr);
    return file_name;
}

void RobotrainerEditorPanel::dataServiceFetchFileList(){
    
    for (int i = 0; i < service_file_combobox->count(); i++)
    {
      service_file_combobox->removeItem(i);
    }
    service_files.clear();
  
    robotrainer_data_service::FileList srv;
    if (ros::service::call("get_list", srv))
    {
      std::string list = srv.response.list;
      if(!srv.response.success) ROS_ERROR("Could not Fetch list of Yamls form robotrainer_data_service: %s",  list.c_str());
      else
      {
        size_t pos = 0;
        while ((pos = list.find(";")) != std::string::npos)
        {
            service_files.push_back(list.substr(0, pos));
            list.erase(0, pos + 1);
        }
        for (auto s :service_files)
        {
            service_file_combobox->addItem(QString::fromStdString(s), QString::fromStdString(s));
            ROS_INFO_STREAM("Added "+s+" to the File selection.");
        }
        
      }
    }
    else
    {
      ROS_ERROR("Failed to fetch File List from robotrainer_data_service");
  }
}

} //end namespace robotrainer_editor_panel
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotrainer_editor_panel::RobotrainerEditorPanel,rviz::Panel )
