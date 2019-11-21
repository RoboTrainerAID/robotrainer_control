robotrainer
==========================================

## ROS Distro Support

|         | Kinetic | Melodic |
|:-------:|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://gitlab.ipr.kit.edu/IIROB/robotrainer/tree/kinetic-devel) | 
| Status  | [![build status](https://gitlab.ipr.kit.edu/IIROB/robotrainer/badges/kinetic-devel/pipeline.svg)](https://gitlab.ipr.kit.edu/IIROB/robotrainer/commits/kinetic-devel) | |

## Status and ToDos

Status:
- Controller restructuring complete. `FTSBaseController` is now the base controller class, from which individual controllers can be derived. 
  As it is a meta controller containing all base functionalities separated into subfunctions for easier adaption at any point in the control loop,
  it cannot be used to actually drive the robot. For the basic functionality, the controller `FTSController` has to be used instead. 
- Modalities have not been tested yet with new parameter server structure (wheel of robot was broken). But all modalities *should* be working fine. 
- PathTracking has untested changes (pathtracking on path sections instead of on whole path)

ToDos:
- integrate `<robotrainer>/yamls/modalities_chain.yaml` into launchfile. Currently, this needs to be loaded manually on startup (see instructions below)
- integrate package `robotrainer_bringup` (launchfiles) into robotrainer package

## Dependencies and build problems

The workspace on SR2 requires this package and the the package `robotrainer` on branch `master_zumkeller` in order to work properly.


## Starting adaptive FTS controller and viewing live input and output

+ Open 3 terminal windows:
  - In terminal 1: 
    launch robot with adaptive FTS controller (or alternative controllers):
    ```
    ssh -X modalities-sr2
    roslaunch robotrainer_bringup sr2_robotrainer_adaptive.launch
    ```
    initialize robot by pushung `START+RB` on the joystick
  - In terminal 2:
    start rqt_reconfigure to change parameters on the fly:
    ```
    rosrun rqt_reconfigure rqt_reconfigure
    ```
  - In terminal 3:
    start rqt_plot to view robot input and output
    ```
    rqt_plot /base/robotrainer_controllers/force_input_limited:velocity_output
    ```


## Starting robotrainer with demo scenario

+ Open 4 terminal windows.
  - In terminal 1: load scenario to parameter server and setup filter chain
    ```
    rosparam load <robotrainer>/yamls/demo_scenario.yaml
    rosparam load <robotrainer>/yamls/modalities_chain.yaml
    ```
  - In terminal 2: start robot
    ```
    ssh modalities-sr2
    roslaunch robotrainer_bringup sr2_robotrainer.launch
    ```
  - In terminal 1: initialize robot (or use joystick `START+RB`)
    ```
    rosservice call /base/driver/init
    ```
  - In terminal 3: start RViz
    ```
    rviz
    ```
  - In terminal 4: start rqt
    ```
    rqt
    ```
+ You can now drive around with the robotrainer. Modalities are not yet activated!
+ In RViz, check the position of the robot on the map. Correct it manually, if necessary with '2D Pose Estimate'.
+ In rqt
  - Open dynamic reconfigure: Plugins > Configuration > Dynamic Reonfigure
  - Navigate to `/base` node
  - **Attention:** Before continuing, move the robot to a place where no virtual forces will be (check the scenario). Otherwise the robot might suddenly start moving.
  - check `use_modalities` in `\base` to activate modalities
  - Optional: Change speed and sensivity of the robot. Important: If you change something in `\base` make sure that you change the corrensponding values in the modality subnodes as well. Otherwise you e.g. might be able to drive through walls if you push hard enough. There are 3 preconfigured settings stored in YAML files for your convenience: slow (loaded by default), normal, fast.
    * Load base parameters from YAML file (click on folder icon): `base_slow` (default) or `base_normal` or `base_fast` in folder `<robotrainer>/yaml/rqt_params`
    * reload dynamic reconfigre (click on small reload icon in top right corner). Now you can see a new node `/modalities` with subnodes `virtual_areas`, `virtual_forces`, `virtual_walls`, `pathtracking`.
    * In each modality subnode, load parameters from the corresponding YAML file. Choose `<modality>_normal` if you've selected `base_normal` and so on.
  - **Modalities are now ready to go**
+ Stop Robot:
  - in rqt, uncheck `use_modalities` (otherwise modalities will be activated automatically on next startup unless robot has been restarted.)
  - in Terminal 2: `Ctrl+C`


## Creating scenarios (with the Robotrainer Editor)

+ Starting the robotrainer_editor:
  - Run RViz. (Make shure roscore is running)
    ```
    rviz
    ```
  - Open robotrainer_panel plugin as RViz panel: Panels > Add New Panel > robotrainer_panel/RobotrainerEditorPanel
  - ![alt text](robotrainer_panel/documentation/01OpenRobotrainerEditorPanel.png?raw=true "Open the Panel")
    
  - Open the RViz display to see the grafic editor data: (In "Displays" Screen) > Add > By topic > robotrainer/scenario/update/InteractiveMarkers
  - ![alt text](robotrainer_panel/documentation/02OpenRobotrainerEditorDisplay.png?raw=true "Open the Display")
    
  - If you don't want to need to repeat this every time, save the Rviz Configuration uppon closing.
  - ![alt text](robotrainer_panel/documentation/03SaveOnExit.png?raw=true "Save On Exit")
    
  - *Optional: Run 'rosrun robotrainer_data_service data_srv' in a command prompt - this is necessary if you want to use this service to load/save scenario files. Loading and saving files is also possible without it, but this service shall be used for database communication in the future.
  
+ Navigating the Panel:
  - The panel is split into multiple tabs: "Params", "Actions", "Session", "Load/Save" and "Data Service"
  - ![alt text](robotrainer_panel/documentation/10Tabs.png?raw=true "Tabs for Navigation")
  
+ Understanding the Display:
  - The Path: the path is represented as a red line with white cubes along it. The white cubes represent interactive points, at which certain modifiers can be added - so far only the "forces".
  - The Forces: represented as a blue circle and a blue arrow, these represent an area along the path in which a force presses onto the robot, thus deflecting the probant.
  - The Sections: green sections along the path represent the segments where the robot is forced to stay close to the path.
  - The Areas: Areas where different modifiers can take effect, represented as green circles.
  - The Walls: Walls are meant to delimit the experiment area, for example to prevent the robot from being dirven into a wall in the real world. The virtual walls are represented by a yellow line. They come with an "area of effect", determining how close the robot must to the wall to be repulsed by it.
  - ![alt text](robotrainer_panel/documentation/04Display.png?raw=true "Scenario Representation in the Display")
  
+ Recording a robot path:
  - Paths cannot be created in the tool, only recorded. This is this way to make shure that every path is reproducible in the real world.
  - To record a path, one must open the "Actions" tab and "Toggle Expermient Mode".
  - Now, the "Record Single Point" button is enabled. By pressing it, the current position of the robot is recorded and connected to the last position recorded.
  - ![alt text](robotrainer_panel/documentation/05ToggleExperimentMode.png?raw=true "Toggle Expermient Mode") ![alt text](robotrainer_panel/documentation/06RecordSinglePoint.png?raw=true "Record Single Point")
  
+ Editing a scenario:
  - Navigate to the "Actions" tab.
  - Here, you can see multiple Buttons, enabling you to different operations.
  - After clicking "Enable editing", you can drag around the allready created objects and use the context menus, that can now be opend by right-clicking.
  - By clicking "Create Wall" / "Create Area", you can create an object of that respective type. They are spawned at default positions.
  - By clicking "Create/Move Force Arrows", you are enabled to drag on one of the "interactive path points", represented as white cubes, to create a new force-field. The white cube now represents the end of the arrow. By draging arround the blue cube on its edge, the size of the field can be modified.
  - By clicking "Enable Section Creation", the red path segments gain a new context menu, opened by right-clicking, in which new tracking-sections can be created along the path. They can be modifed or deleted by right-clicking them afterwards.
  - ![alt text](robotrainer_panel/documentation/07Editing.png?raw=true "Editing the Scenario")
  
+ Loading/Saving:
  - Open the Load/Save tab.
  - Each existing modifier type (see: "Understanding the Display") can save it's own file, or a whole scenario can be saved into one file.
  - ![alt text](robotrainer_panel/documentation/08LoadSave.png?raw=true "Loading and Saving Files")
  - Alternatively, you can load/save scenarios (only the full scenario, in this case) in the "Data Service" tab. Here, a service is used to load/save from a default folder. In the future, this service shall communicate with a database.
  - In the "Data Service" tab, you can see a dropdown menu that will show all scenarios in the services folder (or database).
  - There is a button to update that list, in case you saved something new.
  - ![alt text](robotrainer_panel/documentation/11DataService.png?raw=true "Using external service to load/save files")
  
+ Sessions:
  - Multiple scenarios can be loaded at once, whilst only one is displayed in the 3D display, and only one is modfying the robot behaviour (not necessarily the same one).
  - To switch between multiple scenarios, i.e. their respective components, navigate to the "Sessions" tab.
  - Here, you can load sessions of the different modifier types (see: "Understanding the Display") independantly.
  - Sessions are saved automatically when being replaced by another session that is loaded (from file or session manager).
  - ![alt text](robotrainer_panel/documentation/09Sessions.png?raw=true "Using the Session Management")


## Performing Experiments (with the Robotrainer Editor)

+ Activating a Scenario:
  - Open the "Actions" tab.
  - Here, you will see a button called "Set Session to Active".
  - This button will load the Scenario that you are seeing in the Display from the Editor namespace into the experiment namespace and allert the modalities process.
  - Thus, it will cause the robot to actually enter this scenario.
  
+ Loading a Scenario as Active:
  - Open the "Data Service" tab.
  - Here, you will see a button called "Load to Active".
  - When loading a file using this button, it will be loaded directly into the experiment namespace, and not into the editor namespace (hence, you will not see it in the display!).
  
+ Loading a Scenario from Active:
  - Open the "Session" tab.
  - Here, you will see a button called "Load Session from Active Scenario".
  - When pressing this button, you will load the Scenario that is currently in the experiment namespace into the editor namespace, so you will be able to see it in the display.
  - Note that editing this scenario here will not influence the version in the experiment namespace, and thus not influence the robots behaviour!
  - To update the edited scenario in the experiment namespace, "Activate" the scenario in the "Actions" tab.

### Notes

+ Requirements to the path:
  - The distance of two pathpoints needs to be consistent and much smaller than max_deviation setting in a pathtracking section. Good value: 5cm.
  - The radius of a curve of the path should be a little bigger than max_deviation
+ Pathtracking
  - The robot will go into pathtracking mode if it comes within 30cm of the startpoint of a (pathtracking) section. You will feel it 'snap' to the startpoint. At the end of the section, the robot will leave the pathtracking mode.
  - There can be multiple section on a path. If the robot is in pathtracking mode on one section, it will ignore all other sections until the robot has reached the end of the tracked section.
+ Virtual Areas:
  - Attention when using invert_y: The robot should not be moved sideways (more than 45°) into the area or out of the area. Otherwise there will be bouncing at the area border.
  - Attention when using invert_direction: When moving the robot out of the area: Make sure not to move it vertical to entry-direction. Otherwise the robot will bouce back into the area (with new entry-direction). 

## Parameter Server

The following shows the params on parameter server that are relevant for the robotrainer.

```yaml
base: #Namespace of the FTS Controller
  use_modalities: true #default: false. Turn on/off the modalities.
robotrainer: #Common namespace for all robotrainer related params
  modalities_config: #Configuration valid for all elements of the corresponding modality. Dynamically reconfigurable in rqt.
    virtual_areas:
    virtual_forces:
      compensate_velocity: false
      compensation_reference_velocity: 0.3
      controller_update_rate: 100.0
      max_force: 110.0
      max_velocity: 0.8
      time_const_T: 0.6
      trapezoid_max_at_percent_radius: 0.25
    virtual_walls:
      controller_update_rate: 100
      max_force: 165
      max_velocity: 1.21
      time_const_T: 0.6
      trapezoid_max_at_percent_radius: 0.25
      wall_force: 165
    path_tracking:
      controller_update_rate: 100
      max_force: 110
      max_velocity: 1.21
      time_const_T: 0.6
  modalities_chain_config: #Configuration of the filter chain. Structure required by the filter chain. The order of the filters here defines the order of execution.
    - name: areas_modality
      type: robotrainer_modalities/VirtualAreasFilter
      params: {}
    - name: forces_modalitiy
      type: robotrainer_modalities/VirtualForcesFilter
      params: {}
    - name: walls_modality
      type: robotrainer_modalities/VirtualWallsFilter
      params: {}
    - name: pathtracking_modality
      type: robotrainer_modalities/PathTrackingFilter
      params: {}
  scenario: #Active scenario created in the robotrainer editor. When it gets changed on parameter server, the following service needs to be called in order to push the scenario to the modalities: /base/configure_modalities.
    #Both robotrainer_modalities and robotrainer_editor depend on this structure: the editor configures it, the modalities read it.
    area:
      config:
        area_names: [area_0]
      data:
        area_0:
          area: {X: -0.5, Y: 1.5, Z: 0.0} #area center point
          margin: {X: -0.5, Y: 0.8, Z: 0.0} #point on area margin  
          amplification: 1.0 #amplification of velocities within area
          invert_direction: false #Invert direction within area
          invert_rot: false #Invert rotation within area
          invert_y: false #Invert y-axis within area
          keep_direction: false #Keep the direction constant within area
          keep_rotation: false #Keep the rotation constant within area
    force: 
      config:
        display_path_file_name: /home/groten/path_yaml_test.yaml #relevant in editor but not for modalities
        force_names: [force_1]
        newton_per_meter: 30.0
      data:
        force_1:
          area: {X: -0.5, Y: 1.5, Z: 0.0} #area center point
          arrow: {X: 0, Y: -25.0, Z: 0.0} #relative force vector in Newton
          margin: {X: -0.5, Y: 0.8, Z: 0.0} #point on area margin     
          force_distance_function: 0 #0=trapezoidal 1=trigonometical 2=gaussian
    path:
      points: [point1, point2, point3]
      point1: {x: 0.32290059083819167, y: 0.024954837149349096, z: 0.0}
      point2: {x: 0.42197575569747503, y: 0.03852361410623921, z: 0.0}
      point3: {x: 0.5210509205567584, y: 0.05209239106312933, z: 0.0}
    section: #sections of the path in which pathtracking will be enabled
      config:
        section_names: [section_0]
      data:
        section_0:
          start: point1
          end: point3
          force_distance_function: 0 #0=linear 1=quadratic
          max_deviation: 0.5 #Maximum deviation from the path in meters, min=0.3
    wall:
      config:
        wall_names: [wall_0]
      data:
        wall_0:
          L: {X: 2 Y: -1.2, Z: 0.0}
          R: {X: -1.0, Y: 0.0, Z: 0.0}
          area: 
            area: 0.5
            cube: {X: 0.5, Y: 0.0, Z: 0.0} #not relevant for modalities
          force_distance_function: 0 #0=trapezoidal 1=trigonometical 2=gaussian

```
