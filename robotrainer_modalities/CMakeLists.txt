cmake_minimum_required(VERSION 2.8.3)
project(robotrainer_modalities)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  geometry_msgs
  std_msgs
  rosparam_handler
  dynamic_reconfigure
  tf2_ros
  filters
  robotrainer_parameters
  robotrainer_helper_types
  realtime_tools
)

# needed for rosparam_handler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (Cpp11CompilerFlag)
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

endif()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_ros_parameter_files(
  cfg/PathTracking.params
  cfg/VirtualForces.params
  cfg/VirtualAreas.params
  cfg/VirtualWalls.params
  cfg/ModalitiesVFController.params
  cfg/ModalitiesVirtualWallsController.params
  cfg/ModalitiesPathTrackingController.params
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS robotrainer_parameters robotrainer_helper_types
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/virtual_forces.cpp
   src/path_tracking.cpp
   src/virtual_areas.cpp
   src/virtual_walls.cpp
   src/modalities_virtual_forces_controller.cpp
   src/modalities_virtual_walls_controller.cpp
   src/modalities_path_tracking_controller.cpp
   
   include/${PROJECT_NAME}/virtual_forces.h
   include/${PROJECT_NAME}/modality_base.h
   include/${PROJECT_NAME}/path_tracking.h
   include/${PROJECT_NAME}/virtual_areas.h
   include/${PROJECT_NAME}/virtual_walls.h
   include/${PROJECT_NAME}/modalities_controller_base.h
   include/${PROJECT_NAME}/modalities_virtual_forces_controller.h
   include/${PROJECT_NAME}/modalities_virtual_walls_controller.h
   include/${PROJECT_NAME}/modalities_path_tracking_controller.h

 )

 # make sure configure headers are built before any node using them
 add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg  # For dynamic_reconfigure
  ${PROJECT_NAME}_genparam  # For rosparam_handler
  )
  
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
 
