cmake_minimum_required(VERSION 2.8.3)
project(robotrainer_panel)

find_package(catkin REQUIRED COMPONENTS 
  rviz 
  rosparam_handler 
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  rosbag
  roscpp
  rospy
  visualization_msgs
  std_msgs
  robotrainer_data_service
  robotrainer_parameters
)

# set compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (${Cpp11CompilerFlag})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "17")
	#no additional flag is required
else()
	message(FATAL_ERROR "Compiler does not have c++11 support. Use at least g++4.9 or Visual Studio 2013 and newer.")
endif()

generate_ros_parameter_files(
  cfg/robotrainer_editor.params
)

add_service_files(DIRECTORY ../robotrainer_data_service/srv FILES DataOperation.srv FileList.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS robotrainer_parameters)#INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets) #find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets) trough the error: 'Could not find a configuration file for package "Qt5" that exactly matches requested version "5.5.1".'
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## The tutorial prefers the Qt signals and slots to avoid defining "emit",
##"slots", etc because they can conflict with boost signals, so define 
## QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(HDR_FILES
  include/robotrainer_panel/robotrainer_editor_tool.h
  include/robotrainer_panel/robotrainer_editor_force.h
  include/robotrainer_panel/robotrainer_editor_display_path.h
  include/robotrainer_panel/robotrainer_editor_panel.h
  include/robotrainer_panel/robotrainer_editor_wall.h
  include/robotrainer_panel/robotrainer_editor_area.h
  include/robotrainer_panel/robotrainer_editor_section.h
)

set(SRC_FILES
  src/robotrainer_editor_tool.cpp
  src/robotrainer_editor_force.cpp
  src/robotrainer_editor_panel.cpp
  src/robotrainer_editor_display_path.cpp
  src/robotrainer_editor_wall.cpp
  src/robotrainer_editor_area.cpp
  src/robotrainer_editor_section.cpp
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` and specify the list of
## source files we collected above in ``${SRC_FILES}``.
add_library(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Install rules

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#target_link_libraries(rosparam_handler ${catkin_LIBRARIES})
#install(TARGETS rosparam_handler LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
