cmake_minimum_required(VERSION 2.8.3)
project(robotrainer_controllers)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    cob_base_controller_utils
    cob_omni_drive_controller
    controller_interface
    force_torque_sensor
    hardware_interface
    geometry_msgs
    realtime_tools
    rosparam_handler
    robotrainer_modalities
    robotrainer_helper_types
    tf2
    tf2_ros
    tf2_geometry_msgs
    leg_tracker
    iirob_led
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
else()
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})
endif()

generate_dynamic_reconfigure_options(
  cfg/FTSBaseController.cfg
  cfg/FTSAdaptiveForceController.cfg
  cfg/IntegralCompareCtrl.cfg
  cfg/PassiveBehaviorController.cfg
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (Cpp11CompilerFlag)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS controller_interface hardware_interface geometry_msgs realtime_tools cob_omni_drive_controller leg_tracker iirob_led robotrainer_modalities robotrainer_helper_types
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen
  )


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/fts_controller.cpp
  src/fts_base_controller.cpp
  src/fts_adaptive_force_controller.cpp
  src/integral_compare_ctrl.cpp
  src/passive_behavior_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robotrainer_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
