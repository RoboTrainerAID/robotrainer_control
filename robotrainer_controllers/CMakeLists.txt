cmake_minimum_required(VERSION 2.8.3)
project(robotrainer_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    visualization_msgs
    cob_base_controller_utils
    cob_omni_drive_controller
    controller_interface
    diagnostic_updater
    force_torque_sensor
    geometry_msgs
    hardware_interface
    iirob_led
    leg_tracker
    message_generation
    realtime_tools
    rosparam_handler
    robotrainer_modalities
    robotrainer_helper_types
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
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

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (Cpp11CompilerFlag)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

add_message_files(
  FILES 
  DriveModeParameters.msg 
  VirtualAxle.msg 
  CastorParams.msg 
  ICR.msg 
  A_and_b.msg
)

add_service_files(
  FILES
  GetRearWheelsState.srv
  SetRearWheelsState.srv
  ucdm_cmd.srv
)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/FTSBaseController.cfg
  cfg/FTSAdaptiveForceController.cfg
  cfg/IntegralCompareCtrl.cfg
  cfg/PassiveBehaviorController.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS controller_interface hardware_interface geometry_msgs realtime_tools cob_omni_drive_controller leg_tracker iirob_led robotrainer_modalities robotrainer_helper_types message_runtime
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
  src/odometry_controller.cpp
  src/passive_behavior_controller.cpp
  src/twist_controller.cpp

  src/undercarriage_drive_mode_service.cpp
  src/drive_modes/drive_mode_base_class.cpp
  src/drive_modes/omnidirectional.cpp
  src/drive_modes/differential.cpp
  src/drive_modes/differential_with_matrices.cpp
  src/drive_modes/ackermann.cpp
  src/drive_modes/ackermann_smoother.cpp
  src/drive_modes/synchro.cpp
  src/drive_modes/omni_with_max_accel.cpp
  src/drive_modes/pivot.cpp
  src/drive_modes/dead_reckon.cpp
  src/drive_modes/line2dof.cpp
  src/drive_modes/AxPLUSb.cpp
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

install(DIRECTORY include/${PROJECT_NAME}/ include/drive_modes/ include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robotrainer_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
