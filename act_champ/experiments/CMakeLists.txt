cmake_minimum_required(VERSION 2.8.3)
project(experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  std_msgs
  geometry_msgs
  roscpp
  rospy
  tf
  cmake_modules
  message_generation
  visualization_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)
  
find_package(Eigen3 REQUIRED)
#message("Eigen Lib path=${EIGEN3_INCLUDE_DIRS}")
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
  FILES
  frame_data.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   ar_track_alvar_msgs  std_msgs geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES experiments
   CATKIN_DEPENDS ar_track_alvar_msgs std_msgs geometry_msgs roscpp rospy tf message_runtime moveit_core moveit_ros_planning_interface visualization_msgs
	DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  	${catkin_INCLUDE_DIRS}
  	${EIGEN3_INCLUDE_DIRS}
    ${changepoint_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

## moveit Based arm shaowing
add_executable(shadow_arm src/shadow_arm.cpp)
target_link_libraries(shadow_arm ${catkin_LIBRARIES})

add_executable(target_frame_broadcaster src/target_frame_broadcaster.cpp)
target_link_libraries(target_frame_broadcaster ${catkin_LIBRARIES})

# SHADOW
add_executable(action_data_publisher_shadow src/action_data_publisher_shadow.cpp)
target_link_libraries(action_data_publisher_shadow ${catkin_LIBRARIES})

# TELEOP
add_executable(action_data_publisher_teleop src/action_data_publisher_teleop.cpp)
target_link_libraries(action_data_publisher_teleop ${catkin_LIBRARIES})

add_executable(pose_data_publisher src/pose_data_publisher.cpp)
target_link_libraries(pose_data_publisher ${catkin_LIBRARIES})

##### TESTS #########
add_executable(test_visualization test/test_visualization.cpp )
target_link_libraries(test_visualization ${catkin_LIBRARIES})
  
##### DEPRECATED #####
# add_executable(action_data_publisher src/action_data_publisher.cpp)
# target_link_libraries(action_data_publisher ${catkin_LIBRARIES})

# add_executable(force_data_filter src/force_data_filter.cpp)
# target_link_libraries(force_data_filter ${catkin_LIBRARIES})

