cmake_minimum_required(VERSION 2.8.3)
project(articulation_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation 
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArticulatedObjectMsg.msg
  ModelMsg.msg
  ParamMsg.msg
  TrackMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AlignModelSrv.srv
  ArticulatedObjectSrv.srv
  GetModelPriorSrv.srv
  SetModelPriorSrv.srv
  TrackModelSrv.srv
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
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pomdp_kc
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs sensor_msgs visualization_msgs
)

# ####################################
# ########DEPRECATED##################
# ####################################

# cmake_minimum_required(VERSION 2.4.6)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# # Set the build type.  Options are:
# #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
# #  Debug          : w/ debug symbols, w/o optimization
# #  Release        : w/o debug symbols, w/ optimization
# #  RelWithDebInfo : w/ debug symbols, w/ optimization
# #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# #set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild_init()

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# #uncomment if you have defined messages
# rosbuild_genmsg()
# #uncomment if you have defined services
# rosbuild_gensrv()

# #common commands for building c++ executables and libraries
# #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
# #target_link_libraries(${PROJECT_NAME} another_library)
# #rosbuild_add_boost_directories()
# #rosbuild_link_boost(${PROJECT_NAME} thread)
# #rosbuild_add_executable(example examples/example.cpp)
# #target_link_libraries(example ${PROJECT_NAME})
