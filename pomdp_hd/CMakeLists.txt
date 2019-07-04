cmake_minimum_required(VERSION 2.8.3)
project(pomdp_hd)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
  articulation_msgs
  message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_message_files(
  FILES
  DataPoint.msg
)

# Generate services in the 'srv' folder
# add_service_files(
#  FILES
#  frame_data.srv
#)

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
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
  INCLUDE_DIRS src/pomdp_hd/include
  LIBRARIES pomdp_hd
  CATKIN_DEPENDS roscpp rospy tf std_msgs geometry_msgs articulation_msgs tf2_ros
                 tf2_geometry_msgs moveit_msgs message_runtime
  DEPENDS EIGEN3 Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src/pomdp_hd/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(problem
    # src/${PROJECT_NAME}/src/problem_definitions/example_1_linear_dynamics.cpp
    #src/${PROJECT_NAME}/src/problem_definitions/microwave_door.cpp
    # src/${PROJECT_NAME}/src/problem_definitions/drawer.cpp
    src/${PROJECT_NAME}/src/problem_definitions/stapler.cpp
    )

add_library(utils  src/${PROJECT_NAME}/src/utils.cpp)

add_library(simulator  src/${PROJECT_NAME}/src/simulator.cpp)

add_library(stapler
    src/${PROJECT_NAME}/src/dynamics_models/stapler/stapler_dynamics.cpp
    )

add_library(dynamics
    src/${PROJECT_NAME}/src/dynamics_models/linear_dynamics.cpp
    src/${PROJECT_NAME}/src/dynamics_models/generic_kinematic_pair.cpp
    src/${PROJECT_NAME}/src/dynamics_models/revolute_pair.cpp
    src/${PROJECT_NAME}/src/dynamics_models/prismatic_pair.cpp
    )
target_link_libraries(dynamics ${catkin_LIBRARIES} ${Boost_LIBRARIES} utils stapler)

add_library(filters
   src/${PROJECT_NAME}/src/filters/kalman_filter.cpp
   src/${PROJECT_NAME}/src/filters/ukf.cpp
 )
target_link_libraries(filters ${catkin_LIBRARIES} problem dynamics)

###############
##Executables##
###############
add_executable(test_filters src/${PROJECT_NAME}/tests/filter_test.cpp )
target_link_libraries(test_filters ${catkin_LIBRARIES} problem utils dynamics simulator filters)

add_executable(test_kp src/${PROJECT_NAME}/tests/kp_test.cpp)
target_link_libraries(test_kp ${catkin_LIBRARIES} problem utils dynamics simulator filters)
add_dependencies(test_kp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(stapler_test src/${PROJECT_NAME}/tests/stapler_test.cpp)
# target_link_libraries(stapler_test ${catkin_LIBRARIES} problem utils dynamics simulator filters)
target_link_libraries(stapler_test ${catkin_LIBRARIES} utils dynamics)
add_dependencies(stapler_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(feedback_publisher src/${PROJECT_NAME}/src/feedback_publisher.cpp)
target_link_libraries(feedback_publisher ${catkin_LIBRARIES} dynamics utils)
add_dependencies(feedback_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(bel_evolution src/${PROJECT_NAME}/src/belief_evolution.cpp)
# target_link_libraries(bel_evolution ${catkin_LIBRARIES} problem utils dynamics simulator filters)
# add_dependencies(bel_evolution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   src/pomdp_hd/cython/beliefEvolution
#   src/pomdp_hd/src/blqr
#   src/pomdp_hd/src/trajOpt
#   src/pomdp_hd/src/trajOpt_snopt
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pomdp_hd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
