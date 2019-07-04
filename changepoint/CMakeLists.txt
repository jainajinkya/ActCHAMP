cmake_minimum_required(VERSION 2.8.3)
project(changepoint)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
    endif()

find_package(Boost REQUIRED)
find_package(GSL REQUIRED)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  tf
  std_msgs
  articulation_msgs
  sensor_msgs
  geometry_msgs
  genmsg
  pluginlib
  visualization_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CPParams.msg
  DataPoint.msg
  ModelSegment.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectChangepoints.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  articulation_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ArticulationFitter
  DEPENDS Eigen3 Boost GSL
  CATKIN_DEPENDS roscpp rospy tf pluginlib message_runtime std_msgs articulation_msgs sensor_msgs geometry_msgs rosbag visualization_msgs
)

include_directories(
	include 
  ${EIGEN3_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIRS}
	)


## Plugins
add_library(ArticulationFitter 
  src/articulation.cpp  
  src/utils.cpp
  src/models/generic_model.cpp
  src/models/prismatic_model.cpp
  src/models/rigid_model.cpp
  src/models/factory.cpp
  src/models/rotational_model.cpp
  src/models/helical_model.cpp)

target_link_libraries(ArticulationFitter ${GSL_LIBRARIES})

add_library(Gauss1DFitter src/gauss1D.cpp)

## Nodes
# message(STATUS ${EIGEN3_INCLUDE_DIRS})

add_library(changepoint src/cp_detector.cpp)

add_executable(changepoint_server nodes/changepoint_server.cpp)
target_link_libraries(changepoint_server changepoint ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(changepoint_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(test_articulation test/test_articulation_models.cpp)
target_link_libraries(test_articulation ArticulationFitter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_articulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(demo_quality_analysis test/demo_quality_analysis.cpp)
target_link_libraries(demo_quality_analysis ArticulationFitter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(demo_quality_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

## TESTS
add_executable(test_visualization test/test_cp_visual.cpp )
target_link_libraries(test_visualization ArticulationFitter ${catkin_LIBRARIES})

## Installing Headers to be used by other packages
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

# install(TARGETS ArticulationFitter
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
