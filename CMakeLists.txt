cmake_minimum_required(VERSION 2.8.3)
project(frasier_openrave)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -march=native -mno-avx)
#set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS}")

# Use Trajopt directory for global installation
set(TRAJOPT_DIR "$ENV{TRAJOPT_HOME}")

if(TRAJOPT_DIR)            
    message(STATUS "TRAJOPT_HOME at ${TRAJOPT_DIR}")
else()
    message(FATAL_ERROR "Undefined TRAJOPT_HOME env var.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
# openrave_catkin
# or_octomap_plugin
# moveit_core
  geometry_msgs
  roscpp
  sensor_msgs
  pcl_msgs
  roslib
  actionlib
  ecl_manipulators
  pcl_conversions
#  hacd_interface
)

## System dependencies are found with CMake's conventions
# find_package(Eigen REQUIRED COMPONENTS)
find_package(OpenRAVE REQUIRED)
find_package(Eigen3 REQUIRED)
set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
set(trajopt_LIBRARIES
	${TRAJOPT_DIR}/build/lib/libjson.so 
	${TRAJOPT_DIR}/build/lib/libtrajopt.so
	${TRAJOPT_DIR}/build/lib/libsco.so 
	${TRAJOPT_DIR}/build/lib/libutils.so
	${TRAJOPT_DIR}/build/lib/libosgviewer.so
)




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
 LIBRARIES frasier_openrave frasier_openrave_controller
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs pcl_conversions
 # DEPENDS Eigen
)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	SYSTEM
	${OpenRAVE_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OSG_INCLUDE_DIRS}
)

include_directories(
	${TRAJOPT_DIR}/ext
	${TRAJOPT_DIR}/src
	${TRAJOPT_DIR}/ext/json/include
)

### LIBRARIES ###

set(FRASIER_OPENRAVE_SRC 
	src/frasier_openrave.cpp 
	src/frasier_openrave_planner.cpp
  src/frasier_openrave_grasp_planner.cpp
	src/fraiser_openrave_perception.cpp
  src/frasier_openrave_utilities.cpp
)

add_library(${PROJECT_NAME} ${FRASIER_OPENRAVE_SRC})
target_link_libraries(${PROJECT_NAME} 
	${catkin_LIBRARIES} 
	${Boost_LIBRARIES} 
	${OpenRAVE_BOTH_LIBRARIES} 
	${trajopt_LIBRARIES}
)
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++0x")
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_controller 
	src/frasier_openrave_controller.cpp
)
target_link_libraries(${PROJECT_NAME}_controller ${catkin_LIBRARIES})
#set_target_properties(frasier_controller PROPERTIES COMPILE_FLAGS "-std=c++0x")

### EXECUTABLES ###

#add_executable(test_openrave tests/frasier_openrave_test.cpp src/frasier_openrave.cpp)
#target_link_libraries(test_openrave
#        ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${OpenRAVE_BOTH_LIBRARIES}
#        ${trajopt_LIBRARIES})
#set_target_properties(test_openrave PROPERTIES COMPILE_FLAGS "-std=c++0x")
#
#add_executable(test_grasp_planner tests/frasier_grasp_planner_test.cpp src/frasier_openrave.cpp)
#target_link_libraries(test_grasp_planner
#        ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${OpenRAVE_BOTH_LIBRARIES}
#        ${trajopt_LIBRARIES})
#set_target_properties(test_grasp_planner PROPERTIES COMPILE_FLAGS "-std=c++0x")

#add_executable(test_controller tests/frasier_controller_test.cpp src/frasier_controller.cpp ${FRASIER_OPENRAVE_SRC})
#target_link_libraries(test_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} frasier_openrave ${OpenRAVE_BOTH_LIBRARIES})
#set_target_properties(test_controller PROPERTIES COMPILE_FLAGS "-std=c++0x")

#add_executable(test_perception tests/frasier_perception_test.cpp ${FRASIER_OPENRAVE_SRC})
#target_link_libraries(test_perception ${catkin_LIBRARIES} ${Boost_LIBRARIES} frasier_openrave ${OpenRAVE_BOTH_LIBRARIES})
#set_target_properties(test_perception PROPERTIES COMPILE_FLAGS "-std=c++0x")

# add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

