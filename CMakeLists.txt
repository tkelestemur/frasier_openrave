cmake_minimum_required(VERSION 2.8.3)
project(frasier_openrave)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -march=native -mno-avx)
#set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  roslib
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Eigen REQUIRED COMPONENTS)
find_package(OpenRAVE REQUIRED)
find_package(Eigen3 REQUIRED)
set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
set(trajopt_LIBRARIES /home/tarik/develop/robotics/trajopt/build/lib/libjson.so /home/tarik/develop/robotics/trajopt/build/lib/libtrajopt.so
/home/tarik/develop/robotics/trajopt/build/lib/libsco.so /home/tarik/develop/robotics/trajopt/build/lib/libutils.so
/home/tarik/develop/robotics/trajopt/build/lib/libosgviewer.so)




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
 LIBRARIES frasier_openrave frasier_controller
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
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
  /home/tarik/develop/robotics/trajopt/ext
  /home/tarik/develop/robotics/trajopt/src
  /home/tarik/develop/robotics/trajopt/ext/json/include
)

### LIBRARIES ###

set(FRASIER_OPENRAVE_SRC src/frasier_openrave.cpp src/frasier_openrave_planner.cpp src/fraiser_openrave_perception.cpp)

add_library(${PROJECT_NAME} ${FRASIER_OPENRAVE_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${trajopt_LIBRARIES})
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++0x")
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(frasier_controller src/frasier_controller.cpp)
target_link_libraries(frasier_controller ${catkin_LIBRARIES})
#set_target_properties(frasier_controller PROPERTIES COMPILE_FLAGS "-std=c++0x")

### EXECUTABLES ###

#add_executable(test_planner nodes/frasier_openrave_test.cpp ${FRASIER_OPENRAVE_SRC})
#target_link_libraries(test_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} frasier_openrave ${OpenRAVE_BOTH_LIBRARIES})
#set_target_properties(test_openrave PROPERTIES COMPILE_FLAGS "-std=c++0x")

#add_executable(test_controller nodes/frasier_controller_test.cpp src/frasier_controller.cpp ${FRASIER_OPENRAVE_SRC})
#target_link_libraries(test_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} frasier_openrave ${OpenRAVE_BOTH_LIBRARIES})
#set_target_properties(test_controller PROPERTIES COMPILE_FLAGS "-std=c++0x")

#add_executable(test_perception nodes/frasier_perception_test.cpp ${FRASIER_OPENRAVE_SRC})
#target_link_libraries(test_perception ${catkin_LIBRARIES} ${Boost_LIBRARIES} frasier_openrave ${OpenRAVE_BOTH_LIBRARIES})
#set_target_properties(test_perception PROPERTIES COMPILE_FLAGS "-std=c++0x")

# add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_frasier_openrave.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
