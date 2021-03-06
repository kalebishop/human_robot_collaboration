# Copyright: (C) 2017 Social Robotics Lab - Yale University
# Author: Alessandro Roncone <alessandro.roncone@yale.edu>
# CopyPolicy: Released under the terms of the LGPLv2.1.

cmake_minimum_required(VERSION 2.8.3)
project(human_robot_collaboration_lib)

## Add C++14 support
if(CMAKE_COMPILER_IS_GNUCXX)
    # message("Setting C++14 and C++11 support..")
    set(CMAKE_CXX_FLAGS "-std=c++14")
    # message(${CMAKE_CXX_FLAGS})
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # message("Setting C++14 and C++11 support..")
    set(CMAKE_CXX_FLAGS "-std=c++14 -stdlib=libc++")
    # message(${CMAKE_CXX_FLAGS})
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             cmake_modules
             aruco
             aruco_ros
             aruco_msgs
             human_robot_collaboration_msgs
             cv_bridge
             image_transport
             rosconsole
             roscpp
             tf # Even if I'm not explicitly using it, adding tf is the only way
                # I found to avoid the undefined reference error to ros::init.
                # See http://goo.gl/GlbKgM
             trac_ik_lib
             intera_core_msgs
             intera_motion_msgs
             )

find_package(OpenCV 3.2 REQUIRED) # KB i changed this from 3.3
find_package(Eigen3     REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_utils robot_interface robot_perception
  DEPENDS Eigen trac_ik_lib aruco_ros aruco aruco_msgs
)

###########
## Build ##
###########

ADD_DEFINITIONS ( -Wall -Wextra -Wno-unused-parameter -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# For the record, robot_utils is decoupled from robot_interface
# so that we can link robot_perception against robot_utils only
# without the need to link against the full robot_interface lib,
# which would be an unnecessary overhead.
add_library(robot_utils     include/robot_utils/utils.h
                            include/robot_utils/thread_safe.h
                            include/robot_utils/rviz_publisher.h
                            include/robot_utils/particle_thread.h
                            include/robot_utils/hiro_trac_ik.h
                            include/robot_utils/ros_thread_image.h
                            src/robot_utils/utils.cpp
                            src/robot_utils/rviz_publisher.cpp
                            src/robot_utils/particle_thread.cpp
                            src/robot_utils/hiro_trac_ik.cpp
                            src/robot_utils/ros_thread_image.cpp)

add_library(robot_perception    include/robot_perception/cartesian_estimator.h
                                include/robot_perception/cartesian_estimator_hsv.h
                                include/robot_perception/hsv_detection.h
                                include/robot_perception/client_template.h
                                include/robot_perception/perception_client_impl.h
                                src/robot_perception/cartesian_estimator.cpp
                                src/robot_perception/cartesian_estimator_hsv.cpp
                                src/robot_perception/hsv_detection.cpp)

add_library(robot_interface include/robot_interface/robot_interface.h
                            include/robot_interface/gripper.h
                            include/robot_interface/arm_ctrl.h
                            include/robot_interface/arm_perception_ctrl.h
                            src/robot_interface/robot_interface.cpp
                            src/robot_interface/gripper.cpp
                            src/robot_interface/arm_ctrl.cpp
                            src/robot_interface/arm_perception_ctrl.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(robot_utils        ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_perception   robot_utils
                                    ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_interface    robot_utils
                                    ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_utils           ${catkin_LIBRARIES})

target_link_libraries(robot_perception      robot_utils
                                            ${OpenCV_LIBS}
                                            ${catkin_LIBRARIES})

target_link_libraries(robot_interface       robot_perception
                                            ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark libraries for installation
install (TARGETS robot_utils robot_interface robot_perception
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY   ${PROJECT_SOURCE_DIR}/include/robot_utils
                    ${PROJECT_SOURCE_DIR}/include/robot_interface
                    ${PROJECT_SOURCE_DIR}/include/robot_perception
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
