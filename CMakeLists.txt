cmake_minimum_required(VERSION 2.8.3)
project(uwds_physics_clients)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  uwds_msgs
  uwds
)

#find_package(Bullet REQUIRED COMPONENTS system)

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
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy uwds_msgs uwds
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
#add_library(${PROJECT_NAME}
   #ssrc/bullet3_client.cpp
   #src/gravity_filter.cpp
   #src/visibility_monitor.cpp
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME} ${BULLET_LIBRAIRIES}
#   ${catkin_LIBRARIES})
