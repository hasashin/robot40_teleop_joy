cmake_minimum_required(VERSION 2.8.3)
project(robot40_teleop_joy)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  joy
  roscpp
  rospy
  sensor_msgs
)

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
  CATKIN_DEPENDS std_msgs geometry_msgs joy roscpp rospy sensor_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(robot40_telejoy_node src/robot40_telejoy_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot40_telejoy_node ${catkin_LIBRARIES})
