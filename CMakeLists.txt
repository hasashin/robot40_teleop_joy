cmake_minimum_required(VERSION 2.8.3)
project(robot40_teleop_joy)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
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
  CATKIN_DEPENDS geometry_msgs joy roscpp rospy sensor_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(teleop_joy_node src/teleop_joy_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(teleop_joy_node teleop_joy_node_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(teleop_joy_node ${catkin_LIBRARIES})
