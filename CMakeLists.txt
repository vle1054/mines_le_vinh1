cmake_minimum_required(VERSION 2.8.3)
project(mines_le_vinh)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
## same as for the library above

add_executable(robot_draw_node src/mines_le_vinh.cpp)
target_link_libraries(robot_draw_node ${catkin_LIBRARIES})
add_dependencies(robot_draw_node beginner_tutorials_gen_cpp)
