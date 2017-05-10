cmake_minimum_required(VERSION 2.8.3)
project(wire_viz)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  wire_msgs
  visualization_msgs
  problib
  geometry_msgs
)

# Declare a catkin package                                                                                                                                
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    tf
    wire_msgs
    visualization_msgs
    problib
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}") # C++ 11
set(CMAKE_CXX_FLAGS "-Wreturn-type ${CMAKE_CXX_FLAGS}") # Return type checking

add_library(wire_viz
    src/Visualizer.cpp
)

target_link_libraries(wire_viz ${catkin_LIBRARIES})

# ROS nodes
add_executable(visualization src/visualization.cpp) 
target_link_libraries(visualization wire_viz)
