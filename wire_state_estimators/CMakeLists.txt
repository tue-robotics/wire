cmake_minimum_required(VERSION 3.0.2)
project(wire_state_estimators)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  wire_core
  pluginlib
)

# Declare a catkin package                                                                                                                                
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    wire_core
    pluginlib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build

add_library(position_estimator src/PositionFilter.cpp src/KalmanFilter.cpp)
add_library(multimodel_estimator src/MultiModelFilter.cpp src/PositionFilter.cpp)
add_library(discrete_estimator src/DiscreteFilter.cpp)
add_library(fixed_estimator src/FixedState.cpp)

target_link_libraries(position_estimator ${catkin_LIBRARIES})
target_link_libraries(multimodel_estimator ${catkin_LIBRARIES})
target_link_libraries(discrete_estimator ${catkin_LIBRARIES})
target_link_libraries(fixed_estimator ${catkin_LIBRARIES})
