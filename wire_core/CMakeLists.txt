cmake_minimum_required(VERSION 2.8.3)
project(wire_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  wire_msgs
  pluginlib
  problib
  std_srvs
  cmake_modules
)

# TinyXML
find_package(TinyXML REQUIRED)

# Declare a catkin package                                                                                                                                
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    tf
    wire_msgs
    pluginlib
    problib
    std_srvs
  DEPENDS
    TinyXML
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

## Build
add_compile_options(-std=c++0x) # C++ 11
add_compile_options(-Wreturn-type) # Return type checking


add_library(wire
    src/WorldModelROS.cpp
    src/models/FixedState.cpp
    src/core/datatypes.cpp
    src/core/ClassModel.cpp
    src/core/Property.cpp
    src/core/Evidence.cpp
    src/core/EvidenceSet.cpp
    src/core/PropertySet.cpp
    src/storage/SemanticObject.cpp
    src/storage/ObjectStorage.cpp
    src/storage/KnowledgeDatabase.cpp
    src/util/ObjectModelParser.cpp
    src/logic/Assignment.cpp
    src/logic/AssignmentSet.cpp
    src/logic/AssignmentMatrix.cpp
    src/logic/Hypothesis.cpp
    src/logic/HypothesesTree.cpp
    src/state_estimators/DiscreteFilter.cpp
    src/state_estimators/FixedState.cpp
    src/state_estimators/KalmanFilter.cpp
    src/state_estimators/DiscreteFilter.cpp
    src/state_estimators/MultiModelFilter.cpp
    src/state_estimators/PositionFilter.cpp
    include/wire/WorldModelROS.h
    include/wire/models/FixedState.h
    include/wire/core/datatypes.h
    include/wire/core/ClassModel.h
    include/wire/core/Property.h
    include/wire/core/Evidence.h
    include/wire/core/EvidenceSet.h
    include/wire/core/PropertySet.h
    include/wire/storage/SemanticObject.h
    include/wire/storage/ObjectStorage.h
    include/wire/storage/KnowledgeDatabase.h
    include/wire/state_estimators/DiscreteFilter.h
    include/wire/state_estimators/FixedState.h
    include/wire/state_estimators/KalmanFilter.h
    include/wire/state_estimators/MultiModelFilter.h
    include/wire/state_estimators/PositionFilter.h
    include/wire/util/ObjectModelParser.h
    include/wire/logic/Assignment.h
    include/wire/logic/AssignmentSet.h
    include/wire/logic/AssignmentMatrix.h
    include/wire/logic/Hypothesis.h
    include/wire/logic/HypothesesTree.h
    
)

target_link_libraries(wire ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# ROS nodes
add_executable(wire_server src/world_model.cpp) 
target_link_libraries(wire_server wire)

add_executable(dummy_publisher src/dummy_evidence_publisher.cpp) 
target_link_libraries(dummy_publisher wire)
