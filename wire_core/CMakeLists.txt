cmake_minimum_required(VERSION 3.0.2)
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

# TinyXML2
find_package(TinyXML2 REQUIRED)

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
    TinyXML2
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

## Build
add_compile_options(-Wreturn-type) # Return type checking
add_compile_options(-Wreorder)


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
    include/wire/util/ObjectModelParser.h
    include/wire/logic/Assignment.h
    include/wire/logic/AssignmentSet.h
    include/wire/logic/AssignmentMatrix.h
    include/wire/logic/Hypothesis.h
    include/wire/logic/HypothesesTree.h
    
)

target_link_libraries(wire ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

# ROS nodes
add_executable(wire_server src/world_model.cpp) 
target_link_libraries(wire_server wire)

add_executable(dummy_publisher src/dummy_evidence_publisher.cpp) 
target_link_libraries(dummy_publisher wire)

if(CATKIN_ENABLE_TESTING)
  add_library(test_plugin EXCLUDE_FROM_ALL SHARED test/test_plugin.cpp)
  target_link_libraries(test_plugin wire)

  catkin_add_gtest(test_parser test/test_parser.cpp)
  target_link_libraries(test_parser wire)
  add_dependencies(test_parser test_plugin)
endif()
