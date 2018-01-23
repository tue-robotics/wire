cmake_minimum_required(VERSION 2.8.3)
project(problib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  armadillo_matrix
  genmsg
)

# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES DimensionDescription.msg PDFImpl.msg PDF.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES problib
  CATKIN_DEPENDS message_runtime armadillo_matrix
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

########## feature extraction ###########

add_library(problib
        src/pdfs/PDF.cpp
        src/pdfs/Gaussian.cpp
        src/pdfs/Uniform.cpp
        src/pdfs/Mixture.cpp
        src/pdfs/Hybrid.cpp
        src/pdfs/PMF.cpp
        src/conversions.cpp
        include/${PROJECT_NAME}/conversions.h
        include/${PROJECT_NAME}/datatypes.h
        include/${PROJECT_NAME}/globals.h
        include/${PROJECT_NAME}/pdfs/PDF.h
        include/${PROJECT_NAME}/pdfs/Gaussian.h
        include/${PROJECT_NAME}/pdfs/Uniform.h
        include/${PROJECT_NAME}/pdfs/Mixture.h
        include/${PROJECT_NAME}/pdfs/Hybrid.h
        include/${PROJECT_NAME}/pdfs/PMF.h)
target_link_libraries(problib ${catkin_LIBRARIES})
add_dependencies(problib ${${PROJECT_NAME}_EXPORTED_TARGETS})


# add test executable
add_executable(problib_test src/test.cpp)
target_link_libraries(problib_test problib)
