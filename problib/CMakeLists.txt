cmake_minimum_required(VERSION 3.0.2)
project(problib)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  problib_msgs
  roscpp
)
find_package(Armadillo REQUIRED)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES problib
  CATKIN_DEPENDS problib_msgs
  DEPENDS ARMADILLO
)

###########
## Build ##
###########

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
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
target_link_libraries(problib armadillo ${catkin_LIBRARIES})
add_dependencies(problib ${catkin_EXPORTED_TARGETS})


# add test executable
add_executable(problib_test src/test.cpp)
target_link_libraries(problib_test problib)
