cmake_minimum_required(VERSION 3.0.2)
project(problib_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES DimensionDescription.msg PDFImpl.msg PDF.msg
)

generate_messages(
  DEPENDENCIES
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS message_runtime
  DEPENDS
)
