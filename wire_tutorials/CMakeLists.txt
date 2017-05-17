cmake_minimum_required(VERSION 2.8.3)
project(wire_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  wire_msgs
  problib
)

# Declare a catkin package                                                                                                                                
#catkin_package(
#  CATKIN_DEPENDS
#    roscpp
#    wire_msgs
#    problib
#)


include_directories(include ${catkin_INCLUDE_DIRS})

## Build

# ROS nodes
add_executable(get_world_state src/get_world_state.cpp) 
target_link_libraries(get_world_state ${catkin_LIBRARIES})

add_executable(generate_evidence src/generate_evidence.cpp) 
target_link_libraries(generate_evidence ${catkin_LIBRARIES})


#rosbuild_download_test_data(https://roboticssrv.wtb.tue.nl/svn/data/recorded/rosbags/world_model_tutorials/demo01.tar.gz data/demo01.tar.gz b87ae83fdd8eee9c0dcfbb81074474db)
#rosbuild_download_test_data(https://roboticssrv.wtb.tue.nl/svn/data/recorded/rosbags/world_model_tutorials/demo02.tar.gz data/demo02.tar.gz c7b73565d908227e3e00b084986309d1)
#rosbuild_download_test_data(https://roboticssrv.wtb.tue.nl/svn/data/recorded/rosbags/world_model_tutorials/demo03.tar.gz data/demo03.tar.gz 76906155deb8fccfeb76ff089341dd86)
#rosbuild_download_test_data(https://roboticssrv.wtb.tue.nl/svn/data/recorded/rosbags/world_model_tutorials/demo04.tar.gz data/demo04.tar.gz 4a79460232bd8605a53cc764a5024c71)
#rosbuild_download_test_data(https://roboticssrv.wtb.tue.nl/svn/data/recorded/rosbags/world_model_tutorials/demo05.tar.gz data/demo05.tar.gz acfd06ccc5293599dbf4d274af78c22e)
#rosbuild_download_test_data(https://roboticssrv.wtb.tue.nl/svn/data/recorded/rosbags/world_model_tutorials/demo_amigo.tar.gz data/demo_amigo.tar.gz a74e18691918c8a50f6896c486c8e8be)

#rosbuild_untar_file(data/demo01.tar.gz data/demo01 tests)
#rosbuild_untar_file(data/demo02.tar.gz data/demo02 tests)
#rosbuild_untar_file(data/demo03.tar.gz data/demo03 tests)
#rosbuild_untar_file(data/demo04.tar.gz data/demo04 tests)
#rosbuild_untar_file(data/demo05.tar.gz data/demo05 tests)
#rosbuild_untar_file(data/demo_amigo.tar.gz data/demo_amigo tests)
