cmake_minimum_required(VERSION 2.8.3)
project(wire_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  problib
)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
    ObjectEvidence.msg
    ObjectState.msg
    Property.msg
    WorldEvidence.msg
    WorldState.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    problib
)

catkin_package(
  CATKIN_DEPENDS problib std_msgs message_runtime
)
