cmake_minimum_required(VERSION 3.0.2)
project(wire_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  problib_msgs
  std_msgs
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
    problib_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS problib_msgs std_msgs message_runtime
)
