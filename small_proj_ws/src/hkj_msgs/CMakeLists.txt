cmake_minimum_required(VERSION 2.8.3)
project(hkj_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg/perception/
  FILES
  RoadCondition.msg
  RoadConditionVector.msg
  ObstacleRect.msg
)

add_message_files(
  DIRECTORY msg/pnc/
  FILES
  VehicleActuator.msg
  VehicleState.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)