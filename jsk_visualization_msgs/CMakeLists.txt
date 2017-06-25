cmake_minimum_required(VERSION 2.8.3)
project(jsk_visualization_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ObjectFitCommand.msg
  OverlayMenu.msg
  OverlayText.msg
  PictogramArray.msg
  Pictogram.msg
  RecordCommand.msg
  StringStamped.msg
  TransformableMarkerOperate.msg
)

add_service_files(
  FILES
  EusCommand.srv
  RequestMarkerOperate.srv
  Screenshot.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
