cmake_minimum_required(VERSION 2.8.3)
project(posedetection_msgs)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  message_filters
  roscpp
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files(FILES
  Curve1D.msg
  Feature0D.msg
  Feature1D.msg
  ImageFeature0D.msg
  ImageFeature1D.msg
  Object6DPose.msg
  ObjectDetection.msg
)

add_service_files(FILES
  Detect.srv
  Feature0DDetect.srv
  Feature1DDetect.srv
  TargetObj.srv
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
    INCLUDE_DIRS include
)

add_executable(feature0d_view src/feature0d_view.cpp)
target_link_libraries(feature0d_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_view posedetection_msgs_gencpp)

add_executable(feature0d_to_image src/feature0d_to_image.cpp)
target_link_libraries(feature0d_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_to_image posedetection_msgs_gencpp)

add_executable(object_detection_aggregator src/object_detection_aggregator.cpp)
target_link_libraries(object_detection_aggregator ${catkin_LIBRARIES})
add_dependencies(object_detection_aggregator posedetection_msgs_gencpp)

install(TARGETS feature0d_view feature0d_to_image object_detection_aggregator
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
