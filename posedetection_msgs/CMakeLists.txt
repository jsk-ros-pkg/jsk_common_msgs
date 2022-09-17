cmake_minimum_required(VERSION 3.5)
project(posedetection_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

set(msg_files
  msg/Curve1D.msg
  msg/Feature0D.msg
  msg/Feature1D.msg
  msg/ImageFeature0D.msg
  msg/ImageFeature1D.msg
  msg/Object6DPose.msg
  msg/ObjectDetection.msg
)

set(srv_files
  srv/Detect.srv
  srv/Feature0DDetect.srv
  srv/Feature1DDetect.srv
  srv/TargetObj.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

if(cpp_typesupport_target)
  find_package(OpenCV REQUIRED)
  add_library(feature0d_view SHARED
    src/feature0d_view.cpp
  )

  target_include_directories(feature0d_view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

  target_link_libraries(feature0d_view
    cv_bridge::cv_bridge
    opencv_core
    opencv_highgui
    rclcpp::rclcpp
    rclcpp_components::component
    message_filters::message_filters
    "${cpp_typesupport_target}"
  )

  rclcpp_components_register_node(feature0d_view
    PLUGIN "posedetection_msgs::Feature0DView"
    EXECUTABLE feature0d_view_exe
  )

  add_library(feature0d_to_image SHARED
    src/feature0d_to_image.cpp
  )

  target_include_directories(feature0d_to_image PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

  target_link_libraries(feature0d_to_image
    cv_bridge::cv_bridge
    opencv_core
    rclcpp::rclcpp
    rclcpp_components::component
    message_filters::message_filters
    "${cpp_typesupport_target}"
  )

  rclcpp_components_register_node(feature0d_to_image
    PLUGIN "posedetection_msgs::Feature0DToImage"
    EXECUTABLE feature0d_to_image_exe
  )

  install(
    TARGETS feature0d_view
    EXPORT export_feature0d_view
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    TARGETS feature0d_to_image
    EXPORT export_feature0d_to_image
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()