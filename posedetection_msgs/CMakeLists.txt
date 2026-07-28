# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
# lint_cmake: -whitespace/indent
# Fall back to ROS1 when $ROS_VERSION is unset (pre-ros_environment distros,
# e.g. indigo); every ROS2 environment always sets it via ros_environment.
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
  cmake_minimum_required(VERSION 2.8.12)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.5)
endif()
project(posedetection_msgs)

set(_msgs
  Curve1D.msg
  Feature0D.msg
  Feature1D.msg
  ImageFeature0D.msg
  ImageFeature1D.msg
  Object6DPose.msg
  ObjectDetection.msg
)
set(_srvs
  Detect.srv
  Feature0DDetect.srv
  Feature1DDetect.srv
  TargetObj.srv
)
set(_deps
  std_msgs
  sensor_msgs
  geometry_msgs
)

if("$ENV{ROS_VERSION}" EQUAL 1)
find_package(catkin REQUIRED COMPONENTS roscpp ${_deps} cv_bridge message_generation message_filters)

include_directories(SYSTEM include
                           ${catkin_INCLUDE_DIRS})

add_message_files(FILES
  ${_msgs}
)

add_service_files(FILES
  ${_srvs}
)

# to compatible with fuerte directories
generate_messages(
  DEPENDENCIES ${_deps}
)

catkin_package(
    CATKIN_DEPENDS roscpp ${_deps} message_runtime
    DEPENDS
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

find_package(OpenCV)
add_executable(feature0d_view src/feature0d_view.cpp)
target_link_libraries(feature0d_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_view posedetection_msgs_gencpp)
add_executable(feature0d_to_image src/feature0d_to_image.cpp)
target_link_libraries(feature0d_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_to_image posedetection_msgs_gencpp)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros_environment REQUIRED)
  foreach(_dep ${_deps} cv_bridge message_filters)
    find_package(${_dep} REQUIRED)
  endforeach()

  rosidl_auto_generate_interfaces(ADD_LINTER_TESTS)

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
      ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
  endif()

  if(cpp_typesupport_target)
    find_package(OpenCV REQUIRED)
    set(_libs
      cv_bridge::cv_bridge
      rclcpp::rclcpp
      message_filters::message_filters
      ${OpenCV_LIBRARIES}
      "${cpp_typesupport_target}"
    )

    foreach(_exe feature0d_view feature0d_to_image)
      add_executable(${_exe} src/${_exe}.cpp)
      target_include_directories(${_exe} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      )
      target_link_libraries(${_exe} ${_libs})
    endforeach()
  endif()

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()

if(TARGET feature0d_view)
  # CATKIN_PACKAGE_*_DESTINATION are only defined once catkin_package()
  # (called above, in the ROS1 branch) has run.
  if("$ENV{ROS_VERSION}" EQUAL 1)
    set(_runtime_dest ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(_lib_dest ${CATKIN_PACKAGE_LIB_DESTINATION})
  elseif("$ENV{ROS_VERSION}" EQUAL 2)
    set(_runtime_dest lib/${PROJECT_NAME})
    set(_lib_dest lib)
  endif()

  install(TARGETS feature0d_view feature0d_to_image
    RUNTIME DESTINATION ${_runtime_dest}
    ARCHIVE DESTINATION ${_lib_dest}
    LIBRARY DESTINATION ${_lib_dest}
  )
endif()
