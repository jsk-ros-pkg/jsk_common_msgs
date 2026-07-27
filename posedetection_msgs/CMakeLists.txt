# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
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

install(TARGETS feature0d_view feature0d_to_image
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  foreach(_dep ${_deps})
    find_package(${_dep} REQUIRED)
  endforeach()

  set(msg_files)
  foreach(_f ${_msgs})
    list(APPEND msg_files "msg/${_f}")
  endforeach()

  set(srv_files)
  foreach(_f ${_srvs})
    list(APPEND srv_files "srv/${_f}")
  endforeach()

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES
      ${_deps}
    ADD_LINTER_TESTS
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
    )
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()
