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
project(jsk_common_msgs)
if("$ENV{ROS_VERSION}" EQUAL 1)
find_package(catkin REQUIRED)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()
