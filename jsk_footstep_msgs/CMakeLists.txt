# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
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
project(jsk_footstep_msgs)

set(_msgs
  Footstep.msg
  FootstepArray.msg
)
set(_actions
  ExecFootsteps.action
  PlanFootsteps.action
)
set(_deps
  geometry_msgs
  actionlib_msgs
  std_msgs
)

if("$ENV{ROS_VERSION}" EQUAL 1)
  set(DURATION_TYPE "duration")
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  set(DURATION_TYPE "builtin_interfaces/Duration")
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/action/PlanFootsteps.action.in
  ${CMAKE_CURRENT_SOURCE_DIR}/action/PlanFootsteps.action
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/Footstep.msg.in
  ${CMAKE_CURRENT_SOURCE_DIR}/msg/Footstep.msg
)

if("$ENV{ROS_VERSION}" EQUAL 1)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS ${_deps} message_generation)

add_message_files(
  DIRECTORY msg
  FILES ${_msgs}
)

#add_service_files(
#  FILES
#)

add_action_files(
  DIRECTORY action
  FILES ${_actions}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES ${_deps}
)

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS
    CATKIN_DEPENDS  ${_deps} message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(ros_environment REQUIRED)
  foreach(_dep ${_deps})
    # actionlib_msgs is ROS1-only; ROS2 actions compile natively via rosidl
    if(NOT _dep STREQUAL "actionlib_msgs")
      find_package(${_dep} REQUIRED)
    endif()
  endforeach()

  rosidl_auto_generate_interfaces(ADD_LINTER_TESTS)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()
