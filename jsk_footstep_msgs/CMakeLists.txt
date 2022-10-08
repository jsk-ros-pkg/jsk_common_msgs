if($ENV{ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 2.8.12)
elseif($ENV{ROS_VERSION} EQUAL 2)
  cmake_minimum_required(VERSION 3.5)
endif()
project(jsk_footstep_msgs)

if($ENV{ROS_VERSION} EQUAL 1)
  # Load catkin and all dependencies required for this package
  # TODO: remove all from COMPONENTS that are not catkin packages.
  find_package(catkin REQUIRED COMPONENTS geometry_msgs actionlib_msgs message_generation)

  set(DURATION_TYPE "duration")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/action/PlanFootsteps.action.in
    ${CMAKE_CURRENT_SOURCE_DIR}/action/PlanFootsteps.action
  )
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/Footstep.msg.in
    ${CMAKE_CURRENT_SOURCE_DIR}/msg/Footstep.msg
  )

  add_message_files(
    DIRECTORY msg
    FILES Footstep.msg FootstepArray.msg
  )

  #add_service_files(
  #  FILES
  #)

  add_action_files(
    DIRECTORY action
    FILES PlanFootsteps.action ExecFootsteps.action
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES geometry_msgs actionlib_msgs
  )

  # TODO: fill in what other packages will need to use this package
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
      DEPENDS
      CATKIN_DEPENDS  geometry_msgs actionlib_msgs message_runtime
      INCLUDE_DIRS # TODO include
      LIBRARIES # TODO
  )
elseif($ENV{ROS_VERSION} EQUAL 2)
  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(actionlib_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  set(DURATION_TYPE "builtin_interfaces/Duration")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/action/PlanFootsteps.action.in
    ${CMAKE_CURRENT_SOURCE_DIR}/action/PlanFootsteps.action
  )
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/Footstep.msg.in
    ${CMAKE_CURRENT_SOURCE_DIR}/msg/Footstep.msg
  )

  set(msg_files
    msg/Footstep.msg
    msg/FootstepArray.msg
  )

  set(action_files
    action/ExecFootsteps.action
    action/PlanFootsteps.action
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${action_files}
    DEPENDENCIES
      actionlib_msgs
      builtin_interfaces
      geometry_msgs
      std_msgs
    ADD_LINTER_TESTS
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()

