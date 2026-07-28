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
project(speech_recognition_msgs)

set(_msgs
  Grammar.msg
  PhraseRule.msg
  SpeechRecognitionCandidates.msg
  Vocabulary.msg
)
set(_srvs
  SpeechRecognition.srv
)
set(_deps
  std_msgs
)

if("$ENV{ROS_VERSION}" EQUAL 1)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${_deps}
)

add_message_files(
  FILES
  ${_msgs}
)

add_service_files(
  FILES
  ${_srvs}
)

generate_messages(
  DEPENDENCIES ${_deps}
)

catkin_package(
    CATKIN_DEPENDS ${_deps} message_runtime
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
  find_package(ros_environment REQUIRED)
  foreach(_dep ${_deps})
    find_package(${_dep} REQUIRED)
  endforeach()

  rosidl_auto_generate_interfaces(ADD_LINTER_TESTS)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()
