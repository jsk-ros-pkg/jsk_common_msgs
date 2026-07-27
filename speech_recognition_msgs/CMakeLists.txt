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
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()
