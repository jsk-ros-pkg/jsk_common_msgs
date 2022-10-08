if($ENV{ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 2.8.12)
elseif($ENV{ROS_VERSION} EQUAL 2)
  cmake_minimum_required(VERSION 3.5)
endif()
project(speech_recognition_msgs)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(
    FILES
    Grammar.msg
    PhraseRule.msg
    SpeechRecognitionCandidates.msg
    Vocabulary.msg
  )

  add_service_files(
    FILES
    SpeechRecognition.srv
  )

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
      CATKIN_DEPENDS std_msgs message_runtime
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
  find_package(std_msgs REQUIRED)

  set(msg_files
    msg/Grammar.msg
    msg/PhraseRule.msg
    msg/SpeechRecognitionCandidates.msg
    msg/Vocabulary.msg
  )

  set(srv_files
    srv/SpeechRecognition.srv
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES
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
