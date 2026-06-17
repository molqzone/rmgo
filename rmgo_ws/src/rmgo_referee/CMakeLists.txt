cmake_minimum_required(VERSION 3.8)
project(rmgo_referee)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmgo_msg REQUIRED)
find_package(rmgo_utility REQUIRED)

add_executable(
  referee_node
  src/interaction.cpp
  src/node/referee_node.cpp
  src/protocol.cpp
  src/status.cpp
  src/ui/interaction_ui.cpp
  src/ui/profile.cpp
  src/ui/shape.cpp
)

target_compile_features(referee_node PUBLIC cxx_std_23)

ament_target_dependencies(
  referee_node
  rclcpp
  rmgo_msg
  rmgo_utility
)

target_include_directories(
  referee_node
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

install(TARGETS referee_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
