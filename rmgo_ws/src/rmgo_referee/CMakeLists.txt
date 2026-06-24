cmake_minimum_required(VERSION 3.8)
project(rmgo_referee)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rmgo_msg REQUIRED)
find_package(rmgo_utility REQUIRED)

add_library(
  referee_node_component SHARED
  src/app/node.cpp
  src/app/ui/ui.cpp
  src/app/ui/omni_infantry.cpp
  src/app/ui/shape/shape.cpp
)

target_compile_features(referee_node_component PUBLIC cxx_std_23)

ament_target_dependencies(
  referee_node_component
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  realtime_tools
  rmgo_msg
  rmgo_utility
)

target_include_directories(
  referee_node_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

rclcpp_components_register_nodes(
  referee_node_component
  PLUGIN "rmgo_referee::RefereeNode"
)

add_executable(referee_node src/app/main.cpp)
target_compile_features(referee_node PUBLIC cxx_std_23)
target_link_libraries(referee_node referee_node_component)
ament_target_dependencies(referee_node rclcpp)
target_include_directories(
  referee_node
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

install(
  TARGETS referee_node_component referee_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
