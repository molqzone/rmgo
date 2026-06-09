cmake_minimum_required(VERSION 3.8)
project(rmgo_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

generate_parameter_library(
  chassis_controller_config src/controller/chassis/chassis_controller_config.yaml
)

generate_parameter_library(
  omni_wheel_controller_config src/controller/chassis/omni_wheel_controller_config.yaml
)

generate_parameter_library(
  teleop_remote_controller_config
  src/controller/remote/teleop_remote_controller_config.yaml
)

generate_parameter_library(
  simple_gimbal_controller_config
  src/controller/gimbal/simple_gimbal_controller_config.yaml
)

generate_parameter_library(
  gimbal_simulation_controller_config
  src/controller/gimbal/gimbal_simulation_controller_config.yaml
)

add_library(
  rmgo_chassis_controller SHARED
  src/controller/chassis/chassis_controller.cpp
  src/controller/chassis/chassis_power_controller.cpp
  src/controller/chassis/omni_wheel_controller.cpp
  src/controller/gimbal/gimbal_simulation_controller.cpp
  src/controller/gimbal/simple_gimbal_controller.cpp
  src/controller/remote/teleop_remote_controller.cpp
)

target_compile_features(rmgo_chassis_controller PUBLIC cxx_std_20)

ament_target_dependencies(
  rmgo_chassis_controller
  controller_interface
  angles
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

target_link_libraries(
  rmgo_chassis_controller chassis_controller_config omni_wheel_controller_config
  teleop_remote_controller_config simple_gimbal_controller_config
  gimbal_simulation_controller_config
)

target_include_directories(
  rmgo_chassis_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                 $<INSTALL_INTERFACE:include>
)

install(TARGETS rmgo_chassis_controller DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(controller_interface rmgo_core_plugins.xml)

install(FILES rmgo_core_plugins.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
