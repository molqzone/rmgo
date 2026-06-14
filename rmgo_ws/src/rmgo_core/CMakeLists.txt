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
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rmgo_description REQUIRED)
find_package(rmgo_utility REQUIRED)
find_package(std_msgs REQUIRED)

generate_parameter_library(
  chassis_controller_config config/controller/chassis/chassis_controller_config.yaml
)

generate_parameter_library(
  chassis_power_controller_config
  config/controller/chassis/chassis_power_controller_config.yaml
)

generate_parameter_library(
  omni_wheel_controller_config config/controller/chassis/omni_wheel_controller_config.yaml
)

generate_parameter_library(
  teleop_remote_controller_config
  config/controller/remote/teleop_remote_controller_config.yaml
)

generate_parameter_library(
  simple_gimbal_controller_config
  config/controller/gimbal/simple_gimbal_controller_config.yaml
)

generate_parameter_library(
  joint_velocity_pid_controller_config
  config/controller/pid/joint_velocity_pid_controller_config.yaml
)

generate_parameter_library(
  error_pid_controller_config config/controller/pid/error_pid_controller_config.yaml
)

generate_parameter_library(
  friction_wheel_controller_config
  config/controller/shooter/friction_wheel_controller_config.yaml
)

generate_parameter_library(
  heat_controller_config config/controller/shooter/heat_controller_config.yaml
)

generate_parameter_library(
  bullet_feeder_controller_17mm_config
  config/controller/shooter/bullet_feeder_controller_17mm_config.yaml
)

add_library(
  rmgo_chassis_controller SHARED
  src/controller/chassis/chassis_controller.cpp
  src/controller/chassis/chassis_power_controller.cpp
  src/controller/chassis/omni_wheel_controller.cpp
  src/controller/gimbal/simple_gimbal_controller.cpp
  src/controller/pid/error_pid_controller.cpp
  src/controller/pid/joint_velocity_pid_controller.cpp
  src/controller/remote/teleop_remote_controller.cpp
  src/controller/referee/referee_ui_controller.cpp
  src/controller/shooter/bullet_feeder_controller_17mm.cpp
  src/controller/shooter/friction_wheel_controller.cpp
  src/controller/shooter/heat_controller.cpp
  src/hardware/command_endpoint_gz_interface.cpp
  src/hardware/omni_infantry/omni_infantry_gz_interface.cpp
  src/hardware/refere/referee_gz_system_interface.cpp
  src/hardware/refere/referee_interaction.cpp
  src/hardware/refere/referee_protocol.cpp
  src/hardware/refere/referee_system_interface.cpp
  src/hardware/refere/referee_transfer_registry.cpp
  src/referee/ui/profile.cpp
  src/referee/ui/referee_ui.cpp
)

target_compile_features(rmgo_chassis_controller PUBLIC cxx_std_23)

ament_target_dependencies(
  rmgo_chassis_controller
  controller_interface
  angles
  geometry_msgs
  gz_sim_vendor
  gz_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rmgo_description
  rmgo_utility
  std_msgs
)

target_link_libraries(
  rmgo_chassis_controller
  bullet_feeder_controller_17mm_config
  chassis_controller_config
  chassis_power_controller_config
  error_pid_controller_config
  friction_wheel_controller_config
  gz-sim8::gz-sim8
  heat_controller_config
  joint_velocity_pid_controller_config
  omni_wheel_controller_config
  teleop_remote_controller_config
  simple_gimbal_controller_config
)

target_include_directories(
  rmgo_chassis_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

install(TARGETS rmgo_chassis_controller DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(controller_interface rmgo_core_plugins.xml)
pluginlib_export_plugin_description_file(gz_ros2_control rmgo_hardware_plugins.xml)
pluginlib_export_plugin_description_file(hardware_interface rmgo_hardware_plugins.xml)

install(FILES rmgo_core_plugins.xml rmgo_hardware_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
