cmake_minimum_required(VERSION 3.8)
project(rmgo_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CapacitorStatus.msg
  msg/ChassisStatus.msg
  msg/GimbalStatus.msg
  msg/RefereeStatus.msg
  msg/RemoteStatus.msg
  msg/ShooterStatus.msg
  msg/TargetStatus.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
