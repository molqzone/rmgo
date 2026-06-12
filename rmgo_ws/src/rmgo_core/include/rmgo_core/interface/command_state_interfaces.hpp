#pragma once

#include <array>

namespace rmgo_core::command_state_interfaces {

inline constexpr const char* chassis_linear_x_velocity = "command/chassis/linear/x/velocity";
inline constexpr const char* chassis_linear_y_velocity = "command/chassis/linear/y/velocity";
inline constexpr const char* chassis_angular_z_velocity = "command/chassis/angular/z/velocity";
inline constexpr const char* chassis_mode = "command/chassis/mode";

inline constexpr std::array chassis_interfaces{
    chassis_linear_x_velocity,
    chassis_linear_y_velocity,
    chassis_angular_z_velocity,
    chassis_mode,
};
inline constexpr auto chassis_command_interfaces = chassis_interfaces;

inline constexpr const char* gimbal_yaw_velocity = "command/gimbal/yaw/velocity";
inline constexpr const char* gimbal_pitch_velocity = "command/gimbal/pitch/velocity";
inline constexpr const char* gimbal_enabled = "command/gimbal/enabled";

inline constexpr std::array gimbal_interfaces{
    gimbal_yaw_velocity,
    gimbal_pitch_velocity,
    gimbal_enabled,
};
inline constexpr auto gimbal_command_interfaces = gimbal_interfaces;

inline constexpr const char* shooter_mode = "command/shooter/mode";
inline constexpr const char* shooter_sequence = "command/shooter/sequence";

inline constexpr std::array shooter_interfaces{
    shooter_mode,
    shooter_sequence,
};
inline constexpr auto shooter_command_interfaces = shooter_interfaces;

inline constexpr std::array all_interfaces{
    chassis_linear_x_velocity,
    chassis_linear_y_velocity,
    chassis_angular_z_velocity,
    chassis_mode,
    gimbal_yaw_velocity,
    gimbal_pitch_velocity,
    gimbal_enabled,
    shooter_mode,
    shooter_sequence,
};
inline constexpr auto all_command_interfaces = all_interfaces;

} // namespace rmgo_core::command_state_interfaces
