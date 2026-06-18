#pragma once

#include <array>

namespace rmgo_core::reference_interfaces {

inline constexpr const char* chassis_linear_x_velocity = "linear/x/velocity";
inline constexpr const char* chassis_linear_y_velocity = "linear/y/velocity";
inline constexpr const char* chassis_angular_z_velocity = "angular/z/velocity";
inline constexpr const char* chassis_mode = "mode";
inline constexpr const char* chassis_power_limit = "power_limit";

inline constexpr std::array chassis_interfaces{
    chassis_linear_x_velocity,
    chassis_linear_y_velocity,
    chassis_angular_z_velocity,
    chassis_mode,
};

inline constexpr const char* gimbal_yaw_velocity = "yaw/velocity";
inline constexpr const char* gimbal_pitch_velocity = "pitch/velocity";
inline constexpr const char* gimbal_enabled = "enabled";

inline constexpr std::array gimbal_interfaces{
    gimbal_yaw_velocity,
    gimbal_pitch_velocity,
    gimbal_enabled,
};

inline constexpr const char* shooter_mode = "mode";
inline constexpr const char* shooter_request_sequence = "request_sequence";

inline constexpr std::array shooter_mode_interfaces{
    shooter_mode,
};

inline constexpr std::array shooter_trigger_interfaces{
    shooter_mode,
    shooter_request_sequence,
};

inline constexpr std::array shooter_command_interfaces{
    shooter_mode,
    shooter_request_sequence,
};

} // namespace rmgo_core::reference_interfaces
