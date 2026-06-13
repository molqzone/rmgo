#pragma once

#include <array>

namespace rmgo_core::io_state_interfaces {

inline constexpr const char* remote_dr16_ch0 = "remote/dr16/ch0";
inline constexpr const char* remote_dr16_ch1 = "remote/dr16/ch1";
inline constexpr const char* remote_dr16_ch2 = "remote/dr16/ch2";
inline constexpr const char* remote_dr16_ch3 = "remote/dr16/ch3";
inline constexpr const char* remote_dr16_s1 = "remote/dr16/s1";
inline constexpr const char* remote_dr16_s2 = "remote/dr16/s2";
inline constexpr const char* remote_dr16_online = "remote/dr16/online";

inline constexpr std::array remote_state_interfaces{
    remote_dr16_ch0, remote_dr16_ch1, remote_dr16_ch2,    remote_dr16_ch3,
    remote_dr16_s1,  remote_dr16_s2,  remote_dr16_online,
};

inline constexpr const char* gimbal_imu_orientation_w = "gimbal/imu/orientation.w";
inline constexpr const char* gimbal_imu_orientation_x = "gimbal/imu/orientation.x";
inline constexpr const char* gimbal_imu_orientation_y = "gimbal/imu/orientation.y";
inline constexpr const char* gimbal_imu_orientation_z = "gimbal/imu/orientation.z";
inline constexpr const char* gimbal_imu_angular_velocity_x = "gimbal/imu/angular_velocity.x";
inline constexpr const char* gimbal_imu_angular_velocity_y = "gimbal/imu/angular_velocity.y";
inline constexpr const char* gimbal_imu_angular_velocity_z = "gimbal/imu/angular_velocity.z";
inline constexpr const char* gimbal_imu_linear_acceleration_x = "gimbal/imu/linear_acceleration.x";
inline constexpr const char* gimbal_imu_linear_acceleration_y = "gimbal/imu/linear_acceleration.y";
inline constexpr const char* gimbal_imu_linear_acceleration_z = "gimbal/imu/linear_acceleration.z";
inline constexpr const char* gimbal_yaw_velocity_imu = "gimbal/yaw/velocity_imu";
inline constexpr const char* gimbal_pitch_velocity_imu = "gimbal/pitch/velocity_imu";

// This quaternion represents the transform from PitchLink to OdomImu.
inline constexpr std::array gimbal_imu_orientation_state_interfaces{
    gimbal_imu_orientation_w,
    gimbal_imu_orientation_x,
    gimbal_imu_orientation_y,
    gimbal_imu_orientation_z,
};

inline constexpr std::array gimbal_imu_state_interfaces{
    gimbal_imu_orientation_w,         gimbal_imu_orientation_x,
    gimbal_imu_orientation_y,         gimbal_imu_orientation_z,
    gimbal_imu_angular_velocity_x,    gimbal_imu_angular_velocity_y,
    gimbal_imu_angular_velocity_z,    gimbal_imu_linear_acceleration_x,
    gimbal_imu_linear_acceleration_y, gimbal_imu_linear_acceleration_z,
    gimbal_yaw_velocity_imu,          gimbal_pitch_velocity_imu,
};

inline constexpr const char* referee_online = "referee/online";
inline constexpr const char* referee_id = "referee/id";
inline constexpr const char* referee_game_stage = "referee/game/stage";
inline constexpr const char* referee_game_stage_remain_time = "referee/game/stage_remain_time";
inline constexpr const char* referee_hp = "referee/hp";
inline constexpr const char* referee_max_hp = "referee/max_hp";
inline constexpr const char* referee_shooter_cooling = "referee/shooter/cooling";
inline constexpr const char* referee_shooter_heat_limit = "referee/shooter/heat_limit";
inline constexpr const char* referee_shooter_bullet_allowance = "referee/shooter/bullet_allowance";
inline constexpr const char* referee_shooter_1_heat = "referee/shooter/1/heat";
inline constexpr const char* referee_shooter_2_heat = "referee/shooter/2/heat";
inline constexpr const char* referee_chassis_power_limit = "referee/chassis/power_limit";
inline constexpr const char* referee_chassis_power = "referee/chassis/power";
inline constexpr const char* referee_chassis_buffer_energy = "referee/chassis/buffer_energy";

inline constexpr std::array chassis_power_state_interfaces{
    referee_chassis_power,
    referee_chassis_buffer_energy,
    referee_chassis_power_limit,
};

inline constexpr std::array referee_state_interfaces{
    referee_online,
    referee_id,
    referee_game_stage,
    referee_game_stage_remain_time,
    referee_hp,
    referee_max_hp,
    referee_shooter_cooling,
    referee_shooter_heat_limit,
    referee_shooter_bullet_allowance,
    referee_shooter_1_heat,
    referee_shooter_2_heat,
    referee_chassis_power_limit,
    referee_chassis_power,
    referee_chassis_buffer_energy,
};

inline constexpr const char* referee_command_ui_clear_layer = "referee/command/ui/clear_layer";
inline constexpr const char* referee_command_ui_clear_all = "referee/command/ui/clear_all";
inline constexpr const char* referee_command_sequence = "referee/command/sequence";

inline constexpr std::array referee_command_interfaces{
    referee_command_ui_clear_layer,
    referee_command_ui_clear_all,
    referee_command_sequence,
};

} // namespace rmgo_core::io_state_interfaces
