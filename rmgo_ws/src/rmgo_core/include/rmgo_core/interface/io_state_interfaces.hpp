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

} // namespace rmgo_core::io_state_interfaces
