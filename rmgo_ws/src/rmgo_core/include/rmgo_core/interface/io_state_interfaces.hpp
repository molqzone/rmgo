#pragma once

#include <array>

namespace rmgo_core::io_state_interfaces {

inline constexpr std::array remote_state_interfaces{
    "remote/dr16/ch0", "remote/dr16/ch1", "remote/dr16/ch2",    "remote/dr16/ch3",
    "remote/dr16/s1",  "remote/dr16/s2",  "remote/dr16/online",
};

// This quaternion represents the transform from PitchLink to OdomImu.
inline constexpr std::array gimbal_imu_orientation_state_interfaces{
    "gimbal/imu/orientation.w",
    "gimbal/imu/orientation.x",
    "gimbal/imu/orientation.y",
    "gimbal/imu/orientation.z",
};

inline constexpr std::array gimbal_imu_state_interfaces{
    "gimbal/imu/orientation.w",         "gimbal/imu/orientation.x",
    "gimbal/imu/orientation.y",         "gimbal/imu/orientation.z",
    "gimbal/imu/angular_velocity.x",    "gimbal/imu/angular_velocity.y",
    "gimbal/imu/angular_velocity.z",    "gimbal/imu/linear_acceleration.x",
    "gimbal/imu/linear_acceleration.y", "gimbal/imu/linear_acceleration.z",
    "gimbal/yaw/velocity_imu",          "gimbal/pitch/velocity_imu",
};

inline constexpr std::array referee_state_interfaces{
    "referee/chassis/power",   "referee/chassis/power_buffer", "referee/chassis/voltage",
    "referee/chassis/current", "referee/shooter/heat",         "referee/shooter/heat_limit",
    "referee/robot/hp",        "referee/robot/max_hp",         "referee/robot/level",
};

} // namespace rmgo_core::io_state_interfaces
