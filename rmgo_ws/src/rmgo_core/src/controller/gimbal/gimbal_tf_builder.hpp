#pragma once

#include <eigen3/Eigen/Geometry>

#include <rmgo_description/tf_description.hpp>

namespace rmgo_core::gimbal {

struct GimbalTfState {
    double yaw = 0.0;
    double pitch = 0.0;
    Eigen::Quaterniond pitch_link_to_odom_imu = Eigen::Quaterniond::Identity();
};

inline void update_gimbal_tf(rmgo_description::Tf& tf, const GimbalTfState& state) {
    tf.set_state<rmgo_description::BaseLink, rmgo_description::YawLink>(state.yaw);
    tf.set_state<rmgo_description::YawLink, rmgo_description::PitchLink>(state.pitch);

    Eigen::Quaterniond normalized = state.pitch_link_to_odom_imu;
    normalized.normalize();
    tf.set_transform<rmgo_description::PitchLink, rmgo_description::OdomImu>(normalized);
}

inline void update_encoder_gimbal_tf(rmgo_description::Tf& tf, double yaw, double pitch) {
    tf.set_state<rmgo_description::BaseLink, rmgo_description::YawLink>(yaw);
    tf.set_state<rmgo_description::YawLink, rmgo_description::PitchLink>(pitch);

    const Eigen::Quaterniond base_to_pitch{
        Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}
        * Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()}};
    tf.set_transform<rmgo_description::PitchLink, rmgo_description::OdomImu>(
        base_to_pitch.conjugate());
}

} // namespace rmgo_core::gimbal
