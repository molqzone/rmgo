#pragma once

#include <numbers>

#include <fast_tf/fast_tf.hpp>
#include <fast_tf/impl/link.hpp>

namespace rmgo_description {

inline constexpr double wheel_distance_x = 0.31794;
inline constexpr double wheel_distance_y = 0.31794;
inline constexpr double gimbal_center_height = 0.32059;

struct BaseLink : fast_tf::Link<BaseLink> {
  static constexpr char name[] = "base_link";
};

struct YawLink : fast_tf::Link<YawLink> {
  static constexpr char name[] = "yaw_link";
};

struct PitchLink : fast_tf::Link<PitchLink> {
  static constexpr char name[] = "pitch_link";
};

struct ImuLink : fast_tf::Link<ImuLink> {
  static constexpr char name[] = "imu_link";
};

struct MuzzleLink : fast_tf::Link<MuzzleLink> {
  static constexpr char name[] = "muzzle_link";
};

struct CameraLink : fast_tf::Link<CameraLink> {
  static constexpr char name[] = "camera_link";
};

struct LeftFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
  static constexpr char name[] = "left_front_wheel_link";
};

struct LeftBackWheelLink : fast_tf::Link<LeftBackWheelLink> {
  static constexpr char name[] = "left_back_wheel_link";
};

struct RightBackWheelLink : fast_tf::Link<RightBackWheelLink> {
  static constexpr char name[] = "right_back_wheel_link";
};

struct RightFrontWheelLink : fast_tf::Link<RightFrontWheelLink> {
  static constexpr char name[] = "right_front_wheel_link";
};

namespace detail {

inline Eigen::Isometry3d make_wheel_transform(double x, double y, double yaw, double angle)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d{x, y, 0.0};
  transform.linear() =
    (Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()} *
    Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()})
      .matrix();
  return transform;
}

}  // namespace detail

}  // namespace rmgo_description

template <>
struct fast_tf::Joint<rmgo_description::YawLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::BaseLink;

  void set_state(double angle) { angle_ = angle; }

  auto get_transform() const
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() =
      Eigen::Vector3d{0.0, 0.0, rmgo_description::gimbal_center_height};
    transform.linear() = Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitZ()}.matrix();
    return transform;
  }

private:
  double angle_ = 0.0;
};

template <>
struct fast_tf::Joint<rmgo_description::PitchLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::YawLink;

  void set_state(double angle) { angle_ = angle; }
  auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; }

private:
  double angle_ = 0.0;
};

template <>
struct fast_tf::Joint<rmgo_description::ImuLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::PitchLink;

  Eigen::Quaterniond transform{
    Eigen::AngleAxisd{std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()}};
};

template <>
struct fast_tf::Joint<rmgo_description::MuzzleLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::PitchLink;

  Eigen::Translation3d transform{0.059, 0.0, 0.0};
};

template <>
struct fast_tf::Joint<rmgo_description::CameraLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::PitchLink;

  Eigen::Translation3d transform{0.06603, 0.0, 0.082};
};

template <>
struct fast_tf::Joint<rmgo_description::LeftFrontWheelLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::BaseLink;

  void set_state(double angle)
  {
    transform = rmgo_description::detail::make_wheel_transform(
      rmgo_description::wheel_distance_x / 2.0,
      rmgo_description::wheel_distance_y / 2.0,
      std::numbers::pi / 4.0,
      angle);
  }

  Eigen::Isometry3d transform = rmgo_description::detail::make_wheel_transform(
    rmgo_description::wheel_distance_x / 2.0,
    rmgo_description::wheel_distance_y / 2.0,
    std::numbers::pi / 4.0,
    0.0);
};

template <>
struct fast_tf::Joint<rmgo_description::LeftBackWheelLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::BaseLink;

  void set_state(double angle)
  {
    transform = rmgo_description::detail::make_wheel_transform(
      -rmgo_description::wheel_distance_x / 2.0,
      rmgo_description::wheel_distance_y / 2.0,
      3.0 * std::numbers::pi / 4.0,
      angle);
  }

  Eigen::Isometry3d transform = rmgo_description::detail::make_wheel_transform(
    -rmgo_description::wheel_distance_x / 2.0,
    rmgo_description::wheel_distance_y / 2.0,
    3.0 * std::numbers::pi / 4.0,
    0.0);
};

template <>
struct fast_tf::Joint<rmgo_description::RightBackWheelLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::BaseLink;

  void set_state(double angle)
  {
    transform = rmgo_description::detail::make_wheel_transform(
      -rmgo_description::wheel_distance_x / 2.0,
      -rmgo_description::wheel_distance_y / 2.0,
      -3.0 * std::numbers::pi / 4.0,
      angle);
  }

  Eigen::Isometry3d transform = rmgo_description::detail::make_wheel_transform(
    -rmgo_description::wheel_distance_x / 2.0,
    -rmgo_description::wheel_distance_y / 2.0,
    -3.0 * std::numbers::pi / 4.0,
    0.0);
};

template <>
struct fast_tf::Joint<rmgo_description::RightFrontWheelLink> : fast_tf::ModificationTrackable {
  using Parent = rmgo_description::BaseLink;

  void set_state(double angle)
  {
    transform = rmgo_description::detail::make_wheel_transform(
      rmgo_description::wheel_distance_x / 2.0,
      -rmgo_description::wheel_distance_y / 2.0,
      -std::numbers::pi / 4.0,
      angle);
  }

  Eigen::Isometry3d transform = rmgo_description::detail::make_wheel_transform(
    rmgo_description::wheel_distance_x / 2.0,
    -rmgo_description::wheel_distance_y / 2.0,
    -std::numbers::pi / 4.0,
    0.0);
};

namespace rmgo_description {

using Tf = fast_tf::JointCollection<
  YawLink,
  PitchLink,
  ImuLink,
  MuzzleLink,
  CameraLink,
  LeftFrontWheelLink,
  LeftBackWheelLink,
  RightBackWheelLink,
  RightFrontWheelLink>;

}  // namespace rmgo_description
