#pragma once

#include <concepts>

#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace rmgo_core {

struct RemoteCommand
{
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  bool valid = false;
};

template<typename Device>
concept RemoteDevice =
  std::default_initializable<Device> &&
  requires(
    Device device,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const rclcpp::Time & time)
  {
    { device.configure(node) } -> std::same_as<controller_interface::CallbackReturn>;
    { device.activate() } -> std::same_as<controller_interface::CallbackReturn>;
    { device.deactivate() } -> std::same_as<controller_interface::CallbackReturn>;
    { device.calculate(time) } -> std::same_as<RemoteCommand>;
  };

}  // namespace rmgo_core
