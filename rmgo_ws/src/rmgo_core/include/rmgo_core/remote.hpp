#pragma once

#include <memory>

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

class RemoteBase {
public:
  virtual ~RemoteBase() = default;

  virtual controller_interface::CallbackReturn configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) = 0;

  virtual controller_interface::CallbackReturn activate() = 0;

  virtual controller_interface::CallbackReturn deactivate() = 0;

  virtual void set_enabled(bool enabled) = 0;

  virtual RemoteCommand calculate(const rclcpp::Time & time) = 0;
};

}  // namespace rmgo_core
