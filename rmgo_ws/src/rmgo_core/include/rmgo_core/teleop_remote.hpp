#pragma once

#include <atomic>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "rmgo_core/remote.hpp"

namespace rmgo_core {

class TeleopRemote final : public RemoteBase {
public:
  TeleopRemote(std::string cmd_vel_topic, double command_timeout);

  controller_interface::CallbackReturn configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) override;

  controller_interface::CallbackReturn activate() override;

  controller_interface::CallbackReturn deactivate() override;

  void set_enabled(bool enabled) override;

  RemoteCommand calculate(const rclcpp::Time & time) override;

private:
  struct BufferedCommand
  {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid = false;
  };

  std::string cmd_vel_topic_;
  double command_timeout_ = 0.25;
  std::atomic_bool enabled_{false};
  realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
};

}  // namespace rmgo_core
