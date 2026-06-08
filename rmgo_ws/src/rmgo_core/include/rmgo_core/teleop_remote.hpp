#pragma once

#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "rmgo_core/remote.hpp"

namespace rmgo_core {

class TeleopRemote {
public:
  TeleopRemote()
  {
    command_buffer_.initRT(BufferedCommand{});
  }

  controller_interface::CallbackReturn configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
  {
    node_ = node;

    if (!node_->has_parameter("cmd_vel_topic")) {
      node_->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    }
    if (!node_->has_parameter("command_timeout")) {
      node_->declare_parameter<double>("command_timeout", 0.25);
    }

    const std::string cmd_vel_topic = node_->get_parameter("cmd_vel_topic").as_string();
    const double command_timeout = node_->get_parameter("command_timeout").as_double();
    if (cmd_vel_topic.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "cmd_vel_topic must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (cmd_vel_subscriber_ && cmd_vel_topic_ != cmd_vel_topic) {
      cmd_vel_subscriber_.reset();
    }

    cmd_vel_topic_ = cmd_vel_topic;
    command_timeout_ = command_timeout;

    if (!cmd_vel_subscriber_) {
      cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          command_buffer_.writeFromNonRT(BufferedCommand{
            msg->linear.x,
            msg->linear.y,
            msg->angular.z,
            node_->now(),
            true,
          });
        });
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn activate()
  {
    command_buffer_.writeFromNonRT(BufferedCommand{});
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn deactivate()
  {
    command_buffer_.writeFromNonRT(BufferedCommand{});
    return controller_interface::CallbackReturn::SUCCESS;
  }

  RemoteCommand calculate(const rclcpp::Time & time)
  {
    const BufferedCommand command = *command_buffer_.readFromRT();
    if (
      !command.valid ||
      (command_timeout_ > 0.0 && (time - command.stamp).seconds() > command_timeout_))
    {
      return {};
    }

    return {
      command.vx,
      command.vy,
      command.wz,
      true,
    };
  }

private:
  struct BufferedCommand
  {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid = false;
  };

  std::string cmd_vel_topic_ = "/cmd_vel";
  double command_timeout_ = 0.25;
  realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
};

}  // namespace rmgo_core
