#include "rmgo_core/teleop_remote.hpp"

#include <utility>

#include <rclcpp/qos.hpp>

namespace rmgo_core {

TeleopRemote::TeleopRemote(std::string cmd_vel_topic, double command_timeout)
: cmd_vel_topic_(std::move(cmd_vel_topic)),
  command_timeout_(command_timeout)
{
  command_buffer_.initRT(BufferedCommand{});
}

controller_interface::CallbackReturn TeleopRemote::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  node_ = node;

  if (!cmd_vel_subscriber_) {
    cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!enabled_.load()) {
          return;
        }

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

controller_interface::CallbackReturn TeleopRemote::activate()
{
  enabled_.store(true);
  command_buffer_.writeFromNonRT(BufferedCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TeleopRemote::deactivate()
{
  enabled_.store(false);
  command_buffer_.writeFromNonRT(BufferedCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

void TeleopRemote::set_enabled(bool enabled)
{
  enabled_.store(enabled);
  if (!enabled) {
    command_buffer_.writeFromNonRT(BufferedCommand{});
  }
}

RemoteCommand TeleopRemote::calculate(const rclcpp::Time & time)
{
  if (!enabled_.load()) {
    return {};
  }

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

}  // namespace rmgo_core
