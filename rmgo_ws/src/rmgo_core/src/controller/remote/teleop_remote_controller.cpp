#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace rmgo_core
{

class TeleopRemoteController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    target_controller_name_ = auto_declare<std::string>("target_controller_name", "chassis_controller");
    command_buffer_.initRT(BufferedCommand{});
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string target_controller_name = get_target_controller_name();
    config.names.reserve(command_interface_suffixes.size());
    for (const char* suffix : command_interface_suffixes)
    {
      config.names.push_back(target_controller_name + "/" + suffix);
    }

    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::NONE,
      {},
    };
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    node_ = get_node();
    target_controller_name_ = get_target_controller_name();
    if (target_controller_name_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!node_->has_parameter("cmd_vel_topic"))
    {
      node_->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    }
    if (!node_->has_parameter("command_timeout"))
    {
      node_->declare_parameter<double>("command_timeout", 0.25);
    }
    if (!node_->has_parameter("trace_commands"))
    {
      node_->declare_parameter<bool>("trace_commands", false);
    }

    const std::string cmd_vel_topic = node_->get_parameter("cmd_vel_topic").as_string();
    const double command_timeout = node_->get_parameter("command_timeout").as_double();
    trace_commands_.store(node_->get_parameter("trace_commands").as_bool());
    if (cmd_vel_topic.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "cmd_vel_topic must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (cmd_vel_subscriber_ && cmd_vel_topic_ != cmd_vel_topic)
    {
      cmd_vel_subscriber_.reset();
    }

    cmd_vel_topic_ = cmd_vel_topic;
    command_timeout_ = command_timeout;

    // Subscribe cmd_vel for command buffering
    if (!cmd_vel_subscriber_)
    {
      cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          cmd_vel_topic_, rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            command_buffer_.writeFromNonRT(BufferedCommand{
                msg->linear.x,
                msg->linear.y,
                msg->angular.z,
                steady_clock_.now(),
                true,
            });
            if (trace_commands_.load() && received_trace_counter_.fetch_add(1) % 20 == 0)
            {
              RCLCPP_INFO(node_->get_logger(), "[trace] received cmd_vel: vx=%.3f vy=%.3f wz=%.3f", msg->linear.x,
                          msg->linear.y, msg->angular.z);
            }
          });
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    if (command_interfaces_.size() != command_interface_suffixes.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                   command_interface_suffixes.size(), command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    command_buffer_.writeFromNonRT(BufferedCommand{});
    return write_command({ 0.0, 0.0, 0.0 }) ? controller_interface::CallbackReturn::SUCCESS :
                                              controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    command_buffer_.writeFromNonRT(BufferedCommand{});
    return write_command({ 0.0, 0.0, 0.0 }) ? controller_interface::CallbackReturn::SUCCESS :
                                              controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) override
  {
    (void)time;
    const BufferedCommand command = *command_buffer_.readFromRT();
    const rclcpp::Time now = steady_clock_.now();
    const bool valid =
        command.valid && (command_timeout_ <= 0.0 || (now - command.stamp).seconds() <= command_timeout_);
    const auto values =
        valid ? std::array<double, 3>{ command.vx, command.vy, command.wz } : std::array<double, 3>{ 0.0, 0.0, 0.0 };

    if (trace_commands_.load() && ++update_trace_counter_ % 100 == 0)
    {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[trace] writing %s reference: vx=%.3f vy=%.3f wz=%.3f valid=%s age=%.3f timeout=%.3f",
                  target_controller_name_.c_str(), values[0], values[1], values[2], valid ? "true" : "false",
                  command.valid ? (now - command.stamp).seconds() : -1.0, command_timeout_);
    }

    return write_command(values) ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

private:
  struct BufferedCommand
  {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    rclcpp::Time stamp{ 0, 0, RCL_STEADY_TIME };
    bool valid = false;
  };

  static constexpr std::array<const char*, 3> command_interface_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

  std::string get_target_controller_name() const
  {
    if (const auto node = get_node())
    {
      const auto parameter = node->get_parameter("target_controller_name");
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING && !parameter.as_string().empty())
      {
        return parameter.as_string();
      }
    }

    return target_controller_name_;
  }

  bool write_command(const std::array<double, 3>& values)
  {
    for (std::size_t index = 0; index < values.size(); ++index)
    {
      if (!command_interfaces_[index].set_value(values[index]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to write reference command '%s/%s'",
                     target_controller_name_.c_str(), command_interface_suffixes[index]);
        return false;
      }
    }

    return true;
  }

  std::string target_controller_name_;
  std::string cmd_vel_topic_ = "/cmd_vel";
  double command_timeout_ = 0.25;
  std::atomic_bool trace_commands_{ false };
  std::atomic_size_t received_trace_counter_{ 0 };
  std::size_t update_trace_counter_ = 0;
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
  realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
};

}  // namespace rmgo_core

PLUGINLIB_EXPORT_CLASS(rmgo_core::TeleopRemoteController, controller_interface::ControllerInterface)
