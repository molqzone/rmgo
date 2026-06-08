#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace rmgo_core::controller::chassis {

class ChassisController : public controller_interface::ChainableControllerInterface {
public:
  ChassisController() = default;

  controller_interface::CallbackReturn on_init() override
  {
    wheel_joints_ = auto_declare<std::vector<std::string>>(
      "wheel_joints",
      std::vector<std::string>{
        "left_front_wheel_joint",
        "left_back_wheel_joint",
        "right_back_wheel_joint",
        "right_front_wheel_joint"});
    cmd_vel_topic_ = auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    command_interface_name_ =
      auto_declare<std::string>("command_interface_name", hardware_interface::HW_IF_VELOCITY);
    wheel_radius_ = auto_declare<double>("wheel_radius", 0.07);
    chassis_radius_x_ = auto_declare<double>("chassis_radius_x", 0.15897);
    chassis_radius_y_ = auto_declare<double>("chassis_radius_y", 0.15897);
    command_timeout_ = auto_declare<double>("command_timeout", 0.25);
    max_wheel_velocity_ = auto_declare<double>("max_wheel_velocity", 40.0);

    command_buffer_.initRT(TwistCommand{});
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(wheel_joints_.size());
    for (const auto & joint_name : wheel_joints_) {
      config.names.push_back(joint_name + "/" + command_interface_name_);
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

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override
  {
    reference_interfaces_.assign(3, 0.0);

    const std::string controller_name = get_node()->get_name();
    return {
      std::make_shared<hardware_interface::CommandInterface>(
        controller_name, "linear/x/velocity", &reference_interfaces_[0]),
      std::make_shared<hardware_interface::CommandInterface>(
        controller_name, "linear/y/velocity", &reference_interfaces_[1]),
      std::make_shared<hardware_interface::CommandInterface>(
        controller_name, "angular/z/velocity", &reference_interfaces_[2]),
    };
  }

  std::vector<hardware_interface::StateInterface::SharedPtr>
  on_export_state_interfaces_list() override
  {
    return {};
  }

  bool on_set_chained_mode(bool chained_mode) override
  {
    subscriber_enabled_.store(!chained_mode);
    return true;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
    cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
    command_interface_name_ = get_node()->get_parameter("command_interface_name").as_string();
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    chassis_radius_x_ = get_node()->get_parameter("chassis_radius_x").as_double();
    chassis_radius_y_ = get_node()->get_parameter("chassis_radius_y").as_double();
    command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
    max_wheel_velocity_ = get_node()->get_parameter("max_wheel_velocity").as_double();

    if (wheel_joints_.size() != 4) {
      RCLCPP_ERROR(get_node()->get_logger(), "wheel_joints must contain exactly 4 joints");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_radius_ <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "wheel_radius must be positive");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (chassis_radius_x_ < 0.0 || chassis_radius_y_ < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "chassis radii must be non-negative");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!cmd_vel_subscriber_) {
      cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          if (!subscriber_enabled_.load()) {
            return;
          }
          command_buffer_.writeFromNonRT(TwistCommand{
            msg->linear.x,
            msg->linear.y,
            msg->angular.z,
            get_node()->now(),
            true,
          });
        });
    }

    reset_references();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (command_interfaces_.size() != wheel_joints_.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Expected %zu command interfaces, got %zu",
        wheel_joints_.size(),
        command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    subscriber_enabled_.store(!is_in_chained_mode());
    reset_references();
    if (!write_wheel_commands({0.0, 0.0, 0.0, 0.0})) {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    subscriber_enabled_.store(false);
    reset_references();
    if (!write_wheel_commands({0.0, 0.0, 0.0, 0.0})) {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time,
    const rclcpp::Duration & /*period*/) override
  {
    const TwistCommand command = *command_buffer_.readFromRT();
    if (
      !command.valid ||
      (command_timeout_ > 0.0 && (time - command.stamp).seconds() > command_timeout_))
    {
      reset_references();
      return controller_interface::return_type::OK;
    }

    reference_interfaces_[0] = command.vx;
    reference_interfaces_[1] = command.vy;
    reference_interfaces_[2] = command.wz;
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    std::array<double, 4> wheel_commands = inverse_kinematics(
      reference_interfaces_[0],
      reference_interfaces_[1],
      reference_interfaces_[2]);
    constrain_wheel_commands(wheel_commands);

    return write_wheel_commands(wheel_commands) ? controller_interface::return_type::OK
                                                : controller_interface::return_type::ERROR;
  }

private:
  struct TwistCommand
  {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid = false;
  };

  std::array<double, 4> inverse_kinematics(double vx, double vy, double wz) const
  {
    constexpr double sqrt2 = 1.4142135623730951;
    const double lever_arm = chassis_radius_x_ + chassis_radius_y_;
    const double scale = -1.0 / (sqrt2 * wheel_radius_);

    return {
      scale * (-vx + vy + lever_arm * wz),
      scale * (-vx - vy + lever_arm * wz),
      scale * (vx - vy + lever_arm * wz),
      scale * (vx + vy + lever_arm * wz),
    };
  }

  void constrain_wheel_commands(std::array<double, 4> & wheel_commands) const
  {
    if (max_wheel_velocity_ <= 0.0) {
      return;
    }

    double max_command = 0.0;
    for (const double command : wheel_commands) {
      max_command = std::max(max_command, std::abs(command));
    }
    if (max_command <= max_wheel_velocity_) {
      return;
    }

    const double scale = max_wheel_velocity_ / max_command;
    for (double & command : wheel_commands) {
      command *= scale;
    }
  }

  void reset_references()
  {
    if (reference_interfaces_.size() < 3) {
      reference_interfaces_.assign(3, 0.0);
      return;
    }
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
    reference_interfaces_[2] = 0.0;
  }

  bool write_wheel_commands(const std::array<double, 4> & wheel_commands)
  {
    for (std::size_t index = 0; index < wheel_commands.size(); ++index) {
      if (!command_interfaces_[index].set_value(wheel_commands[index])) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to write %s command for joint %s",
          command_interface_name_.c_str(),
          wheel_joints_[index].c_str());
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> wheel_joints_;
  std::string cmd_vel_topic_;
  std::string command_interface_name_;
  double wheel_radius_ = 0.07;
  double chassis_radius_x_ = 0.15897;
  double chassis_radius_y_ = 0.15897;
  double command_timeout_ = 0.25;
  double max_wheel_velocity_ = 40.0;
  std::atomic_bool subscriber_enabled_{true};

  realtime_tools::RealtimeBuffer<TwistCommand> command_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
};

}  // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
  rmgo_core::controller::chassis::ChassisController,
  controller_interface::ChainableControllerInterface)
