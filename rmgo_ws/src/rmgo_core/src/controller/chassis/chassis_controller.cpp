#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/pid/pid_calculator.hpp"

namespace rmgo_core::controller::chassis
{

class ChassisController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    target_controller_name_ = auto_declare<std::string>("target_controller_name", "chassis_power_controller");
    yaw_joint_name_ = auto_declare<std::string>("yaw_joint_name", "yaw_joint");
    yaw_state_interface_name_ =
        auto_declare<std::string>("yaw_state_interface_name", hardware_interface::HW_IF_POSITION);
    command_source_frame_ = auto_declare<std::string>("command_source_frame", "yaw");
    trace_commands_ = auto_declare<bool>("trace_commands", false);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string target_controller_name = get_target_controller_name();
    config.names.reserve(chassis_command_suffixes.size());
    for (const char* suffix : chassis_command_suffixes)
    {
      config.names.push_back(target_controller_name + "/" + suffix);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(get_yaw_state_interface_name());
    return config;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override
  {
    reset_references();

    const std::string controller_name = get_node()->get_name();
    return {
      std::make_shared<hardware_interface::CommandInterface>(controller_name, remote_command_suffixes[0],
                                                             &remote_command_reference_[0]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, remote_command_suffixes[1],
                                                             &remote_command_reference_[1]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, remote_command_suffixes[2],
                                                             &remote_command_reference_[2]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, remote_command_suffixes[3],
                                                             &remote_command_reference_[3]),
    };
  }

  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list() override
  {
    return {};
  }

  bool on_set_chained_mode(bool chained_mode) override
  {
    (void)chained_mode;
    return true;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    target_controller_name_ = get_target_controller_name();
    yaw_joint_name_ = get_string_parameter_or("yaw_joint_name", yaw_joint_name_);
    yaw_state_interface_name_ = get_string_parameter_or("yaw_state_interface_name", yaw_state_interface_name_);
    command_source_frame_ = get_string_parameter_or("command_source_frame", command_source_frame_);
    trace_commands_.store(get_node()->get_parameter("trace_commands").as_bool());

    if (target_controller_name_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (yaw_joint_name_.empty() || yaw_state_interface_name_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "yaw joint state interface must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (command_source_frame_ != "yaw" && command_source_frame_ != "base_link")
    {
      RCLCPP_ERROR(get_node()->get_logger(), "command_source_frame must be 'yaw' or 'base_link'");
      return controller_interface::CallbackReturn::ERROR;
    }

    declare_parameter_if_missing<double>("twist_angular", std::numbers::pi / 6.0);
    declare_parameter_if_missing<double>("follow_velocity_feedforward", 0.0);
    twist_angular_ = get_node()->get_parameter("twist_angular").as_double();
    follow_velocity_feedforward_ = get_node()->get_parameter("follow_velocity_feedforward").as_double();

    auto& node = *get_node();
    follow_pid_ = rmgo_core::pid::make_pid_calculator(node, "follow_", 0.0, 0.0, 0.0);

    reset_references();
    reset_pid();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    if (command_interfaces_.size() != chassis_command_suffixes.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                   chassis_command_suffixes.size(), command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (state_interfaces_.size() != 1)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected yaw state interface, got %zu", state_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    reset_references();
    reset_pid();
    last_mode_ = Mode::kRaw;
    return write_command({ 0.0, 0.0, 0.0 }) ? controller_interface::CallbackReturn::SUCCESS :
                                              controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    reset_references();
    reset_pid();
    last_mode_ = Mode::kRaw;
    return write_command({ 0.0, 0.0, 0.0 }) ? controller_interface::CallbackReturn::SUCCESS :
                                              controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/) override
  {
    if (!is_in_chained_mode())
    {
      reset_references();
      reset_pid();
      last_mode_ = Mode::kRaw;
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) override
  {
    const RemoteCommand command{
      remote_command_reference_[0],
      remote_command_reference_[1],
      remote_command_reference_[2],
    };
    const Mode mode = mode_from_value(remote_command_reference_[3]);
    if (mode != last_mode_)
    {
      reset_pid();
      last_mode_ = mode;
      if (trace_commands_.load())
      {
        RCLCPP_INFO(get_node()->get_logger(), "[trace] chassis mode -> %s", mode_name(mode));
      }
    }

    const auto values = calculate_command(mode, command);
    if (trace_commands_.load() && ++update_trace_counter_ % 100 == 0)
    {
      RCLCPP_INFO(get_node()->get_logger(), "[trace] mode=%s out->%s: vx=%.3f vy=%.3f wz=%.3f", mode_name(mode),
                  target_controller_name_.c_str(), values[0], values[1], values[2]);
    }

    return write_command(values) ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

private:
  enum class Mode : std::uint8_t
  {
    kRaw = 0,
    kFollow = 1,
    kTwist = 2,
  };

  struct RemoteCommand
  {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
  };

  static constexpr std::array<const char*, 4> remote_command_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
    "mode",
  };

  static constexpr std::array<const char*, 3> chassis_command_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

  template <typename T>
  void declare_parameter_if_missing(const std::string& name, const T& default_value)
  {
    if (!get_node()->has_parameter(name))
    {
      get_node()->declare_parameter<T>(name, default_value);
    }
  }

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

  std::string get_yaw_state_interface_name() const
  {
    std::string joint_name = yaw_joint_name_;
    std::string interface_name = yaw_state_interface_name_;
    if (const auto node = get_node())
    {
      const auto joint_parameter = node->get_parameter("yaw_joint_name");
      const auto interface_parameter = node->get_parameter("yaw_state_interface_name");
      if (joint_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING && !joint_parameter.as_string().empty())
      {
        joint_name = joint_parameter.as_string();
      }
      if (interface_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
          !interface_parameter.as_string().empty())
      {
        interface_name = interface_parameter.as_string();
      }
    }
    return joint_name + "/" + interface_name;
  }

  std::string get_string_parameter_or(const std::string& name, const std::string& fallback) const
  {
    const auto parameter = get_node()->get_parameter(name);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING && !parameter.as_string().empty())
    {
      return parameter.as_string();
    }
    return fallback;
  }

  static Mode mode_from_value(double value)
  {
    if (!std::isfinite(value))
    {
      return Mode::kRaw;
    }

    switch (static_cast<std::uint8_t>(std::llround(value)))
    {
      case static_cast<std::uint8_t>(Mode::kFollow):
        return Mode::kFollow;
      case static_cast<std::uint8_t>(Mode::kTwist):
        return Mode::kTwist;
      case static_cast<std::uint8_t>(Mode::kRaw):
      default:
        return Mode::kRaw;
    }
  }

  static const char* mode_name(Mode mode)
  {
    switch (mode)
    {
      case Mode::kFollow:
        return "FOLLOW";
      case Mode::kTwist:
        return "TWIST";
      case Mode::kRaw:
      default:
        return "RAW";
    }
  }

  std::array<double, 3> calculate_command(Mode mode, const RemoteCommand& command)
  {
    const double yaw = read_yaw_position();
    std::array<double, 3> values = command_to_base_link(command, yaw);
    switch (mode)
    {
      case Mode::kFollow:
        values[2] = calculate_follow_angular_velocity(yaw, command.wz);
        break;
      case Mode::kTwist:
        values[2] = calculate_twist_angular_velocity(yaw);
        break;
      case Mode::kRaw:
      default:
        break;
    }
    return values;
  }

  std::array<double, 3> command_to_base_link(const RemoteCommand& command, double yaw) const
  {
    if (command_source_frame_ == "base_link")
    {
      return { command.vx, command.vy, command.wz };
    }

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    return {
      cos_yaw * command.vx - sin_yaw * command.vy,
      sin_yaw * command.vx + cos_yaw * command.vy,
      command.wz,
    };
  }

  double calculate_follow_angular_velocity(double yaw, double feedforward)
  {
    return follow_pid_.update(normalize_angle(yaw)) + feedforward + follow_velocity_feedforward_;
  }

  double calculate_twist_angular_velocity(double yaw)
  {
    constexpr std::array<double, 4> candidate_offsets{
      -std::numbers::pi / 4.0,
      std::numbers::pi / 4.0,
      3.0 * std::numbers::pi / 4.0,
      -3.0 * std::numbers::pi / 4.0,
    };

    double offset = 0.0;
    for (const double candidate : candidate_offsets)
    {
      if (std::abs(shortest_angular_distance(yaw, candidate)) < 0.79)
      {
        offset = candidate;
        break;
      }
    }

    const double desired_yaw =
        twist_angular_ * std::sin(2.0 * std::numbers::pi * steady_clock_.now().seconds()) + offset;
    return follow_pid_.update(-shortest_angular_distance(yaw, desired_yaw));
  }

  double read_yaw_position() const
  {
    if (state_interfaces_.empty())
    {
      return 0.0;
    }

    const std::optional<double> yaw = state_interfaces_[0].get_optional();
    return yaw.has_value() && std::isfinite(*yaw) ? *yaw : 0.0;
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double shortest_angular_distance(double from, double to)
  {
    return normalize_angle(to - from);
  }

  void reset_references()
  {
    remote_command_reference_.fill(0.0);
  }

  void reset_pid()
  {
    follow_pid_.reset();
  }

  bool write_command(const std::array<double, 3>& values)
  {
    for (std::size_t index = 0; index < values.size(); ++index)
    {
      if (!command_interfaces_[index].set_value(values[index]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to write reference command '%s/%s'",
                     target_controller_name_.c_str(), chassis_command_suffixes[index]);
        return false;
      }
    }
    return true;
  }

  std::string target_controller_name_;
  std::string yaw_joint_name_;
  std::string yaw_state_interface_name_;
  std::string command_source_frame_ = "yaw";
  double twist_angular_ = std::numbers::pi / 6.0;
  double follow_velocity_feedforward_ = 0.0;
  std::atomic_bool trace_commands_{ false };
  std::size_t update_trace_counter_ = 0;
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
  std::array<double, 4> remote_command_reference_{ 0.0, 0.0, 0.0, 0.0 };
  Mode last_mode_ = Mode::kRaw;
  rmgo_core::pid::PidCalculator follow_pid_;
};

}  // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(rmgo_core::controller::chassis::ChassisController,
                       controller_interface::ChainableControllerInterface)
