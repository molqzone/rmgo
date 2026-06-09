#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <eigen3/Eigen/Dense>
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
  using ChassisCommand = Eigen::Vector3d;
  using WheelState = Eigen::Vector4d;
  using ChassisToWheelMatrix = Eigen::Matrix<double, 4, 3>;
  using WheelToChassisMatrix = Eigen::Matrix<double, 3, 4>;

  ChassisController() = default;

  controller_interface::CallbackReturn on_init() override
  {
    target_controller_name_ = auto_declare<std::string>("target_controller_name", "chassis_power_controller");
    wheel_joints_ = auto_declare<std::vector<std::string>>(
        "wheel_joints", std::vector<std::string>{ "left_front_wheel_joint", "left_back_wheel_joint",
                                                  "right_back_wheel_joint", "right_front_wheel_joint" });
    wheel_state_interface_name_ =
        auto_declare<std::string>("wheel_state_interface_name", hardware_interface::HW_IF_VELOCITY);
    wheel_radius_ = auto_declare<double>("wheel_radius", 0.07);
    chassis_radius_x_ = auto_declare<double>("chassis_radius_x", 0.15897);
    chassis_radius_y_ = auto_declare<double>("chassis_radius_y", 0.15897);
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
    config.names.reserve(wheel_joints_.size());
    for (const auto& joint_name : wheel_joints_)
    {
      config.names.push_back(joint_name + "/" + wheel_state_interface_name_);
    }
    return config;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override
  {
    reset_references();

    const std::string controller_name = get_node()->get_name();
    return {
      std::make_shared<hardware_interface::CommandInterface>(controller_name, "linear/x/velocity",
                                                             &chassis_reference_[0]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, "linear/y/velocity",
                                                             &chassis_reference_[1]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, "angular/z/velocity",
                                                             &chassis_reference_[2]),
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
    wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
    wheel_state_interface_name_ = get_node()->get_parameter("wheel_state_interface_name").as_string();
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    chassis_radius_x_ = get_node()->get_parameter("chassis_radius_x").as_double();
    chassis_radius_y_ = get_node()->get_parameter("chassis_radius_y").as_double();
    trace_commands_ = get_node()->get_parameter("trace_commands").as_bool();

    if (target_controller_name_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_joints_.size() != 4)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "wheel_joints must contain exactly 4 joints");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_state_interface_name_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "wheel_state_interface_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_radius_ <= 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "wheel_radius must be positive");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (chassis_radius_x_ < 0.0 || chassis_radius_y_ < 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "chassis radii must be non-negative");
      return controller_interface::CallbackReturn::ERROR;
    }
    if ((chassis_radius_x_ + chassis_radius_y_) <= 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "sum of chassis radii must be positive");
      return controller_interface::CallbackReturn::ERROR;
    }

    chassis_to_wheel_ = make_chassis_to_wheel_matrix();
    wheel_to_chassis_ = make_wheel_to_chassis_matrix(chassis_to_wheel_);

    auto& node = *get_node();
    linear_x_pid_ = rmgo_core::pid::make_pid_calculator(node, "linear_x_", 0.0, 0.0, 0.0);
    linear_y_pid_ = rmgo_core::pid::make_pid_calculator(node, "linear_y_", 0.0, 0.0, 0.0);
    angular_z_pid_ = rmgo_core::pid::make_pid_calculator(node, "angular_z_", 0.0, 0.0, 0.0);

    reset_references();
    reset_pid_calculators();
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
    if (state_interfaces_.size() != wheel_joints_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu state interfaces, got %zu", wheel_joints_.size(),
                   state_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    reset_references();
    reset_pid_calculators();
    if (!write_chassis_commands(ChassisCommand::Zero()))
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    reset_references();
    reset_pid_calculators();
    if (!write_chassis_commands(ChassisCommand::Zero()))
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/) override
  {
    if (!is_in_chained_mode())
    {
      reset_references();
      reset_pid_calculators();
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) override
  {
    const ChassisCommand desired_command{ chassis_reference_[0], chassis_reference_[1], chassis_reference_[2] };

    const std::optional<ChassisCommand> measured_command = measure_chassis_command();
    const ChassisCommand corrected_command =
        measured_command.has_value() ? apply_pid(desired_command, *measured_command) : desired_command;

    if (trace_commands_ && ++trace_counter_ % 100 == 0)
    {
      const ChassisCommand measured = measured_command.value_or(ChassisCommand::Constant(std::nan("")));
      RCLCPP_INFO(get_node()->get_logger(),
                  "[trace] chassis in=(%.3f %.3f %.3f) measured=(%.3f %.3f %.3f) out->%s=(%.3f %.3f %.3f)",
                  desired_command.x(), desired_command.y(), desired_command.z(), measured.x(), measured.y(),
                  measured.z(), target_controller_name_.c_str(), corrected_command.x(), corrected_command.y(),
                  corrected_command.z());
    }

    if (!measured_command.has_value())
    {
      reset_pid_calculators();
    }

    return write_chassis_commands(corrected_command) ? controller_interface::return_type::OK :
                                                       controller_interface::return_type::ERROR;
  }

private:
  static constexpr std::array<const char*, 3> chassis_command_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

  ChassisToWheelMatrix make_chassis_to_wheel_matrix() const
  {
    const double lever_arm = chassis_radius_x_ + chassis_radius_y_;

    ChassisToWheelMatrix matrix;
    matrix << -1.0, 1.0, lever_arm, -1.0, -1.0, lever_arm, 1.0, -1.0, lever_arm, 1.0, 1.0, lever_arm;
    matrix *= -1.0 / (std::numbers::sqrt2 * wheel_radius_);
    return matrix;
  }

  static WheelToChassisMatrix make_wheel_to_chassis_matrix(const ChassisToWheelMatrix& chassis_to_wheel)
  {
    return (chassis_to_wheel.transpose() * chassis_to_wheel).inverse() * chassis_to_wheel.transpose();
  }

  void reset_references()
  {
    chassis_reference_.fill(0.0);
  }

  void reset_pid_calculators()
  {
    linear_x_pid_.reset();
    linear_y_pid_.reset();
    angular_z_pid_.reset();
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

  std::optional<ChassisCommand> measure_chassis_command() const
  {
    if (state_interfaces_.size() != wheel_joints_.size())
    {
      return std::nullopt;
    }

    WheelState wheel_state;
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index)
    {
      const double value = state_interfaces_[index].get_value();
      if (!std::isfinite(value))
      {
        return std::nullopt;
      }
      wheel_state[static_cast<Eigen::Index>(index)] = value;
    }

    return wheel_to_chassis_ * wheel_state;
  }

  ChassisCommand apply_pid(const ChassisCommand& desired_command, const ChassisCommand& measured_command)
  {
    // Keep the desired command as feedforward, and use PID as a correction term.
    ChassisCommand corrected_command = desired_command;
    corrected_command.x() += linear_x_pid_.update(desired_command.x() - measured_command.x());
    corrected_command.y() += linear_y_pid_.update(desired_command.y() - measured_command.y());
    corrected_command.z() += angular_z_pid_.update(desired_command.z() - measured_command.z());
    return corrected_command;
  }

  bool write_chassis_commands(const ChassisCommand& commands)
  {
    for (std::size_t index = 0; index < chassis_command_suffixes.size(); ++index)
    {
      if (!command_interfaces_[index].set_value(commands[static_cast<Eigen::Index>(index)]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to write chained command '%s/%s'",
                     target_controller_name_.c_str(), chassis_command_suffixes[index]);
        return false;
      }
    }
    return true;
  }

  std::string target_controller_name_;
  std::vector<std::string> wheel_joints_;
  std::string wheel_state_interface_name_;
  double wheel_radius_ = 0.07;
  double chassis_radius_x_ = 0.15897;
  double chassis_radius_y_ = 0.15897;
  bool trace_commands_ = false;
  std::size_t trace_counter_ = 0;
  std::array<double, 3> chassis_reference_{ 0.0, 0.0, 0.0 };
  ChassisToWheelMatrix chassis_to_wheel_ = ChassisToWheelMatrix::Zero();
  WheelToChassisMatrix wheel_to_chassis_ = WheelToChassisMatrix::Zero();
  rmgo_core::pid::PidCalculator linear_x_pid_;
  rmgo_core::pid::PidCalculator linear_y_pid_;
  rmgo_core::pid::PidCalculator angular_z_pid_;
};

}  // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(rmgo_core::controller::chassis::ChassisController,
                       controller_interface::ChainableControllerInterface)
