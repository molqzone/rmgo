#include <algorithm>
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

class OmniWheelController : public controller_interface::ChainableControllerInterface
{
public:
  using BaseLinkVelocityCommand = Eigen::Vector3d;
  using WheelState = Eigen::Vector4d;
  using WheelCommand = Eigen::Vector4d;
  using BaseLinkToWheelMatrix = Eigen::Matrix<double, 4, 3>;
  using WheelToBaseLinkMatrix = Eigen::Matrix<double, 3, 4>;

  controller_interface::CallbackReturn on_init() override
  {
    wheel_joints_ = auto_declare<std::vector<std::string>>(
        "wheel_joints", std::vector<std::string>{ "left_front_wheel_joint", "left_back_wheel_joint",
                                                  "right_back_wheel_joint", "right_front_wheel_joint" });
    command_interface_name_ = auto_declare<std::string>("command_interface_name", hardware_interface::HW_IF_VELOCITY);
    wheel_state_interface_name_ =
        auto_declare<std::string>("wheel_state_interface_name", hardware_interface::HW_IF_VELOCITY);
    wheel_radius_ = auto_declare<double>("wheel_radius", 0.07);
    chassis_radius_x_ = auto_declare<double>("chassis_radius_x", 0.15897);
    chassis_radius_y_ = auto_declare<double>("chassis_radius_y", 0.15897);
    max_wheel_velocity_ = auto_declare<double>("max_wheel_velocity", 40.0);
    trace_commands_ = auto_declare<bool>("trace_commands", false);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(wheel_joints_.size());
    for (const auto& joint_name : wheel_joints_)
    {
      config.names.push_back(joint_name + "/" + command_interface_name_);
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
      std::make_shared<hardware_interface::CommandInterface>(controller_name, base_link_velocity_suffixes[0],
                                                             &base_link_velocity_reference_[0]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, base_link_velocity_suffixes[1],
                                                             &base_link_velocity_reference_[1]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, base_link_velocity_suffixes[2],
                                                             &base_link_velocity_reference_[2]),
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
    wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
    command_interface_name_ = get_node()->get_parameter("command_interface_name").as_string();
    wheel_state_interface_name_ = get_node()->get_parameter("wheel_state_interface_name").as_string();
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    chassis_radius_x_ = get_node()->get_parameter("chassis_radius_x").as_double();
    chassis_radius_y_ = get_node()->get_parameter("chassis_radius_y").as_double();
    max_wheel_velocity_ = get_node()->get_parameter("max_wheel_velocity").as_double();
    trace_commands_ = get_node()->get_parameter("trace_commands").as_bool();

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

    base_link_to_wheel_ = make_base_link_to_wheel_matrix();
    wheel_to_base_link_ = make_wheel_to_base_link_matrix(base_link_to_wheel_);
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
    if (command_interfaces_.size() != wheel_joints_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu", wheel_joints_.size(),
                   command_interfaces_.size());
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
    return write_wheel_commands(WheelCommand::Zero()) ? controller_interface::CallbackReturn::SUCCESS :
                                                        controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    reset_references();
    reset_pid_calculators();
    return write_wheel_commands(WheelCommand::Zero()) ? controller_interface::CallbackReturn::SUCCESS :
                                                        controller_interface::CallbackReturn::ERROR;
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
    const BaseLinkVelocityCommand base_link_velocity_command{ base_link_velocity_reference_[0],
                                                              base_link_velocity_reference_[1],
                                                              base_link_velocity_reference_[2] };
    const std::optional<BaseLinkVelocityCommand> measured_velocity = measure_base_link_velocity();
    const BaseLinkVelocityCommand control_velocity = measured_velocity.has_value() ?
                                                         apply_pid(base_link_velocity_command, *measured_velocity) :
                                                         base_link_velocity_command;

    if (!measured_velocity.has_value())
    {
      reset_pid_calculators();
    }

    WheelCommand wheel_commands = inverse_kinematics(control_velocity);
    constrain_wheel_commands(wheel_commands);

    if (trace_commands_ && ++trace_counter_ % 100 == 0)
    {
      const BaseLinkVelocityCommand measured =
          measured_velocity.value_or(BaseLinkVelocityCommand::Constant(std::nan("")));
      RCLCPP_INFO(get_node()->get_logger(),
                  "[trace] omni wheel base_link_velocity=(%.3f %.3f %.3f) measured=(%.3f %.3f %.3f) "
                  "control=(%.3f %.3f %.3f) wheel_cmd=(%.3f %.3f %.3f %.3f)",
                  base_link_velocity_command.x(), base_link_velocity_command.y(), base_link_velocity_command.z(),
                  measured.x(), measured.y(), measured.z(), control_velocity.x(), control_velocity.y(),
                  control_velocity.z(), wheel_commands.x(), wheel_commands.y(), wheel_commands.z(), wheel_commands.w());
    }

    return write_wheel_commands(wheel_commands) ? controller_interface::return_type::OK :
                                                  controller_interface::return_type::ERROR;
  }

private:
  // ros2_control exchanges scalar reference interfaces, but the three values
  // are semantically a BaseLink-frame chassis velocity command, matching the
  // RMCS BaseLink::DirectionVector convention at this controller boundary.
  static constexpr std::array<const char*, 3> base_link_velocity_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

  BaseLinkToWheelMatrix make_base_link_to_wheel_matrix() const
  {
    const double lever_arm = chassis_radius_x_ + chassis_radius_y_;

    BaseLinkToWheelMatrix matrix;
    matrix << -1.0, 1.0, lever_arm, -1.0, -1.0, lever_arm, 1.0, -1.0, lever_arm, 1.0, 1.0, lever_arm;
    matrix *= -1.0 / (std::numbers::sqrt2 * wheel_radius_);
    return matrix;
  }

  static WheelToBaseLinkMatrix make_wheel_to_base_link_matrix(const BaseLinkToWheelMatrix& base_link_to_wheel)
  {
    return (base_link_to_wheel.transpose() * base_link_to_wheel).inverse() * base_link_to_wheel.transpose();
  }

  WheelCommand inverse_kinematics(const BaseLinkVelocityCommand& base_link_velocity_command) const
  {
    return base_link_to_wheel_ * base_link_velocity_command;
  }

  std::optional<BaseLinkVelocityCommand> measure_base_link_velocity() const
  {
    if (state_interfaces_.size() != wheel_joints_.size())
    {
      return std::nullopt;
    }

    WheelState wheel_state;
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index)
    {
      const std::optional<double> value = state_interfaces_[index].get_optional();
      if (!value.has_value() || !std::isfinite(*value))
      {
        return std::nullopt;
      }
      wheel_state[static_cast<Eigen::Index>(index)] = *value;
    }

    return wheel_to_base_link_ * wheel_state;
  }

  BaseLinkVelocityCommand apply_pid(const BaseLinkVelocityCommand& desired_command,
                                    const BaseLinkVelocityCommand& measured_command)
  {
    BaseLinkVelocityCommand control_command = desired_command;
    control_command.x() += linear_x_pid_.update(desired_command.x() - measured_command.x());
    control_command.y() += linear_y_pid_.update(desired_command.y() - measured_command.y());
    control_command.z() += angular_z_pid_.update(desired_command.z() - measured_command.z());
    return control_command;
  }

  void constrain_wheel_commands(WheelCommand& wheel_commands) const
  {
    if (max_wheel_velocity_ <= 0.0)
    {
      return;
    }

    const double max_command = wheel_commands.cwiseAbs().maxCoeff();
    if (max_command <= max_wheel_velocity_)
    {
      return;
    }

    wheel_commands *= max_wheel_velocity_ / max_command;
  }

  void reset_references()
  {
    base_link_velocity_reference_.fill(0.0);
  }

  void reset_pid_calculators()
  {
    linear_x_pid_.reset();
    linear_y_pid_.reset();
    angular_z_pid_.reset();
  }

  bool write_wheel_commands(const WheelCommand& wheel_commands)
  {
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index)
    {
      if (!command_interfaces_[index].set_value(wheel_commands[static_cast<Eigen::Index>(index)]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to write %s command for joint %s",
                     command_interface_name_.c_str(), wheel_joints_[index].c_str());
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> wheel_joints_;
  std::string command_interface_name_;
  std::string wheel_state_interface_name_;
  double wheel_radius_ = 0.07;
  double chassis_radius_x_ = 0.15897;
  double chassis_radius_y_ = 0.15897;
  double max_wheel_velocity_ = 40.0;
  bool trace_commands_ = false;
  std::size_t trace_counter_ = 0;
  std::array<double, 3> base_link_velocity_reference_{ 0.0, 0.0, 0.0 };
  BaseLinkToWheelMatrix base_link_to_wheel_ = BaseLinkToWheelMatrix::Zero();
  WheelToBaseLinkMatrix wheel_to_base_link_ = WheelToBaseLinkMatrix::Zero();
  rmgo_core::pid::PidCalculator linear_x_pid_;
  rmgo_core::pid::PidCalculator linear_y_pid_;
  rmgo_core::pid::PidCalculator angular_z_pid_;
};

}  // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(rmgo_core::controller::chassis::OmniWheelController,
                       controller_interface::ChainableControllerInterface)
