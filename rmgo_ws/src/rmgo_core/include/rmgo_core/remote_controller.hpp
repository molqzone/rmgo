#pragma once

#include <array>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/remote.hpp"

namespace rmgo_core {

template<RemoteDevice Device>
class RemoteController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override
  {
    target_controller_name_ = auto_declare<std::string>("target_controller_name", "chassis_controller");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string target_controller_name = get_target_controller_name();
    config.names.reserve(remote_command_suffixes.size());
    for (const char * suffix : remote_command_suffixes) {
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

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    target_controller_name_ = get_target_controller_name();
    if (target_controller_name_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    return device_.configure(get_node());
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (command_interfaces_.size() != remote_command_suffixes.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Expected %zu command interfaces, got %zu",
        remote_command_suffixes.size(),
        command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    if (device_.activate() != controller_interface::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate remote device");
      return controller_interface::CallbackReturn::ERROR;
    }

    return write_command({0.0, 0.0, 0.0}) ? controller_interface::CallbackReturn::SUCCESS
                                           : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const bool zeroed = write_command({0.0, 0.0, 0.0});
    const auto deactivate_result = device_.deactivate();

    if (!zeroed) {
      return controller_interface::CallbackReturn::ERROR;
    }
    if (deactivate_result != controller_interface::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to deactivate remote device");
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & /*period*/) override
  {
    const auto command = device_.calculate(time);
    const auto values = command.valid ? std::array<double, 3>{command.vx, command.vy, command.wz}
                                      : std::array<double, 3>{0.0, 0.0, 0.0};

    return write_command(values) ? controller_interface::return_type::OK
                                 : controller_interface::return_type::ERROR;
  }

private:
  static constexpr std::array<const char *, 3> remote_command_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

  std::string get_target_controller_name() const
  {
    if (const auto node = get_node()) {
      const auto parameter = node->get_parameter("target_controller_name");
      if (
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
        !parameter.as_string().empty())
      {
        return parameter.as_string();
      }
    }

    return target_controller_name_;
  }

  bool write_command(const std::array<double, 3> & values)
  {
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (!command_interfaces_[index].set_value(values[index])) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to write reference command '%s/%s'",
          target_controller_name_.c_str(),
          remote_command_suffixes[index]);
        return false;
      }
    }

    return true;
  }

  std::string target_controller_name_;
  Device device_{};
};

}  // namespace rmgo_core
