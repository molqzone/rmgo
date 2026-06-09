#include <array>
#include <string>

#include <controller_interface/chainable_controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace rmgo_core::controller::chassis
{

class ChassisPowerController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    target_controller_name_ = auto_declare<std::string>("target_controller_name", "omni_wheel_controller");
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
    return {
      controller_interface::interface_configuration_type::NONE,
      {},
    };
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override
  {
    reset_references();

    const std::string controller_name = get_node()->get_name();
    return {
      std::make_shared<hardware_interface::CommandInterface>(controller_name, chassis_command_suffixes[0],
                                                             &chassis_reference_[0]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, chassis_command_suffixes[1],
                                                             &chassis_reference_[1]),
      std::make_shared<hardware_interface::CommandInterface>(controller_name, chassis_command_suffixes[2],
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
    if (target_controller_name_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    reset_references();
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

    reset_references();
    return write_chassis_commands({ 0.0, 0.0, 0.0 }) ? controller_interface::CallbackReturn::SUCCESS :
                                                       controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    reset_references();
    return write_chassis_commands({ 0.0, 0.0, 0.0 }) ? controller_interface::CallbackReturn::SUCCESS :
                                                       controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/) override
  {
    if (!is_in_chained_mode())
    {
      reset_references();
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) override
  {
    // RMCS keeps chassis power limiting policy inside ChassisPowerController itself.
    // This controller stays in the chain so rmgo matches that layering now, but it
    // intentionally forwards commands unchanged until referee, supercap and remote
    // inputs are wired into this controller instead of a temporary external source.
    return write_chassis_commands({ chassis_reference_[0], chassis_reference_[1], chassis_reference_[2] }) ?
               controller_interface::return_type::OK :
               controller_interface::return_type::ERROR;
  }

private:
  static constexpr std::array<const char*, 3> chassis_command_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

  void reset_references()
  {
    chassis_reference_.fill(0.0);
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

  bool write_chassis_commands(const std::array<double, 3>& commands)
  {
    for (std::size_t index = 0; index < commands.size(); ++index)
    {
      if (!command_interfaces_[index].set_value(commands[index]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to write chained command '%s/%s'",
                     target_controller_name_.c_str(), chassis_command_suffixes[index]);
        return false;
      }
    }
    return true;
  }

  std::string target_controller_name_;
  std::array<double, 3> chassis_reference_{ 0.0, 0.0, 0.0 };
};

}  // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(rmgo_core::controller::chassis::ChassisPowerController,
                       controller_interface::ChainableControllerInterface)
