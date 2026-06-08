#include <algorithm>
#include <array>
#include <string>

#include <controller_interface/chainable_controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmgo_core::controller::chassis {

class ChassisPowerController : public controller_interface::ChainableControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override
  {
    target_controller_name_ =
      auto_declare<std::string>("target_controller_name", "omni_wheel_controller");
    power_limit_topic_ = auto_declare<std::string>("power_limit_topic", "/chassis_power_limit");
    default_power_limit_ = auto_declare<double>("default_power_limit", 80.0);
    nominal_power_limit_ = auto_declare<double>("nominal_power_limit", 80.0);
    min_power_scale_ = auto_declare<double>("min_power_scale", 0.0);
    power_limit_timeout_ = auto_declare<double>("power_limit_timeout", 0.5);
    power_limit_buffer_.writeFromNonRT(PowerLimitSample{});
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string target_controller_name = get_target_controller_name();
    config.names.reserve(chassis_command_suffixes.size());
    for (const char * suffix : chassis_command_suffixes) {
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

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override
  {
    reference_interfaces_.assign(3, 0.0);

    const std::string controller_name = get_node()->get_name();
    return {
      std::make_shared<hardware_interface::CommandInterface>(
        controller_name, chassis_command_suffixes[0], &reference_interfaces_[0]),
      std::make_shared<hardware_interface::CommandInterface>(
        controller_name, chassis_command_suffixes[1], &reference_interfaces_[1]),
      std::make_shared<hardware_interface::CommandInterface>(
        controller_name, chassis_command_suffixes[2], &reference_interfaces_[2]),
    };
  }

  std::vector<hardware_interface::StateInterface::SharedPtr>
  on_export_state_interfaces_list() override
  {
    return {};
  }

  bool on_set_chained_mode(bool chained_mode) override
  {
    (void)chained_mode;
    return true;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    target_controller_name_ = get_target_controller_name();
    power_limit_topic_ = get_node()->get_parameter("power_limit_topic").as_string();
    default_power_limit_ = get_node()->get_parameter("default_power_limit").as_double();
    nominal_power_limit_ = get_node()->get_parameter("nominal_power_limit").as_double();
    min_power_scale_ = get_node()->get_parameter("min_power_scale").as_double();
    power_limit_timeout_ = get_node()->get_parameter("power_limit_timeout").as_double();

    if (target_controller_name_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (nominal_power_limit_ <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "nominal_power_limit must be positive");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (min_power_scale_ < 0.0 || min_power_scale_ > 1.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "min_power_scale must be within [0.0, 1.0]");
      return controller_interface::CallbackReturn::ERROR;
    }

    power_limit_subscriber_.reset();
    if (!power_limit_topic_.empty()) {
      power_limit_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        power_limit_topic_,
        rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          power_limit_buffer_.writeFromNonRT(
            PowerLimitSample{std::max(0.0, msg->data), get_node()->now(), true});
        });
    }

    reset_references();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (command_interfaces_.size() != chassis_command_suffixes.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Expected %zu command interfaces, got %zu",
        chassis_command_suffixes.size(),
        command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    power_limit_buffer_.writeFromNonRT(
      PowerLimitSample{std::max(0.0, default_power_limit_), get_node()->now(), false});
    reset_references();
    return write_chassis_commands({0.0, 0.0, 0.0}) ? controller_interface::CallbackReturn::SUCCESS
                                                    : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    reset_references();
    return write_chassis_commands({0.0, 0.0, 0.0}) ? controller_interface::CallbackReturn::SUCCESS
                                                    : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    if (!is_in_chained_mode()) {
      reset_references();
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & /*period*/) override
  {
    const double power_scale = calculate_power_scale(time);
    return write_chassis_commands(
             {
               reference_interfaces_[0] * power_scale,
               reference_interfaces_[1] * power_scale,
               reference_interfaces_[2] * power_scale,
             }) ?
             controller_interface::return_type::OK :
             controller_interface::return_type::ERROR;
  }

private:
  struct PowerLimitSample
  {
    double value = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid = false;
  };

  static constexpr std::array<const char *, 3> chassis_command_suffixes = {
    "linear/x/velocity",
    "linear/y/velocity",
    "angular/z/velocity",
  };

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

  double calculate_power_scale(const rclcpp::Time & time)
  {
    double power_limit = std::max(0.0, default_power_limit_);
    const auto sample = *power_limit_buffer_.readFromRT();
    const bool sample_fresh =
      sample.valid &&
      (power_limit_timeout_ <= 0.0 || (time - sample.stamp).seconds() <= power_limit_timeout_);

    if (sample_fresh) {
      power_limit = sample.value;
    }

    return std::clamp(power_limit / nominal_power_limit_, min_power_scale_, 1.0);
  }

  bool write_chassis_commands(const std::array<double, 3> & commands)
  {
    for (std::size_t index = 0; index < commands.size(); ++index) {
      if (!command_interfaces_[index].set_value(commands[index])) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to write chained command '%s/%s'",
          target_controller_name_.c_str(),
          chassis_command_suffixes[index]);
        return false;
      }
    }
    return true;
  }

  std::string target_controller_name_;
  std::string power_limit_topic_;
  double default_power_limit_ = 80.0;
  double nominal_power_limit_ = 80.0;
  double min_power_scale_ = 0.0;
  double power_limit_timeout_ = 0.5;
  realtime_tools::RealtimeBuffer<PowerLimitSample> power_limit_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr power_limit_subscriber_;
};

}  // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
  rmgo_core::controller::chassis::ChassisPowerController,
  controller_interface::ChainableControllerInterface)
