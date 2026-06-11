#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/chassis_power_controller_config.hpp"
#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::chassis {

class ChassisPowerController
    : public controller_interface::ChainableControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        target_controller_name_ = params_.target_controller_name;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(params_.target_controller_name, chassis_command_suffixes);
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return build_individual_config(
            rmgo_core::io_state_interfaces::chassis_power_state_interfaces);
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(chassis_reference_);
        return make_reference_interfaces(chassis_command_suffixes, chassis_reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        target_controller_name_ = params_.target_controller_name;
        if (target_controller_name_.empty()) {
            logging::error("target_controller_name must not be empty");
            return controller_interface::CallbackReturn::ERROR;
        }
        reset_references(chassis_reference_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references(chassis_reference_);
        return write_chassis_commands({0.0, 0.0, 0.0})
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references(chassis_reference_);
        return write_chassis_commands({0.0, 0.0, 0.0})
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references(chassis_reference_);
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const double control_power_limit = calculate_control_power_limit();
        return write_chassis_commands(limit_chassis_command(control_power_limit))
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::array<const char*, 3> chassis_command_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
    };
    static constexpr std::size_t power_buffer_index = 1;
    static constexpr std::size_t power_limit_index = 2;

    double calculate_control_power_limit() {
        const std::optional<double> referee_power_limit = read_state(power_limit_index);
        if (!referee_power_limit.has_value()) {
            return params_.safety_power;
        }

        const double referee_buffer_energy =
            read_state(power_buffer_index).value_or(params_.buffer_threshold);
        const double extra_power =
            (referee_buffer_energy - params_.buffer_threshold) * params_.power_gain;
        return std::clamp(*referee_power_limit + extra_power, 0.0, params_.max_power_limit);
    }

    std::array<double, 3> limit_chassis_command(double control_power_limit) const {
        if (!std::isfinite(control_power_limit) || control_power_limit <= 0.0) {
            return {0.0, 0.0, 0.0};
        }

        const double power_ratio =
            std::clamp(control_power_limit / params_.full_speed_power_limit, 0.0, 1.0);
        const auto& [linear_x_reference, linear_y_reference, angular_z_reference] =
            chassis_reference_;
        return {
            clamp_abs(linear_x_reference, params_.max_linear_x_velocity * power_ratio),
            clamp_abs(linear_y_reference, params_.max_linear_y_velocity * power_ratio),
            clamp_abs(angular_z_reference, params_.max_angular_z_velocity * power_ratio),
        };
    }

    static double clamp_abs(double value, double limit) {
        if (!std::isfinite(value) || limit <= 0.0) {
            return 0.0;
        }
        return std::clamp(value, -limit, limit);
    }

    std::optional<double> read_state(std::size_t index) const {
        return read_finite_interface(state_interfaces_, index);
    }

    bool write_chassis_commands(const std::array<double, 3>& commands) {
        return write_safe_commands(
            command_interfaces_, commands, target_controller_name_, chassis_command_suffixes,
            "chained command");
    }

    std::string target_controller_name_;
    std::array<double, 3> chassis_reference_{0.0, 0.0, 0.0};
    std::shared_ptr<::chassis_power_controller::ParamListener> param_listener_;
    ::chassis_power_controller::Params params_;
};

} // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::chassis::ChassisPowerController,
    controller_interface::ChainableControllerInterface)
