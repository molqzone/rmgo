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
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/chassis_power_controller_config.hpp"
#include "rmgo_core/interface/io_state_interfaces.hpp"

namespace rmgo_core::controller::chassis {

class ChassisPowerController : public controller_interface::ChainableControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override {
        param_listener_ = std::make_shared<::chassis_power_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        target_controller_name_ = params_.target_controller_name;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        const std::string target_controller_name = params_.target_controller_name;
        config.names.reserve(chassis_command_suffixes.size());
        for (const char* suffix : chassis_command_suffixes) {
            config.names.push_back(target_controller_name + "/" + suffix);
        }

        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(rmgo_core::io_state_interfaces::chassis_power_state_interfaces.size());
        for (const char* name : rmgo_core::io_state_interfaces::chassis_power_state_interfaces) {
            config.names.emplace_back(name);
        }
        return config;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references();

        const std::string controller_name = get_node()->get_name();
        return {
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, chassis_command_suffixes[0], &chassis_reference_[0]),
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, chassis_command_suffixes[1], &chassis_reference_[1]),
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, chassis_command_suffixes[2], &chassis_reference_[2]),
        };
    }

    std::vector<hardware_interface::StateInterface::SharedPtr>
        on_export_state_interfaces_list() override {
        return {};
    }

    bool on_set_chained_mode(bool /*chained_mode*/) override { return true; }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        params_ = param_listener_->get_params();
        target_controller_name_ = params_.target_controller_name;
        if (target_controller_name_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "target_controller_name must not be empty");
            return controller_interface::CallbackReturn::ERROR;
        }
        reset_references();
        reset_power_state();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (command_interfaces_.size() != chassis_command_suffixes.size()) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                chassis_command_suffixes.size(), command_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        const std::size_t expected_state_interfaces =
            rmgo_core::io_state_interfaces::chassis_power_state_interfaces.size();
        if (state_interfaces_.size() != expected_state_interfaces) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu chassis power state interfaces, got %zu",
                expected_state_interfaces, state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        reset_references();
        reset_power_state();
        return write_chassis_commands({0.0, 0.0, 0.0})
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references();
        reset_power_state();
        return write_chassis_commands({0.0, 0.0, 0.0})
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references();
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
        const double scale = calculate_power_scale(period);
        return write_chassis_commands({
                   scale * chassis_reference_[0],
                   scale * chassis_reference_[1],
                   scale * chassis_reference_[2],
               })
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::array<const char*, 3> chassis_command_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
    };
    static constexpr std::size_t power_index = 0;
    static constexpr std::size_t power_buffer_index = 1;
    static constexpr std::size_t power_limit_index = 2;
    static constexpr double full_scale = 1.0;

    void reset_references() { chassis_reference_.fill(0.0); }

    void reset_power_state() {
        virtual_buffer_energy_ = params_.virtual_buffer_energy_limit;
        chassis_power_limit_expected_ = params_.fallback_power_limit;
    }

    double calculate_power_scale(const rclcpp::Duration& period) {
        const double referee_power_limit =
            read_state(power_limit_index).value_or(params_.fallback_power_limit);
        const double measured_power = read_state(power_index).value_or(0.0);
        const double referee_buffer_energy =
            read_state(power_buffer_index).value_or(params_.virtual_buffer_energy_limit);

        update_virtual_buffer_energy(period, referee_buffer_energy, measured_power);

        chassis_power_limit_expected_ = referee_power_limit;
        const double control_power_limit =
            (referee_power_limit + params_.excess_power_limit) * virtual_buffer_ratio();

        double scale = control_power_limit / referee_power_limit;
        if (measured_power > control_power_limit && control_power_limit > 0.0) {
            scale = std::min(scale, control_power_limit / measured_power);
        }
        return std::clamp(scale, params_.min_scale, full_scale);
    }

    void update_virtual_buffer_energy(
        const rclcpp::Duration& period, double referee_buffer_energy, double measured_power) {
        const double dt = std::max(0.0, period.seconds());
        virtual_buffer_energy_ += dt * (chassis_power_limit_expected_ - measured_power);
        const double upper_bound =
            std::clamp(referee_buffer_energy, 0.0, params_.virtual_buffer_energy_limit);
        virtual_buffer_energy_ = std::clamp(virtual_buffer_energy_, 0.0, upper_bound);
    }

    double virtual_buffer_ratio() const {
        if (params_.virtual_buffer_energy_limit <= 0.0) {
            return full_scale;
        }
        return std::clamp(virtual_buffer_energy_ / params_.virtual_buffer_energy_limit, 0.0, 1.0);
    }

    std::optional<double> read_state(std::size_t index) const {
        if (index >= state_interfaces_.size()) {
            return std::nullopt;
        }

        const std::optional<double> value = state_interfaces_[index].get_optional();
        return value.has_value() && std::isfinite(*value) ? value : std::nullopt;
    }

    bool write_chassis_commands(const std::array<double, 3>& commands) {
        for (std::size_t index = 0; index < commands.size(); ++index) {
            if (!command_interfaces_[index].set_value(commands[index])) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Failed to write chained command '%s/%s'",
                    target_controller_name_.c_str(), chassis_command_suffixes[index]);
                return false;
            }
        }
        return true;
    }

    std::string target_controller_name_;
    std::array<double, 3> chassis_reference_{0.0, 0.0, 0.0};
    double virtual_buffer_energy_ = 0.0;
    double chassis_power_limit_expected_ = 0.0;
    std::shared_ptr<::chassis_power_controller::ParamListener> param_listener_;
    ::chassis_power_controller::Params params_;
};

} // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::chassis::ChassisPowerController,
    controller_interface::ChainableControllerInterface)
