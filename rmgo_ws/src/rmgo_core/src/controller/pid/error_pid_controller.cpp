#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "../pid/pid_calculator.hpp"
#include "rmgo_core/error_pid_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::pid {

class ErrorPidController
    : public controller_interface::ChainableControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(
            std::array{params_.target_name + "/" + params_.target_interface_name});
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return {
            controller_interface::interface_configuration_type::NONE,
            {},
        };
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(reference_);
        return make_reference_interfaces(reference_suffixes, reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        update_pid_from_parameters();
        reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset();
        return write_command(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                  : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset();
        return write_command(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                  : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references(reference_);
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const double error = reference_[0];
        if (!std::isfinite(error)) {
            pid_.reset();
            return write_command(0.0) ? controller_interface::return_type::OK
                                      : controller_interface::return_type::ERROR;
        }

        const double command = params_.feedforward + pid_.update(error);
        return write_command(std::isfinite(command) ? command : 0.0)
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::array<const char*, 1> reference_suffixes = {"control_angle_error"};

    void update_pid_from_parameters() {
        pid_ = rmgo_core::pid::PidCalculator{params_.kp, params_.ki, params_.kd};
        pid_.integral_min = params_.integral_min;
        pid_.integral_max = params_.integral_max;
        pid_.integral_split_min = params_.integral_split_min;
        pid_.integral_split_max = params_.integral_split_max;
        pid_.output_min = params_.output_min;
        pid_.output_max = params_.output_max;
    }

    void reset() {
        reset_references(reference_);
        pid_.reset();
    }

    bool write_command(double value) {
        return write_safe_commands(
            command_interfaces_, std::array{value}, params_.target_name,
            std::array{params_.target_interface_name.c_str()}, "error PID output");
    }

    std::array<double, 1> reference_{0.0};
    rmgo_core::pid::PidCalculator pid_;
    std::shared_ptr<::error_pid_controller::ParamListener> param_listener_;
    ::error_pid_controller::Params params_;
};

} // namespace rmgo_core::controller::pid

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::pid::ErrorPidController,
    controller_interface::ChainableControllerInterface)
