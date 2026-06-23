#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "../pid/pid_calculator.hpp"
#include "rmgo_core/joint_velocity_pid_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::pid {

class JointVelocityPidController
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
            std::array{params_.joint_name + "/" + params_.command_interface_name});
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return build_individual_config(
            std::array{state_prefix() + "/" + params_.state_interface_name});
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
        const double target_velocity = reference_[0];
        if (!std::isfinite(target_velocity)) {
            pid_.reset();
            return write_command(0.0) ? controller_interface::return_type::OK
                                      : controller_interface::return_type::ERROR;
        }

        double command = params_.velocity_feedforward * target_velocity;
        const double measured_velocity = read_interface_value(state_interfaces_, 0);
        if (std::isfinite(measured_velocity)) {
            command += pid_.update(target_velocity - measured_velocity);
        } else {
            pid_.reset();
        }

        return write_command(std::isfinite(command) ? command : 0.0)
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::array<const char*, 1> reference_suffixes = {"control_velocity"};

    std::string state_prefix() const {
        return params_.state_joint_name.empty() ? params_.joint_name : params_.state_joint_name;
    }

    void update_pid_from_parameters() {
        pid_ = rmgo_core::pid::PidCalculator{params_.kp, params_.ki, params_.kd};
        pid_.output_min = params_.output_min;
        pid_.output_max = params_.output_max;
    }

    void reset() {
        reset_references(reference_);
        pid_.reset();
    }

    bool write_command(double value) {
        if (command_interfaces_.empty()) {
            logging::error("Missing command interface for joint '{}'", params_.joint_name);
            return false;
        }
        if (!command_interfaces_[0].set_value(value)) {
            logging::error(
                "Failed to write {} command for joint '{}'", params_.command_interface_name,
                params_.joint_name);
            return false;
        }
        return true;
    }

    std::array<double, 1> reference_{0.0};
    rmgo_core::pid::PidCalculator pid_;
    std::shared_ptr<::joint_velocity_pid_controller::ParamListener> param_listener_;
    ::joint_velocity_pid_controller::Params params_;
};

} // namespace rmgo_core::controller::pid

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::pid::JointVelocityPidController,
    controller_interface::ChainableControllerInterface)
