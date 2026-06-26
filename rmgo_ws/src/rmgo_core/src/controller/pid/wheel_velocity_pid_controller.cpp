#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <eigen3/Eigen/Dense>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "../pid/pid_calculator.hpp"
#include "rmgo_core/interface/reference_interfaces.hpp"
#include "rmgo_core/wheel_velocity_pid_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::pid {

class WheelVelocityPidController
    : public controller_interface::ChainableControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    using WheelState = Eigen::Vector4d;
    using WheelVelocityCommand = Eigen::Vector4d;
    using WheelCommand = Eigen::Vector4d;

    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_joint_interface_config(params_.wheel_joints, params_.command_interface_name);
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return build_joint_interface_config(
            params_.wheel_joints, params_.wheel_state_interface_name);
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(wheel_velocity_reference_);
        power_limit_reference_ = 0.0;

        auto interfaces =
            make_reference_interfaces(wheel_velocity_suffixes, wheel_velocity_reference_);
        interfaces.emplace_back(
            std::make_shared<hardware_interface::CommandInterface>(
                node_name(), rmgo_core::reference_interfaces::chassis_power_limit,
                &power_limit_reference_));
        return interfaces;
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        std::ranges::copy(params_.wheel_joints, wheel_joints_.begin());
        update_pids_from_parameters();

        if (params_.command_interface_name != hardware_interface::HW_IF_EFFORT) {
            logging::warn(
                "WheelVelocityPidController is tuned for '{}' wheel commands; configured '{}'",
                hardware_interface::HW_IF_EFFORT, params_.command_interface_name);
        }

        reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset();
        return write_wheel_commands(WheelCommand::Zero())
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset();
        return write_wheel_commands(WheelCommand::Zero())
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset();
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const WheelVelocityCommand wheel_velocity_command = read_wheel_velocity_reference();
        if (!wheel_velocity_command.allFinite()) {
            reset_pids();
            return write_wheel_commands(WheelCommand::Zero())
                     ? controller_interface::return_type::OK
                     : controller_interface::return_type::ERROR;
        }

        const WheelState wheel_state = read_wheel_state();
        WheelCommand wheel_commands = calculate_wheel_commands(wheel_velocity_command, wheel_state);
        constrain_chassis_power(wheel_commands, wheel_state);

        return write_wheel_commands(wheel_commands) ? controller_interface::return_type::OK
                                                    : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::size_t wheel_count = 4;

    static constexpr std::array<const char*, wheel_count> wheel_velocity_suffixes = {
        "left_front_wheel/control_velocity",
        "left_back_wheel/control_velocity",
        "right_back_wheel/control_velocity",
        "right_front_wheel/control_velocity",
    };

    WheelVelocityCommand read_wheel_velocity_reference() const {
        WheelVelocityCommand wheel_velocity_command;
        for (std::size_t index = 0; index < wheel_count; ++index) {
            wheel_velocity_command[static_cast<Eigen::Index>(index)] =
                wheel_velocity_reference_[index];
        }
        return wheel_velocity_command;
    }

    WheelState read_wheel_state() const {
        assert(state_interfaces_.size() == params_.wheel_joints.size());

        WheelState wheel_state;
        for (std::size_t index = 0; index < params_.wheel_joints.size(); ++index) {
            wheel_state[static_cast<Eigen::Index>(index)] =
                read_interface_value(state_interfaces_, index);
        }
        return wheel_state;
    }

    WheelCommand calculate_wheel_commands(
        const WheelVelocityCommand& wheel_velocity_command, const WheelState& wheel_state) {
        WheelCommand wheel_commands = WheelCommand::Zero();
        for (std::size_t index = 0; index < wheel_count; ++index) {
            const auto eigen_index = static_cast<Eigen::Index>(index);
            const double target_velocity = wheel_velocity_command[eigen_index];
            const double measured_velocity = wheel_state[eigen_index];
            if (!std::isfinite(target_velocity)) {
                wheel_velocity_pids_[index].reset();
                continue;
            }

            double command = params_.wheel_velocity_feedforward * target_velocity;
            if (std::isfinite(measured_velocity)) {
                command += wheel_velocity_pids_[index].update(target_velocity - measured_velocity);
            } else {
                wheel_velocity_pids_[index].reset();
            }

            wheel_commands[eigen_index] =
                std::isfinite(command)
                    ? std::clamp(command, -params_.max_wheel_effort, params_.max_wheel_effort)
                    : 0.0;
        }
        return wheel_commands;
    }

    void
        constrain_chassis_power(WheelCommand& wheel_commands, const WheelState& wheel_state) const {
        const double control_power_limit = power_limit_reference_;
        if (!std::isfinite(control_power_limit) || control_power_limit <= 0.0) {
            wheel_commands.setZero();
            return;
        }

        wheel_commands *=
            calculate_rm_controllers_power_scale(wheel_commands, wheel_state, control_power_limit);
    }

    double calculate_rm_controllers_power_scale(
        const WheelCommand& wheel_commands, const WheelState& wheel_state,
        double control_power_limit) const {
        if (!wheel_commands.allFinite() || !wheel_state.allFinite()) {
            return 0.0;
        }

        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        for (std::size_t index = 0; index < wheel_count; ++index) {
            const auto eigen_index = static_cast<Eigen::Index>(index);
            const double command = wheel_commands[eigen_index];
            const double velocity = wheel_state[eigen_index];
            a += square(command);
            b += std::abs(command * velocity);
            c += square(velocity);
        }
        a *= params_.effort_coeff;
        c = c * params_.vel_coeff - params_.power_offset - control_power_limit;

        return solve_power_scale(a, b, c);
    }

    static double solve_power_scale(double a, double b, double c) {
        if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c)) {
            return 0.0;
        }

        constexpr double epsilon = std::numeric_limits<double>::epsilon();
        if (a <= epsilon) {
            if (b <= epsilon) {
                return c <= 0.0 ? 1.0 : 0.0;
            }
            return std::clamp(-c / b, 0.0, 1.0);
        }

        const double discriminant = square(b) - 4.0 * a * c;
        if (discriminant <= 0.0) {
            return 0.0;
        }

        const double scale = (-b + std::sqrt(discriminant)) / (2.0 * a);
        return std::isfinite(scale) ? std::clamp(scale, 0.0, 1.0) : 0.0;
    }

    static constexpr double square(double value) { return value * value; }

    void update_pids_from_parameters() {
        for (auto& pid : wheel_velocity_pids_) {
            pid = rmgo_core::pid::PidCalculator{
                params_.wheel_velocity_kp,
                params_.wheel_velocity_ki,
                params_.wheel_velocity_kd,
            };
            pid.output_min = -params_.max_wheel_effort;
            pid.output_max = params_.max_wheel_effort;
        }
    }

    void reset() {
        reset_references(wheel_velocity_reference_);
        power_limit_reference_ = 0.0;
        reset_pids();
    }

    void reset_pids() {
        for (auto& pid : wheel_velocity_pids_) {
            pid.reset();
        }
    }

    bool write_wheel_commands(const WheelCommand& wheel_commands) {
        return write_safe_joint_commands(
            command_interfaces_,
            std::span<const double, wheel_count>{wheel_commands.data(), wheel_count},
            std::span<const std::string, wheel_count>{wheel_joints_},
            params_.command_interface_name);
    }

    std::array<std::string, wheel_count> wheel_joints_{};
    std::array<double, wheel_count> wheel_velocity_reference_{0.0, 0.0, 0.0, 0.0};
    double power_limit_reference_ = 0.0;
    std::array<rmgo_core::pid::PidCalculator, wheel_count> wheel_velocity_pids_;
    std::shared_ptr<::wheel_velocity_pid_controller::ParamListener> param_listener_;
    ::wheel_velocity_pid_controller::Params params_;
};

} // namespace rmgo_core::controller::pid

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::pid::WheelVelocityPidController,
    controller_interface::ChainableControllerInterface)
