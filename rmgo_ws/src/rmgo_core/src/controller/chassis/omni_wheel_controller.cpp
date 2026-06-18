#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <numbers>
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
#include "rmgo_core/omni_wheel_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::chassis {

class OmniWheelController
    : public controller_interface::ChainableControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    using BaseLinkVelocityCommand = Eigen::Vector3d;
    using WheelState = Eigen::Vector4d;
    using WheelCommand = Eigen::Vector4d;
    using BaseLinkToWheelMatrix = Eigen::Matrix<double, 4, 3>;
    using WheelToBaseLinkMatrix = Eigen::Matrix<double, 3, 4>;

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
        reset_references(base_link_velocity_reference_);
        power_limit_reference_ = 0.0;
        auto interfaces =
            make_reference_interfaces(base_link_velocity_suffixes, base_link_velocity_reference_);
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

        base_link_to_wheel_ = make_base_link_to_wheel_matrix();
        wheel_to_base_link_ = make_wheel_to_base_link_matrix(base_link_to_wheel_);
        auto& node = *get_node();
        linear_x_pid_ = rmgo_core::pid::make_pid_calculator(node, "linear_x_", 0.0, 0.0, 0.0);
        linear_y_pid_ = rmgo_core::pid::make_pid_calculator(node, "linear_y_", 0.0, 0.0, 0.0);
        angular_z_pid_ = rmgo_core::pid::make_pid_calculator(node, "angular_z_", 0.0, 0.0, 0.0);

        reset_references(base_link_velocity_reference_);
        reset_pid_calculators();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references(base_link_velocity_reference_);
        power_limit_reference_ = 0.0;
        reset_pid_calculators();
        return write_wheel_commands(WheelCommand::Zero())
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references(base_link_velocity_reference_);
        power_limit_reference_ = 0.0;
        reset_pid_calculators();
        return write_wheel_commands(WheelCommand::Zero())
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references(base_link_velocity_reference_);
            power_limit_reference_ = 0.0;
            reset_pid_calculators();
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const auto& [linear_x_reference, linear_y_reference, angular_z_reference] =
            base_link_velocity_reference_;
        const BaseLinkVelocityCommand base_link_velocity_command{
            linear_x_reference,
            linear_y_reference,
            angular_z_reference,
        };
        if (!base_link_velocity_command.allFinite()) {
            reset_pid_calculators();
            return write_wheel_commands(WheelCommand::Zero())
                     ? controller_interface::return_type::OK
                     : controller_interface::return_type::ERROR;
        }

        const BaseLinkVelocityCommand measured_velocity = measure_base_link_velocity();
        BaseLinkVelocityCommand control_velocity = base_link_velocity_command;
        if (measured_velocity.allFinite()) {
            control_velocity = apply_pid(base_link_velocity_command, measured_velocity);
        } else {
            reset_pid_calculators();
        }
        WheelCommand wheel_commands = inverse_kinematics(control_velocity);
        constrain_wheel_commands(wheel_commands);
        constrain_chassis_power(wheel_commands);

        return write_wheel_commands(wheel_commands) ? controller_interface::return_type::OK
                                                    : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::size_t wheel_count = 4;

    // ros2_control exchanges scalar reference interfaces, but the three values
    // are semantically a BaseLink-frame chassis velocity command.
    static constexpr std::array<const char*, 3> base_link_velocity_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
    };

    BaseLinkToWheelMatrix make_base_link_to_wheel_matrix() const {
        const double lever_arm = params_.chassis_radius_x + params_.chassis_radius_y;

        BaseLinkToWheelMatrix matrix;
        matrix << -1.0, 1.0, lever_arm, -1.0, -1.0, lever_arm, 1.0, -1.0, lever_arm, 1.0, 1.0,
            lever_arm;
        matrix *= -1.0 / (std::numbers::sqrt2 * params_.wheel_radius);
        return matrix;
    }

    static WheelToBaseLinkMatrix
        make_wheel_to_base_link_matrix(const BaseLinkToWheelMatrix& base_link_to_wheel) {
        return (base_link_to_wheel.transpose() * base_link_to_wheel).inverse()
             * base_link_to_wheel.transpose();
    }

    WheelCommand
        inverse_kinematics(const BaseLinkVelocityCommand& base_link_velocity_command) const {
        return base_link_to_wheel_ * base_link_velocity_command;
    }

    BaseLinkVelocityCommand measure_base_link_velocity() const {
        assert(state_interfaces_.size() == params_.wheel_joints.size());

        WheelState wheel_state;
        for (std::size_t index = 0; index < params_.wheel_joints.size(); ++index) {
            wheel_state[static_cast<Eigen::Index>(index)] =
                read_interface_value(state_interfaces_, index);
        }

        return wheel_to_base_link_ * wheel_state;
    }

    BaseLinkVelocityCommand apply_pid(
        const BaseLinkVelocityCommand& desired_command,
        const BaseLinkVelocityCommand& measured_command) {
        BaseLinkVelocityCommand control_command = desired_command;
        control_command.x() += linear_x_pid_.update(desired_command.x() - measured_command.x());
        control_command.y() += linear_y_pid_.update(desired_command.y() - measured_command.y());
        control_command.z() += angular_z_pid_.update(desired_command.z() - measured_command.z());
        return control_command;
    }

    void constrain_wheel_commands(WheelCommand& wheel_commands) const {
        if (params_.max_wheel_velocity <= 0.0) {
            return;
        }

        const double max_command = wheel_commands.cwiseAbs().maxCoeff();
        if (max_command <= params_.max_wheel_velocity) {
            return;
        }

        wheel_commands *= params_.max_wheel_velocity / max_command;
    }

    void constrain_chassis_power(WheelCommand& wheel_commands) const {
        const double control_power_limit = power_limit_reference_;
        if (!std::isfinite(control_power_limit) || control_power_limit <= 0.0) {
            wheel_commands.setZero();
            return;
        }

        const double power_ratio =
            std::clamp(control_power_limit / params_.full_speed_power_limit, 0.0, 1.0);
        wheel_commands *= power_ratio;
    }

    void reset_pid_calculators() {
        linear_x_pid_.reset();
        linear_y_pid_.reset();
        angular_z_pid_.reset();
    }

    bool write_wheel_commands(const WheelCommand& wheel_commands) {
        return write_safe_joint_commands(
            command_interfaces_,
            std::span<const double, wheel_count>{wheel_commands.data(), wheel_count},
            std::span<const std::string, wheel_count>{wheel_joints_},
            params_.command_interface_name);
    }

    std::array<std::string, wheel_count> wheel_joints_{};
    std::array<double, 3> base_link_velocity_reference_{0.0, 0.0, 0.0};
    double power_limit_reference_ = 0.0;
    BaseLinkToWheelMatrix base_link_to_wheel_ = BaseLinkToWheelMatrix::Zero();
    WheelToBaseLinkMatrix wheel_to_base_link_ = WheelToBaseLinkMatrix::Zero();
    rmgo_core::pid::PidCalculator linear_x_pid_;
    rmgo_core::pid::PidCalculator linear_y_pid_;
    rmgo_core::pid::PidCalculator angular_z_pid_;
    std::shared_ptr<::omni_wheel_controller::ParamListener> param_listener_;
    ::omni_wheel_controller::Params params_;
};

} // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::chassis::OmniWheelController,
    controller_interface::ChainableControllerInterface)
