#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "../pid/pid_calculator.hpp"
#include "rmgo_core/chassis_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::chassis {

class ChassisController
    : public controller_interface::ChainableControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        target_controller_name_ = params_.target_controller_name;
        yaw_joint_name_ = params_.yaw_joint_name;
        yaw_state_interface_name_ = params_.yaw_state_interface_name;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(params_.target_controller_name, chassis_command_suffixes);
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return build_individual_config(std::array{get_yaw_state_interface_name()});
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(remote_command_reference_);
        return make_reference_interfaces(remote_command_suffixes, remote_command_reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        target_controller_name_ = params_.target_controller_name;
        yaw_joint_name_ = params_.yaw_joint_name;
        yaw_state_interface_name_ = params_.yaw_state_interface_name;
        auto& node = *get_node();
        follow_pid_ = rmgo_core::pid::make_pid_calculator(node, "follow_", 0.0, 0.0, 0.0);

        reset_internal_state();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!expect_interface_count(
                command_interfaces_, chassis_command_suffixes.size(), "command")) {
            return controller_interface::CallbackReturn::ERROR;
        }
        if (!expect_interface_count(state_interfaces_, 1, "yaw state")) {
            return controller_interface::CallbackReturn::ERROR;
        }

        reset_internal_state();
        return stop_chassis() ? controller_interface::CallbackReturn::SUCCESS
                              : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_internal_state();
        return stop_chassis() ? controller_interface::CallbackReturn::SUCCESS
                              : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_internal_state();
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const auto& [linear_x_reference, linear_y_reference, angular_z_reference, mode_reference] =
            remote_command_reference_;
        const RemoteCommand command{
            linear_x_reference,
            linear_y_reference,
            angular_z_reference,
        };
        const Mode mode = mode_from_value(mode_reference);
        if (mode != last_mode_) {
            reset_pid();
            last_mode_ = mode;
        }

        const auto values = calculate_command(mode, command);

        return write_command(values) ? controller_interface::return_type::OK
                                     : controller_interface::return_type::ERROR;
    }

private:
    enum class Mode : std::uint8_t {
        raw = 0,
        follow = 1,
        twist = 2,
    };

    struct RemoteCommand {
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
    };

    enum class ChassisCommandIndex : std::size_t {
        linear_x = 0,
        linear_y,
        angular_z,
        count,
    };

    static constexpr std::size_t to_index(ChassisCommandIndex index) {
        return std::to_underlying(index);
    }

    static constexpr std::array<const char*, 4> remote_command_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
        "mode",
    };

    static constexpr std::array<const char*, 3> chassis_command_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
    };

    static_assert(
        chassis_command_suffixes.size() == std::to_underlying(ChassisCommandIndex::count));

    std::string get_yaw_state_interface_name() const {
        return params_.yaw_joint_name + "/" + params_.yaw_state_interface_name;
    }

    static Mode mode_from_value(double value) {
        if (!std::isfinite(value)) {
            return Mode::raw;
        }

        const auto mode = static_cast<Mode>(std::llround(value));
        switch (mode) {
        case Mode::follow:
        case Mode::twist: return mode;
        case Mode::raw:
        default: return Mode::raw;
        }
    }

    std::array<double, 3> calculate_command(Mode mode, const RemoteCommand& command) {
        const double yaw = read_yaw_position();
        std::array<double, 3> values = command_to_base_link(command, yaw);
        switch (mode) {
        case Mode::follow:
            values[to_index(ChassisCommandIndex::angular_z)] =
                calculate_follow_angular_velocity(yaw, command.wz);
            break;
        case Mode::twist:
            values[to_index(ChassisCommandIndex::angular_z)] =
                calculate_twist_angular_velocity(yaw);
            break;
        case Mode::raw:
        default: break;
        }
        return values;
    }

    std::array<double, 3> command_to_base_link(const RemoteCommand& command, double yaw) const {
        if (params_.command_source_frame == "base_link") {
            return {command.vx, command.vy, command.wz};
        }

        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        return {
            cos_yaw * command.vx - sin_yaw * command.vy,
            sin_yaw * command.vx + cos_yaw * command.vy,
            command.wz,
        };
    }

    double calculate_follow_angular_velocity(double yaw, double feedforward) {
        return follow_pid_.update(angles::normalize_angle(yaw)) + feedforward
             + params_.follow_velocity_feedforward;
    }

    double calculate_twist_angular_velocity(double yaw) {
        constexpr std::array<double, 4> candidate_offsets{
            -std::numbers::pi / 4.0,
            std::numbers::pi / 4.0,
            3.0 * std::numbers::pi / 4.0,
            -3.0 * std::numbers::pi / 4.0,
        };

        double offset = 0.0;
        for (const double candidate : candidate_offsets) {
            if (std::abs(angles::shortest_angular_distance(yaw, candidate)) < 0.79) {
                offset = candidate;
                break;
            }
        }

        const double desired_yaw =
            params_.twist_angular * std::sin(2.0 * std::numbers::pi * steady_clock_.now().seconds())
            + offset;
        return follow_pid_.update(-angles::shortest_angular_distance(yaw, desired_yaw));
    }

    double read_yaw_position() const { return read_finite_interface_or(state_interfaces_, 0, 0.0); }

    void reset_pid() { follow_pid_.reset(); }

    void reset_internal_state() {
        reset_references(remote_command_reference_);
        reset_pid();
        last_mode_ = Mode::raw;
    }

    bool stop_chassis() { return write_command({0.0, 0.0, 0.0}); }

    bool write_command(const std::array<double, 3>& values) {
        return write_safe_commands(
            command_interfaces_, values, target_controller_name_, chassis_command_suffixes);
    }

    std::string target_controller_name_;
    std::string yaw_joint_name_;
    std::string yaw_state_interface_name_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::array<double, 4> remote_command_reference_{0.0, 0.0, 0.0, 0.0};
    Mode last_mode_ = Mode::raw;
    rmgo_core::pid::PidCalculator follow_pid_;
    std::shared_ptr<::chassis_controller::ParamListener> param_listener_;
    ::chassis_controller::Params params_;
};

} // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::chassis::ChassisController,
    controller_interface::ChainableControllerInterface)
