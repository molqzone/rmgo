#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/chassis_controller_config.hpp"
#include "rmgo_core/pid/pid_calculator.hpp"

namespace rmgo_core::controller::chassis {

class ChassisController : public controller_interface::ChainableControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override {
        param_listener_ = std::make_shared<::chassis_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        target_controller_name_ = params_.target_controller_name;
        yaw_joint_name_ = params_.yaw_joint_name;
        yaw_state_interface_name_ = params_.yaw_state_interface_name;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        const std::string target_controller_name = get_target_controller_name();
        config.names.reserve(chassis_command_suffixes.size());
        for (const char* suffix : chassis_command_suffixes) {
            config.names.push_back(target_controller_name + "/" + suffix);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(get_yaw_state_interface_name());
        return config;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references();

        const std::string controller_name = get_node()->get_name();
        return {
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, remote_command_suffixes[0], &remote_command_reference_[0]),
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, remote_command_suffixes[1], &remote_command_reference_[1]),
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, remote_command_suffixes[2], &remote_command_reference_[2]),
            std::make_shared<hardware_interface::CommandInterface>(
                controller_name, remote_command_suffixes[3], &remote_command_reference_[3]),
        };
    }

    std::vector<hardware_interface::StateInterface::SharedPtr>
        on_export_state_interfaces_list() override {
        return {};
    }

    bool on_set_chained_mode(bool chained_mode) override {
        (void)chained_mode;
        return true;
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        params_ = param_listener_->get_params();
        target_controller_name_ = params_.target_controller_name;
        yaw_joint_name_ = params_.yaw_joint_name;
        yaw_state_interface_name_ = params_.yaw_state_interface_name;
        auto& node = *get_node();
        follow_pid_ = rmgo_core::pid::make_pid_calculator(node, "follow_", 0.0, 0.0, 0.0);

        reset_references();
        reset_pid();
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
        if (state_interfaces_.size() != 1) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected yaw state interface, got %zu",
                state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        reset_references();
        reset_pid();
        last_mode_ = Mode::raw;
        return write_command({0.0, 0.0, 0.0}) ? controller_interface::CallbackReturn::SUCCESS
                                              : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references();
        reset_pid();
        last_mode_ = Mode::raw;
        return write_command({0.0, 0.0, 0.0}) ? controller_interface::CallbackReturn::SUCCESS
                                              : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references();
            reset_pid();
            last_mode_ = Mode::raw;
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const RemoteCommand command{
            remote_command_reference_[0],
            remote_command_reference_[1],
            remote_command_reference_[2],
        };
        const Mode mode = mode_from_value(remote_command_reference_[3]);
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

    std::string get_target_controller_name() const { return params_.target_controller_name; }

    std::string get_yaw_state_interface_name() const {
        return params_.yaw_joint_name + "/" + params_.yaw_state_interface_name;
    }

    static Mode mode_from_value(double value) {
        if (!std::isfinite(value)) {
            return Mode::raw;
        }

        switch (static_cast<std::uint8_t>(std::llround(value))) {
        case static_cast<std::uint8_t>(Mode::follow): return Mode::follow;
        case static_cast<std::uint8_t>(Mode::twist): return Mode::twist;
        case static_cast<std::uint8_t>(Mode::raw):
        default: return Mode::raw;
        }
    }

    std::array<double, 3> calculate_command(Mode mode, const RemoteCommand& command) {
        const double yaw = read_yaw_position();
        std::array<double, 3> values = command_to_base_link(command, yaw);
        switch (mode) {
        case Mode::follow: values[2] = calculate_follow_angular_velocity(yaw, command.wz); break;
        case Mode::twist: values[2] = calculate_twist_angular_velocity(yaw); break;
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

    double read_yaw_position() const {
        if (state_interfaces_.empty()) {
            return 0.0;
        }

        const std::optional<double> yaw = state_interfaces_[0].get_optional();
        return yaw.has_value() && std::isfinite(*yaw) ? *yaw : 0.0;
    }

    void reset_references() { remote_command_reference_.fill(0.0); }

    void reset_pid() { follow_pid_.reset(); }

    bool write_command(const std::array<double, 3>& values) {
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces_[index].set_value(values[index])) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Failed to write reference command '%s/%s'",
                    target_controller_name_.c_str(), chassis_command_suffixes[index]);
                return false;
            }
        }
        return true;
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
