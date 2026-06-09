#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/gimbal/two_axis_gimbal_solver.hpp"
#include "rmgo_core/simple_gimbal_controller_config.hpp"

namespace rmgo_core::controller::gimbal {

class SimpleGimbalController : public controller_interface::ChainableControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override {
        param_listener_ = std::make_shared<::simple_gimbal_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        reset_references();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            params_.yaw_joint_name + "/" + params_.command_interface_name,
            params_.pitch_joint_name + "/" + params_.command_interface_name,
        };
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            params_.yaw_joint_name + "/" + params_.state_interface_name,
            params_.pitch_joint_name + "/" + params_.state_interface_name,
        };
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
        solver_ = rmgo_core::gimbal::TwoAxisGimbalSolver{
            params_.pitch_upper_limit,
            params_.pitch_lower_limit,
        };
        reset_references();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (command_interfaces_.size() != command_interface_names.size()) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                command_interface_names.size(), command_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        if (state_interfaces_.size() != state_interface_names.size()) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
                state_interface_names.size(), state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        const double yaw = read_state(yaw_index);
        const double pitch = read_state(pitch_index);
        target_yaw_ = angles::normalize_angle(yaw);
        target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        reset_references();

        return write_commands(yaw, target_pitch_) ? controller_interface::CallbackReturn::SUCCESS
                                                  : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references();
        return write_commands(read_state(yaw_index), read_state(pitch_index))
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
        update_targets(period.seconds());
        return write_commands(target_yaw_, target_pitch_)
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::size_t yaw_index = 0;
    static constexpr std::size_t pitch_index = 1;
    static constexpr std::array<const char*, 2> command_interface_names = {"yaw", "pitch"};
    static constexpr std::array<const char*, 2> state_interface_names = {
        "yaw",
        "pitch",
    };
    static constexpr std::array<const char*, 3> remote_command_suffixes = {
        "yaw/velocity",
        "pitch/velocity",
        "enabled",
    };

    double read_state(std::size_t index) const {
        if (index >= state_interfaces_.size()) {
            return 0.0;
        }

        const std::optional<double> value = state_interfaces_[index].get_optional();
        return value.has_value() && std::isfinite(*value) ? *value : 0.0;
    }

    void update_targets(double dt) {
        const double yaw = read_state(yaw_index);
        const double pitch = read_state(pitch_index);
        if (!is_enabled_reference()) {
            solver_.update(yaw, pitch, rmgo_core::gimbal::TwoAxisGimbalSolver::SetDisabled{});
            target_yaw_ = angles::normalize_angle(yaw);
            target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
            return;
        }

        const double yaw_velocity = finite_or_zero(remote_command_reference_[0]);
        const double pitch_velocity = finite_or_zero(remote_command_reference_[1]);
        const auto error =
            solver_.enabled()
                ? solver_.update(
                      yaw, pitch,
                      rmgo_core::gimbal::TwoAxisGimbalSolver::SetControlShift{
                          yaw_velocity * dt,
                          pitch_velocity * dt,
                      })
                : solver_.update(yaw, pitch, rmgo_core::gimbal::TwoAxisGimbalSolver::SetToLevel{});

        if (std::isfinite(error.yaw) && std::isfinite(error.pitch)) {
            target_yaw_ = angles::normalize_angle(yaw + error.yaw);
            target_pitch_ = std::clamp(
                pitch + error.pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        } else {
            target_yaw_ = angles::normalize_angle(yaw);
            target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        }
    }

    bool is_enabled_reference() const {
        return std::isfinite(remote_command_reference_[2]) && remote_command_reference_[2] > 0.5;
    }

    static double finite_or_zero(double value) { return std::isfinite(value) ? value : 0.0; }

    void reset_references() { remote_command_reference_.fill(0.0); }

    bool write_commands(double yaw, double pitch) {
        const std::array<double, 2> values = {
            angles::normalize_angle(yaw),
            std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit),
        };
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces_[index].set_value(values[index])) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Failed to write %s gimbal command",
                    command_interface_names[index]);
                return false;
            }
        }
        return true;
    }

    double target_yaw_ = 0.0;
    double target_pitch_ = 0.0;
    std::array<double, 3> remote_command_reference_{0.0, 0.0, 0.0};
    rmgo_core::gimbal::TwoAxisGimbalSolver solver_{
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    std::shared_ptr<::simple_gimbal_controller::ParamListener> param_listener_;
    ::simple_gimbal_controller::Params params_;
};

} // namespace rmgo_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::gimbal::SimpleGimbalController,
    controller_interface::ChainableControllerInterface)
