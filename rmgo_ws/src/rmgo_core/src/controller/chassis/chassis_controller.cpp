#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
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
#include "rmgo_core/interface/reference_interfaces.hpp"
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
        reset_references(chassis_reference_);
        return make_reference_interfaces(
            rmgo_core::reference_interfaces::chassis_interfaces, chassis_reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        target_controller_name_ = params_.target_controller_name;
        yaw_joint_name_ = params_.yaw_joint_name;
        yaw_state_interface_name_ = params_.yaw_state_interface_name;
        auto& node = *get_node();
        follow_pid_ = rmgo_core::pid::make_pid_calculator(node, "follow_", 0.0, 0.0, 0.0);

        reset_references(chassis_reference_);
        reset_internal_state();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_state_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        reset_references(chassis_reference_);
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
            reset_references(chassis_reference_);
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const StateSnapshot state = read_state_snapshot();
        const Mode mode = mode_from_value(state.mode);
        if (mode != last_mode_) {
            reset_pid();
            last_mode_ = mode;
        }

        const auto values = calculate_command(mode, state.reference, state.yaw);

        return write_command(values) ? controller_interface::return_type::OK
                                     : controller_interface::return_type::ERROR;
    }

private:
    enum class Mode : std::uint8_t {
        raw = 0,
        follow = 1,
        twist = 2,
    };

    enum class StateInterfaceIndex : std::size_t {
        yaw = 0,
        count,
    };

    struct ChassisReference {
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
    };

    struct StateSnapshot {
        double yaw = 0.0;
        ChassisReference reference;
        double mode = 0.0;
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

    static constexpr std::size_t to_index(StateInterfaceIndex index) {
        return std::to_underlying(index);
    }

    static constexpr std::array<const char*, 3> chassis_command_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
    };

    static_assert(
        chassis_command_suffixes.size() == std::to_underlying(ChassisCommandIndex::count));
    static_assert(std::to_underlying(StateInterfaceIndex::count) == 1);

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

    std::array<double, 3> calculate_command(
        Mode mode, const ChassisReference& reference, double yaw) {
        std::array<double, 3> values = reference_to_base_link(reference, yaw);
        switch (mode) {
        case Mode::follow:
            values[to_index(ChassisCommandIndex::angular_z)] =
                calculate_follow_angular_velocity(yaw, reference.wz);
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

    std::array<double, 3> reference_to_base_link(
        const ChassisReference& reference, double yaw) const {
        if (params_.command_source_frame == "base_link") {
            return {reference.vx, reference.vy, reference.wz};
        }

        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        return {
            cos_yaw * reference.vx - sin_yaw * reference.vy,
            sin_yaw * reference.vx + cos_yaw * reference.vy,
            reference.wz,
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

    bool bind_state_interfaces() {
        state_indexes_.fill(invalid_index);
        return bind_prefixed_interface_indexes(
            state_interfaces_,
            {
                {&state_indexes_[to_index(StateInterfaceIndex::yaw)], params_.yaw_joint_name,
                 params_.yaw_state_interface_name},
            },
            "chassis state interface");
    }

    StateSnapshot read_state_snapshot() const {
        return StateSnapshot{
            .yaw = read_state(StateInterfaceIndex::yaw),
            .reference =
                ChassisReference{
                    .vx = chassis_reference_[0],
                    .vy = chassis_reference_[1],
                    .wz = chassis_reference_[2],
                },
            .mode = chassis_reference_[3],
        };
    }

    double read_state(StateInterfaceIndex index) const {
        return read_interface_value(state_interfaces_, state_indexes_[to_index(index)]);
    }

    void reset_pid() { follow_pid_.reset(); }

    void reset_internal_state() {
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
    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    std::array<std::size_t, std::to_underlying(StateInterfaceIndex::count)> state_indexes_{};
    std::array<double, rmgo_core::reference_interfaces::chassis_interfaces.size()>
        chassis_reference_{0.0, 0.0, 0.0, 0.0};
    Mode last_mode_ = Mode::raw;
    rmgo_core::pid::PidCalculator follow_pid_;
    std::shared_ptr<::chassis_controller::ParamListener> param_listener_;
    ::chassis_controller::Params params_;
};

} // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::chassis::ChassisController,
    controller_interface::ChainableControllerInterface)
