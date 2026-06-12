#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "gimbal_tf_builder.hpp"
#include "rmgo_core/interface/command_state_interfaces.hpp"
#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "rmgo_core/simple_gimbal_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"
#include "two_axis_gimbal_solver.hpp"

namespace rmgo_core::controller::gimbal {

using GimbalTfState = rmgo_core::gimbal::GimbalTfState;
using TwoAxisGimbalSolver = rmgo_core::gimbal::TwoAxisGimbalSolver;
using rmgo_core::gimbal::update_gimbal_tf;

class SimpleGimbalController
    : public controller_interface::ControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        append_prefixed_interface_names(
            config.names, params_.yaw_target_controller_name, angle_error_suffixes);
        append_prefixed_interface_names(
            config.names, params_.pitch_target_controller_name, angle_error_suffixes);
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        auto config = build_individual_config(std::array{
            params_.yaw_joint_name + "/" + params_.state_interface_name,
            params_.pitch_joint_name + "/" + params_.state_interface_name,
        });
        append_interface_names(
            config.names, rmgo_core::io_state_interfaces::gimbal_imu_orientation_state_interfaces);
        append_interface_names(
            config.names, rmgo_core::command_state_interfaces::gimbal_interfaces);
        return config;
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        solver_ = rmgo_core::gimbal::TwoAxisGimbalSolver{
            params_.pitch_lower_limit,
            params_.pitch_upper_limit,
        };
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_command_interfaces() || !bind_state_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }

        const double yaw = read_state(StateInterfaceIndex::yaw);
        const double pitch = read_state(StateInterfaceIndex::pitch);
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || !update_tf(yaw, pitch)) {
            logging::error("Gimbal state source unavailable during activation");
            return controller_interface::CallbackReturn::ERROR;
        }

        return write_angle_errors({0.0, 0.0}) ? controller_interface::CallbackReturn::SUCCESS
                                              : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return write_angle_errors(disabled_angle_error())
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type
        update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
        const TwoAxisGimbalSolver::AngleError angle_error = calculate_angle_error(period.seconds());
        return write_angle_errors(angle_error) ? controller_interface::return_type::OK
                                               : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    static constexpr std::array<const char*, 1> angle_error_suffixes = {"control_angle_error"};
    static constexpr std::array<const char*, 2> command_interface_names = {
        "yaw_angle_error",
        "pitch_angle_error",
    };

    enum class CommandInterfaceIndex : std::size_t {
        yaw = 0,
        pitch,
        count,
    };

    enum class StateInterfaceIndex : std::size_t {
        yaw = 0,
        pitch,
        imu_w,
        imu_x,
        imu_y,
        imu_z,
        command_yaw_velocity,
        command_pitch_velocity,
        command_enabled,
        count,
    };

    template <typename IndexT>
    static constexpr std::size_t to_index(IndexT index) {
        return std::to_underlying(index);
    }

    static_assert(
        command_interface_names.size() == std::to_underlying(CommandInterfaceIndex::count));
    static_assert(
        std::to_underlying(StateInterfaceIndex::count)
        == 2 + rmgo_core::io_state_interfaces::gimbal_imu_orientation_state_interfaces.size()
               + rmgo_core::command_state_interfaces::gimbal_interfaces.size());

    template <std::size_t N>
    static constexpr std::array<std::size_t, N> invalid_indexes() {
        std::array<std::size_t, N> indexes{};
        indexes.fill(invalid_index);
        return indexes;
    }

    double read_state(StateInterfaceIndex index) const {
        return read_interface_value(state_interfaces_, state_indexes_[to_index(index)]);
    }

    bool bind_command_interfaces() {
        command_indexes_.fill(invalid_index);
        return bind_prefixed_interface_indexes(
            command_interfaces_,
            {
                {&command_indexes_[to_index(CommandInterfaceIndex::yaw)],
                 params_.yaw_target_controller_name, angle_error_suffixes[0]},
                {&command_indexes_[to_index(CommandInterfaceIndex::pitch)],
                 params_.pitch_target_controller_name, angle_error_suffixes[0]},
            },
            "gimbal angle-error command interface");
    }

    // Gimbal self-stabilization depends on this quaternion being read as w/x/y/z.
    // Do not index state_interfaces_ by the request order here: ros2_control and
    // /dynamic_joint_states expose interfaces by full names, and grouped diagnostic
    // output may not preserve the controller's requested order. A swapped IMU
    // quaternion makes the solver see the wrong world frame, so the yaw joint will
    // not cancel chassis rotation even though the controller chain is active.
    bool bind_state_interfaces() {
        using namespace rmgo_core::io_state_interfaces;
        state_indexes_.fill(invalid_index);
        return bind_prefixed_interface_indexes(
                   state_interfaces_,
                   {
                       {&state_indexes_[to_index(StateInterfaceIndex::yaw)], params_.yaw_joint_name,
                        params_.state_interface_name},
                       {&state_indexes_[to_index(StateInterfaceIndex::pitch)],
                        params_.pitch_joint_name, params_.state_interface_name},
                   },
                   "gimbal state interface")
            && bind_interface_indexes(
                   state_interfaces_,
                   {
                       {&state_indexes_[to_index(StateInterfaceIndex::imu_w)],
                        gimbal_imu_orientation_w},
                       {&state_indexes_[to_index(StateInterfaceIndex::imu_x)],
                        gimbal_imu_orientation_x},
                       {&state_indexes_[to_index(StateInterfaceIndex::imu_y)],
                        gimbal_imu_orientation_y},
                       {&state_indexes_[to_index(StateInterfaceIndex::imu_z)],
                        gimbal_imu_orientation_z},
                   },
                   "gimbal state interface")
            && bind_interface_indexes(
                   state_interfaces_,
                   {
                       {&state_indexes_[to_index(StateInterfaceIndex::command_yaw_velocity)],
                        rmgo_core::command_state_interfaces::gimbal_yaw_velocity},
                       {&state_indexes_[to_index(StateInterfaceIndex::command_pitch_velocity)],
                        rmgo_core::command_state_interfaces::gimbal_pitch_velocity},
                       {&state_indexes_[to_index(StateInterfaceIndex::command_enabled)],
                        rmgo_core::command_state_interfaces::gimbal_enabled},
                   },
                   "gimbal command state interface");
    }

    TwoAxisGimbalSolver::AngleError calculate_angle_error(double dt) {
        const double yaw = read_state(StateInterfaceIndex::yaw);
        const double pitch = read_state(StateInterfaceIndex::pitch);
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || !update_tf(yaw, pitch)) {
            return disabled_angle_error();
        }

        const double yaw_reference = read_state(StateInterfaceIndex::command_yaw_velocity);
        const double pitch_reference = read_state(StateInterfaceIndex::command_pitch_velocity);
        const double enabled_reference = read_state(StateInterfaceIndex::command_enabled);
        const bool is_enabled = std::isfinite(enabled_reference) && enabled_reference > 0.5;
        if (!is_enabled) {
            return solver_.update(tf_, pitch, TwoAxisGimbalSolver::SetDisabled{});
        }

        const auto error = solver_.enabled()
                             ? solver_.update(
                                   tf_, pitch,
                                   TwoAxisGimbalSolver::SetControlShift{
                                       finite_or_zero(yaw_reference) * dt,
                                       finite_or_zero(pitch_reference) * dt,
                                   })
                             : solver_.update(tf_, pitch, TwoAxisGimbalSolver::SetToLevel{});

        return error;
    }

    bool update_tf(double yaw, double pitch) {
        const double imu_w = read_state(StateInterfaceIndex::imu_w);
        const double imu_x = read_state(StateInterfaceIndex::imu_x);
        const double imu_y = read_state(StateInterfaceIndex::imu_y);
        const double imu_z = read_state(StateInterfaceIndex::imu_z);
        Eigen::Quaterniond orientation{imu_w, imu_x, imu_y, imu_z};
        if (orientation.norm() <= 1e-6 || !orientation.coeffs().allFinite()) {
            return false;
        }

        update_gimbal_tf(
            tf_, GimbalTfState{
                     .yaw = yaw,
                     .pitch = pitch,
                     .pitch_link_to_odom_imu = orientation,
                 });
        return true;
    }

    static TwoAxisGimbalSolver::AngleError disabled_angle_error() {
        return {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
        };
    }

    static double finite_or_zero(double value) { return std::isfinite(value) ? value : 0.0; }

    bool write_angle_errors(TwoAxisGimbalSolver::AngleError angle_error) {
        const std::array<double, 2> values = {angle_error.yaw, angle_error.pitch};
        return write_safe_indexed_commands(
            command_interfaces_, values, command_indexes_, command_interface_names,
            "gimbal angle error");
    }

    std::array<std::size_t, std::to_underlying(CommandInterfaceIndex::count)> command_indexes_ =
        invalid_indexes<std::to_underlying(CommandInterfaceIndex::count)>();
    std::array<std::size_t, std::to_underlying(StateInterfaceIndex::count)> state_indexes_ =
        invalid_indexes<std::to_underlying(StateInterfaceIndex::count)>();
    rmgo_description::Tf tf_;
    TwoAxisGimbalSolver solver_{
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    std::shared_ptr<::simple_gimbal_controller::ParamListener> param_listener_;
    ::simple_gimbal_controller::Params params_;
};

} // namespace rmgo_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::gimbal::SimpleGimbalController,
    controller_interface::ControllerInterface)
