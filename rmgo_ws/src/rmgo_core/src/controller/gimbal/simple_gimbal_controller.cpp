#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "gimbal_tf_builder.hpp"
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
    : public controller_interface::ChainableControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        reset_references(remote_command_reference_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(std::array{
            params_.yaw_joint_name + "/" + params_.command_interface_name,
            params_.pitch_joint_name + "/" + params_.command_interface_name,
        });
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        auto config = build_individual_config(std::array{
            params_.yaw_joint_name + "/" + params_.state_interface_name,
            params_.pitch_joint_name + "/" + params_.state_interface_name,
        });
        append_interface_names(
            config.names, rmgo_core::io_state_interfaces::gimbal_imu_orientation_state_interfaces);
        return config;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(remote_command_reference_);
        return make_reference_interfaces(remote_command_suffixes, remote_command_reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        solver_ = rmgo_core::gimbal::TwoAxisGimbalSolver{
            params_.pitch_lower_limit,
            params_.pitch_upper_limit,
        };
        reset_references(remote_command_reference_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!expect_interface_count(
                command_interfaces_, command_interface_names.size(), "command")) {
            return controller_interface::CallbackReturn::ERROR;
        }
        if (!expect_interface_count(
                state_interfaces_, std::to_underlying(StateInterfaceIndex::count), "state")) {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!bind_command_interfaces() || !bind_state_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }

        const double yaw = read_state(state_indexes_[to_index(StateInterfaceIndex::yaw)]);
        const double pitch = read_state(state_indexes_[to_index(StateInterfaceIndex::pitch)]);
        update_tf(yaw, pitch);
        target_yaw_ = angles::normalize_angle(yaw);
        target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        reset_references(remote_command_reference_);

        return write_commands(yaw, target_pitch_) ? controller_interface::CallbackReturn::SUCCESS
                                                  : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_references(remote_command_reference_);
        return write_commands(
                   read_state(state_indexes_[to_index(StateInterfaceIndex::yaw)]),
                   read_state(state_indexes_[to_index(StateInterfaceIndex::pitch)]))
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references(remote_command_reference_);
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
    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    static constexpr std::array<const char*, 2> command_interface_names = {"yaw", "pitch"};
    static constexpr std::array<const char*, 3> remote_command_suffixes = {
        "yaw/velocity",
        "pitch/velocity",
        "enabled",
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
        == 2 + rmgo_core::io_state_interfaces::gimbal_imu_orientation_state_interfaces.size());

    template <std::size_t N>
    static constexpr std::array<std::size_t, N> invalid_indexes() {
        std::array<std::size_t, N> indexes{};
        indexes.fill(invalid_index);
        return indexes;
    }

    double read_state(std::size_t index) const {
        return read_finite_interface_or(state_interfaces_, index, 0.0);
    }

    bool bind_command_interfaces() {
        command_indexes_.fill(invalid_index);
        return bind_prefixed_interface_indexes(
            command_interfaces_,
            {
                {&command_indexes_[to_index(CommandInterfaceIndex::yaw)], params_.yaw_joint_name,
                 params_.command_interface_name},
                {&command_indexes_[to_index(CommandInterfaceIndex::pitch)],
                 params_.pitch_joint_name, params_.command_interface_name},
            },
            "gimbal command interface");
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
                   "gimbal state interface");
    }

    void update_targets(double dt) {
        const double yaw = read_state(state_indexes_[to_index(StateInterfaceIndex::yaw)]);
        const double pitch = read_state(state_indexes_[to_index(StateInterfaceIndex::pitch)]);
        update_tf(yaw, pitch);

        const auto& [yaw_reference, pitch_reference, enabled_reference] = remote_command_reference_;
        const bool is_enabled = std::isfinite(enabled_reference) && enabled_reference > 0.5;
        if (!is_enabled) {
            solver_.update(tf_, pitch, TwoAxisGimbalSolver::SetDisabled{});
            target_yaw_ = angles::normalize_angle(yaw);
            target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
            return;
        }

        const double yaw_velocity = finite_or_zero(yaw_reference);
        const double pitch_velocity = finite_or_zero(pitch_reference);
        const auto error = solver_.enabled()
                             ? solver_.update(
                                   tf_, pitch,
                                   TwoAxisGimbalSolver::SetControlShift{
                                       yaw_velocity * dt,
                                       pitch_velocity * dt,
                                   })
                             : solver_.update(tf_, pitch, TwoAxisGimbalSolver::SetToLevel{});

        if (std::isfinite(error.yaw) && std::isfinite(error.pitch)) {
            target_yaw_ = angles::normalize_angle(yaw + error.yaw);
            target_pitch_ = std::clamp(
                pitch + error.pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        } else {
            target_yaw_ = angles::normalize_angle(yaw);
            target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        }
    }

    static double finite_or_zero(double value) { return std::isfinite(value) ? value : 0.0; }

    void update_tf(double yaw, double pitch) {
        Eigen::Quaterniond orientation{
            read_state(state_indexes_[to_index(StateInterfaceIndex::imu_w)]),
            read_state(state_indexes_[to_index(StateInterfaceIndex::imu_x)]),
            read_state(state_indexes_[to_index(StateInterfaceIndex::imu_y)]),
            read_state(state_indexes_[to_index(StateInterfaceIndex::imu_z)]),
        };
        // NOTE: is here lack of check?
        if (orientation.norm() <= 1e-6 || !orientation.coeffs().allFinite()) {
            orientation = Eigen::Quaterniond::Identity();
        }

        update_gimbal_tf(
            tf_, GimbalTfState{
                     .yaw = yaw,
                     .pitch = pitch,
                     .pitch_link_to_odom_imu = orientation,
                 });
    }

    bool write_commands(double yaw, double pitch) {
        const double current_yaw = read_state(state_indexes_[to_index(StateInterfaceIndex::yaw)]);
        const std::array<double, 2> values = {
            current_yaw + angles::shortest_angular_distance(current_yaw, yaw),
            std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit),
        };
        return write_safe_indexed_commands(
            command_interfaces_, values, command_indexes_, command_interface_names,
            "gimbal command");
    }

    double target_yaw_ = 0.0;
    double target_pitch_ = 0.0;
    std::array<std::size_t, std::to_underlying(CommandInterfaceIndex::count)> command_indexes_ =
        invalid_indexes<std::to_underlying(CommandInterfaceIndex::count)>();
    std::array<std::size_t, std::to_underlying(StateInterfaceIndex::count)> state_indexes_ =
        invalid_indexes<std::to_underlying(StateInterfaceIndex::count)>();
    std::array<double, 3> remote_command_reference_{0.0, 0.0, 0.0};
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
    controller_interface::ChainableControllerInterface)
