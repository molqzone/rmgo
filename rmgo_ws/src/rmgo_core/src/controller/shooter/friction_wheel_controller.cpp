#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "rmgo_core/friction_wheel_controller_config.hpp"
#include "rmgo_core/interface/reference_interfaces.hpp"
#include "rmgo_msg/msg/shooter_status.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::shooter {

class FrictionWheelController
    : public controller_interface::ChainableControllerInterface
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
            config.names, params_.left_target_controller_name, control_velocity_suffixes);
        append_prefixed_interface_names(
            config.names, params_.right_target_controller_name, control_velocity_suffixes);
        append_prefixed_interface_names(
            config.names, params_.target_heat_controller_name, heat_event_suffixes);
        append_prefixed_interface_names(
            config.names, params_.target_bullet_feeder_controller_name, bullet_feeder_suffixes);
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        auto config = build_individual_config(
            std::array{
                params_.left_friction_joint_name + "/" + params_.friction_state_interface_name,
                params_.right_friction_joint_name + "/" + params_.friction_state_interface_name,
            });
        return config;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(shooter_reference_);
        return make_reference_interfaces(
            rmgo_core::reference_interfaces::shooter_mode_interfaces, shooter_reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        if (status_publisher_ && status_topic_ != params_.status_topic) {
            status_realtime_publisher_.reset();
            status_publisher_.reset();
        }
        status_topic_ = params_.status_topic;
        if (!status_publisher_) {
            status_publisher_ = get_node()->create_publisher<rmgo_msg::msg::ShooterStatus>(
                status_topic_, rclcpp::SystemDefaultsQoS());
            status_realtime_publisher_ =
                std::make_unique<StatusRealtimePublisher>(status_publisher_);
            const std::lock_guard lock{*status_realtime_publisher_};
            status_realtime_publisher_->msg_.header.frame_id = "shooter";
        }
        reset_references(shooter_reference_);
        reset_internal_state();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_state_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        if (status_publisher_) {
            status_publisher_->on_activate();
        }
        reset_references(shooter_reference_);
        reset_internal_state();
        return write_outputs({0.0, 0.0}, false, false)
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_internal_state();
        const bool stopped = write_outputs({0.0, 0.0}, false, false);
        if (status_publisher_) {
            status_publisher_->on_deactivate();
        }
        return stopped ? controller_interface::CallbackReturn::SUCCESS
                       : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references(shooter_reference_);
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& time, const rclcpp::Duration& period) override {
        const StateSnapshot state = read_state_snapshot();
        const double dt = std::max(0.0, period.seconds());
        const Mode mode = mode_from_value(state.mode);
        const bool friction_requested = mode != Mode::disabled;
        if (!friction_requested) {
            reset_friction_status();
        }

        const std::array<double, 2> commands =
            update_friction_commands(friction_requested && !friction_faulted_, dt);
        const bool friction_ready = update_friction_status(state, friction_requested, commands, dt);
        const bool bullet_fired = friction_ready && detect_bullet_fire(state.left_velocity);
        publish_status(
            time, state, mode, commands, friction_requested, friction_ready, bullet_fired);

        return write_outputs(commands, friction_ready, bullet_fired)
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    enum class Mode : std::uint8_t {
        disabled = 0,
        friction = 1,
        single = 2,
        automatic = 3,
        eject = 4,
        deep_eject = 5,
    };

    enum class StateInterfaceIndex : std::size_t {
        left_velocity = 0,
        right_velocity,
        count,
    };

    struct StateSnapshot {
        double left_velocity = 0.0;
        double right_velocity = 0.0;
        double mode = 0.0;
    };

    static constexpr std::size_t to_index(StateInterfaceIndex index) {
        return std::to_underlying(index);
    }

    static constexpr std::array<const char*, 1> control_velocity_suffixes = {"control_velocity"};
    static constexpr std::array<const char*, 1> heat_event_suffixes = {"bullet_fired"};
    static constexpr std::array<const char*, 2> bullet_feeder_suffixes = {
        "friction_ready",
        "bullet_fired",
    };

    static Mode mode_from_value(double value) {
        if (!std::isfinite(value)) {
            return Mode::disabled;
        }

        const auto mode = static_cast<Mode>(std::llround(value));
        switch (mode) {
        case Mode::friction:
        case Mode::single:
        case Mode::automatic:
        case Mode::eject:
        case Mode::deep_eject: return mode;
        case Mode::disabled:
        default: return Mode::disabled;
        }
    }

    bool bind_state_interfaces() {
        state_indexes_.fill(invalid_index);
        return bind_prefixed_interface_indexes(
            state_interfaces_,
            {
                {&state_indexes_[to_index(StateInterfaceIndex::left_velocity)],
                 params_.left_friction_joint_name, params_.friction_state_interface_name},
                {&state_indexes_[to_index(StateInterfaceIndex::right_velocity)],
                 params_.right_friction_joint_name, params_.friction_state_interface_name},
            },
            "friction wheel state interface");
    }

    StateSnapshot read_state_snapshot() const {
        return StateSnapshot{
            .left_velocity = read_state(StateInterfaceIndex::left_velocity),
            .right_velocity = read_state(StateInterfaceIndex::right_velocity),
            .mode = shooter_reference_[0],
        };
    }

    double read_state(StateInterfaceIndex index) const {
        return read_interface_value(state_interfaces_, state_indexes_[to_index(index)]);
    }

    std::array<double, 2> update_friction_commands(bool enabled, double dt) {
        const double step = dt / params_.friction_soft_start_stop_time;
        friction_scale_ = std::clamp(friction_scale_ + (enabled ? step : -step), 0.0, 1.0);
        return {
            friction_scale_ * params_.left_friction_velocity,
            friction_scale_ * params_.right_friction_velocity,
        };
    }

    bool update_friction_status(
        const StateSnapshot& state, bool friction_requested, const std::array<double, 2>& commands,
        double dt) {
        if (!friction_requested || friction_scale_ < 1.0) {
            friction_fault_time_ = 0.0;
            return false;
        }

        const bool faulty =
            is_velocity_below_ratio(state, 0, commands[0], params_.friction_fault_ratio)
            || is_velocity_below_ratio(state, 1, commands[1], params_.friction_fault_ratio);
        if (!faulty) {
            friction_fault_time_ = 0.0;
            return !is_velocity_below_ratio(state, 0, commands[0], params_.friction_ready_ratio)
                && !is_velocity_below_ratio(state, 1, commands[1], params_.friction_ready_ratio);
        }

        friction_fault_time_ += dt;
        if (friction_fault_time_ >= params_.friction_fault_time && !friction_faulted_) {
            friction_faulted_ = true;
            logging::warn("Friction wheel jam detected, disabling friction wheels");
        }
        return !friction_faulted_;
    }

    bool is_velocity_below_ratio(
        const StateSnapshot& state, std::size_t index, double command, double ratio) const {
        const double command_abs = std::abs(command);
        if (!std::isfinite(command_abs)) {
            return true;
        }
        if (command_abs <= 1e-6) {
            return false;
        }
        const double velocity = index == 0 ? state.left_velocity : state.right_velocity;
        if (!std::isfinite(velocity)) {
            return true;
        }
        return std::abs(velocity) < command_abs * ratio;
    }

    bool detect_bullet_fire(double primary_velocity) {
        bool fired = false;
        if (std::isfinite(last_primary_friction_velocity_)) {
            const double differential = primary_velocity - last_primary_friction_velocity_;
            if (differential < params_.bullet_drop_recovery_threshold) {
                primary_friction_velocity_decrease_integral_ += differential;
            } else {
                if (primary_friction_velocity_decrease_integral_
                        < params_.bullet_drop_integral_threshold
                    && last_primary_friction_velocity_
                           < params_.left_friction_velocity - params_.bullet_drop_velocity_margin) {
                    fired = true;
                }
                primary_friction_velocity_decrease_integral_ = 0.0;
            }
        }
        last_primary_friction_velocity_ = primary_velocity;
        return fired;
    }

    void reset_friction_status() {
        friction_fault_time_ = 0.0;
        friction_faulted_ = false;
        last_primary_friction_velocity_ = nan();
        primary_friction_velocity_decrease_integral_ = 0.0;
    }

    void reset_internal_state() {
        friction_scale_ = 0.0;
        reset_friction_status();
    }

    void publish_status(
        const rclcpp::Time& time, const StateSnapshot& state, Mode mode,
        const std::array<double, 2>& commands, bool friction_requested, bool friction_ready,
        bool bullet_fired) {
        if (status_realtime_publisher_ == nullptr || !status_realtime_publisher_->trylock()) {
            return;
        }

        auto& msg = status_realtime_publisher_->msg_;
        msg.header.stamp = time;
        msg.mode = static_cast<std::uint8_t>(mode);
        msg.friction_requested = friction_requested;
        msg.friction_ready = friction_ready;
        msg.friction_faulted = friction_faulted_;
        msg.bullet_fired = bullet_fired;
        msg.left_friction_velocity = state.left_velocity;
        msg.right_friction_velocity = state.right_velocity;
        msg.left_control_velocity = commands[0];
        msg.right_control_velocity = commands[1];
        status_realtime_publisher_->unlockAndPublish();
    }

    bool write_outputs(
        const std::array<double, 2>& control_velocities, bool friction_ready, bool bullet_fired) {
        std::size_t offset = 0;
        bool ok = write_safe_commands(
            command_interfaces_, std::array{control_velocities[0]},
            params_.left_target_controller_name, control_velocity_suffixes, "friction control",
            offset);
        offset += control_velocity_suffixes.size();
        ok = ok
          && write_safe_commands(
                 command_interfaces_, std::array{control_velocities[1]},
                 params_.right_target_controller_name, control_velocity_suffixes,
                 "friction control", offset);
        offset += control_velocity_suffixes.size();
        ok = ok
          && write_safe_commands(
                 command_interfaces_, std::array{as_double(bullet_fired)},
                 params_.target_heat_controller_name, heat_event_suffixes, "heat event", offset);
        offset += heat_event_suffixes.size();
        ok = ok
          && write_safe_commands(
                 command_interfaces_,
                 std::array{
                     as_double(friction_ready),
                     as_double(bullet_fired),
                 },
                 params_.target_bullet_feeder_controller_name, bullet_feeder_suffixes,
                 "bullet feeder status", offset);
        return ok;
    }

    static constexpr double nan() { return std::numeric_limits<double>::quiet_NaN(); }

    static constexpr double as_double(bool value) { return value ? 1.0 : 0.0; }

    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    std::array<std::size_t, std::to_underlying(StateInterfaceIndex::count)> state_indexes_{};
    std::array<double, rmgo_core::reference_interfaces::shooter_mode_interfaces.size()>
        shooter_reference_{0.0};
    double friction_scale_ = 0.0;
    double friction_fault_time_ = 0.0;
    bool friction_faulted_ = false;
    double last_primary_friction_velocity_ = nan();
    double primary_friction_velocity_decrease_integral_ = 0.0;
    std::string status_topic_ = "/shooter/status";
    rclcpp_lifecycle::LifecyclePublisher<rmgo_msg::msg::ShooterStatus>::SharedPtr status_publisher_;
    using StatusRealtimePublisher = realtime_tools::RealtimePublisher<rmgo_msg::msg::ShooterStatus>;
    std::unique_ptr<StatusRealtimePublisher> status_realtime_publisher_;
    std::shared_ptr<::friction_wheel_controller::ParamListener> param_listener_;
    ::friction_wheel_controller::Params params_;
};

} // namespace rmgo_core::controller::shooter

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::shooter::FrictionWheelController,
    controller_interface::ChainableControllerInterface)
