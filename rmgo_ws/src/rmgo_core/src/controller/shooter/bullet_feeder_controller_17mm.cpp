#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/bullet_feeder_controller_17mm_config.hpp"
#include "rmgo_core/interface/command_state_interfaces.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::shooter {

class BulletFeederController17mm
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
            params_.target_velocity_pid_controller_name, control_velocity_suffixes);
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        auto config = build_individual_config(std::array{
            params_.bullet_feeder_joint_name + "/" + params_.feeder_velocity_state_interface_name,
        });
        append_interface_names(
            config.names, rmgo_core::command_state_interfaces::shooter_interfaces);
        return config;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(reference_);
        return make_reference_interfaces(reference_suffixes, reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        update_derived_parameters();
        reset_internal_state();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_state_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        reset_internal_state();
        return write_feeder_command(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                         : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset_internal_state();
        return write_feeder_command(0.0) ? controller_interface::CallbackReturn::SUCCESS
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
        const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
        const StateSnapshot state = read_state_snapshot();
        const double dt = std::max(0.0, period.seconds());
        const Mode mode = mode_from_value(state.mode);
        const std::uint64_t sequence = sequence_from_value(state.sequence);
        const bool friction_ready = reference_[to_index(ReferenceIndex::friction_ready)] > 0.5;
        const bool bullet_fired = reference_[to_index(ReferenceIndex::bullet_fired)] > 0.5;
        const int heat_allowance = static_cast<int>(
            std::max(0.0, std::floor(reference_[to_index(ReferenceIndex::heat_allowance)])));

        update_mode_events(mode, sequence, bullet_fired);
        const double command = calculate_feeder_command(
            mode, friction_ready, heat_allowance, state.feeder_velocity, dt);
        last_feeder_command_ = command;

        return write_feeder_command(command) ? controller_interface::return_type::OK
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

    enum class ReferenceIndex : std::size_t {
        friction_ready = 0,
        bullet_fired,
        heat_allowance,
        count,
    };

    enum class StateInterfaceIndex : std::size_t {
        feeder_velocity = 0,
        command_mode,
        command_sequence,
        count,
    };

    struct StateSnapshot {
        double feeder_velocity = 0.0;
        double mode = 0.0;
        double sequence = 0.0;
    };

    static constexpr std::size_t to_index(ReferenceIndex index) {
        return std::to_underlying(index);
    }

    static constexpr std::size_t to_index(StateInterfaceIndex index) {
        return std::to_underlying(index);
    }

    static constexpr double stalled_velocity_ratio = 0.5;
    static constexpr std::array<const char*, std::to_underlying(ReferenceIndex::count)>
        reference_suffixes = {
            "friction_ready",
            "bullet_fired",
            "control_bullet_allowance/limited_by_heat",
        };
    static constexpr std::array<const char*, 1> control_velocity_suffixes = {"control_velocity"};

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

    static std::uint64_t sequence_from_value(double value) {
        if (!std::isfinite(value) || value <= 0.0) {
            return 0;
        }
        return static_cast<std::uint64_t>(std::llround(value));
    }

    bool bind_state_interfaces() {
        using namespace rmgo_core::command_state_interfaces;
        state_indexes_.fill(invalid_index);
        return bind_prefixed_interface_indexes(
                   state_interfaces_,
                   {
                       {&state_indexes_[to_index(StateInterfaceIndex::feeder_velocity)],
                        params_.bullet_feeder_joint_name,
                        params_.feeder_velocity_state_interface_name},
                   },
                   "bullet feeder state interface")
            && bind_interface_indexes(
                   state_interfaces_,
                   {
                       {&state_indexes_[to_index(StateInterfaceIndex::command_mode)], shooter_mode},
                       {&state_indexes_[to_index(StateInterfaceIndex::command_sequence)],
                        shooter_sequence},
                   },
                   "bullet feeder command state interface");
    }

    StateSnapshot read_state_snapshot() const {
        return StateSnapshot{
            .feeder_velocity = read_state(StateInterfaceIndex::feeder_velocity),
            .mode = read_state(StateInterfaceIndex::command_mode),
            .sequence = read_state(StateInterfaceIndex::command_sequence),
        };
    }

    double read_state(StateInterfaceIndex index) const {
        return read_interface_value(state_interfaces_, state_indexes_[to_index(index)]);
    }

    void update_derived_parameters() {
        const double bullet_angle_per_shot =
            2.0 * std::numbers::pi / params_.bullets_per_feeder_turn;
        feeder_working_velocity_ = bullet_angle_per_shot * params_.shot_frequency;
        feeder_safe_velocity_ = bullet_angle_per_shot * params_.safe_shot_frequency;
        feeder_eject_velocity_ = -bullet_angle_per_shot * params_.eject_frequency;
        feeder_deep_eject_velocity_ = -bullet_angle_per_shot * params_.deep_eject_frequency;
    }

    void update_mode_events(Mode mode, std::uint64_t sequence, bool bullet_fired) {
        const bool command_event = sequence != 0 && sequence != last_command_sequence_;
        const bool mode_changed = mode != last_mode_;
        if (command_event) {
            last_command_sequence_ = sequence;
        }

        if (mode != Mode::single) {
            single_shot_active_ = false;
        }
        if (bullet_fired) {
            single_shot_active_ = false;
        }
        if (mode == Mode::disabled) {
            feeder_jammed_count_ = 0;
            eject_remaining_time_ = 0.0;
        }

        if (!command_event && !mode_changed) {
            return;
        }

        switch (mode) {
        case Mode::single:
            single_shot_active_ = true;
            single_shot_elapsed_ = 0.0;
            break;
        case Mode::eject: start_eject(false); break;
        case Mode::deep_eject: start_eject(true); break;
        case Mode::friction:
        case Mode::automatic:
        case Mode::disabled:
        default: break;
        }

        last_mode_ = mode;
    }

    double calculate_feeder_command(
        Mode mode, bool friction_ready, int heat_allowance, double feeder_velocity, double dt) {
        if (eject_remaining_time_ > 0.0) {
            eject_remaining_time_ = std::max(0.0, eject_remaining_time_ - dt);
            feeder_working_status_time_ = 0.0;
            return active_eject_velocity_;
        }

        if (!friction_ready) {
            feeder_working_status_time_ = 0.0;
            return 0.0;
        }

        double command = 0.0;
        if (mode == Mode::automatic && heat_allowance > 0) {
            command = heat_allowance > 1 ? feeder_working_velocity_ : feeder_safe_velocity_;
        } else if (mode == Mode::single && single_shot_active_) {
            single_shot_elapsed_ += dt;
            if (single_shot_elapsed_ <= params_.single_shot_max_stop_delay && heat_allowance > 0) {
                command = feeder_safe_velocity_;
            } else if (single_shot_elapsed_ > params_.single_shot_max_stop_delay) {
                single_shot_active_ = false;
            }
        }

        if (command <= 0.0) {
            feeder_working_status_time_ = 0.0;
            return 0.0;
        }

        if (command > last_feeder_command_) {
            feeder_working_status_time_ = std::min(0.0, feeder_working_status_time_);
        }
        if (update_jam_detection(command, feeder_velocity, dt)) {
            return active_eject_velocity_;
        }
        return command;
    }

    bool update_jam_detection(double command, double feeder_velocity, double dt) {
        const double measured_velocity = std::abs(feeder_velocity);
        if (measured_velocity >= std::abs(command) * stalled_velocity_ratio) {
            if (feeder_working_status_time_ < 0.0) {
                feeder_working_status_time_ = 0.0;
            } else if (feeder_working_status_time_ < params_.feeder_jam_time) {
                feeder_working_status_time_ += dt;
            } else {
                feeder_jammed_count_ = 0;
            }
            return false;
        }

        if (feeder_working_status_time_ >= params_.feeder_jam_time) {
            start_eject(++feeder_jammed_count_ > 2);
            single_shot_active_ = false;
            feeder_working_status_time_ = 0.0;
            logging::warn("Bullet feeder instant jam detected, reversing feeder");
            return true;
        }
        if (feeder_working_status_time_ > 0.0) {
            feeder_working_status_time_ = 0.0;
            return false;
        }
        if (feeder_working_status_time_ > -params_.feeder_jam_time) {
            feeder_working_status_time_ -= dt;
            return false;
        }

        start_eject(++feeder_jammed_count_ > 2);
        single_shot_active_ = false;
        feeder_working_status_time_ = 0.0;
        logging::warn("Bullet feeder jam detected, reversing feeder");
        return true;
    }

    void start_eject(bool deep) {
        active_eject_velocity_ = deep ? feeder_deep_eject_velocity_ : feeder_eject_velocity_;
        eject_remaining_time_ = deep ? params_.deep_eject_time : params_.eject_time;
    }

    void reset_internal_state() {
        reset_references(reference_);
        feeder_working_status_time_ = 0.0;
        feeder_jammed_count_ = 0;
        eject_remaining_time_ = 0.0;
        active_eject_velocity_ = 0.0;
        single_shot_active_ = false;
        single_shot_elapsed_ = 0.0;
        last_feeder_command_ = 0.0;
        last_mode_ = Mode::disabled;
        last_command_sequence_ = 0;
    }

    bool write_feeder_command(double value) {
        return write_safe_commands(
            command_interfaces_, std::array{value}, params_.target_velocity_pid_controller_name,
            control_velocity_suffixes, "bullet feeder control");
    }

    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    std::array<std::size_t, std::to_underlying(StateInterfaceIndex::count)> state_indexes_{};
    std::array<double, std::to_underlying(ReferenceIndex::count)> reference_{0.0, 0.0, 0.0};
    double feeder_working_velocity_ = 0.0;
    double feeder_safe_velocity_ = 0.0;
    double feeder_eject_velocity_ = 0.0;
    double feeder_deep_eject_velocity_ = 0.0;
    double feeder_working_status_time_ = 0.0;
    int feeder_jammed_count_ = 0;
    double eject_remaining_time_ = 0.0;
    double active_eject_velocity_ = 0.0;
    bool single_shot_active_ = false;
    double single_shot_elapsed_ = 0.0;
    double last_feeder_command_ = 0.0;
    Mode last_mode_ = Mode::disabled;
    std::uint64_t last_command_sequence_ = 0;
    std::shared_ptr<::bullet_feeder_controller_17mm::ParamListener> param_listener_;
    ::bullet_feeder_controller_17mm::Params params_;
};

} // namespace rmgo_core::controller::shooter

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::shooter::BulletFeederController17mm,
    controller_interface::ChainableControllerInterface)
