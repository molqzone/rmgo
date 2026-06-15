#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/command_state_interfaces.hpp"
#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "referee/transfer_registry.hpp"
#include "referee/ui/ui.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::referee {

class RefereeUiController
    : public controller_interface::ControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        auto node = get_node();
        transfer_path_ = node->declare_parameter<std::string>(
            "transfer_path", std::string{rmgo_core::referee::default_transfer_path});
        profile_name_ = node->declare_parameter<std::string>("profile", "omni_infantry");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return {
            controller_interface::interface_configuration_type::NONE,
            {},
        };
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        auto config = build_individual_config(rmgo_core::io_state_interfaces::referee_state_interfaces);
        append_interface_names(config.names, rmgo_core::command_state_interfaces::all_interfaces);
        return config;
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        get_node()->get_parameter("transfer_path", transfer_path_);
        get_node()->get_parameter("profile", profile_name_);
        ui_profile_ = rmgo_core::referee::ui::make_ui_profile(profile_name_, interaction_ui_);
        if (ui_profile_ == nullptr) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Unknown referee UI profile '%s'", profile_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        transfer_endpoint_ = {};
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_state_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        refresh_endpoint();
        interaction_ui_.reset_remote_state();
        last_game_stage_ = unknown_game_stage;
        last_online_ = false;
        if (ui_profile_ == nullptr) {
            return controller_interface::CallbackReturn::ERROR;
        }
        ui_profile_->on_activate();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (ui_profile_ != nullptr) {
            ui_profile_->on_deactivate();
        }
        if (auto endpoint = refresh_endpoint(); endpoint != nullptr) {
            (void)interaction_ui_.clear_remote_state(*endpoint);
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
        update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const bool online = read_state(StateIndex::online) > 0.5;
        const auto game_stage = read_state(StateIndex::game_stage);
        if (online
            && ((!last_online_ && online)
                || (last_game_stage_ == unknown_game_stage && game_stage != unknown_game_stage)
                || (last_game_stage_ != preparation_game_stage
                    && game_stage == preparation_game_stage))) {
            interaction_ui_.reset_remote_state();
        }
        last_online_ = online;
        last_game_stage_ = game_stage;

        if (ui_profile_ != nullptr) {
            ui_profile_->update(
                rmgo_core::referee::ui::RefereeUiState{
                    .online = online,
                    .robot_id = read_state(StateIndex::robot_id),
                    .game_stage = game_stage,
                    .stage_remain_time = read_state(StateIndex::stage_remain_time),
                    .hp = read_state(StateIndex::hp),
                    .max_hp = read_state(StateIndex::max_hp),
                    .shooter_cooling = read_state(StateIndex::shooter_cooling),
                    .shooter_heat_limit = read_state(StateIndex::shooter_heat_limit),
                    .shooter_bullet_allowance = read_state(StateIndex::shooter_bullet_allowance),
                    .shooter_1_heat = read_state(StateIndex::shooter_1_heat),
                    .shooter_2_heat = read_state(StateIndex::shooter_2_heat),
                    .chassis_power_limit = read_state(StateIndex::chassis_power_limit),
                    .chassis_power = read_state(StateIndex::chassis_power),
                    .chassis_buffer_energy = read_state(StateIndex::chassis_buffer_energy),
                    .chassis_output_status = read_state(StateIndex::chassis_output_status),
                    .chassis_mode = read_state(StateIndex::chassis_mode),
                    .gimbal_enabled = read_state(StateIndex::gimbal_enabled),
                    .shooter_mode = read_state(StateIndex::shooter_mode),
                });
        }
        if (auto endpoint = refresh_endpoint(); endpoint != nullptr) {
            (void)interaction_ui_.update(*endpoint);
        }
        return controller_interface::return_type::OK;
    }

private:
    enum class StateIndex : std::size_t {
        online,
        robot_id,
        game_stage,
        stage_remain_time,
        hp,
        max_hp,
        shooter_cooling,
        shooter_heat_limit,
        shooter_bullet_allowance,
        shooter_1_heat,
        shooter_2_heat,
        chassis_power_limit,
        chassis_power,
        chassis_buffer_energy,
        chassis_output_status,
        chassis_mode,
        gimbal_enabled,
        shooter_mode,
        count,
    };

    double read_state(StateIndex index) const {
        return read_interface_value(state_interfaces_, state_indexes_[to_index(index)]);
    }

    bool bind_state_interfaces() {
        using namespace rmgo_core::command_state_interfaces;
        using namespace rmgo_core::io_state_interfaces;
        state_indexes_.fill(invalid_index);
        return bind_interface_indexes(
            state_interfaces_,
            {
                {&state_indexes_[to_index(StateIndex::online)], referee_online},
                {&state_indexes_[to_index(StateIndex::robot_id)], referee_id},
                {&state_indexes_[to_index(StateIndex::game_stage)], referee_game_stage},
                {&state_indexes_[to_index(StateIndex::stage_remain_time)],
                 referee_game_stage_remain_time},
                {&state_indexes_[to_index(StateIndex::hp)], referee_hp},
                {&state_indexes_[to_index(StateIndex::max_hp)], referee_max_hp},
                {&state_indexes_[to_index(StateIndex::shooter_cooling)], referee_shooter_cooling},
                {&state_indexes_[to_index(StateIndex::shooter_heat_limit)],
                 referee_shooter_heat_limit},
                {&state_indexes_[to_index(StateIndex::shooter_bullet_allowance)],
                 referee_shooter_bullet_allowance},
                {&state_indexes_[to_index(StateIndex::shooter_1_heat)], referee_shooter_1_heat},
                {&state_indexes_[to_index(StateIndex::shooter_2_heat)], referee_shooter_2_heat},
                {&state_indexes_[to_index(StateIndex::chassis_power_limit)],
                 referee_chassis_power_limit},
                {&state_indexes_[to_index(StateIndex::chassis_power)], referee_chassis_power},
                {&state_indexes_[to_index(StateIndex::chassis_buffer_energy)],
                 referee_chassis_buffer_energy},
                {&state_indexes_[to_index(StateIndex::chassis_output_status)],
                 referee_chassis_output_status},
                {&state_indexes_[to_index(StateIndex::chassis_mode)], chassis_mode},
                {&state_indexes_[to_index(StateIndex::gimbal_enabled)], gimbal_enabled},
                {&state_indexes_[to_index(StateIndex::shooter_mode)], shooter_mode},
            },
            "referee UI state interface");
    }

    std::shared_ptr<rmgo_core::referee::RefereeTransferEndpoint> refresh_endpoint() {
        auto endpoint = transfer_endpoint_.lock();
        if (endpoint == nullptr) {
            endpoint = rmgo_core::referee::get_referee_transfer_endpoint(transfer_path_);
            transfer_endpoint_ = endpoint;
        }
        return endpoint;
    }

    std::string transfer_path_{rmgo_core::referee::default_transfer_path};
    std::string profile_name_{"omni_infantry"};
    std::weak_ptr<rmgo_core::referee::RefereeTransferEndpoint> transfer_endpoint_;
    double last_game_stage_ = unknown_game_stage;
    bool last_online_ = false;
    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    static constexpr std::size_t to_index(StateIndex index) {
        return static_cast<std::size_t>(index);
    }
    std::array<std::size_t, static_cast<std::size_t>(StateIndex::count)> state_indexes_{};
    rmgo_core::referee::ui::InteractionUi interaction_ui_;
    std::unique_ptr<rmgo_core::referee::ui::UiProfile> ui_profile_;

    static constexpr double unknown_game_stage = 0.0;
    static constexpr double preparation_game_stage = 1.0;
};

} // namespace rmgo_core::controller::referee

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::referee::RefereeUiController, controller_interface::ControllerInterface)
