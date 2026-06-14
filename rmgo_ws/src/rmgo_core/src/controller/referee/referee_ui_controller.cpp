#include <cstddef>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "rmgo_core/referee/referee_transfer_registry.hpp"
#include "rmgo_core/referee/referee_ui.hpp"
#include "rmgo_core/referee/ui/profile.hpp"
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
        profile_name_ = node->declare_parameter<std::string>("profile", "infantry");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return {
            controller_interface::interface_configuration_type::NONE,
            {},
        };
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return build_individual_config(rmgo_core::io_state_interfaces::referee_state_interfaces);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        get_node()->get_parameter("transfer_path", transfer_path_);
        get_node()->get_parameter("profile", profile_name_);
        ui_profile_ = rmgo_core::referee::ui::make_ui_profile(profile_name_, ui_scheduler_);
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
        refresh_endpoint();
        ui_scheduler_.reset_remote_state();
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
            (void)ui_scheduler_.update(*endpoint);
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
            ui_scheduler_.reset_remote_state();
        }
        last_online_ = online;
        last_game_stage_ = game_stage;

        if (ui_profile_ != nullptr) {
            ui_profile_->update(
                rmgo_core::referee::ui::RefereeUiState{
                    .online = online,
                    .game_stage = game_stage,
                    .chassis_power_limit = read_state(StateIndex::chassis_power_limit),
                    .chassis_power = read_state(StateIndex::chassis_power),
                });
        }
        if (auto endpoint = refresh_endpoint(); endpoint != nullptr) {
            (void)ui_scheduler_.update(*endpoint);
        }
        return controller_interface::return_type::OK;
    }

private:
    enum class StateIndex : std::size_t {
        online = 0,
        game_stage = 2,
        chassis_power_limit = 11,
        chassis_power = 12,
    };

    double read_state(StateIndex index) const {
        return read_interface_value(state_interfaces_, static_cast<std::size_t>(index));
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
    std::string profile_name_{"infantry"};
    std::weak_ptr<rmgo_core::referee::RefereeTransferEndpoint> transfer_endpoint_;
    double last_game_stage_ = unknown_game_stage;
    bool last_online_ = false;
    rmgo_core::referee::ui::UiScheduler ui_scheduler_;
    std::unique_ptr<rmgo_core::referee::ui::UiProfile> ui_profile_;

    static constexpr double unknown_game_stage = 0.0;
    static constexpr double preparation_game_stage = 1.0;
};

} // namespace rmgo_core::controller::referee

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::referee::RefereeUiController, controller_interface::ControllerInterface)
