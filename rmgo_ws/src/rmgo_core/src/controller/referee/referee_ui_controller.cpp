#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "rmgo_core/referee/referee_transfer_registry.hpp"
#include "rmgo_core/referee/referee_ui.hpp"
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
        transfer_endpoint_ = {};
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        refresh_endpoint();
        ui_scheduler_.reset_remote_state();
        last_game_stage_ = unknown_game_stage;
        last_online_ = false;
        chassis_power_ui_.set_visible(true);
        chassis_power_limit_ui_.set_visible(true);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        chassis_power_ui_.set_visible(false);
        chassis_power_limit_ui_.set_visible(false);
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

        chassis_power_ui_.set_visible(online);
        chassis_power_limit_ui_.set_visible(online);
        if (online) {
            chassis_power_ui_.set_value(
                static_cast<std::int32_t>(std::lround(read_state(StateIndex::chassis_power))));
            chassis_power_limit_ui_.set_value(
                static_cast<std::int32_t>(
                    std::lround(read_state(StateIndex::chassis_power_limit))));
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
    std::weak_ptr<rmgo_core::referee::RefereeTransferEndpoint> transfer_endpoint_;
    double last_game_stage_ = unknown_game_stage;
    bool last_online_ = false;
    rmgo_core::referee::ui::UiScheduler ui_scheduler_;
    rmgo_core::referee::ui::Integer chassis_power_ui_{
        ui_scheduler_,
        rmgo_core::referee::ui::Color::white,
        15,
        2,
        rmgo_core::referee::ui::x_center,
        100,
        0};
    rmgo_core::referee::ui::Integer chassis_power_limit_ui_{
        ui_scheduler_,
        rmgo_core::referee::ui::Color::white,
        15,
        2,
        rmgo_core::referee::ui::x_center,
        150,
        0};

    static constexpr double unknown_game_stage = 0.0;
    static constexpr double preparation_game_stage = 1.0;
};

} // namespace rmgo_core::controller::referee

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::referee::RefereeUiController, controller_interface::ControllerInterface)
