#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/heat_controller_config.hpp"
#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::shooter {

class HeatController
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
            params_.target_bullet_feeder_controller_name, bullet_allowance_suffixes);
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        using namespace rmgo_core::io_state_interfaces;
        return build_individual_config(std::array{
            std::string{referee_shooter_cooling},
            std::string{referee_shooter_heat_limit},
        });
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_reference_interfaces_list() override {
        reset_references(reference_);
        return make_reference_interfaces(reference_suffixes, reference_);
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset();
        return write_allowance(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                    : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        reset();
        return write_allowance(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                    : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        if (!is_in_chained_mode()) {
            reset_references(reference_);
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
        const double cooling = read_state(cooling_index);
        const double heat_limit = read_state(heat_limit_index);
        heat_ = std::max(0.0, heat_ - std::max(0.0, cooling) * std::max(0.0, period.seconds()));

        const bool bullet_fired = reference_[0] > 0.5;
        if (bullet_fired && !last_bullet_fired_) {
            heat_ += params_.heat_per_shot + params_.extra_heat_per_shot;
        }
        last_bullet_fired_ = bullet_fired;

        const double allowance =
            std::floor((heat_limit - heat_ - params_.reserved_heat) / params_.heat_per_shot);
        return write_allowance(std::max(0.0, allowance)) ? controller_interface::return_type::OK
                                                         : controller_interface::return_type::ERROR;
    }

private:
    static constexpr std::array<const char*, 1> reference_suffixes = {"bullet_fired"};
    static constexpr std::array<const char*, 1> bullet_allowance_suffixes = {
        "control_bullet_allowance/limited_by_heat",
    };
    static constexpr std::size_t cooling_index = 0;
    static constexpr std::size_t heat_limit_index = 1;

    double read_state(std::size_t index) const {
        return read_interface_value(state_interfaces_, index);
    }

    void reset() {
        reset_references(reference_);
        heat_ = 0.0;
        last_bullet_fired_ = false;
    }

    bool write_allowance(double allowance) {
        return write_safe_commands(
            command_interfaces_, std::array{allowance},
            params_.target_bullet_feeder_controller_name, bullet_allowance_suffixes,
            "heat-limited bullet allowance");
    }

    std::array<double, 1> reference_{0.0};
    double heat_ = 0.0;
    bool last_bullet_fired_ = false;
    std::shared_ptr<::heat_controller::ParamListener> param_listener_;
    ::heat_controller::Params params_;
};

} // namespace rmgo_core::controller::shooter

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::shooter::HeatController,
    controller_interface::ChainableControllerInterface)
