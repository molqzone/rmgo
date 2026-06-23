#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "rmgo_core/chassis_power_controller_config.hpp"
#include "rmgo_core/interface/reference_interfaces.hpp"
#include "rmgo_msg/msg/game_robot_status.hpp"
#include "rmgo_msg/msg/power_heat_data.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::controller::chassis {

class ChassisPowerController
    : public controller_interface::ControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        robot_status_buffer_.initRT(BufferedRobotStatus{});
        power_heat_buffer_.initRT(BufferedPowerHeatData{});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(
            params_.target_controller_name,
            std::array{rmgo_core::reference_interfaces::chassis_power_limit});
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return {
            controller_interface::interface_configuration_type::NONE,
            {},
        };
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        update_parameters(param_listener_, params_);
        target_controller_name_ = params_.target_controller_name;
        if (target_controller_name_.empty()) {
            logging::error("target_controller_name must not be empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (game_robot_status_topic_ != params_.game_robot_status_topic) {
            game_robot_status_subscriber_.reset();
        }
        if (power_heat_data_topic_ != params_.power_heat_data_topic) {
            power_heat_data_subscriber_.reset();
        }
        game_robot_status_topic_ = params_.game_robot_status_topic;
        power_heat_data_topic_ = params_.power_heat_data_topic;
        status_timeout_ = params_.status_timeout;
        policy_ = ChassisPowerPolicy{
            .safety_power_limit = params_.safety_power_limit,
            .buffer_threshold = params_.buffer_threshold,
            .power_gain = params_.power_gain,
            .max_power_limit = params_.max_power_limit,
        };

        if (!game_robot_status_subscriber_) {
            game_robot_status_subscriber_ =
                get_node()->create_subscription<rmgo_msg::msg::GameRobotStatus>(
                    game_robot_status_topic_, rclcpp::SystemDefaultsQoS(),
                    [this](const rmgo_msg::msg::GameRobotStatus& msg) {
                        robot_status_buffer_.writeFromNonRT(BufferedRobotStatus{
                            .power_limit = static_cast<double>(msg.chassis_power_limit),
                            .stamp = steady_clock_.now(),
                            .valid = true,
                        });
                    });
        }
        if (!power_heat_data_subscriber_) {
            power_heat_data_subscriber_ =
                get_node()->create_subscription<rmgo_msg::msg::PowerHeatData>(
                    power_heat_data_topic_, rclcpp::SystemDefaultsQoS(),
                    [this](const rmgo_msg::msg::PowerHeatData& msg) {
                        power_heat_buffer_.writeFromNonRT(BufferedPowerHeatData{
                            .buffer_energy = static_cast<double>(msg.chassis_buffer_energy),
                            .stamp = steady_clock_.now(),
                            .valid = true,
                        });
                    });
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_power_limit_interface()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        return write_power_limit(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                      : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return write_power_limit(0.0) ? controller_interface::CallbackReturn::SUCCESS
                                      : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type
        update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const auto now = steady_clock_.now();
        const auto robot = *robot_status_buffer_.readFromRT();
        const auto power_heat = *power_heat_buffer_.readFromRT();
        const auto referee_power_limit =
            is_fresh(robot, now) ? std::optional<double>{robot.power_limit} : std::nullopt;
        const auto buffer_energy = is_fresh(power_heat, now)
                                     ? std::optional<double>{power_heat.buffer_energy}
                                     : std::nullopt;
        const double power_limit = policy_.calculate(referee_power_limit, buffer_energy);

        return write_power_limit(power_limit) ? controller_interface::return_type::OK
                                              : controller_interface::return_type::ERROR;
    }

private:
    struct BufferedRobotStatus {
        double power_limit = 0.0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    struct BufferedPowerHeatData {
        double buffer_energy = 0.0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    struct ChassisPowerPolicy {
        double safety_power_limit = 0.0;
        double buffer_threshold = 0.0;
        double power_gain = 0.0;
        double max_power_limit = 0.0;

        double calculate(
            std::optional<double> referee_power_limit, std::optional<double> buffer_energy) const {
            if (!referee_power_limit.has_value() || !std::isfinite(*referee_power_limit)) {
                return std::clamp(safety_power_limit, 0.0, max_power_limit);
            }

            const double extra_power = buffer_energy.has_value() && std::isfinite(*buffer_energy)
                                         ? (*buffer_energy - buffer_threshold) * power_gain
                                         : 0.0;
            return std::clamp(*referee_power_limit + extra_power, 0.0, max_power_limit);
        }
    };

    template <typename Packet>
    bool is_fresh(const Packet& packet, const rclcpp::Time& now) const {
        return packet.valid
            && (status_timeout_ <= 0.0 || (now - packet.stamp).seconds() <= status_timeout_);
    }

    bool bind_power_limit_interface() {
        power_limit_interface_index_ = invalid_index;
        return bind_prefixed_interface_indexes(
            command_interfaces_,
            {
                {
                    &power_limit_interface_index_,
                    target_controller_name_,
                    rmgo_core::reference_interfaces::chassis_power_limit,
                },
            },
            "chassis power limit reference interface");
    }

    bool write_power_limit(double value) {
        if (power_limit_interface_index_ >= command_interfaces_.size()) [[unlikely]] {
            logging::error(
                "Chassis power limit reference interface '{}/{}' is not bound",
                target_controller_name_, rmgo_core::reference_interfaces::chassis_power_limit);
            return false;
        }
        if (!command_interfaces_[power_limit_interface_index_].set_value(value)) [[unlikely]] {
            logging::error(
                "Failed to write chassis power limit reference interface '{}/{}'",
                target_controller_name_, rmgo_core::reference_interfaces::chassis_power_limit);
            return false;
        }
        return true;
    }

    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();

    std::string target_controller_name_;
    std::string game_robot_status_topic_;
    std::string power_heat_data_topic_;
    double status_timeout_ = 0.0;
    ChassisPowerPolicy policy_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::size_t power_limit_interface_index_ = invalid_index;
    realtime_tools::RealtimeBuffer<BufferedRobotStatus> robot_status_buffer_;
    realtime_tools::RealtimeBuffer<BufferedPowerHeatData> power_heat_buffer_;
    rclcpp::Subscription<rmgo_msg::msg::GameRobotStatus>::SharedPtr game_robot_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::PowerHeatData>::SharedPtr power_heat_data_subscriber_;
    std::shared_ptr<::chassis_power_controller::ParamListener> param_listener_;
    ::chassis_power_controller::Params params_;
};

} // namespace rmgo_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::chassis::ChassisPowerController,
    controller_interface::ControllerInterface)
