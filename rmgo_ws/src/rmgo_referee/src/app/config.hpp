#pragma once

#include <chrono>
#include <cmath>
#include <exception>
#include <optional>
#include <string>
#include <string_view>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_referee {

struct RefereeConfig {
    std::string device = "/dev/usbReferee";
    std::string status_topic = "/referee/status";
    std::string game_status_topic = "/referee/game_status";
    std::string game_robot_status_topic = "/referee/game_robot_status";
    std::string power_heat_data_topic = "/referee/power_heat_data";
    std::string chassis_status_topic = "/chassis/status";
    std::string gimbal_status_topic = "/gimbal/status";
    std::string shooter_status_topic = "/shooter/status";
    std::string remote_status_topic = "/remote/status";
    std::string target_status_topic = "/target/status";
    std::string capacitor_status_topic = "/capacitor/status";
    std::string profile_name = "omni_infantry";
    int rx_buffer_size = 1024;
    int tx_queue_capacity = 64;
    double online_timeout = 1.0;
    std::chrono::duration<double> publish_period{0.02};
    std::chrono::duration<double> ui_period{0.01};
    std::chrono::duration<double> transport_watchdog_period{0.5};
};

class RefereeConfigLoader final : public rmgo_utility::NodeMixin {
public:
    explicit RefereeConfigLoader(rclcpp_lifecycle::LifecycleNode& node) noexcept
        : node_(node) {}

    rclcpp_lifecycle::LifecycleNode* get_node() noexcept { return &node_; }
    const rclcpp_lifecycle::LifecycleNode* get_node() const noexcept { return &node_; }

    std::optional<RefereeConfig> load() {
        auto config = RefereeConfig{};
        const auto parameters_valid = load_parameters(config);
        const auto values_valid = validate(config);
        if (!parameters_valid || !values_valid) {
            return std::nullopt;
        }
        return config;
    }

private:
    bool load_parameters(RefereeConfig& config) {
        auto valid = true;
        const auto load = [this, &valid](const std::string& name, auto& output) {
            if (!load_parameter(name, output)) {
                valid = false;
            }
        };

        load("device", config.device);
        load("rx_buffer_size", config.rx_buffer_size);
        load("tx_queue_capacity", config.tx_queue_capacity);
        load("online_timeout", config.online_timeout);
        load("status_topic", config.status_topic);
        load("game_status_topic", config.game_status_topic);
        load("game_robot_status_topic", config.game_robot_status_topic);
        load("power_heat_data_topic", config.power_heat_data_topic);
        load("chassis_status_topic", config.chassis_status_topic);
        load("gimbal_status_topic", config.gimbal_status_topic);
        load("shooter_status_topic", config.shooter_status_topic);
        load("remote_status_topic", config.remote_status_topic);
        load("target_status_topic", config.target_status_topic);
        load("capacitor_status_topic", config.capacitor_status_topic);
        load("profile", config.profile_name);

        auto publish_period = config.publish_period.count();
        auto ui_period = config.ui_period.count();
        auto transport_watchdog_period = config.transport_watchdog_period.count();
        load("publish_period", publish_period);
        load("ui_period", ui_period);
        load("transport_watchdog_period", transport_watchdog_period);
        config.publish_period = std::chrono::duration<double>{publish_period};
        config.ui_period = std::chrono::duration<double>{ui_period};
        config.transport_watchdog_period = std::chrono::duration<double>{transport_watchdog_period};

        return valid;
    }

    template <typename T>
    bool load_parameter(const std::string& name, T& output) {
        try {
            if (node_.has_parameter(name)) {
                output = node_.get_parameter(name).get_value<T>();
            } else {
                output = node_.declare_parameter<T>(name);
            }
            return true;
        } catch (const std::exception& exception) {
            logging::error(
                "Missing or invalid required referee parameter '{}': {}", name, exception.what());
            return false;
        }
    }

    bool validate(const RefereeConfig& config) {
        auto valid = true;
        const auto require_text = [this, &valid](std::string_view name, const std::string& value) {
            if (value.empty()) {
                logging::error("Required referee parameter '{}' must not be empty", name);
                valid = false;
            }
        };

        require_text("device", config.device);
        require_text("status_topic", config.status_topic);
        require_text("game_status_topic", config.game_status_topic);
        require_text("game_robot_status_topic", config.game_robot_status_topic);
        require_text("power_heat_data_topic", config.power_heat_data_topic);
        require_text("chassis_status_topic", config.chassis_status_topic);
        require_text("gimbal_status_topic", config.gimbal_status_topic);
        require_text("shooter_status_topic", config.shooter_status_topic);
        require_text("remote_status_topic", config.remote_status_topic);
        require_text("target_status_topic", config.target_status_topic);
        require_text("capacitor_status_topic", config.capacitor_status_topic);
        require_text("profile", config.profile_name);

        if (config.rx_buffer_size <= 0 || config.tx_queue_capacity <= 0) {
            logging::error("rx_buffer_size and tx_queue_capacity must be positive");
            valid = false;
        }
        if (!std::isfinite(config.online_timeout) || config.online_timeout < 0.0) {
            logging::error("online_timeout must be finite and non-negative");
            valid = false;
        }
        if (!std::isfinite(config.publish_period.count()) || config.publish_period.count() <= 0.0
            || !std::isfinite(config.ui_period.count()) || config.ui_period.count() <= 0.0
            || !std::isfinite(config.transport_watchdog_period.count())
            || config.transport_watchdog_period.count() <= 0.0) {
            logging::error(
                "publish_period, ui_period, and transport_watchdog_period must be finite and "
                "positive");
            valid = false;
        }
        return valid;
    }

    rclcpp_lifecycle::LifecycleNode& node_;
};

} // namespace rmgo_referee
