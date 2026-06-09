#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rmgo_core/teleop_remote_controller_config.hpp"

namespace rmgo_core {

class TeleopRemoteController : public controller_interface::ControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override {
        param_listener_ = std::make_shared<::teleop_remote_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        target_controller_name_ = params_.target_controller_name;
        target_gimbal_controller_name_ = params_.target_gimbal_controller_name;
        command_buffer_.initRT(BufferedCommand{});
        gimbal_command_buffer_.initRT(BufferedGimbalCommand{});
        mode_buffer_.initRT(raw_mode);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        const std::string target_controller_name = params_.target_controller_name;
        config.names.reserve(
            command_interface_suffixes.size()
            + (params_.target_gimbal_controller_name.empty()
                   ? 0
                   : gimbal_command_interface_suffixes.size()));
        for (const char* suffix : command_interface_suffixes) {
            config.names.push_back(target_controller_name + "/" + suffix);
        }
        if (!params_.target_gimbal_controller_name.empty()) {
            for (const char* suffix : gimbal_command_interface_suffixes) {
                config.names.push_back(params_.target_gimbal_controller_name + "/" + suffix);
            }
        }

        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return {
            controller_interface::interface_configuration_type::NONE,
            {},
        };
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        node_ = get_node();
        params_ = param_listener_->get_params();
        target_controller_name_ = params_.target_controller_name;
        target_gimbal_controller_name_ = params_.target_gimbal_controller_name;
        if (cmd_vel_subscriber_ && cmd_vel_topic_ != params_.cmd_vel_topic) {
            cmd_vel_subscriber_.reset();
        }
        if (cmd_gimbal_subscriber_ && cmd_gimbal_topic_ != params_.cmd_gimbal_topic) {
            cmd_gimbal_subscriber_.reset();
        }
        if (mode_subscriber_ && mode_topic_ != params_.mode_topic) {
            mode_subscriber_.reset();
        }

        cmd_vel_topic_ = params_.cmd_vel_topic;
        cmd_gimbal_topic_ = params_.cmd_gimbal_topic;
        mode_topic_ = params_.mode_topic;
        command_timeout_ = params_.command_timeout;

        if (!cmd_vel_subscriber_) {
            cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
                [this](const geometry_msgs::msg::Twist& msg) {
                    command_buffer_.writeFromNonRT(
                        BufferedCommand{
                            msg.linear.x,
                            msg.linear.y,
                            msg.angular.z,
                            steady_clock_.now(),
                            true,
                        });
                });
        }
        if (!target_gimbal_controller_name_.empty() && !cmd_gimbal_subscriber_) {
            cmd_gimbal_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                cmd_gimbal_topic_, rclcpp::SystemDefaultsQoS(),
                [this](const geometry_msgs::msg::Twist& msg) {
                    gimbal_command_buffer_.writeFromNonRT(
                        BufferedGimbalCommand{
                            msg.angular.z,
                            msg.angular.y,
                            steady_clock_.now(),
                            true,
                        });
                });
        }
        if (!mode_subscriber_) {
            mode_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
                mode_topic_, rclcpp::SystemDefaultsQoS(),
                [this](const std_msgs::msg::UInt8& msg) { mode_buffer_.writeFromNonRT(msg.data); });
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        const std::size_t expected_interfaces =
            command_interface_suffixes.size()
            + (target_gimbal_controller_name_.empty() ? 0
                                                      : gimbal_command_interface_suffixes.size());
        if (command_interfaces_.size() != expected_interfaces) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                expected_interfaces, command_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        command_buffer_.writeFromNonRT(BufferedCommand{});
        gimbal_command_buffer_.writeFromNonRT(BufferedGimbalCommand{});
        mode_buffer_.writeFromNonRT(raw_mode);
        return write_command({0.0, 0.0, 0.0, raw_mode})
                    && write_gimbal_command({0.0, 0.0, disabled_gimbal})
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        command_buffer_.writeFromNonRT(BufferedCommand{});
        gimbal_command_buffer_.writeFromNonRT(BufferedGimbalCommand{});
        mode_buffer_.writeFromNonRT(raw_mode);
        return write_command({0.0, 0.0, 0.0, raw_mode})
                    && write_gimbal_command({0.0, 0.0, disabled_gimbal})
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type
        update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const BufferedCommand command = *command_buffer_.readFromRT();
        const rclcpp::Time now = steady_clock_.now();
        const bool valid =
            command.valid
            && (command_timeout_ <= 0.0 || (now - command.stamp).seconds() <= command_timeout_);
        const double mode = static_cast<double>(*mode_buffer_.readFromRT());
        const auto values = valid ? std::array<double, 4>{command.vx, command.vy, command.wz, mode}
                                  : std::array<double, 4>{0.0, 0.0, 0.0, mode};

        const BufferedGimbalCommand gimbal_command = *gimbal_command_buffer_.readFromRT();
        const bool gimbal_valid = gimbal_command.valid
                               && (command_timeout_ <= 0.0
                                   || (now - gimbal_command.stamp).seconds() <= command_timeout_);
        const auto gimbal_values =
            gimbal_valid
                ? std::array<double, 3>{gimbal_command.yaw, gimbal_command.pitch, enabled_gimbal}
                : std::array<double, 3>{0.0, 0.0, disabled_gimbal};

        return write_command(values) && write_gimbal_command(gimbal_values)
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    struct BufferedCommand {
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    struct BufferedGimbalCommand {
        double yaw = 0.0;
        double pitch = 0.0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    static constexpr std::uint8_t raw_mode = 0;
    static constexpr double disabled_gimbal = 0.0;
    static constexpr double enabled_gimbal = 1.0;

    static constexpr std::array<const char*, 4> command_interface_suffixes = {
        "linear/x/velocity",
        "linear/y/velocity",
        "angular/z/velocity",
        "mode",
    };

    static constexpr std::array<const char*, 3> gimbal_command_interface_suffixes = {
        "yaw/velocity",
        "pitch/velocity",
        "enabled",
    };

    bool write_command(const std::array<double, 4>& values) {
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces_[index].set_value(values[index])) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Failed to write reference command '%s/%s'",
                    target_controller_name_.c_str(), command_interface_suffixes[index]);
                return false;
            }
        }

        return true;
    }

    bool write_gimbal_command(const std::array<double, 3>& values) {
        if (target_gimbal_controller_name_.empty()) {
            return true;
        }

        const std::size_t offset = command_interface_suffixes.size();
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces_[offset + index].set_value(values[index])) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Failed to write reference command '%s/%s'",
                    target_gimbal_controller_name_.c_str(),
                    gimbal_command_interface_suffixes[index]);
                return false;
            }
        }

        return true;
    }

    std::string target_controller_name_;
    std::string target_gimbal_controller_name_;
    std::string cmd_vel_topic_ = "/cmd_vel";
    std::string cmd_gimbal_topic_ = "/cmd_gimbal";
    std::string mode_topic_ = "/cmd_chassis_mode";
    double command_timeout_ = 0.25;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
    realtime_tools::RealtimeBuffer<BufferedGimbalCommand> gimbal_command_buffer_;
    realtime_tools::RealtimeBuffer<std::uint8_t> mode_buffer_;
    std::shared_ptr<::teleop_remote_controller::ParamListener> param_listener_;
    ::teleop_remote_controller::Params params_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_gimbal_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_subscriber_;
};

} // namespace rmgo_core

PLUGINLIB_EXPORT_CLASS(rmgo_core::TeleopRemoteController, controller_interface::ControllerInterface)
