#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rmgo_core/interface/command_state_interfaces.hpp"
#include "rmgo_core/teleop_remote_controller_config.hpp"
#include "rmgo_utility/controller_interface_mixin.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core {

class TeleopRemoteController
    : public controller_interface::ControllerInterface
    , public rmgo_utility::ControllerInterfaceMixin
    , public rmgo_utility::NodeMixin {
public:
    controller_interface::CallbackReturn on_init() override {
        init_parameters(param_listener_, params_);
        command_buffer_.initRT(BufferedCommand{});
        gimbal_command_buffer_.initRT(BufferedGimbalCommand{});
        mode_buffer_.initRT(raw_mode);
        shooter_mode_buffer_.initRT(BufferedShooterMode{});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(rmgo_core::command_state_interfaces::all_command_interfaces);
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
        update_parameters(param_listener_, params_);
        if (cmd_vel_subscriber_ && cmd_vel_topic_ != params_.cmd_vel_topic) {
            cmd_vel_subscriber_.reset();
        }
        if (cmd_gimbal_subscriber_ && cmd_gimbal_topic_ != params_.cmd_gimbal_topic) {
            cmd_gimbal_subscriber_.reset();
        }
        if (mode_subscriber_ && mode_topic_ != params_.mode_topic) {
            mode_subscriber_.reset();
        }
        if (shooter_mode_subscriber_ && shooter_mode_topic_ != params_.shooter_mode_topic) {
            shooter_mode_subscriber_.reset();
        }

        cmd_vel_topic_ = params_.cmd_vel_topic;
        cmd_gimbal_topic_ = params_.cmd_gimbal_topic;
        mode_topic_ = params_.mode_topic;
        shooter_mode_topic_ = params_.shooter_mode_topic;
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
        if (!cmd_gimbal_subscriber_) {
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
        if (!shooter_mode_subscriber_) {
            shooter_mode_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
                shooter_mode_topic_, rclcpp::SystemDefaultsQoS(),
                [this](const std_msgs::msg::UInt8& msg) {
                    shooter_mode_buffer_.writeFromNonRT(
                        BufferedShooterMode{
                            msg.data,
                            steady_clock_.now(),
                            true,
                            ++shooter_command_sequence_,
                        });
                });
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        command_buffer_.writeFromNonRT(BufferedCommand{});
        gimbal_command_buffer_.writeFromNonRT(BufferedGimbalCommand{});
        mode_buffer_.writeFromNonRT(raw_mode);
        shooter_mode_buffer_.writeFromNonRT(BufferedShooterMode{});
        return write_command_bus({
                   0.0,
                   0.0,
                   0.0,
                   raw_mode,
                   0.0,
                   0.0,
                   enabled_gimbal,
                   disabled_shooter,
                   0.0,
               })
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        command_buffer_.writeFromNonRT(BufferedCommand{});
        gimbal_command_buffer_.writeFromNonRT(BufferedGimbalCommand{});
        mode_buffer_.writeFromNonRT(raw_mode);
        shooter_mode_buffer_.writeFromNonRT(BufferedShooterMode{});
        return write_command_bus({
                   0.0,
                   0.0,
                   0.0,
                   raw_mode,
                   0.0,
                   0.0,
                   disabled_gimbal,
                   disabled_shooter,
                   0.0,
               })
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
        const std::array<double, 4> chassis_values =
            valid ? std::array<double, 4>{command.vx, command.vy, command.wz, mode}
                  : std::array<double, 4>{0.0, 0.0, 0.0, mode};

        const BufferedGimbalCommand gimbal_command = *gimbal_command_buffer_.readFromRT();
        const bool gimbal_valid = gimbal_command.valid
                               && (command_timeout_ <= 0.0
                                   || (now - gimbal_command.stamp).seconds() <= command_timeout_);
        // Keep the gimbal controller enabled on stale teleop input so it keeps stabilizing.
        const std::array<double, 3> gimbal_values =
            gimbal_valid
                ? std::array<double, 3>{gimbal_command.yaw, gimbal_command.pitch, enabled_gimbal}
                : std::array<double, 3>{0.0, 0.0, enabled_gimbal};

        const BufferedShooterMode shooter_mode = *shooter_mode_buffer_.readFromRT();
        const bool shooter_valid = shooter_mode.valid
                                && (command_timeout_ <= 0.0
                                    || (now - shooter_mode.stamp).seconds() <= command_timeout_);
        const std::array<double, 2> shooter_values{
            static_cast<double>(shooter_valid ? shooter_mode.mode : disabled_shooter),
            static_cast<double>(shooter_valid ? shooter_mode.sequence : 0),
        };

        return write_command_bus({
                   chassis_values[0],
                   chassis_values[1],
                   chassis_values[2],
                   chassis_values[3],
                   gimbal_values[0],
                   gimbal_values[1],
                   gimbal_values[2],
                   shooter_values[0],
                   shooter_values[1],
               })
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

    struct BufferedShooterMode {
        std::uint8_t mode = 0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
        std::uint64_t sequence = 0;
    };

    static constexpr std::uint8_t raw_mode = 0;
    static constexpr double disabled_gimbal = 0.0;
    static constexpr double enabled_gimbal = 1.0;
    static constexpr double disabled_shooter = 0.0;

    bool write_command_bus(
        const std::array<
            double, rmgo_core::command_state_interfaces::all_command_interfaces.size()>& values) {
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces_[index].set_value(values[index])) [[unlikely]] {
                logging::error(
                    "Failed to write command bus interface '{}'",
                    rmgo_core::command_state_interfaces::all_command_interfaces[index]);
                return false;
            }
        }
        return true;
    }
    std::string cmd_vel_topic_ = "/cmd_vel";
    std::string cmd_gimbal_topic_ = "/cmd_gimbal";
    std::string mode_topic_ = "/cmd_chassis_mode";
    std::string shooter_mode_topic_ = "/cmd_shooter_mode";
    double command_timeout_ = 0.25;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
    realtime_tools::RealtimeBuffer<BufferedGimbalCommand> gimbal_command_buffer_;
    realtime_tools::RealtimeBuffer<std::uint8_t> mode_buffer_;
    realtime_tools::RealtimeBuffer<BufferedShooterMode> shooter_mode_buffer_;
    std::uint64_t shooter_command_sequence_ = 0;
    std::shared_ptr<::teleop_remote_controller::ParamListener> param_listener_;
    ::teleop_remote_controller::Params params_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_gimbal_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr shooter_mode_subscriber_;
};

} // namespace rmgo_core

PLUGINLIB_EXPORT_CLASS(rmgo_core::TeleopRemoteController, controller_interface::ControllerInterface)
