#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rmgo_core/interface/reference_interfaces.hpp"
#include "rmgo_core/teleop_remote_controller_config.hpp"
#include "rmgo_msg/msg/remote_status.hpp"
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
        shooter_fire_buffer_.initRT(BufferedShooterFire{});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return build_individual_config(reference_interface_names());
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
        if (shooter_fire_subscriber_ && shooter_fire_topic_ != params_.shooter_fire_topic) {
            shooter_fire_subscriber_.reset();
        }

        cmd_vel_topic_ = params_.cmd_vel_topic;
        cmd_gimbal_topic_ = params_.cmd_gimbal_topic;
        mode_topic_ = params_.mode_topic;
        shooter_mode_topic_ = params_.shooter_mode_topic;
        shooter_fire_topic_ = params_.shooter_fire_topic;
        command_timeout_ = params_.command_timeout;
        if (!remote_status_publisher_) {
            remote_status_publisher_ = node_->create_publisher<rmgo_msg::msg::RemoteStatus>(
                "/remote/status", rclcpp::SystemDefaultsQoS());
        }

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
                        });
                });
        }
        if (!shooter_fire_subscriber_) {
            shooter_fire_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
                shooter_fire_topic_, rclcpp::SystemDefaultsQoS(),
                [this](const std_msgs::msg::Bool& msg) {
                    shooter_fire_buffer_.writeFromNonRT(
                        BufferedShooterFire{
                            msg.data,
                            steady_clock_.now(),
                            true,
                        });
                });
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!bind_command_interfaces()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        command_buffer_.writeFromNonRT(BufferedCommand{});
        gimbal_command_buffer_.writeFromNonRT(BufferedGimbalCommand{});
        mode_buffer_.writeFromNonRT(raw_mode);
        shooter_mode_buffer_.writeFromNonRT(BufferedShooterMode{});
        shooter_fire_buffer_.writeFromNonRT(BufferedShooterFire{});
        last_fire_pressed_ = false;
        shooter_request_sequence_ = 0;
        if (remote_status_publisher_) {
            remote_status_publisher_->on_activate();
        }
        return write_reference_interfaces({
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
        shooter_fire_buffer_.writeFromNonRT(BufferedShooterFire{});
        last_fire_pressed_ = false;
        if (remote_status_publisher_) {
            remote_status_publisher_->on_deactivate();
        }
        const bool wrote_references = write_reference_interfaces({
            0.0,
            0.0,
            0.0,
            raw_mode,
            0.0,
            0.0,
            disabled_gimbal,
            disabled_shooter,
            0.0,
        });
        return wrote_references ? controller_interface::CallbackReturn::SUCCESS
                                : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type
        update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) override {
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
        const std::array<double, 1> shooter_values{
            static_cast<double>(shooter_valid ? shooter_mode.mode : disabled_shooter),
        };
        const ShooterFireStatus shooter_fire =
            update_shooter_request_sequence(now, shooter_values[0]);
        publish_remote_status(time, valid || gimbal_valid || shooter_valid, shooter_fire.pressed);

        return write_reference_interfaces({
                   chassis_values[0],
                   chassis_values[1],
                   chassis_values[2],
                   chassis_values[3],
                   gimbal_values[0],
                   gimbal_values[1],
                   gimbal_values[2],
                   shooter_values[0],
                   shooter_fire.request_sequence,
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
    };

    struct BufferedShooterFire {
        bool pressed = false;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    struct ShooterFireStatus {
        bool pressed = false;
        double request_sequence = 0.0;
    };

    static constexpr std::size_t teleop_reference_count =
        rmgo_core::reference_interfaces::chassis_interfaces.size()
        + rmgo_core::reference_interfaces::gimbal_interfaces.size()
        + rmgo_core::reference_interfaces::shooter_mode_interfaces.size()
        + rmgo_core::reference_interfaces::shooter_trigger_interfaces.size();
    static constexpr std::uint8_t raw_mode = 0;
    static constexpr double disabled_gimbal = 0.0;
    static constexpr double enabled_gimbal = 1.0;
    static constexpr double disabled_shooter = 0.0;
    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();

    std::vector<std::string> reference_interface_names() const {
        controller_interface::InterfaceConfiguration config;
        append_prefixed_interface_names(
            config.names, params_.chassis_controller_name,
            rmgo_core::reference_interfaces::chassis_interfaces);
        append_prefixed_interface_names(
            config.names, params_.gimbal_controller_name,
            rmgo_core::reference_interfaces::gimbal_interfaces);
        append_prefixed_interface_names(
            config.names, params_.shooter_controller_name,
            rmgo_core::reference_interfaces::shooter_mode_interfaces);
        append_prefixed_interface_names(
            config.names, params_.bullet_feeder_controller_name,
            rmgo_core::reference_interfaces::shooter_trigger_interfaces);
        return std::move(config.names);
    }

    ShooterFireStatus update_shooter_request_sequence(const rclcpp::Time& now, double shooter_mode) {
        const BufferedShooterFire shooter_fire = *shooter_fire_buffer_.readFromRT();
        const bool fresh = shooter_fire.valid
                        && (command_timeout_ <= 0.0
                            || (now - shooter_fire.stamp).seconds() <= command_timeout_);
        const bool fire_pressed = fresh && shooter_fire.pressed;
        const bool rising_edge = fire_pressed && !last_fire_pressed_;
        last_fire_pressed_ = fire_pressed;

        if (shooter_mode <= disabled_shooter || !rising_edge) {
            return {
                .pressed = fire_pressed,
                .request_sequence = 0.0,
            };
        }
        return {
            .pressed = fire_pressed,
            .request_sequence = static_cast<double>(++shooter_request_sequence_),
        };
    }

    void publish_remote_status(const rclcpp::Time& time, bool active, bool fire_pressed) {
        if (!remote_status_publisher_) {
            return;
        }

        auto msg = rmgo_msg::msg::RemoteStatus{};
        msg.header.stamp = time;
        msg.header.frame_id = "remote";
        msg.active = active;
        msg.fire_pressed = fire_pressed;
        msg.cover_open = false;
        msg.gimbal_eject = false;
        msg.power_limit_state = rmgo_msg::msg::RemoteStatus::POWER_LIMIT_UNKNOWN;
        msg.shoot_frequency = rmgo_msg::msg::RemoteStatus::SHOOT_FREQUENCY_UNKNOWN;
        msg.target = rmgo_msg::msg::RemoteStatus::TARGET_UNKNOWN;
        msg.armor_target = rmgo_msg::msg::RemoteStatus::ARMOR_UNKNOWN;
        msg.target_color_red = false;
        remote_status_publisher_->publish(msg);
    }

    bool bind_command_interfaces() {
        command_indexes_.fill(invalid_index);
        const auto names = reference_interface_names();
        for (std::size_t target_index = 0; target_index < names.size(); ++target_index) {
            std::size_t interface_index = 0;
            bool found = false;
            for (const auto& interface : command_interfaces_) {
                if (std::string_view{interface.get_name()} == names[target_index]) {
                    command_indexes_[target_index] = interface_index;
                    found = true;
                    break;
                }
                ++interface_index;
            }
            if (!found) {
                logging::error("Missing teleop reference interface '{}'", names[target_index]);
                return false;
            }
        }
        return true;
    }

    bool write_reference_interfaces(
        const std::array<double, teleop_reference_count>& values) {
        const auto names = reference_interface_names();
        for (std::size_t index = 0; index < values.size(); ++index) {
            const std::size_t command_index = command_indexes_[index];
            if (command_index >= command_interfaces_.size()) [[unlikely]] {
                logging::error("Teleop reference interface '{}' is not bound", names[index]);
                return false;
            }
            if (!command_interfaces_[command_index].set_value(values[index])) [[unlikely]] {
                logging::error(
                    "Failed to write reference interface '{}'",
                    command_interfaces_[command_index].get_name());
                return false;
            }
        }
        return true;
    }
    std::string cmd_vel_topic_ = "/cmd_vel";
    std::string cmd_gimbal_topic_ = "/cmd_gimbal";
    std::string mode_topic_ = "/cmd_chassis_mode";
    std::string shooter_mode_topic_ = "/cmd_shooter_mode";
    std::string shooter_fire_topic_ = "/cmd_shooter_fire";
    double command_timeout_ = 0.25;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
    realtime_tools::RealtimeBuffer<BufferedGimbalCommand> gimbal_command_buffer_;
    realtime_tools::RealtimeBuffer<std::uint8_t> mode_buffer_;
    realtime_tools::RealtimeBuffer<BufferedShooterMode> shooter_mode_buffer_;
    realtime_tools::RealtimeBuffer<BufferedShooterFire> shooter_fire_buffer_;
    std::uint64_t shooter_request_sequence_ = 0;
    bool last_fire_pressed_ = false;
    std::array<std::size_t, teleop_reference_count> command_indexes_{};
    std::shared_ptr<::teleop_remote_controller::ParamListener> param_listener_;
    ::teleop_remote_controller::Params params_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp_lifecycle::LifecyclePublisher<rmgo_msg::msg::RemoteStatus>::SharedPtr
        remote_status_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_gimbal_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr shooter_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shooter_fire_subscriber_;
};

} // namespace rmgo_core

PLUGINLIB_EXPORT_CLASS(rmgo_core::TeleopRemoteController, controller_interface::ControllerInterface)
