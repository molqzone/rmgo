#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <angles/angles.h>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "rmgo_core/simple_gimbal_controller_config.hpp"

namespace rmgo_core::controller::gimbal {

class SimpleGimbalController : public controller_interface::ControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override {
        param_listener_ = std::make_shared<::simple_gimbal_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        update_joint_names();
        command_buffer_.initRT(BufferedCommand{});
        base_odometry_buffer_.initRT(BufferedBaseOdometry{});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            yaw_joint_name_ + "/" + params_.command_interface_name,
            pitch_joint_name_ + "/" + params_.command_interface_name,
        };
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            yaw_joint_name_ + "/" + params_.state_interface_name,
            pitch_joint_name_ + "/" + params_.state_interface_name,
        };
        return config;
    }

    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        params_ = param_listener_->get_params();
        update_joint_names();
        command_topic_ = params_.command_topic;
        base_odometry_topic_ = params_.base_odometry_topic;
        command_timeout_ = params_.command_timeout;
        base_odometry_timeout_ = params_.base_odometry_timeout;

        auto node = get_node();
        command_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
            command_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist& msg) {
                command_buffer_.writeFromNonRT(BufferedCommand{
                    msg.angular.z,
                    msg.angular.y,
                    steady_clock_.now(),
                    true,
                });
            });
        base_odometry_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
            base_odometry_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry& msg) {
                base_odometry_buffer_.writeFromNonRT(BufferedBaseOdometry{
                    yaw_from_odometry(msg),
                    steady_clock_.now(),
                    true,
                });
            });

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (command_interfaces_.size() != command_interface_names.size()) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                command_interface_names.size(), command_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        if (state_interfaces_.size() != state_interface_names.size()) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
                state_interface_names.size(), state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        const double yaw = read_state(yaw_index);
        const double pitch = read_state(pitch_index);
        const auto base_odometry = read_base_odometry();
        world_yaw_initialized_from_odometry_ = base_odometry.has_value();
        target_world_yaw_ = base_odometry.has_value()
                              ? angles::normalize_angle(base_odometry->yaw + yaw)
                              : angles::normalize_angle(yaw);
        target_pitch_ = std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit);
        last_update_time_ = steady_clock_.now();

        return write_commands(yaw, target_pitch_) ? controller_interface::CallbackReturn::SUCCESS
                                                  : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        command_buffer_.writeFromNonRT(BufferedCommand{});
        world_yaw_initialized_from_odometry_ = false;
        return write_commands(read_state(yaw_index), read_state(pitch_index))
                 ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::return_type
        update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        const rclcpp::Time now = steady_clock_.now();
        const double dt = std::max(0.0, (now - last_update_time_).seconds());
        last_update_time_ = now;

        const BufferedCommand command = *command_buffer_.readFromRT();
        if (is_command_valid(command, now)) {
            target_world_yaw_ = angles::normalize_angle(
                target_world_yaw_ + command.yaw_velocity * params_.yaw_velocity_scale * dt);
            target_pitch_ = std::clamp(
                target_pitch_ + command.pitch_velocity * params_.pitch_velocity_scale * dt,
                params_.pitch_lower_limit, params_.pitch_upper_limit);
        }

        const double yaw_command = calculate_yaw_command(now);
        return write_commands(yaw_command, target_pitch_)
                 ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
    }

private:
    struct BufferedCommand {
        double yaw_velocity = 0.0;
        double pitch_velocity = 0.0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    struct BufferedBaseOdometry {
        double yaw = 0.0;
        rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
        bool valid = false;
    };

    static constexpr std::size_t yaw_index = 0;
    static constexpr std::size_t pitch_index = 1;
    static constexpr std::array<const char*, 2> command_interface_names = {"yaw", "pitch"};
    static constexpr std::array<const char*, 2> state_interface_names = {"yaw", "pitch"};

    void update_joint_names() {
        yaw_joint_name_ = params_.yaw_joint_name;
        pitch_joint_name_ = params_.pitch_joint_name;
    }

    static double yaw_from_odometry(const nav_msgs::msg::Odometry& odometry) {
        const auto& q = odometry.pose.pose.orientation;
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    bool is_command_valid(const BufferedCommand& command, const rclcpp::Time& now) const {
        return command.valid
            && (command_timeout_ <= 0.0 || (now - command.stamp).seconds() <= command_timeout_);
    }

    std::optional<BufferedBaseOdometry> read_base_odometry() {
        const BufferedBaseOdometry odometry = *base_odometry_buffer_.readFromRT();
        if (!odometry.valid) {
            return std::nullopt;
        }
        return odometry;
    }

    bool is_base_odometry_valid(
        const BufferedBaseOdometry& odometry, const rclcpp::Time& now) const {
        return odometry.valid
            && (base_odometry_timeout_ <= 0.0
                || (now - odometry.stamp).seconds() <= base_odometry_timeout_);
    }

    double calculate_yaw_command(const rclcpp::Time& now) {
        const BufferedBaseOdometry odometry = *base_odometry_buffer_.readFromRT();
        if (!is_base_odometry_valid(odometry, now)) {
            return read_state(yaw_index);
        }

        if (!world_yaw_initialized_from_odometry_) {
            target_world_yaw_ = angles::normalize_angle(odometry.yaw + read_state(yaw_index));
            world_yaw_initialized_from_odometry_ = true;
        }
        return angles::normalize_angle(target_world_yaw_ - odometry.yaw);
    }

    double read_state(std::size_t index) const {
        if (index >= state_interfaces_.size()) {
            return 0.0;
        }

        const std::optional<double> value = state_interfaces_[index].get_optional();
        return value.has_value() && std::isfinite(*value) ? *value : 0.0;
    }

    bool write_commands(double yaw, double pitch) {
        const std::array<double, 2> values = {
            angles::normalize_angle(yaw),
            std::clamp(pitch, params_.pitch_lower_limit, params_.pitch_upper_limit),
        };
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces_[index].set_value(values[index])) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Failed to write %s gimbal command",
                    command_interface_names[index]);
                return false;
            }
        }
        return true;
    }

    std::string yaw_joint_name_;
    std::string pitch_joint_name_;
    std::string command_topic_ = "/cmd_gimbal";
    std::string base_odometry_topic_ = "/model/omni_infantry/odometry";
    double command_timeout_ = 0.25;
    double base_odometry_timeout_ = 0.5;
    double target_world_yaw_ = 0.0;
    double target_pitch_ = 0.0;
    bool world_yaw_initialized_from_odometry_ = false;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time last_update_time_{0, 0, RCL_STEADY_TIME};
    realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
    realtime_tools::RealtimeBuffer<BufferedBaseOdometry> base_odometry_buffer_;
    std::shared_ptr<::simple_gimbal_controller::ParamListener> param_listener_;
    ::simple_gimbal_controller::Params params_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_odometry_subscriber_;
};

} // namespace rmgo_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::controller::gimbal::SimpleGimbalController,
    controller_interface::ControllerInterface)
