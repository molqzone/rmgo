#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/io_state_interfaces.hpp"

namespace rmgo_core::interface {

class MockIoSystemInterface final : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override {
        if (hardware_interface::SystemInterface::on_init(params)
            != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_interfaces_.clear();
        for (const auto& joint : info_.joints) {
            auto interfaces = JointInterfaces{};
            interfaces.name = joint.name;
            for (const auto& state_interface : joint.state_interfaces) {
                if (state_interface.name == hardware_interface::HW_IF_POSITION) {
                    interfaces.position_state_name =
                        make_interface_name(joint.name, hardware_interface::HW_IF_POSITION);
                } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
                    interfaces.velocity_state_name =
                        make_interface_name(joint.name, hardware_interface::HW_IF_VELOCITY);
                }
            }
            for (const auto& command_interface : joint.command_interfaces) {
                if (command_interface.name == hardware_interface::HW_IF_POSITION) {
                    interfaces.position_command_name =
                        make_interface_name(joint.name, hardware_interface::HW_IF_POSITION);
                } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
                    interfaces.velocity_command_name =
                        make_interface_name(joint.name, hardware_interface::HW_IF_VELOCITY);
                }
            }
            joint_interfaces_.push_back(std::move(interfaces));
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::InterfaceDescription>
        export_unlisted_state_interface_descriptions() override {
        auto interfaces = std::vector<hardware_interface::InterfaceDescription>{};
        append_unlisted_state_interface_descriptions(
            interfaces, rmgo_core::io_state_interfaces::remote_state_interfaces);
        append_unlisted_state_interface_descriptions(
            interfaces, rmgo_core::io_state_interfaces::gimbal_imu_state_interfaces);
        append_unlisted_state_interface_descriptions(
            interfaces, rmgo_core::io_state_interfaces::referee_state_interfaces);
        return interfaces;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
        on_export_state_interfaces() override {
        auto interfaces = hardware_interface::SystemInterface::on_export_state_interfaces();
        cache_state_handles();
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_command_interfaces() override {
        auto interfaces = hardware_interface::SystemInterface::on_export_command_interfaces();
        cache_command_handles();
        return interfaces;
    }

    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
        read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
        const double dt = std::max(0.0, period.seconds());
        update_joint_states(dt);
        update_mock_remote_states();
        update_mock_gimbal_imu_states();
        update_mock_referee_states();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        return hardware_interface::return_type::OK;
    }

private:
    struct InterfaceName {
        std::string prefix;
        std::string name;
    };

    struct JointInterfaces {
        std::string name;
        std::optional<std::string> position_state_name;
        std::optional<std::string> velocity_state_name;
        std::optional<std::string> position_command_name;
        std::optional<std::string> velocity_command_name;
        hardware_interface::StateInterface::SharedPtr position_state;
        hardware_interface::StateInterface::SharedPtr velocity_state;
        hardware_interface::CommandInterface::SharedPtr position_command;
        hardware_interface::CommandInterface::SharedPtr velocity_command;
    };

    static std::string make_interface_name(std::string_view prefix, std::string_view interface) {
        auto name = std::string{prefix};
        name += "/";
        name += interface;
        return name;
    }

    static std::optional<InterfaceName> split_interface_name(std::string_view full_name) {
        const auto slash = full_name.rfind('/');
        if (slash == std::string_view::npos || slash == 0 || slash == full_name.size() - 1) {
            return std::nullopt;
        }
        return InterfaceName{
            .prefix = std::string{full_name.substr(0, slash)},
            .name = std::string{full_name.substr(slash + 1)},
        };
    }

    template <typename Names>
    static void append_unlisted_state_interface_descriptions(
        std::vector<hardware_interface::InterfaceDescription>& interfaces, const Names& names) {
        for (std::string_view full_name : names) {
            const auto split_name = split_interface_name(full_name);
            if (!split_name.has_value()) {
                continue;
            }
            auto interface_info = hardware_interface::InterfaceInfo{};
            interface_info.name = split_name->name;
            interface_info.initial_value = "0.0";
            interfaces.emplace_back(split_name->prefix, interface_info);
        }
    }

    void cache_state_handles() {
        for (auto& interfaces : joint_interfaces_) {
            interfaces.position_state = state_handle(interfaces.position_state_name);
            interfaces.velocity_state = state_handle(interfaces.velocity_state_name);
        }
    }

    void cache_command_handles() {
        for (auto& interfaces : joint_interfaces_) {
            interfaces.position_command = command_handle(interfaces.position_command_name);
            interfaces.velocity_command = command_handle(interfaces.velocity_command_name);
        }
    }

    hardware_interface::StateInterface::SharedPtr
        state_handle(const std::optional<std::string>& name) const {
        if (!name.has_value() || !has_state(*name)) {
            return {};
        }
        return get_state_interface_handle(*name);
    }

    hardware_interface::CommandInterface::SharedPtr
        command_handle(const std::optional<std::string>& name) const {
        if (!name.has_value() || !has_command(*name)) {
            return {};
        }
        return get_command_interface_handle(*name);
    }

    void update_joint_states(double dt) {
        for (auto& interfaces : joint_interfaces_) {
            if (interfaces.position_command != nullptr && interfaces.position_state != nullptr) {
                double command = 0.0;
                if (get_command(interfaces.position_command, command, false)) {
                    set_state(interfaces.position_state, command, false);
                }
            }

            if (interfaces.velocity_command != nullptr) {
                double velocity = 0.0;
                if (!get_command(interfaces.velocity_command, velocity, false)) {
                    continue;
                }
                if (interfaces.velocity_state != nullptr) {
                    set_state(interfaces.velocity_state, velocity, false);
                }
                if (interfaces.position_state != nullptr) {
                    double position = 0.0;
                    std::ignore = get_state(interfaces.position_state, position, false);
                    set_state(interfaces.position_state, position + velocity * dt, false);
                }
            }
        }
    }

    void set_mock_state(std::string_view name, double value) {
        set_state(std::string{name}, value);
    }

    void update_mock_remote_states() {
        set_mock_state("remote/dr16/ch0", 0.0);
        set_mock_state("remote/dr16/ch1", 0.0);
        set_mock_state("remote/dr16/ch2", 0.0);
        set_mock_state("remote/dr16/ch3", 0.0);
        set_mock_state("remote/dr16/s1", 2.0);
        set_mock_state("remote/dr16/s2", 2.0);
        set_mock_state("remote/dr16/online", 1.0);
    }

    void update_mock_gimbal_imu_states() {
        set_mock_state("gimbal/imu/orientation.w", 1.0);
        set_mock_state("gimbal/imu/orientation.x", 0.0);
        set_mock_state("gimbal/imu/orientation.y", 0.0);
        set_mock_state("gimbal/imu/orientation.z", 0.0);
        set_mock_state("gimbal/imu/angular_velocity.x", 0.0);
        set_mock_state("gimbal/imu/angular_velocity.y", 0.0);
        set_mock_state("gimbal/imu/angular_velocity.z", 0.0);
        set_mock_state("gimbal/imu/linear_acceleration.x", 0.0);
        set_mock_state("gimbal/imu/linear_acceleration.y", 0.0);
        set_mock_state("gimbal/imu/linear_acceleration.z", 0.0);
        set_mock_state("gimbal/yaw/velocity_imu", 0.0);
        set_mock_state("gimbal/pitch/velocity_imu", 0.0);
    }

    void update_mock_referee_states() {
        set_mock_state("referee/chassis/power", 0.0);
        set_mock_state("referee/chassis/power_buffer", 60.0);
        set_mock_state("referee/chassis/voltage", 24.0);
        set_mock_state("referee/chassis/current", 0.0);
        set_mock_state("referee/shooter/heat", 0.0);
        set_mock_state("referee/shooter/heat_limit", 200.0);
        set_mock_state("referee/robot/hp", 400.0);
        set_mock_state("referee/robot/max_hp", 400.0);
        set_mock_state("referee/robot/level", 1.0);
    }

    std::vector<JointInterfaces> joint_interfaces_;
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::MockIoSystemInterface, hardware_interface::SystemInterface)
