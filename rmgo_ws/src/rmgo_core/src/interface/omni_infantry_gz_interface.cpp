#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <gz/sim/EntityComponentManager.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/io_state_interfaces.hpp"

namespace rmgo_core::interface {

class OmniInfantryGzInterface final : public gz_ros2_control::GazeboSimSystemInterface {
public:
    bool initSim(
        rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
        const hardware_interface::HardwareInfo& hardware_info, gz::sim::EntityComponentManager& ecm,
        unsigned int update_rate) override {
        (void)model_nh;
        (void)joints;
        (void)ecm;
        (void)update_rate;

        auto params = hardware_interface::HardwareComponentInterfaceParams{};
        params.hardware_info = hardware_info;
        return on_init(params) == hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override {
        if (hardware_interface::SystemInterface::on_init(params)
            != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
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
        update_mock_remote_states();
        update_mock_gimbal_imu_states();
        update_mock_referee_states();
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

    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
        read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
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

    void set_mock_state(std::string_view name, double value) {
        set_state(std::string{name}, value);
    }

    void update_mock_remote_states() {
        using namespace rmgo_core::io_state_interfaces;
        set_mock_state(remote_dr16_ch0, 0.0);
        set_mock_state(remote_dr16_ch1, 0.0);
        set_mock_state(remote_dr16_ch2, 0.0);
        set_mock_state(remote_dr16_ch3, 0.0);
        set_mock_state(remote_dr16_s1, 2.0);
        set_mock_state(remote_dr16_s2, 2.0);
        set_mock_state(remote_dr16_online, 1.0);
    }

    void update_mock_gimbal_imu_states() {
        using namespace rmgo_core::io_state_interfaces;
        set_mock_state(gimbal_imu_orientation_w, 1.0);
        set_mock_state(gimbal_imu_orientation_x, 0.0);
        set_mock_state(gimbal_imu_orientation_y, 0.0);
        set_mock_state(gimbal_imu_orientation_z, 0.0);
        set_mock_state(gimbal_imu_angular_velocity_x, 0.0);
        set_mock_state(gimbal_imu_angular_velocity_y, 0.0);
        set_mock_state(gimbal_imu_angular_velocity_z, 0.0);
        set_mock_state(gimbal_imu_linear_acceleration_x, 0.0);
        set_mock_state(gimbal_imu_linear_acceleration_y, 0.0);
        set_mock_state(gimbal_imu_linear_acceleration_z, 0.0);
        set_mock_state(gimbal_yaw_velocity_imu, 0.0);
        set_mock_state(gimbal_pitch_velocity_imu, 0.0);
    }

    void update_mock_referee_states() {
        using namespace rmgo_core::io_state_interfaces;
        set_mock_state(referee_chassis_power, 0.0);
        set_mock_state(referee_chassis_power_buffer, 60.0);
        set_mock_state(referee_chassis_power_limit, 80.0);
        set_mock_state(referee_chassis_voltage, 24.0);
        set_mock_state(referee_chassis_current, 0.0);
        set_mock_state(referee_shooter_heat, 0.0);
        set_mock_state(referee_shooter_heat_limit, 200.0);
        set_mock_state(referee_robot_hp, 400.0);
        set_mock_state(referee_robot_max_hp, 400.0);
        set_mock_state(referee_robot_level, 1.0);
    }
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::OmniInfantryGzInterface, gz_ros2_control::GazeboSimSystemInterface)
