#include <array>
#include <map>
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
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/command_state_interfaces.hpp"

namespace rmgo_core::interface {

class CommandGzInterface final : public gz_ros2_control::GazeboSimSystemInterface {
public:
    bool initSim(
        rclcpp::Node::SharedPtr& /*model_nh*/, std::map<std::string, gz::sim::Entity>& /*joints*/,
        const hardware_interface::HardwareInfo& hardware_info,
        gz::sim::EntityComponentManager& /*ecm*/, unsigned int /*update_rate*/) override {
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

        commands_.fill(0.0);
        states_.fill(0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        auto interfaces = std::vector<hardware_interface::StateInterface>{};
        interfaces.reserve(command_interfaces_.size());
        for (const auto& command_interface : command_interfaces_) {
            interfaces.emplace_back(
                command_interface.prefix, command_interface.name,
                &states_[command_interface.index]);
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        auto interfaces = std::vector<hardware_interface::CommandInterface>{};
        interfaces.reserve(command_interfaces_.size());
        for (const auto& command_interface : command_interfaces_) {
            interfaces.emplace_back(
                command_interface.prefix, command_interface.name,
                &commands_[command_interface.index]);
        }
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

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& /*start_interfaces*/,
        const std::vector<std::string>& /*stop_interfaces*/) override {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        states_ = commands_;
        return hardware_interface::return_type::OK;
    }

private:
    static constexpr std::size_t command_state_count =
        rmgo_core::command_state_interfaces::all_interfaces.size();

    struct InterfaceName {
        std::string prefix;
        std::string name;
    };

    struct BusInterface {
        std::string prefix;
        std::string name;
        std::size_t index = 0;
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

    static std::vector<BusInterface> make_command_interfaces() {
        auto interfaces = std::vector<BusInterface>{};
        interfaces.reserve(rmgo_core::command_state_interfaces::all_interfaces.size());
        for (std::string_view full_name : rmgo_core::command_state_interfaces::all_interfaces) {
            const auto split_name = split_interface_name(full_name);
            if (!split_name.has_value()) {
                continue;
            }
            interfaces.push_back(
                BusInterface{
                    .prefix = split_name->prefix,
                    .name = split_name->name,
                    .index = interfaces.size(),
                });
        }
        return interfaces;
    }

    std::array<double, command_state_count> commands_{};
    std::array<double, command_state_count> states_{};
    std::vector<BusInterface> command_interfaces_ = make_command_interfaces();
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::CommandGzInterface, gz_ros2_control::GazeboSimSystemInterface)
