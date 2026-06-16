#include <array>
#include <cstddef>
#include <map>
#include <string>
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
#include "rmgo_utility/node_mixin.hpp"
#include "rmgo_utility/scalar_interface.hpp"

namespace rmgo_core::interface {

class CommandGzInterface final
    : public gz_ros2_control::GazeboSimSystemInterface
    , public rmgo_utility::NodeMixin {
public:
    bool initSim(
        rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& /*joints*/,
        const hardware_interface::HardwareInfo& hardware_info,
        gz::sim::EntityComponentManager& /*ecm*/, unsigned int /*update_rate*/) override {
        node_ = model_nh;
        auto params = hardware_interface::HardwareComponentInterfaceParams{};
        params.hardware_info = hardware_info;
        return on_init(params) == hardware_interface::CallbackReturn::SUCCESS;
    }

    rclcpp::Node::SharedPtr get_node() const override { return node_; }

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
        return rmgo_utility::scalar_interface::export_state_interfaces(
            command_interfaces_, states_);
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        return rmgo_utility::scalar_interface::export_command_interfaces(
            command_interfaces_, commands_);
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

    std::array<double, command_state_count> commands_{};
    std::array<double, command_state_count> states_{};
    std::vector<rmgo_utility::scalar_interface::Interface> command_interfaces_ =
        rmgo_utility::scalar_interface::make_interfaces(
            rmgo_core::command_state_interfaces::all_interfaces);
    rclcpp::Node::SharedPtr node_;
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::CommandGzInterface, gz_ros2_control::GazeboSimSystemInterface)
