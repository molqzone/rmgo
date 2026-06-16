#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
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

#include "rmgo_utility/node_mixin.hpp"
#include "rmgo_utility/scalar_interface.hpp"

namespace rmgo_core::interface {

class CommandEndpointGzInterface final
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

        command_interfaces_.clear();
        state_interfaces_.clear();
        state_to_command_indices_.clear();

        for (const auto& endpoint : get_hardware_info().gpios) {
            append_endpoint_command_interfaces(endpoint);
            append_endpoint_state_interfaces(endpoint);
        }

        commands_.assign(command_interfaces_.size(), 0.0);
        states_.assign(state_interfaces_.size(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        return rmgo_utility::scalar_interface::export_state_interfaces(state_interfaces_, states_);
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
        for (std::size_t state_index = 0; state_index < state_to_command_indices_.size();
             ++state_index) {
            const auto command_index = state_to_command_indices_[state_index];
            if (command_index != npos) {
                states_[state_index] = commands_[command_index];
            }
        }
        return hardware_interface::return_type::OK;
    }

private:
    static rmgo_utility::scalar_interface::Interface make_scalar_interface(
        const std::string& prefix, const std::string& name, std::size_t index) {
        return rmgo_utility::scalar_interface::Interface{
            .prefix = prefix,
            .name = name,
            .index = index,
        };
    }

    static bool same_scalar_interface(
        const rmgo_utility::scalar_interface::Interface& lhs,
        const rmgo_utility::scalar_interface::Interface& rhs) {
        return lhs.prefix == rhs.prefix && lhs.name == rhs.name;
    }

    std::size_t
        find_command_index_for_state(const rmgo_utility::scalar_interface::Interface& state) const {
        const auto command = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [&](const rmgo_utility::scalar_interface::Interface& candidate) {
                return same_scalar_interface(candidate, state);
            });
        return command == command_interfaces_.end() ? npos : command->index;
    }

    void append_endpoint_command_interfaces(const hardware_interface::ComponentInfo& endpoint) {
        for (const auto& command_interface : endpoint.command_interfaces) {
            command_interfaces_.push_back(make_scalar_interface(
                endpoint.name, command_interface.name, command_interfaces_.size()));
        }
    }

    void append_endpoint_state_interfaces(const hardware_interface::ComponentInfo& endpoint) {
        for (const auto& state_interface : endpoint.state_interfaces) {
            auto scalar_interface = make_scalar_interface(
                endpoint.name, state_interface.name, state_interfaces_.size());
            state_to_command_indices_.push_back(find_command_index_for_state(scalar_interface));
            state_interfaces_.push_back(std::move(scalar_interface));
        }
    }

    std::vector<double> commands_;
    std::vector<double> states_;
    std::vector<std::size_t> state_to_command_indices_;
    std::vector<rmgo_utility::scalar_interface::Interface> command_interfaces_;
    std::vector<rmgo_utility::scalar_interface::Interface> state_interfaces_;
    rclcpp::Node::SharedPtr node_;

    static constexpr auto npos = std::numeric_limits<std::size_t>::max();
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::CommandEndpointGzInterface, gz_ros2_control::GazeboSimSystemInterface)
