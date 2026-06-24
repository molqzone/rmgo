#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <angles/angles.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "rmgo_utility/node_mixin.hpp"

namespace rmgo_core::interface {

class OmniInfantryGzInterface final
    : public gz_ros2_control::GazeboSimSystemInterface
    , public rmgo_utility::NodeMixin {
public:
    bool initSim(
        rclcpp::Node::SharedPtr& model_nh, std::map<std::string, gz::sim::Entity>& joints,
        const hardware_interface::HardwareInfo& hardware_info, gz::sim::EntityComponentManager& ecm,
        unsigned int /*update_rate*/) override {
        node_ = model_nh;
        ecm_ = &ecm;
        gazebo_joints_ = joints;

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

        joint_interfaces_.clear();
        for (const auto& joint : get_hardware_info().joints) {
            auto joint_interface = JointInterface{};
            joint_interface.name = joint.name;
            for (const auto& command_interface : joint.command_interfaces) {
                if (command_interface.name == hardware_interface::HW_IF_POSITION) {
                    const auto position_servo =
                        position_servo_parameters(command_interface, joint.name);
                    if (!position_servo.has_value()) {
                        return hardware_interface::CallbackReturn::ERROR;
                    }

                    joint_interface.position_servo_kp = position_servo->kp;
                    joint_interface.position_servo_kd = position_servo->kd;
                    joint_interface.position_servo_max_effort = position_servo->max_effort;
                    joint_interface.position_servo_max_velocity = position_servo->max_velocity;
                    joint_interface.position_servo_deadband = position_servo->deadband;
                    joint_interface.position_servo_velocity_feedforward =
                        position_servo->velocity_feedforward;
                }
                joint_interface.command_interfaces.push_back(JointCommandInterface{
                    .name = command_interface.name,
                    .value = 0.0,
                });
            }
            for (const auto& state_interface : joint.state_interfaces) {
                joint_interface.state_interfaces.push_back(JointStateInterface{
                    .name = state_interface.name,
                    .value = initial_state_value(state_interface),
                });
            }
            joint_interfaces_.push_back(std::move(joint_interface));
        }

        mock_states_.fill(0.0);
        update_mock_states();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        auto interfaces = std::vector<hardware_interface::StateInterface>{};
        for (auto& joint : joint_interfaces_) {
            for (auto& state_interface : joint.state_interfaces) {
                interfaces.emplace_back(joint.name, state_interface.name, &state_interface.value);
            }
        }

        for (const auto& mock_interface : mock_state_interfaces_) {
            interfaces.emplace_back(
                mock_interface.prefix, mock_interface.name, &mock_states_[mock_interface.index]);
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        auto interfaces = std::vector<hardware_interface::CommandInterface>{};
        for (auto& joint : joint_interfaces_) {
            for (auto& command_interface : joint.command_interfaces) {
                interfaces.emplace_back(
                    joint.name, command_interface.name, &command_interface.value);
            }
        }
        return interfaces;
    }

    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        resolve_gazebo_joint_entities();
        resolve_gazebo_link_entities();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override {
        for (const auto& interface : stop_interfaces) {
            const auto split = split_interface_name(interface);
            if (!split.has_value()) {
                continue;
            }
            if (auto* joint = find_joint(split->prefix); joint != nullptr) {
                joint->control_method = gz_ros2_control::GazeboSimSystemInterface::ControlMethod{};
                joint->previous_position_command.reset();
            }
        }

        for (const auto& interface : start_interfaces) {
            const auto split = split_interface_name(interface);
            if (!split.has_value()) {
                continue;
            }
            if (auto* joint = find_joint(split->prefix); joint != nullptr) {
                joint->control_method = control_method_from_interface(split->name);
                joint->previous_position_command.reset();
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        read_joint_states();
        update_mock_states();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override {
        write_joint_commands(period.seconds());
        return hardware_interface::return_type::OK;
    }

private:
    static constexpr std::size_t mock_state_count =
        rmgo_core::io_state_interfaces::remote_state_interfaces.size()
        + rmgo_core::io_state_interfaces::gimbal_imu_state_interfaces.size();

    struct JointCommandInterface {
        std::string name;
        double value = 0.0;
    };

    struct JointStateInterface {
        std::string name;
        double value = 0.0;
    };

    struct JointInterface {
        std::string name;
        gz::sim::Entity entity = gz::sim::kNullEntity;
        gz_ros2_control::GazeboSimSystemInterface::ControlMethod control_method;
        std::optional<double> previous_position_command;
        double position_servo_kp = 0.0;
        double position_servo_kd = 0.0;
        double position_servo_max_effort = 0.0;
        double position_servo_max_velocity = 0.0;
        double position_servo_deadband = 0.0;
        double position_servo_velocity_feedforward = 0.0;
        std::vector<JointCommandInterface> command_interfaces;
        std::vector<JointStateInterface> state_interfaces;
    };

    struct PositionServoParameters {
        double kp = 0.0;
        double kd = 0.0;
        double max_effort = 0.0;
        double max_velocity = 0.0;
        double deadband = 0.0;
        double velocity_feedforward = 0.0;
    };

    struct InterfaceName {
        std::string prefix;
        std::string name;
    };

    struct MockStateInterface {
        std::string prefix;
        std::string name;
        std::size_t index = 0;
    };

    static double initial_state_value(const hardware_interface::InterfaceInfo& interface) {
        if (interface.initial_value.empty()) {
            return 0.0;
        }
        try {
            return std::stod(interface.initial_value);
        } catch (const std::exception&) {
            return 0.0;
        }
    }

    std::optional<PositionServoParameters> position_servo_parameters(
        const hardware_interface::InterfaceInfo& interface, std::string_view joint_name) {
        const auto kp = required_interface_parameter(interface, joint_name, "sim_servo_kp");
        const auto kd = required_interface_parameter(interface, joint_name, "sim_servo_kd");
        const auto max_effort =
            required_interface_parameter(interface, joint_name, "sim_servo_max_effort");
        const auto max_velocity =
            required_interface_parameter(interface, joint_name, "sim_servo_max_velocity");
        const auto deadband =
            required_interface_parameter(interface, joint_name, "sim_servo_deadband");
        const auto velocity_feedforward =
            required_interface_parameter(interface, joint_name, "sim_servo_velocity_feedforward");

        if (!kp.has_value() || !kd.has_value() || !max_effort.has_value()
            || !max_velocity.has_value() || !deadband.has_value()
            || !velocity_feedforward.has_value()) {
            return std::nullopt;
        }

        return PositionServoParameters{
            .kp = *kp,
            .kd = *kd,
            .max_effort = *max_effort,
            .max_velocity = *max_velocity,
            .deadband = *deadband,
            .velocity_feedforward = *velocity_feedforward,
        };
    }

    std::optional<double> required_interface_parameter(
        const hardware_interface::InterfaceInfo& interface, std::string_view joint_name,
        std::string_view name) {
        const auto parameter_name = std::string{name};
        const auto joint_name_string = std::string{joint_name};
        const auto parameter = interface.parameters.find(parameter_name);
        if (parameter == interface.parameters.end()) {
            logging::error(
                "Missing required parameter '{}' on command interface '{}/{}'.", parameter_name,
                joint_name_string, interface.name);
            return std::nullopt;
        }

        try {
            std::size_t parsed = 0;
            const double value = std::stod(parameter->second, &parsed);
            if (!std::isfinite(value) || !has_only_trailing_whitespace(parameter->second, parsed)) {
                logging::error(
                    "Invalid parameter '{}' on command interface '{}/{}': '{}'.", parameter_name,
                    joint_name_string, interface.name, parameter->second);
                return std::nullopt;
            }
            return value;
        } catch (const std::exception& exception) {
            logging::error(
                "Invalid parameter '{}' on command interface '{}/{}': '{}' ({}).", parameter_name,
                joint_name_string, interface.name, parameter->second, exception.what());
            return std::nullopt;
        }
    }

    static bool has_only_trailing_whitespace(std::string_view text, std::size_t offset) {
        for (std::size_t index = offset; index < text.size(); ++index) {
            if (std::isspace(static_cast<unsigned char>(text[index])) == 0) {
                return false;
            }
        }
        return true;
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

    static gz_ros2_control::GazeboSimSystemInterface::ControlMethod
        control_method_from_interface(std::string_view name) {
        if (name == hardware_interface::HW_IF_POSITION) {
            return gz_ros2_control::GazeboSimSystemInterface::ControlMethod{
                gz_ros2_control::GazeboSimSystemInterface::POSITION};
        }
        if (name == hardware_interface::HW_IF_VELOCITY) {
            return gz_ros2_control::GazeboSimSystemInterface::ControlMethod{
                gz_ros2_control::GazeboSimSystemInterface::VELOCITY};
        }
        if (name == hardware_interface::HW_IF_EFFORT) {
            return gz_ros2_control::GazeboSimSystemInterface::ControlMethod{
                gz_ros2_control::GazeboSimSystemInterface::EFFORT};
        }
        return gz_ros2_control::GazeboSimSystemInterface::ControlMethod{};
    }

    static std::vector<MockStateInterface> make_mock_state_interfaces() {
        auto interfaces = std::vector<MockStateInterface>{};
        const auto append = [&](const auto& names) {
            for (std::string_view full_name : names) {
                const auto split_name = split_interface_name(full_name);
                if (!split_name.has_value()) {
                    continue;
                }
                interfaces.push_back(MockStateInterface{
                    .prefix = split_name->prefix,
                    .name = split_name->name,
                    .index = interfaces.size(),
                });
            }
        };
        append(rmgo_core::io_state_interfaces::remote_state_interfaces);
        append(rmgo_core::io_state_interfaces::gimbal_imu_state_interfaces);
        return interfaces;
    }

    void resolve_gazebo_joint_entities() {
        for (auto& joint : joint_interfaces_) {
            const auto entity = gazebo_joints_.find(joint.name);
            if (entity != gazebo_joints_.end()) {
                joint.entity = entity->second;
            } else if (ecm_ != nullptr) {
                joint.entity = ecm_->EntityByComponents(
                    gz::sim::components::Name(joint.name), gz::sim::components::Joint());
            }
            if (joint.entity != gz::sim::kNullEntity && ecm_ != nullptr) {
                const auto gazebo_joint = gz::sim::Joint{joint.entity};
                gazebo_joint.EnablePositionCheck(*ecm_);
                gazebo_joint.EnableVelocityCheck(*ecm_);
                create_gazebo_joint_state_components(joint.entity);
            }
        }
    }

    void resolve_gazebo_link_entities() {
        if (ecm_ == nullptr) {
            return;
        }

        pitch_link_entity_ = ecm_->EntityByComponents(
            gz::sim::components::Name("pitch_link"), gz::sim::components::Link());
    }

    void read_joint_states() {
        if (ecm_ == nullptr) {
            return;
        }

        for (auto& joint : joint_interfaces_) {
            if (joint.entity == gz::sim::kNullEntity) {
                continue;
            }
            for (auto& state_interface : joint.state_interfaces) {
                if (state_interface.name == hardware_interface::HW_IF_POSITION) {
                    state_interface.value = read_gazebo_vector(
                        gz::sim::Joint{joint.entity}.Position(*ecm_), state_interface.value);
                } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
                    state_interface.value = read_gazebo_vector(
                        gz::sim::Joint{joint.entity}.Velocity(*ecm_), state_interface.value);
                }
            }
        }
    }

    static double
        read_gazebo_vector(const std::optional<std::vector<double>>& value, double fallback) {
        if (!value.has_value() || value->empty()) {
            return fallback;
        }
        return value->front();
    }

    JointInterface* find_joint(std::string_view name) {
        const auto joint = std::find_if(
            joint_interfaces_.begin(), joint_interfaces_.end(),
            [&](const JointInterface& candidate) { return candidate.name == name; });
        return joint == joint_interfaces_.end() ? nullptr : &*joint;
    }

    static const JointCommandInterface*
        find_joint_command_interface(const JointInterface& joint, std::string_view name) {
        const auto command_interface = std::find_if(
            joint.command_interfaces.begin(), joint.command_interfaces.end(),
            [&](const JointCommandInterface& candidate) { return candidate.name == name; });
        return command_interface == joint.command_interfaces.end() ? nullptr : &*command_interface;
    }

    static double joint_state_value(const JointInterface& joint, std::string_view name) {
        const auto state_interface = std::find_if(
            joint.state_interfaces.begin(), joint.state_interfaces.end(),
            [&](const JointStateInterface& candidate) { return candidate.name == name; });
        return state_interface == joint.state_interfaces.end() ? 0.0 : state_interface->value;
    }

    double joint_state_value_or(
        std::string_view joint_name, std::string_view interface_name, double fallback) {
        const auto* joint = find_joint(joint_name);
        return joint == nullptr ? fallback : joint_state_value(*joint, interface_name);
    }

    void create_gazebo_joint_state_components(gz::sim::Entity entity) {
        if (ecm_ == nullptr || entity == gz::sim::kNullEntity) {
            return;
        }
        if (ecm_->Component<gz::sim::components::JointPosition>(entity) == nullptr) {
            ecm_->CreateComponent(entity, gz::sim::components::JointPosition({0.0}));
        }
        if (ecm_->Component<gz::sim::components::JointVelocity>(entity) == nullptr) {
            ecm_->CreateComponent(entity, gz::sim::components::JointVelocity({0.0}));
        }
    }

    void set_gazebo_joint_velocity_command(gz::sim::Entity entity, double value) {
        if (ecm_ == nullptr || entity == gz::sim::kNullEntity) {
            return;
        }
        if (!ecm_->SetComponentData<gz::sim::components::JointVelocityCmd>(entity, {value})) {
            ecm_->CreateComponent(entity, gz::sim::components::JointVelocityCmd({value}));
        }
    }

    void set_gazebo_joint_force_command(gz::sim::Entity entity, double value) {
        if (ecm_ == nullptr || entity == gz::sim::kNullEntity) {
            return;
        }
        if (!ecm_->SetComponentData<gz::sim::components::JointForceCmd>(entity, {value})) {
            ecm_->CreateComponent(entity, gz::sim::components::JointForceCmd({value}));
        }
    }

    void write_joint_commands(double dt) {
        if (ecm_ == nullptr) {
            return;
        }

        for (auto& joint : joint_interfaces_) {
            if (joint.entity == gz::sim::kNullEntity) {
                continue;
            }

            auto control_method = joint.control_method;
            if (control_method & gz_ros2_control::GazeboSimSystemInterface::VELOCITY) {
                if (const auto* command =
                        find_joint_command_interface(joint, hardware_interface::HW_IF_VELOCITY);
                    command != nullptr) {
                    set_gazebo_joint_velocity_command(joint.entity, command->value);
                }
            } else if (control_method & gz_ros2_control::GazeboSimSystemInterface::POSITION) {
                if (const auto* command =
                        find_joint_command_interface(joint, hardware_interface::HW_IF_POSITION);
                    command != nullptr) {
                    const double position =
                        joint_state_value(joint, hardware_interface::HW_IF_POSITION);
                    const double velocity =
                        joint_state_value(joint, hardware_interface::HW_IF_VELOCITY);
                    const double position_error =
                        angles::shortest_angular_distance(position, command->value);
                    double target_velocity = 0.0;
                    if (dt > 0.0 && joint.previous_position_command.has_value()) {
                        target_velocity = angles::shortest_angular_distance(
                                              *joint.previous_position_command, command->value)
                                        / dt;
                    }
                    joint.previous_position_command = command->value;

                    if (std::abs(position_error) < joint.position_servo_deadband
                        && std::abs(velocity) < joint.position_servo_deadband) {
                        set_gazebo_joint_force_command(joint.entity, 0.0);
                        continue;
                    }

                    const double velocity_error = std::clamp(
                        joint.position_servo_velocity_feedforward * target_velocity - velocity,
                        -joint.position_servo_max_velocity, joint.position_servo_max_velocity);
                    const double effort = std::clamp(
                        joint.position_servo_kp * position_error
                            + joint.position_servo_kd * velocity_error,
                        -joint.position_servo_max_effort, joint.position_servo_max_effort);
                    set_gazebo_joint_force_command(joint.entity, effort);
                }
            } else if (control_method & gz_ros2_control::GazeboSimSystemInterface::EFFORT) {
                if (const auto* command =
                        find_joint_command_interface(joint, hardware_interface::HW_IF_EFFORT);
                    command != nullptr) {
                    set_gazebo_joint_force_command(joint.entity, command->value);
                }
            }
        }
    }

    void set_mock_state(std::string_view name, double value) {
        const auto split_name = split_interface_name(name);
        if (!split_name.has_value()) {
            return;
        }

        const auto interface = std::find_if(
            mock_state_interfaces_.begin(), mock_state_interfaces_.end(),
            [&](const MockStateInterface& candidate) {
                return candidate.prefix == split_name->prefix && candidate.name == split_name->name;
            });
        if (interface != mock_state_interfaces_.end()) {
            mock_states_[interface->index] = value;
        }
    }

    void update_mock_states() {
        update_mock_remote_states();
        update_mock_gimbal_imu_states();
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
        const double yaw_velocity =
            joint_state_value_or("yaw_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
        const double pitch_velocity =
            joint_state_value_or("pitch_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
        const auto odom_imu_to_pitch_link = read_odom_imu_to_pitch_link();
        set_mock_state(gimbal_imu_orientation_w, odom_imu_to_pitch_link.W());
        set_mock_state(gimbal_imu_orientation_x, odom_imu_to_pitch_link.X());
        set_mock_state(gimbal_imu_orientation_y, odom_imu_to_pitch_link.Y());
        set_mock_state(gimbal_imu_orientation_z, odom_imu_to_pitch_link.Z());
        set_mock_state(gimbal_imu_angular_velocity_x, 0.0);
        set_mock_state(gimbal_imu_angular_velocity_y, pitch_velocity);
        set_mock_state(gimbal_imu_angular_velocity_z, yaw_velocity);
        set_mock_state(gimbal_imu_linear_acceleration_x, 0.0);
        set_mock_state(gimbal_imu_linear_acceleration_y, 0.0);
        set_mock_state(gimbal_imu_linear_acceleration_z, 0.0);
        set_mock_state(gimbal_yaw_velocity_imu, yaw_velocity);
        set_mock_state(gimbal_pitch_velocity_imu, pitch_velocity);
    }

    gz::math::Quaterniond read_odom_imu_to_pitch_link() {
        if (ecm_ == nullptr) {
            return gz::math::Quaterniond::Identity;
        }
        if (pitch_link_entity_ == gz::sim::kNullEntity) {
            resolve_gazebo_link_entities();
        }
        if (pitch_link_entity_ == gz::sim::kNullEntity) {
            return gz::math::Quaterniond::Identity;
        }

        const auto world_pose = gz::sim::Link{pitch_link_entity_}.WorldPose(*ecm_);
        return world_pose.has_value() ? world_pose->Rot().Inverse()
                                      : gz::math::Quaterniond::Identity;
    }

    gz::sim::EntityComponentManager* ecm_ = nullptr;
    std::map<std::string, gz::sim::Entity> gazebo_joints_;
    gz::sim::Entity pitch_link_entity_ = gz::sim::kNullEntity;
    std::vector<JointInterface> joint_interfaces_;
    std::array<double, mock_state_count> mock_states_{};
    std::vector<MockStateInterface> mock_state_interfaces_ = make_mock_state_interfaces();
    rclcpp::Node::SharedPtr node_;
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::OmniInfantryGzInterface, gz_ros2_control::GazeboSimSystemInterface)
