#include <array>
#include <cstddef>
#include <map>
#include <memory>
#include <span>
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

#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "referee/protocol.hpp"
#include "referee/transfer_registry.hpp"
#include "rmgo_utility/node_mixin.hpp"
#include "rmgo_utility/scalar_interface_mixin.hpp"

namespace rmgo_core::interface {

class RefereeGzSystemInterface final
    : public gz_ros2_control::GazeboSimSystemInterface,
      public rmgo_utility::NodeMixin,
      public rmgo_utility::ScalarInterfaceMixin {
  class Endpoint;

public:
  bool initSim(rclcpp::Node::SharedPtr &model_nh,
               std::map<std::string, gz::sim::Entity> & /*joints*/,
               const hardware_interface::HardwareInfo &hardware_info,
               gz::sim::EntityComponentManager & /*ecm*/,
               unsigned int /*update_rate*/) override {
    node_ = model_nh;
    auto params = hardware_interface::HardwareComponentInterfaceParams{};
    params.hardware_info = hardware_info;
    return on_init(params) == hardware_interface::CallbackReturn::SUCCESS;
  }

  rclcpp::Node::SharedPtr get_node() const override { return node_; }

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto transfer_path =
        get_hardware_info().hardware_parameters.find("transfer_path");
    transfer_path_ =
        transfer_path == get_hardware_info().hardware_parameters.end()
            ? std::string{rmgo_core::referee::default_transfer_path}
            : transfer_path->second;
    endpoint_ = std::make_shared<Endpoint>(*this);
    update_mock_states();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    return export_scalar_state_interfaces(referee_interfaces_, states_);
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    return export_scalar_command_interfaces(command_interfaces_, commands_);
  }

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    update_mock_states();
    rmgo_core::referee::register_referee_transfer_endpoint(transfer_path_,
                                                           endpoint_);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    rmgo_core::referee::unregister_referee_transfer_endpoint(transfer_path_,
                                                             endpoint_.get());
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> & /*start_interfaces*/,
      const std::vector<std::string> & /*stop_interfaces*/) override {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  read(const rclcpp::Time & /*time*/,
       const rclcpp::Duration & /*period*/) override {
    update_mock_states();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  write(const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/) override {
    previous_commands_ = commands_;
    return hardware_interface::return_type::OK;
  }

private:
  static constexpr double robot_id = 3.0;
  static constexpr double chassis_power = 0.0;
  static constexpr double chassis_power_buffer = 60.0;
  static constexpr double chassis_power_limit = 80.0;
  static constexpr double shooter_cooling = 40.0;
  static constexpr double shooter_heat = 0.0;
  static constexpr double shooter_heat_limit = 50000.0;
  static constexpr double robot_hp = 400.0;
  static constexpr double robot_max_hp = 400.0;
  static constexpr double mock_online = 1.0;
  static constexpr double game_progress = 4.0;
  static constexpr double stage_remain_time = 0.0;
  static constexpr double projectile_allowance_17mm = 50.0;

  static constexpr std::size_t state_count =
      rmgo_core::io_state_interfaces::referee_state_interfaces.size();
  static constexpr std::size_t command_count =
      rmgo_core::io_state_interfaces::referee_command_interfaces.size();

  class Endpoint final : public rmgo_core::referee::RefereeTransferEndpoint {
  public:
    explicit Endpoint(RefereeGzSystemInterface & /*owner*/) {}

    std::uint16_t self_robot_id() const noexcept override {
      return static_cast<std::uint16_t>(robot_id);
    }

    rmgo_core::referee::RefereeTransferResult
    send_frame(std::uint16_t command_id,
               std::span<const std::byte> payload) noexcept override {
      return command_id == 0 || payload.size() >
                                    rmgo_core::referee::max_referee_payload_size
                 ? rmgo_core::referee::RefereeTransferResult::InvalidFrame
                 : rmgo_core::referee::RefereeTransferResult::Accepted;
    }

    rmgo_core::referee::RefereeTransferResult
    clear_ui(std::uint8_t layer) noexcept override {
      return layer > 9 ? rmgo_core::referee::RefereeTransferResult::InvalidFrame
                       : rmgo_core::referee::RefereeTransferResult::Accepted;
    }

  };

  void set_state(std::string_view name, double value) {
    (void)set_scalar_interface_value(referee_interfaces_, states_, name, value);
  }

  void update_mock_states() {
    using namespace rmgo_core::io_state_interfaces;
    set_state(referee_id, robot_id);
    set_state(referee_game_stage, game_progress);
    set_state(referee_game_stage_remain_time, stage_remain_time);
    set_state(referee_hp, robot_hp);
    set_state(referee_max_hp, robot_max_hp);
    set_state(referee_shooter_cooling, shooter_cooling);
    set_state(referee_shooter_heat_limit, shooter_heat_limit);
    set_state(referee_shooter_bullet_allowance, projectile_allowance_17mm);
    set_state(referee_shooter_1_heat, shooter_heat);
    set_state(referee_shooter_2_heat, 0.0);
    set_state(referee_chassis_power, chassis_power);
    set_state(referee_chassis_buffer_energy, chassis_power_buffer);
    set_state(referee_chassis_power_limit, chassis_power_limit);
    set_state(referee_radar_mark_hero, 0.0);
    set_state(referee_radar_mark_engineer, 0.0);
    set_state(referee_radar_mark_infantry_3, 0.0);
    set_state(referee_radar_mark_infantry_4, 0.0);
    set_state(referee_radar_mark_infantry_5, 0.0);
    set_state(referee_radar_mark_sentry, 0.0);
    set_state(referee_radar_double_effect_chance, 0.0);
    set_state(referee_radar_double_effect_active, 0.0);
    set_state(referee_dart_remaining_time, 0.0);
    set_state(referee_dart_latest_hit_target, 0.0);
    set_state(referee_dart_hit_count, 0.0);
    set_state(referee_dart_selected_target, 0.0);
    set_state(referee_online, mock_online);
  }

  std::array<double, state_count> states_{};
  std::array<double, command_count> commands_{};
  std::array<double, command_count> previous_commands_{};
  std::vector<rmgo_utility::ScalarInterface> referee_interfaces_ =
      make_scalar_interfaces(
          rmgo_core::io_state_interfaces::referee_state_interfaces);
  std::vector<rmgo_utility::ScalarInterface> command_interfaces_ =
      make_scalar_interfaces(
          rmgo_core::io_state_interfaces::referee_command_interfaces);
  rclcpp::Node::SharedPtr node_;
  std::string transfer_path_{rmgo_core::referee::default_transfer_path};
  std::shared_ptr<Endpoint> endpoint_;
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(rmgo_core::interface::RefereeGzSystemInterface,
                       gz_ros2_control::GazeboSimSystemInterface)
