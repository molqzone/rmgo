#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "app/ui/ui.hpp"
#include "command/endpoint.hpp"
#include "rmgo_msg/msg/capacitor_status.hpp"
#include "rmgo_msg/msg/chassis_status.hpp"
#include "rmgo_msg/msg/gimbal_status.hpp"
#include "rmgo_msg/msg/remote_status.hpp"
#include "rmgo_msg/msg/shooter_status.hpp"
#include "rmgo_msg/msg/target_status.hpp"
#include "status/status.hpp"

namespace rmgo_referee {

class UiStateAdapter final {
public:
    struct Config {
        std::string chassis_status_topic;
        std::string gimbal_status_topic;
        std::string shooter_status_topic;
        std::string remote_status_topic;
        std::string target_status_topic;
        std::string capacitor_status_topic;
        std::string profile_name;
        double online_timeout = 1.0;
        std::chrono::duration<double> update_period{0.01};
    };

    UiStateAdapter(
        rclcpp::Node& node, StatusStore& status, TransferEndpoint& endpoint, Config config)
        : node_(node)
        , status_(status)
        , endpoint_(endpoint)
        , config_(std::move(config)) {
        create_subscribers();
        activate_profile();
        create_timer();
    }

    ~UiStateAdapter() {
        stopping_.store(true, std::memory_order_release);
        stop_callbacks();
        { const std::scoped_lock lock{state_mutex_}; }
        const std::scoped_lock lock{ui_mutex_};
        deactivate_profile();
    }

    UiStateAdapter(const UiStateAdapter&) = delete;
    UiStateAdapter& operator=(const UiStateAdapter&) = delete;

private:
    using ChassisStatus = rmgo_msg::msg::ChassisStatus;
    using GimbalStatus = rmgo_msg::msg::GimbalStatus;
    using ShooterStatus = rmgo_msg::msg::ShooterStatus;
    using RemoteStatus = rmgo_msg::msg::RemoteStatus;
    using TargetStatus = rmgo_msg::msg::TargetStatus;
    using CapacitorStatus = rmgo_msg::msg::CapacitorStatus;
    using UiState = ui::RefereeUiState;
    using UiProfile = ui::UiProfile;

    template <typename Message>
    using Subscription = typename rclcpp::Subscription<Message>::SharedPtr;

    static rclcpp::SystemDefaultsQoS default_qos() { return rclcpp::SystemDefaultsQoS{}; }

    template <typename Message, typename Callback>
    Subscription<Message> make_subscription(const std::string& topic, Callback&& callback) {
        return node_.create_subscription<Message>(
            topic, default_qos(), std::forward<Callback>(callback));
    }

    void create_subscribers() {
        chassis_status_subscriber_ = make_subscription<ChassisStatus>(
            config_.chassis_status_topic, [this](const ChassisStatus& msg) { update(msg); });
        gimbal_status_subscriber_ = make_subscription<GimbalStatus>(
            config_.gimbal_status_topic, [this](const GimbalStatus& msg) { update(msg); });
        shooter_status_subscriber_ = make_subscription<ShooterStatus>(
            config_.shooter_status_topic, [this](const ShooterStatus& msg) { update(msg); });
        remote_status_subscriber_ = make_subscription<RemoteStatus>(
            config_.remote_status_topic, [this](const RemoteStatus& msg) { update(msg); });
        target_status_subscriber_ = make_subscription<TargetStatus>(
            config_.target_status_topic, [this](const TargetStatus& msg) { update(msg); });
        capacitor_status_subscriber_ = make_subscription<CapacitorStatus>(
            config_.capacitor_status_topic, [this](const CapacitorStatus& msg) { update(msg); });
    }

    void create_timer() {
        timer_ = node_.create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(config_.update_period),
            [this] { update(); });
    }

    void activate_profile() {
        profile_ = ui::make_ui_profile(config_.profile_name, ui_);
        if (profile_ == nullptr) {
            throw std::invalid_argument(
                "Unknown referee UI profile '" + config_.profile_name + "'");
        }
        profile_->on_activate();
    }

    void deactivate_profile() noexcept {
        if (profile_ != nullptr) {
            profile_->on_deactivate();
        }
    }

    void stop_callbacks() noexcept {
        if (timer_ != nullptr) {
            timer_->cancel();
        }
        timer_.reset();
        chassis_status_subscriber_.reset();
        gimbal_status_subscriber_.reset();
        shooter_status_subscriber_.reset();
        remote_status_subscriber_.reset();
        target_status_subscriber_.reset();
        capacitor_status_subscriber_.reset();
    }

    void update(const ChassisStatus& msg) {
        update_state([&](auto& state) {
            state.chassis_mode = msg.mode;
            state.chassis_yaw = msg.yaw;
            state.chassis_linear_x_reference = msg.linear_x_reference;
            state.chassis_linear_y_reference = msg.linear_y_reference;
            state.chassis_angular_z_reference = msg.angular_z_reference;
        });
    }

    void update(const GimbalStatus& msg) {
        update_state([&](auto& state) {
            state.gimbal_enabled = msg.enabled ? 1.0 : 0.0;
            state.gimbal_yaw = msg.yaw;
            state.gimbal_pitch = msg.pitch;
            state.gimbal_yaw_reference = msg.yaw_reference;
            state.gimbal_pitch_reference = msg.pitch_reference;
        });
    }

    void update(const ShooterStatus& msg) {
        update_state([&](auto& state) {
            state.shooter_mode = msg.mode;
            state.shooter_friction_requested = msg.friction_requested ? 1.0 : 0.0;
            state.shooter_friction_ready = msg.friction_ready ? 1.0 : 0.0;
            state.shooter_friction_faulted = msg.friction_faulted ? 1.0 : 0.0;
            state.shooter_left_control_velocity = msg.left_control_velocity;
            state.shooter_right_control_velocity = msg.right_control_velocity;
        });
    }

    void update(const RemoteStatus& msg) {
        update_state([&](auto& state) {
            state.remote_active = msg.active ? 1.0 : 0.0;
            state.remote_fire_pressed = msg.fire_pressed ? 1.0 : 0.0;
            state.remote_cover_open = msg.cover_open ? 1.0 : 0.0;
            state.remote_gimbal_eject = msg.gimbal_eject ? 1.0 : 0.0;
            state.remote_power_limit_state = msg.power_limit_state;
            state.remote_shoot_frequency = msg.shoot_frequency;
            state.remote_target = msg.target;
            state.remote_armor_target = msg.armor_target;
            state.remote_target_color_red = msg.target_color_red ? 1.0 : 0.0;
        });
    }

    void update(const TargetStatus& msg) {
        update_state([&](auto& state) {
            state.target_locked = msg.locked ? 1.0 : 0.0;
            state.target_id = msg.id;
            state.target_distance = msg.distance;
        });
    }

    void update(const CapacitorStatus& msg) {
        update_state([&](auto& state) {
            state.capacitor_online = msg.online ? 1.0 : 0.0;
            state.capacitor_resetting = msg.resetting ? 1.0 : 0.0;
            state.capacitor_charge_ratio = msg.charge_ratio;
        });
    }

    void update() {
        if (stopping_.load(std::memory_order_acquire)) {
            return;
        }

        auto state = state_snapshot();
        apply_referee_state(state);

        const std::scoped_lock lock{ui_mutex_};
        if (stopping_.load(std::memory_order_acquire)) {
            return;
        }
        reset_remote_state_if_needed(state);
        if (profile_ != nullptr) {
            profile_->update(state);
        }
        (void)ui_.update(endpoint_);
    }

    void apply_referee_state(UiState& state) const noexcept {
        state.online =
            status_.get(StatusField::online) > 0.5 && status_.is_fresh(config_.online_timeout);
        state.robot_id = status_.get(StatusField::id);
        state.game_stage = status_.get(StatusField::game_stage);
        state.stage_remain_time = status_.get(StatusField::game_stage_remain_time);
        state.hp = status_.get(StatusField::hp);
        state.max_hp = status_.get(StatusField::max_hp);
        state.shooter_cooling = status_.get(StatusField::shooter_cooling);
        state.shooter_heat_limit = status_.get(StatusField::shooter_heat_limit);
        state.shooter_bullet_allowance = status_.get(StatusField::shooter_bullet_allowance);
        state.shooter_1_heat = status_.get(StatusField::shooter_1_heat);
        state.shooter_2_heat = status_.get(StatusField::shooter_2_heat);
        state.chassis_power_limit = status_.get(StatusField::chassis_power_limit);
        state.chassis_power = status_.get(StatusField::chassis_power);
        state.chassis_buffer_energy = status_.get(StatusField::chassis_buffer_energy);
        state.chassis_output_status = status_.get(StatusField::chassis_output_status);
    }

    void reset_remote_state_if_needed(const UiState& state) {
        const auto game_stage = state.game_stage;
        if (state.online
            && ((!last_online_ && state.online)
                || (last_game_stage_ == unknown_game_stage && game_stage != unknown_game_stage)
                || (last_game_stage_ != preparation_game_stage
                    && game_stage == preparation_game_stage))) {
            ui_.reset_remote_state();
        }
        last_online_ = state.online;
        last_game_stage_ = game_stage;
    }

    template <typename Update>
    void update_state(Update&& update) {
        if (stopping_.load(std::memory_order_acquire)) {
            return;
        }
        const std::scoped_lock lock{state_mutex_};
        if (stopping_.load(std::memory_order_acquire)) {
            return;
        }
        std::forward<Update>(update)(state_);
    }

    UiState state_snapshot() const {
        const std::scoped_lock lock{state_mutex_};
        return state_;
    }

    rclcpp::Node& node_;
    StatusStore& status_;
    TransferEndpoint& endpoint_;
    Config config_;
    ui::Ui ui_;
    std::unique_ptr<UiProfile> profile_;
    Subscription<ChassisStatus> chassis_status_subscriber_;
    Subscription<GimbalStatus> gimbal_status_subscriber_;
    Subscription<ShooterStatus> shooter_status_subscriber_;
    Subscription<RemoteStatus> remote_status_subscriber_;
    Subscription<TargetStatus> target_status_subscriber_;
    Subscription<CapacitorStatus> capacitor_status_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    static constexpr double unknown_game_stage = 0.0;
    static constexpr double preparation_game_stage = 1.0;
    mutable std::mutex state_mutex_;
    mutable std::mutex ui_mutex_;
    std::atomic_bool stopping_{false};
    UiState state_;
    bool last_online_ = false;
    double last_game_stage_ = unknown_game_stage;
};

} // namespace rmgo_referee
