#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "frame.hpp"
#include "status/status.hpp"

namespace rmgo_referee {

struct RefereeStatusEvents {
    bool game_status = false;
    bool robot_status = false;
    bool power_heat = false;
};

class RefereeStatusTranslator final {
public:
    explicit RefereeStatusTranslator(RefereeStatusStore& status)
        : status_(status) {}

    RefereeStatusEvents maintain_safety() noexcept {
        const auto safety = status_.maintain_safety();
        return RefereeStatusEvents{
            .game_status = safety.game_status_timeout,
            .robot_status = safety.robot_status_timeout,
            .power_heat = safety.power_heat_timeout,
        };
    }

    RefereeStatusEvents handle_frame(const RefereeFrame& frame) noexcept {
        auto events = RefereeStatusEvents{};
        struct FrameCallbacks {
            RefereeStatusTranslator& translator;
            RefereeStatusEvents& events;

            void on_game_status(const referee_wire::GameStatus& payload) {
                translator.accept_game_status(payload);
                events.game_status = true;
            }

            void on_game_robot_hp(const referee_wire::GameRobotHp& payload) {
                translator.accept_game_robot_hp(payload);
            }

            void on_event_data(const referee_wire::EventData& payload) {
                translator.accept_event_data(payload);
            }

            void on_dart_status(const referee_wire::DartStatus& payload) {
                translator.accept_dart_status(payload);
            }

            void on_robot_status(const referee_wire::RobotStatus& payload) {
                translator.accept_robot_status(payload);
                events.robot_status = true;
            }

            void on_power_heat_data(const referee_wire::PowerHeatData& payload) {
                translator.accept_power_heat_data(payload);
                events.power_heat = true;
            }

            void on_power_heat_data(const referee_wire::PowerHeatDataWith42mm& payload) {
                translator.accept_power_heat_data(payload);
                events.power_heat = true;
            }

            void on_projectile_allowance(const referee_wire::ProjectileAllowance17mm& payload) {
                translator.accept_projectile_allowance(payload);
            }

            void on_projectile_allowance(const referee_wire::ProjectileAllowanceBase& payload) {
                translator.accept_projectile_allowance(payload);
            }

            void on_projectile_allowance(const referee_wire::ProjectileAllowance& payload) {
                translator.accept_projectile_allowance(payload);
            }

            void on_game_robot_position(const referee_wire::GameRobotPosition& payload) {
                translator.accept_game_robot_position(payload);
            }

            void on_radar_mark_progress(const referee_wire::RadarMarkProgress& payload) {
                translator.accept_radar_mark_progress(payload);
            }

            void on_radar_info(const referee_wire::RadarInfo& payload) {
                translator.accept_radar_info(payload);
            }

            void on_sentry_info(const referee_wire::SentryInfo& payload) {
                translator.accept_sentry_info(payload);
            }

            void on_map_command(const referee_wire::MapCommand& payload) {
                translator.accept_map_command(payload);
            }

            void on_radar_map_robot_data(const referee_wire::RadarMapRobotData& payload) {
                translator.accept_radar_map_robot_data(payload);
            }
        };

        mark_online();
        auto callbacks = FrameCallbacks{
            .translator = *this,
            .events = events,
        };
        (void)decode_referee_frame(frame, callbacks);
        return events;
    }

private:
    void mark_online() noexcept {
        status_.set(RefereeStatusField::online, 1.0);
        status_.mark_online(std::chrono::steady_clock::now());
    }

    void accept_game_status(const referee_wire::GameStatus& payload) noexcept {
        status_.set(RefereeStatusField::game_type, static_cast<double>(payload.game_type));
        status_.set(RefereeStatusField::game_stage, static_cast<double>(payload.game_progress));
        status_.set(
            RefereeStatusField::game_stage_remain_time,
            static_cast<double>(payload.stage_remain_time));
        status_.set(
            RefereeStatusField::game_sync_timestamp, static_cast<double>(payload.sync_timestamp));
    }

    void accept_game_robot_hp(const referee_wire::GameRobotHp& payload) noexcept {
        for (std::size_t index = 0; index < referee_robots_hp_fields.size(); ++index) {
            status_.set(referee_robots_hp_fields[index], static_cast<double>(payload.hp[index]));
        }
    }

    void accept_event_data(const referee_wire::EventData& payload) noexcept {
        const auto event_data = payload.event_data;
        status_.set(
            RefereeStatusField::event_ally_small_energy_activation_status,
            static_cast<double>((event_data >> 3U) & 0x03U));
        status_.set(
            RefereeStatusField::event_ally_big_energy_activation_status,
            static_cast<double>((event_data >> 5U) & 0x03U));
        status_.set(
            RefereeStatusField::event_ally_fortress_occupation_status,
            static_cast<double>((event_data >> 25U) & 0x03U));
    }

    void accept_dart_status(const referee_wire::DartStatus& payload) noexcept {
        const auto dart_info = payload.dart_info;
        status_.set(
            RefereeStatusField::dart_remaining_time,
            static_cast<double>(payload.dart_remaining_time));
        status_.set(
            RefereeStatusField::dart_latest_hit_target, static_cast<double>(dart_info & 0x03U));
        status_.set(
            RefereeStatusField::dart_hit_count, static_cast<double>((dart_info >> 3U) & 0x07U));
        status_.set(
            RefereeStatusField::dart_selected_target,
            static_cast<double>((dart_info >> 6U) & 0x03U));
    }

    void accept_robot_status(const referee_wire::RobotStatus& payload) noexcept {
        const auto power_management_status = payload.power_management_status;
        status_.set(RefereeStatusField::id, static_cast<double>(payload.robot_id));
        status_.set(RefereeStatusField::robot_level, static_cast<double>(payload.robot_level));
        status_.set(RefereeStatusField::hp, static_cast<double>(payload.current_hp));
        status_.set(RefereeStatusField::max_hp, static_cast<double>(payload.maximum_hp));
        status_.set(
            RefereeStatusField::shooter_cooling,
            static_cast<double>(payload.shooter_barrel_cooling_value));
        status_.set(
            RefereeStatusField::shooter_heat_limit,
            static_cast<double>(payload.shooter_barrel_heat_limit));
        status_.set(
            RefereeStatusField::chassis_power_limit,
            static_cast<double>(payload.chassis_power_limit));
        status_.set(
            RefereeStatusField::gimbal_output_status,
            static_cast<double>(power_management_status & 0x01U));
        status_.set(
            RefereeStatusField::chassis_output_status,
            static_cast<double>((power_management_status >> 1U) & 0x01U));
        status_.set(
            RefereeStatusField::shooter_output_status,
            static_cast<double>((power_management_status >> 2U) & 0x01U));
    }

    void accept_power_heat_data(const referee_wire::PowerHeatData& payload) noexcept {
        update_power_heat_status(payload);
    }

    void accept_power_heat_data(const referee_wire::PowerHeatDataWith42mm& payload) noexcept {
        update_power_heat_status(payload);
        status_.set(
            RefereeStatusField::shooter_42mm_heat, static_cast<double>(payload.shooter_42mm_heat));
    }

    template <typename PowerHeatPayload>
    void update_power_heat_status(const PowerHeatPayload& payload) noexcept {
        status_.set(
            RefereeStatusField::chassis_voltage, static_cast<double>(payload.chassis_voltage));
        status_.set(
            RefereeStatusField::chassis_current, static_cast<double>(payload.chassis_current));
        status_.set(RefereeStatusField::chassis_power, static_cast<double>(payload.chassis_power));
        status_.set(
            RefereeStatusField::chassis_buffer_energy,
            static_cast<double>(payload.chassis_buffer_energy));
        status_.set(
            RefereeStatusField::shooter_1_heat, static_cast<double>(payload.shooter_1_heat));
        status_.set(
            RefereeStatusField::shooter_2_heat, static_cast<double>(payload.shooter_2_heat));
    }

    void
        accept_projectile_allowance(const referee_wire::ProjectileAllowance17mm& payload) noexcept {
        status_.set(
            RefereeStatusField::shooter_bullet_allowance,
            static_cast<double>(payload.projectile_allowance_17mm));
    }

    void
        accept_projectile_allowance(const referee_wire::ProjectileAllowanceBase& payload) noexcept {
        status_.set(
            RefereeStatusField::shooter_bullet_allowance,
            static_cast<double>(payload.projectile_allowance_17mm));
        status_.set(
            RefereeStatusField::shooter_42mm_bullet_allowance,
            static_cast<double>(payload.projectile_allowance_42mm));
        status_.set(
            RefereeStatusField::remaining_gold_coin,
            static_cast<double>(payload.remaining_gold_coin));
    }

    void accept_projectile_allowance(const referee_wire::ProjectileAllowance& payload) noexcept {
        status_.set(
            RefereeStatusField::shooter_bullet_allowance,
            static_cast<double>(payload.projectile_allowance_17mm));
        status_.set(
            RefereeStatusField::shooter_42mm_bullet_allowance,
            static_cast<double>(payload.projectile_allowance_42mm));
        status_.set(
            RefereeStatusField::remaining_gold_coin,
            static_cast<double>(payload.remaining_gold_coin));
        status_.set(
            RefereeStatusField::shooter_fortress_17mm_bullet_allowance,
            static_cast<double>(payload.projectile_allowance_fortress));
    }

    void accept_game_robot_position(const referee_wire::GameRobotPosition& payload) noexcept {
        for (std::size_t index = 0; index < referee_ally_robot_position_fields.size(); ++index) {
            status_.set(
                referee_ally_robot_position_fields[index],
                static_cast<double>(payload.positions[index]));
        }
    }

    void accept_radar_mark_progress(const referee_wire::RadarMarkProgress& payload) noexcept {
        for (std::size_t index = 0; index < referee_radar_mark_fields.size(); ++index) {
            status_.set(
                referee_radar_mark_fields[index], static_cast<double>(payload.progress[index]));
        }
    }

    void accept_radar_info(const referee_wire::RadarInfo& payload) noexcept {
        const auto radar_info = payload.radar_info;
        status_.set(
            RefereeStatusField::radar_double_effect_chance,
            static_cast<double>(radar_info & 0x03U));
        status_.set(
            RefereeStatusField::radar_double_effect_active,
            static_cast<double>((radar_info >> 2U) & 0x01U));
    }

    void accept_sentry_info(const referee_wire::SentryInfo& payload) noexcept {
        const auto sentry_info = payload.sentry_info;
        const auto sentry_info_2 = payload.sentry_info_2;
        status_.set(
            RefereeStatusField::sentry_exchanged_bullet_allowance,
            static_cast<double>(sentry_info & 0x07FFU));
        status_.set(
            RefereeStatusField::sentry_remote_bullet_exchange_count,
            static_cast<double>((sentry_info >> 11U) & 0x0FU));
        status_.set(
            RefereeStatusField::sentry_can_confirm_free_revive,
            static_cast<double>((sentry_info >> 19U) & 0x01U));
        status_.set(
            RefereeStatusField::sentry_can_exchange_instant_revive,
            static_cast<double>((sentry_info >> 20U) & 0x01U));
        status_.set(
            RefereeStatusField::sentry_instant_revive_cost,
            static_cast<double>((sentry_info >> 21U) & 0x03FFU));
        status_.set(
            RefereeStatusField::sentry_exchangeable_bullet_allowance,
            static_cast<double>((sentry_info_2 >> 1U) & 0x07FFU));
        status_.set(
            RefereeStatusField::sentry_mode, static_cast<double>((sentry_info_2 >> 12U) & 0x03U));
        status_.set(
            RefereeStatusField::sentry_energy_mechanism_activatable,
            static_cast<double>((sentry_info_2 >> 14U) & 0x01U));
    }

    void accept_map_command(const referee_wire::MapCommand& payload) noexcept {
        const auto previous_x = status_.get(RefereeStatusField::map_command_target_position_x);
        const auto previous_y = status_.get(RefereeStatusField::map_command_target_position_y);
        const auto previous_keyboard = status_.get(RefereeStatusField::map_command_keyboard);
        const auto previous_target = status_.get(RefereeStatusField::map_command_target_robot_id);
        const auto previous_source = status_.get(RefereeStatusField::map_command_source);
        const auto target_x = static_cast<double>(payload.target_position_x);
        const auto target_y = static_cast<double>(payload.target_position_y);
        const auto keyboard = static_cast<double>(payload.cmd_keyboard);
        const auto target = static_cast<double>(payload.target_robot_id);
        const auto source = static_cast<double>(payload.cmd_source);
        status_.set(RefereeStatusField::map_command_target_position_x, target_x);
        status_.set(RefereeStatusField::map_command_target_position_y, target_y);
        status_.set(RefereeStatusField::map_command_keyboard, keyboard);
        status_.set(RefereeStatusField::map_command_target_robot_id, target);
        status_.set(RefereeStatusField::map_command_source, source);
        if (target_x != previous_x || target_y != previous_y || keyboard != previous_keyboard
            || target != previous_target || source != previous_source) {
            status_.set(
                RefereeStatusField::map_command_sequence,
                status_.get(RefereeStatusField::map_command_sequence) + 1.0);
        }
    }

    void accept_radar_map_robot_data(const referee_wire::RadarMapRobotData& payload) noexcept {
        constexpr double centimeter_to_meter = 0.01;
        for (std::size_t index = 0; index < referee_opponent_robot_position_fields.size();
             ++index) {
            status_.set(
                referee_opponent_robot_position_fields[index],
                static_cast<double>(payload.positions_cm[index]) * centimeter_to_meter);
        }
    }

    RefereeStatusStore& status_;
};

} // namespace rmgo_referee
