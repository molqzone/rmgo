#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "rmgo_msg/msg/game_robot_status.hpp"
#include "rmgo_msg/msg/game_status.hpp"
#include "rmgo_msg/msg/power_heat_data.hpp"
#include "rmgo_msg/msg/referee_status.hpp"
#include "rmgo_utility/utility/watchdog.hpp"
#include "status/field.hpp"

namespace rmgo_referee {

struct StatusSafetyEvents {
    bool game_status_timeout = false;
    bool robot_status_timeout = false;
    bool power_heat_timeout = false;
};

class StatusStore final {
public:
    StatusStore();

    void reset() noexcept;

    void set(StatusField field, double value) noexcept;
    double get(StatusField field) const noexcept;
    void mark_online(std::chrono::steady_clock::time_point time) noexcept;

    StatusSafetyEvents maintain_safety() noexcept;
    bool is_fresh(double online_timeout) const noexcept;
    bool game_status_fresh() const noexcept;
    bool robot_status_fresh() const noexcept;
    bool power_heat_fresh() const noexcept;
    std::uint16_t robot_id() const noexcept;
    rmgo_msg::msg::GameStatus to_game_status_message() const;
    rmgo_msg::msg::GameRobotStatus to_game_robot_status_message() const;
    rmgo_msg::msg::PowerHeatData to_power_heat_data_message() const;
    rmgo_msg::msg::RefereeStatus to_message(double online_timeout) const;

private:
    using Watchdog = rmgo_utility::utility::Watchdog;

    double load(StatusField field) const noexcept;
    void apply_initial_safety_fallback() noexcept;
    void arm_game_status_watchdog(Watchdog::clock::time_point now) noexcept;
    void arm_robot_status_watchdog(Watchdog::clock::time_point now) noexcept;
    void arm_power_heat_watchdog(Watchdog::clock::time_point now) noexcept;
    void apply_robot_status_fallback() noexcept;
    void apply_power_heat_fallback() noexcept;

    std::array<std::atomic<double>, to_index(StatusField::count)> values_{};
    std::atomic<std::int64_t> last_update_ns_{0};
    std::atomic<std::uint16_t> robot_id_{0};
    Watchdog game_status_watchdog_;
    Watchdog robot_status_watchdog_;
    Watchdog power_heat_watchdog_;
};

} // namespace rmgo_referee

namespace rmgo_referee {
namespace {

using namespace std::chrono_literals;

constexpr double started_game_stage = 4.0;
constexpr double safe_game_stage = 0.0;
constexpr double safe_shooter_cooling = 40.0;
constexpr double safe_shooter_heat_limit = 50'000.0;
constexpr double safe_chassis_power_limit = 45.0;
constexpr double safe_chassis_power = 0.0;
constexpr double safe_chassis_buffer_energy = 60.0;

template <typename T>
T status_integer(double value) noexcept {
    if (!std::isfinite(value) || value <= 0.0) {
        return 0;
    }
    const auto max = static_cast<double>(std::numeric_limits<T>::max());
    return static_cast<T>(std::clamp(value, 0.0, max));
}

float status_float(double value) noexcept {
    if (!std::isfinite(value)) {
        return 0.0F;
    }
    return static_cast<float>(value);
}

bool status_bool(double value) noexcept { return value > 0.5; }

template <typename Array, std::size_t size>
void fill_status_array(
    Array& output, const std::array<StatusField, size>& fields, const StatusStore& store) noexcept {
    using Value = typename Array::value_type;
    for (std::size_t index = 0; index < size; ++index) {
        output[index] = status_integer<Value>(store.get(fields[index]));
    }
}

template <typename Array, std::size_t size>
void fill_status_float_array(
    Array& output, const std::array<StatusField, size>& fields, const StatusStore& store) noexcept {
    for (std::size_t index = 0; index < size; ++index) {
        output[index] = status_float(store.get(fields[index]));
    }
}

} // namespace

inline StatusStore::StatusStore() { reset(); }

inline void StatusStore::reset() noexcept {
    for (auto& value : values_) {
        value.store(0.0, std::memory_order_relaxed);
    }
    last_update_ns_.store(0, std::memory_order_release);
    robot_id_.store(0, std::memory_order_release);
    game_status_watchdog_.disarm();
    robot_status_watchdog_.disarm();
    power_heat_watchdog_.disarm();
    apply_initial_safety_fallback();
}

inline void StatusStore::set(StatusField field, double value) noexcept {
    values_[to_index(field)].store(value, std::memory_order_relaxed);
    if (field == StatusField::id) {
        robot_id_.store(static_cast<std::uint16_t>(value), std::memory_order_release);
    }
    const auto now = Watchdog::clock::now();
    if (field == StatusField::game_stage) {
        arm_game_status_watchdog(now);
    } else if (
        field == StatusField::id || field == StatusField::hp || field == StatusField::max_hp
        || field == StatusField::shooter_cooling || field == StatusField::shooter_heat_limit
        || field == StatusField::chassis_power_limit
        || field == StatusField::chassis_output_status) {
        arm_robot_status_watchdog(now);
    } else if (
        field == StatusField::chassis_power || field == StatusField::chassis_buffer_energy
        || field == StatusField::shooter_1_heat || field == StatusField::shooter_2_heat) {
        arm_power_heat_watchdog(now);
    }
}

inline double StatusStore::get(StatusField field) const noexcept { return load(field); }

inline void StatusStore::mark_online(std::chrono::steady_clock::time_point time) noexcept {
    last_update_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count(),
        std::memory_order_release);
}

inline StatusSafetyEvents StatusStore::maintain_safety() noexcept {
    const auto now = Watchdog::clock::now();
    auto events = StatusSafetyEvents{};
    if (game_status_watchdog_.consume_expiration(now)) {
        values_[to_index(StatusField::game_stage)].store(
            safe_game_stage, std::memory_order_relaxed);
        events.game_status_timeout = true;
    }
    if (robot_status_watchdog_.consume_expiration(now)) {
        apply_robot_status_fallback();
        events.robot_status_timeout = true;
    }
    if (power_heat_watchdog_.consume_expiration(now)) {
        apply_power_heat_fallback();
        events.power_heat_timeout = true;
    }
    return events;
}

inline bool StatusStore::is_fresh(double online_timeout) const noexcept {
    const auto last_update_ns = last_update_ns_.load(std::memory_order_acquire);
    if (last_update_ns <= 0) {
        return false;
    }
    const auto last_update =
        std::chrono::steady_clock::time_point{std::chrono::nanoseconds{last_update_ns}};
    return online_timeout <= 0.0
        || std::chrono::steady_clock::now() - last_update
               <= std::chrono::duration<double>{online_timeout};
}

inline bool StatusStore::game_status_fresh() const noexcept {
    return game_status_watchdog_.fresh(Watchdog::clock::now());
}

inline bool StatusStore::robot_status_fresh() const noexcept {
    return robot_status_watchdog_.fresh(Watchdog::clock::now());
}

inline bool StatusStore::power_heat_fresh() const noexcept {
    return power_heat_watchdog_.fresh(Watchdog::clock::now());
}

inline std::uint16_t StatusStore::robot_id() const noexcept {
    return robot_id_.load(std::memory_order_acquire);
}

inline rmgo_msg::msg::GameStatus StatusStore::to_game_status_message() const {
    auto msg = rmgo_msg::msg::GameStatus{};
    msg.game_type = status_integer<std::uint8_t>(load(StatusField::game_type));
    msg.game_progress = status_integer<std::uint8_t>(load(StatusField::game_stage));
    msg.stage_remain_time =
        status_integer<std::uint16_t>(load(StatusField::game_stage_remain_time));
    msg.sync_timestamp = status_integer<std::uint64_t>(load(StatusField::game_sync_timestamp));
    return msg;
}

inline rmgo_msg::msg::GameRobotStatus StatusStore::to_game_robot_status_message() const {
    auto msg = rmgo_msg::msg::GameRobotStatus{};
    msg.robot_id = status_integer<std::uint8_t>(load(StatusField::id));
    msg.robot_level = status_integer<std::uint8_t>(load(StatusField::robot_level));
    msg.hp = status_integer<std::uint16_t>(load(StatusField::hp));
    msg.max_hp = status_integer<std::uint16_t>(load(StatusField::max_hp));
    msg.shooter_cooling = status_integer<std::uint16_t>(load(StatusField::shooter_cooling));
    msg.shooter_heat_limit = status_integer<std::uint16_t>(load(StatusField::shooter_heat_limit));
    msg.chassis_power_limit = status_integer<std::uint16_t>(load(StatusField::chassis_power_limit));
    msg.gimbal_output_enabled = status_bool(load(StatusField::gimbal_output_status));
    msg.chassis_output_enabled = status_bool(load(StatusField::chassis_output_status));
    msg.shooter_output_enabled = status_bool(load(StatusField::shooter_output_status));
    return msg;
}

inline rmgo_msg::msg::PowerHeatData StatusStore::to_power_heat_data_message() const {
    auto msg = rmgo_msg::msg::PowerHeatData{};
    msg.chassis_voltage = status_integer<std::uint16_t>(load(StatusField::chassis_voltage));
    msg.chassis_current = status_integer<std::uint16_t>(load(StatusField::chassis_current));
    msg.chassis_power = status_float(load(StatusField::chassis_power));
    msg.chassis_buffer_energy =
        status_integer<std::uint16_t>(load(StatusField::chassis_buffer_energy));
    msg.shooter_1_heat = status_integer<std::uint16_t>(load(StatusField::shooter_1_heat));
    msg.shooter_2_heat = status_integer<std::uint16_t>(load(StatusField::shooter_2_heat));
    msg.shooter_42mm_heat = status_integer<std::uint16_t>(load(StatusField::shooter_42mm_heat));
    return msg;
}

inline rmgo_msg::msg::RefereeStatus StatusStore::to_message(double online_timeout) const {
    auto msg = rmgo_msg::msg::RefereeStatus{};
    msg.online = load(StatusField::online) > 0.5 && is_fresh(online_timeout);
    msg.game_status_fresh = game_status_fresh();
    msg.robot_status_fresh = robot_status_fresh();
    msg.power_heat_fresh = power_heat_fresh();
    msg.robot_id = status_integer<std::uint16_t>(load(StatusField::id));
    msg.game_stage = status_integer<std::uint8_t>(load(StatusField::game_stage));
    msg.stage_remain_time =
        status_integer<std::uint16_t>(load(StatusField::game_stage_remain_time));
    msg.game_sync_timestamp = status_integer<std::uint64_t>(load(StatusField::game_sync_timestamp));

    msg.hp = status_integer<std::uint16_t>(load(StatusField::hp));
    msg.max_hp = status_integer<std::uint16_t>(load(StatusField::max_hp));
    fill_status_array(msg.robots_hp, referee_robots_hp_fields, *this);

    msg.shooter_cooling = status_integer<std::uint16_t>(load(StatusField::shooter_cooling));
    msg.shooter_heat_limit = status_integer<std::uint16_t>(load(StatusField::shooter_heat_limit));
    msg.shooter_17mm_bullet_allowance =
        status_integer<std::uint16_t>(load(StatusField::shooter_bullet_allowance));
    msg.shooter_42mm_bullet_allowance =
        status_integer<std::uint16_t>(load(StatusField::shooter_42mm_bullet_allowance));
    msg.shooter_fortress_17mm_bullet_allowance =
        status_integer<std::uint16_t>(load(StatusField::shooter_fortress_17mm_bullet_allowance));
    msg.shooter_1_heat = status_integer<std::uint16_t>(load(StatusField::shooter_1_heat));
    msg.shooter_2_heat = status_integer<std::uint16_t>(load(StatusField::shooter_2_heat));

    msg.chassis_power_limit = status_integer<std::uint16_t>(load(StatusField::chassis_power_limit));
    msg.chassis_power = status_float(load(StatusField::chassis_power));
    msg.chassis_buffer_energy =
        status_integer<std::uint16_t>(load(StatusField::chassis_buffer_energy));
    msg.chassis_output_enabled = status_bool(load(StatusField::chassis_output_status));
    msg.remaining_gold_coin = status_integer<std::uint16_t>(load(StatusField::remaining_gold_coin));

    fill_status_array(msg.radar_mark_progress, referee_radar_mark_fields, *this);
    msg.radar_double_effect_chance =
        status_integer<std::uint8_t>(load(StatusField::radar_double_effect_chance));
    msg.radar_double_effect_active = status_bool(load(StatusField::radar_double_effect_active));

    msg.dart_remaining_time = status_integer<std::uint8_t>(load(StatusField::dart_remaining_time));
    msg.dart_latest_hit_target =
        status_integer<std::uint8_t>(load(StatusField::dart_latest_hit_target));
    msg.dart_hit_count = status_integer<std::uint8_t>(load(StatusField::dart_hit_count));
    msg.dart_selected_target =
        status_integer<std::uint8_t>(load(StatusField::dart_selected_target));

    msg.ally_small_energy_activation_status =
        status_integer<std::uint8_t>(load(StatusField::event_ally_small_energy_activation_status));
    msg.ally_big_energy_activation_status =
        status_integer<std::uint8_t>(load(StatusField::event_ally_big_energy_activation_status));
    msg.ally_fortress_occupation_status =
        status_integer<std::uint8_t>(load(StatusField::event_ally_fortress_occupation_status));

    fill_status_float_array(msg.ally_robot_positions, referee_ally_robot_position_fields, *this);
    fill_status_float_array(
        msg.opponent_robot_positions, referee_opponent_robot_position_fields, *this);

    msg.map_command_target_position_x =
        status_float(load(StatusField::map_command_target_position_x));
    msg.map_command_target_position_y =
        status_float(load(StatusField::map_command_target_position_y));
    msg.map_command_keyboard =
        status_integer<std::uint8_t>(load(StatusField::map_command_keyboard));
    msg.map_command_target_robot_id =
        status_integer<std::uint8_t>(load(StatusField::map_command_target_robot_id));
    msg.map_command_source = status_integer<std::uint16_t>(load(StatusField::map_command_source));
    msg.map_command_sequence =
        status_integer<std::uint32_t>(load(StatusField::map_command_sequence));

    msg.sentry_can_confirm_free_revive =
        status_bool(load(StatusField::sentry_can_confirm_free_revive));
    msg.sentry_can_exchange_instant_revive =
        status_bool(load(StatusField::sentry_can_exchange_instant_revive));
    msg.sentry_instant_revive_cost =
        status_integer<std::uint16_t>(load(StatusField::sentry_instant_revive_cost));
    msg.sentry_exchanged_bullet_allowance =
        status_integer<std::uint16_t>(load(StatusField::sentry_exchanged_bullet_allowance));
    msg.sentry_remote_bullet_exchange_count =
        status_integer<std::uint8_t>(load(StatusField::sentry_remote_bullet_exchange_count));
    msg.sentry_exchangeable_bullet_allowance =
        status_integer<std::uint16_t>(load(StatusField::sentry_exchangeable_bullet_allowance));
    msg.sentry_mode = status_integer<std::uint8_t>(load(StatusField::sentry_mode));
    msg.sentry_energy_mechanism_activatable =
        status_bool(load(StatusField::sentry_energy_mechanism_activatable));
    return msg;
}

inline double StatusStore::load(StatusField field) const noexcept {
    return values_[to_index(field)].load(std::memory_order_relaxed);
}

inline void StatusStore::apply_initial_safety_fallback() noexcept {
    values_[to_index(StatusField::game_stage)].store(safe_game_stage, std::memory_order_relaxed);
    apply_robot_status_fallback();
    apply_power_heat_fallback();
}

inline void StatusStore::arm_game_status_watchdog(Watchdog::clock::time_point now) noexcept {
    const auto timeout = load(StatusField::game_stage) == started_game_stage ? 30s : 5s;
    game_status_watchdog_.reset(now, timeout);
}

inline void StatusStore::arm_robot_status_watchdog(Watchdog::clock::time_point now) noexcept {
    const auto timeout = load(StatusField::game_stage) == started_game_stage ? 60s : 5s;
    robot_status_watchdog_.reset(now, timeout);
}

inline void StatusStore::arm_power_heat_watchdog(Watchdog::clock::time_point now) noexcept {
    power_heat_watchdog_.reset(now, 3s);
}

inline void StatusStore::apply_robot_status_fallback() noexcept {
    values_[to_index(StatusField::shooter_cooling)].store(
        safe_shooter_cooling, std::memory_order_relaxed);
    values_[to_index(StatusField::shooter_heat_limit)].store(
        safe_shooter_heat_limit, std::memory_order_relaxed);
    values_[to_index(StatusField::chassis_power_limit)].store(
        safe_chassis_power_limit, std::memory_order_relaxed);
}

inline void StatusStore::apply_power_heat_fallback() noexcept {
    values_[to_index(StatusField::chassis_power)].store(
        safe_chassis_power, std::memory_order_relaxed);
    values_[to_index(StatusField::chassis_buffer_energy)].store(
        safe_chassis_buffer_energy, std::memory_order_relaxed);
}

} // namespace rmgo_referee
