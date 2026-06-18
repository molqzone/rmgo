#include "status.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

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
    Array& output, const std::array<RefereeStatusField, size>& fields,
    const RefereeStatusStore& store) noexcept {
    using Value = typename Array::value_type;
    for (std::size_t index = 0; index < size; ++index) {
        output[index] = status_integer<Value>(store.get(fields[index]));
    }
}

template <typename Array, std::size_t size>
void fill_status_float_array(
    Array& output, const std::array<RefereeStatusField, size>& fields,
    const RefereeStatusStore& store) noexcept {
    for (std::size_t index = 0; index < size; ++index) {
        output[index] = status_float(store.get(fields[index]));
    }
}

constexpr std::array robots_hp_fields{
    RefereeStatusField::robots_hp_red_1,        RefereeStatusField::robots_hp_red_2,
    RefereeStatusField::robots_hp_red_3,        RefereeStatusField::robots_hp_red_4,
    RefereeStatusField::robots_hp_red_5,        RefereeStatusField::robots_hp_red_7,
    RefereeStatusField::robots_hp_red_outpost,  RefereeStatusField::robots_hp_red_base,
    RefereeStatusField::robots_hp_blue_1,       RefereeStatusField::robots_hp_blue_2,
    RefereeStatusField::robots_hp_blue_3,       RefereeStatusField::robots_hp_blue_4,
    RefereeStatusField::robots_hp_blue_5,       RefereeStatusField::robots_hp_blue_7,
    RefereeStatusField::robots_hp_blue_outpost, RefereeStatusField::robots_hp_blue_base,
};

constexpr std::array radar_mark_fields{
    RefereeStatusField::radar_mark_hero,       RefereeStatusField::radar_mark_engineer,
    RefereeStatusField::radar_mark_infantry_3, RefereeStatusField::radar_mark_infantry_4,
    RefereeStatusField::radar_mark_infantry_5, RefereeStatusField::radar_mark_sentry,
};

constexpr std::array ally_robot_position_fields{
    RefereeStatusField::ally_hero_position_x,       RefereeStatusField::ally_hero_position_y,
    RefereeStatusField::ally_engineer_position_x,   RefereeStatusField::ally_engineer_position_y,
    RefereeStatusField::ally_infantry_3_position_x, RefereeStatusField::ally_infantry_3_position_y,
    RefereeStatusField::ally_infantry_4_position_x, RefereeStatusField::ally_infantry_4_position_y,
    RefereeStatusField::ally_infantry_5_position_x, RefereeStatusField::ally_infantry_5_position_y,
};

constexpr std::array opponent_robot_position_fields{
    RefereeStatusField::opponent_hero_position_x,
    RefereeStatusField::opponent_hero_position_y,
    RefereeStatusField::opponent_engineer_position_x,
    RefereeStatusField::opponent_engineer_position_y,
    RefereeStatusField::opponent_infantry_3_position_x,
    RefereeStatusField::opponent_infantry_3_position_y,
    RefereeStatusField::opponent_infantry_4_position_x,
    RefereeStatusField::opponent_infantry_4_position_y,
    RefereeStatusField::opponent_uav_position_x,
    RefereeStatusField::opponent_uav_position_y,
    RefereeStatusField::opponent_sentry_position_x,
    RefereeStatusField::opponent_sentry_position_y,
};

} // namespace

RefereeStatusStore::RefereeStatusStore() { reset(); }

void RefereeStatusStore::reset() noexcept {
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

void RefereeStatusStore::set(RefereeStatusField field, double value) noexcept {
    values_[to_index(field)].store(value, std::memory_order_relaxed);
    if (field == RefereeStatusField::id) {
        robot_id_.store(static_cast<std::uint16_t>(value), std::memory_order_release);
    }
    const auto now = Watchdog::clock::now();
    if (field == RefereeStatusField::game_stage) {
        arm_game_status_watchdog(now);
    } else if (
        field == RefereeStatusField::id || field == RefereeStatusField::hp
        || field == RefereeStatusField::max_hp || field == RefereeStatusField::shooter_cooling
        || field == RefereeStatusField::shooter_heat_limit
        || field == RefereeStatusField::chassis_power_limit
        || field == RefereeStatusField::chassis_output_status) {
        arm_robot_status_watchdog(now);
    } else if (
        field == RefereeStatusField::chassis_power
        || field == RefereeStatusField::chassis_buffer_energy
        || field == RefereeStatusField::shooter_1_heat
        || field == RefereeStatusField::shooter_2_heat) {
        arm_power_heat_watchdog(now);
    }
}

double RefereeStatusStore::get(RefereeStatusField field) const noexcept { return load(field); }

void RefereeStatusStore::mark_online(std::chrono::steady_clock::time_point time) noexcept {
    last_update_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count(),
        std::memory_order_release);
}

RefereeStatusSafetyEvents RefereeStatusStore::maintain_safety() noexcept {
    const auto now = Watchdog::clock::now();
    auto events = RefereeStatusSafetyEvents{};
    if (game_status_watchdog_.consume_expiration(now)) {
        values_[to_index(RefereeStatusField::game_stage)].store(
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

bool RefereeStatusStore::is_fresh(double online_timeout) const noexcept {
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

bool RefereeStatusStore::game_status_fresh() const noexcept {
    return game_status_watchdog_.fresh(Watchdog::clock::now());
}

bool RefereeStatusStore::robot_status_fresh() const noexcept {
    return robot_status_watchdog_.fresh(Watchdog::clock::now());
}

bool RefereeStatusStore::power_heat_fresh() const noexcept {
    return power_heat_watchdog_.fresh(Watchdog::clock::now());
}

std::uint16_t RefereeStatusStore::robot_id() const noexcept {
    return robot_id_.load(std::memory_order_acquire);
}

ui::RefereeUiState RefereeStatusStore::to_ui_state(double online_timeout) const noexcept {
    return ui::RefereeUiState{
        .online = load(RefereeStatusField::online) > 0.5 && is_fresh(online_timeout),
        .robot_id = load(RefereeStatusField::id),
        .game_stage = load(RefereeStatusField::game_stage),
        .stage_remain_time = load(RefereeStatusField::game_stage_remain_time),
        .hp = load(RefereeStatusField::hp),
        .max_hp = load(RefereeStatusField::max_hp),
        .shooter_cooling = load(RefereeStatusField::shooter_cooling),
        .shooter_heat_limit = load(RefereeStatusField::shooter_heat_limit),
        .shooter_bullet_allowance = load(RefereeStatusField::shooter_bullet_allowance),
        .shooter_1_heat = load(RefereeStatusField::shooter_1_heat),
        .shooter_2_heat = load(RefereeStatusField::shooter_2_heat),
        .chassis_power_limit = load(RefereeStatusField::chassis_power_limit),
        .chassis_power = load(RefereeStatusField::chassis_power),
        .chassis_buffer_energy = load(RefereeStatusField::chassis_buffer_energy),
        .chassis_output_status = load(RefereeStatusField::chassis_output_status),
        .chassis_mode = 0.0,
        .gimbal_enabled = 0.0,
        .shooter_mode = 0.0,
    };
}

rmgo_msg::msg::GameStatus RefereeStatusStore::to_game_status_message() const {
    auto msg = rmgo_msg::msg::GameStatus{};
    msg.game_type = status_integer<std::uint8_t>(load(RefereeStatusField::game_type));
    msg.game_progress = status_integer<std::uint8_t>(load(RefereeStatusField::game_stage));
    msg.stage_remain_time =
        status_integer<std::uint16_t>(load(RefereeStatusField::game_stage_remain_time));
    msg.sync_timestamp =
        status_integer<std::uint64_t>(load(RefereeStatusField::game_sync_timestamp));
    return msg;
}

rmgo_msg::msg::GameRobotStatus RefereeStatusStore::to_game_robot_status_message() const {
    auto msg = rmgo_msg::msg::GameRobotStatus{};
    msg.robot_id = status_integer<std::uint8_t>(load(RefereeStatusField::id));
    msg.robot_level = status_integer<std::uint8_t>(load(RefereeStatusField::robot_level));
    msg.hp = status_integer<std::uint16_t>(load(RefereeStatusField::hp));
    msg.max_hp = status_integer<std::uint16_t>(load(RefereeStatusField::max_hp));
    msg.shooter_cooling = status_integer<std::uint16_t>(load(RefereeStatusField::shooter_cooling));
    msg.shooter_heat_limit =
        status_integer<std::uint16_t>(load(RefereeStatusField::shooter_heat_limit));
    msg.chassis_power_limit =
        status_integer<std::uint16_t>(load(RefereeStatusField::chassis_power_limit));
    msg.gimbal_output_enabled = status_bool(load(RefereeStatusField::gimbal_output_status));
    msg.chassis_output_enabled = status_bool(load(RefereeStatusField::chassis_output_status));
    msg.shooter_output_enabled = status_bool(load(RefereeStatusField::shooter_output_status));
    return msg;
}

rmgo_msg::msg::PowerHeatData RefereeStatusStore::to_power_heat_data_message() const {
    auto msg = rmgo_msg::msg::PowerHeatData{};
    msg.chassis_voltage = status_integer<std::uint16_t>(load(RefereeStatusField::chassis_voltage));
    msg.chassis_current = status_integer<std::uint16_t>(load(RefereeStatusField::chassis_current));
    msg.chassis_power = status_float(load(RefereeStatusField::chassis_power));
    msg.chassis_buffer_energy =
        status_integer<std::uint16_t>(load(RefereeStatusField::chassis_buffer_energy));
    msg.shooter_1_heat = status_integer<std::uint16_t>(load(RefereeStatusField::shooter_1_heat));
    msg.shooter_2_heat = status_integer<std::uint16_t>(load(RefereeStatusField::shooter_2_heat));
    msg.shooter_42mm_heat =
        status_integer<std::uint16_t>(load(RefereeStatusField::shooter_42mm_heat));
    return msg;
}

rmgo_msg::msg::RefereeStatus RefereeStatusStore::to_message(double online_timeout) const {
    const auto state = to_ui_state(online_timeout);
    auto msg = rmgo_msg::msg::RefereeStatus{};
    msg.online = state.online;
    msg.game_status_fresh = game_status_fresh();
    msg.robot_status_fresh = robot_status_fresh();
    msg.power_heat_fresh = power_heat_fresh();
    msg.robot_id = status_integer<std::uint16_t>(state.robot_id);
    msg.game_stage = status_integer<std::uint8_t>(state.game_stage);
    msg.stage_remain_time = status_integer<std::uint16_t>(state.stage_remain_time);
    msg.game_sync_timestamp =
        status_integer<std::uint64_t>(load(RefereeStatusField::game_sync_timestamp));

    msg.hp = status_integer<std::uint16_t>(state.hp);
    msg.max_hp = status_integer<std::uint16_t>(state.max_hp);
    fill_status_array(msg.robots_hp, robots_hp_fields, *this);

    msg.shooter_cooling = status_integer<std::uint16_t>(state.shooter_cooling);
    msg.shooter_heat_limit = status_integer<std::uint16_t>(state.shooter_heat_limit);
    msg.shooter_17mm_bullet_allowance =
        status_integer<std::uint16_t>(state.shooter_bullet_allowance);
    msg.shooter_42mm_bullet_allowance =
        status_integer<std::uint16_t>(load(RefereeStatusField::shooter_42mm_bullet_allowance));
    msg.shooter_fortress_17mm_bullet_allowance = status_integer<std::uint16_t>(
        load(RefereeStatusField::shooter_fortress_17mm_bullet_allowance));
    msg.shooter_1_heat = status_integer<std::uint16_t>(state.shooter_1_heat);
    msg.shooter_2_heat = status_integer<std::uint16_t>(state.shooter_2_heat);

    msg.chassis_power_limit = status_integer<std::uint16_t>(state.chassis_power_limit);
    msg.chassis_power = status_float(state.chassis_power);
    msg.chassis_buffer_energy = status_integer<std::uint16_t>(state.chassis_buffer_energy);
    msg.chassis_output_enabled = status_bool(state.chassis_output_status);
    msg.remaining_gold_coin =
        status_integer<std::uint16_t>(load(RefereeStatusField::remaining_gold_coin));

    fill_status_array(msg.radar_mark_progress, radar_mark_fields, *this);
    msg.radar_double_effect_chance =
        status_integer<std::uint8_t>(load(RefereeStatusField::radar_double_effect_chance));
    msg.radar_double_effect_active =
        status_bool(load(RefereeStatusField::radar_double_effect_active));

    msg.dart_remaining_time =
        status_integer<std::uint8_t>(load(RefereeStatusField::dart_remaining_time));
    msg.dart_latest_hit_target =
        status_integer<std::uint8_t>(load(RefereeStatusField::dart_latest_hit_target));
    msg.dart_hit_count = status_integer<std::uint8_t>(load(RefereeStatusField::dart_hit_count));
    msg.dart_selected_target =
        status_integer<std::uint8_t>(load(RefereeStatusField::dart_selected_target));

    msg.ally_small_energy_activation_status = status_integer<std::uint8_t>(
        load(RefereeStatusField::event_ally_small_energy_activation_status));
    msg.ally_big_energy_activation_status = status_integer<std::uint8_t>(
        load(RefereeStatusField::event_ally_big_energy_activation_status));
    msg.ally_fortress_occupation_status = status_integer<std::uint8_t>(
        load(RefereeStatusField::event_ally_fortress_occupation_status));

    fill_status_float_array(msg.ally_robot_positions, ally_robot_position_fields, *this);
    fill_status_float_array(msg.opponent_robot_positions, opponent_robot_position_fields, *this);

    msg.map_command_target_position_x =
        status_float(load(RefereeStatusField::map_command_target_position_x));
    msg.map_command_target_position_y =
        status_float(load(RefereeStatusField::map_command_target_position_y));
    msg.map_command_keyboard =
        status_integer<std::uint8_t>(load(RefereeStatusField::map_command_keyboard));
    msg.map_command_target_robot_id =
        status_integer<std::uint8_t>(load(RefereeStatusField::map_command_target_robot_id));
    msg.map_command_source =
        status_integer<std::uint16_t>(load(RefereeStatusField::map_command_source));
    msg.map_command_sequence =
        status_integer<std::uint32_t>(load(RefereeStatusField::map_command_sequence));

    msg.sentry_can_confirm_free_revive =
        status_bool(load(RefereeStatusField::sentry_can_confirm_free_revive));
    msg.sentry_can_exchange_instant_revive =
        status_bool(load(RefereeStatusField::sentry_can_exchange_instant_revive));
    msg.sentry_instant_revive_cost =
        status_integer<std::uint16_t>(load(RefereeStatusField::sentry_instant_revive_cost));
    msg.sentry_exchanged_bullet_allowance =
        status_integer<std::uint16_t>(load(RefereeStatusField::sentry_exchanged_bullet_allowance));
    msg.sentry_remote_bullet_exchange_count =
        status_integer<std::uint8_t>(load(RefereeStatusField::sentry_remote_bullet_exchange_count));
    msg.sentry_exchangeable_bullet_allowance = status_integer<std::uint16_t>(
        load(RefereeStatusField::sentry_exchangeable_bullet_allowance));
    msg.sentry_mode = status_integer<std::uint8_t>(load(RefereeStatusField::sentry_mode));
    msg.sentry_energy_mechanism_activatable =
        status_bool(load(RefereeStatusField::sentry_energy_mechanism_activatable));
    return msg;
}

double RefereeStatusStore::load(RefereeStatusField field) const noexcept {
    return values_[to_index(field)].load(std::memory_order_relaxed);
}

void RefereeStatusStore::apply_initial_safety_fallback() noexcept {
    values_[to_index(RefereeStatusField::game_stage)].store(
        safe_game_stage, std::memory_order_relaxed);
    apply_robot_status_fallback();
    apply_power_heat_fallback();
}

void RefereeStatusStore::arm_game_status_watchdog(Watchdog::clock::time_point now) noexcept {
    const auto timeout = load(RefereeStatusField::game_stage) == started_game_stage ? 30s : 5s;
    game_status_watchdog_.reset(now, timeout);
}

void RefereeStatusStore::arm_robot_status_watchdog(Watchdog::clock::time_point now) noexcept {
    const auto timeout = load(RefereeStatusField::game_stage) == started_game_stage ? 60s : 5s;
    robot_status_watchdog_.reset(now, timeout);
}

void RefereeStatusStore::arm_power_heat_watchdog(Watchdog::clock::time_point now) noexcept {
    power_heat_watchdog_.reset(now, 3s);
}

void RefereeStatusStore::apply_robot_status_fallback() noexcept {
    values_[to_index(RefereeStatusField::shooter_cooling)].store(
        safe_shooter_cooling, std::memory_order_relaxed);
    values_[to_index(RefereeStatusField::shooter_heat_limit)].store(
        safe_shooter_heat_limit, std::memory_order_relaxed);
    values_[to_index(RefereeStatusField::chassis_power_limit)].store(
        safe_chassis_power_limit, std::memory_order_relaxed);
}

void RefereeStatusStore::apply_power_heat_fallback() noexcept {
    values_[to_index(RefereeStatusField::chassis_power)].store(
        safe_chassis_power, std::memory_order_relaxed);
    values_[to_index(RefereeStatusField::chassis_buffer_energy)].store(
        safe_chassis_buffer_energy, std::memory_order_relaxed);
}

} // namespace rmgo_referee
