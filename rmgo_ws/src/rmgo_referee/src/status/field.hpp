#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rmgo_referee {

// Wire-frame fields mirrored by the referee status store. Keeping this index
// list separate makes parser/store dependencies explicit without growing a
// semantic "referee" object.
enum class StatusField : std::size_t {
    online = 0,
    id,
    robot_level,
    game_type,
    game_stage,
    game_stage_remain_time,
    hp,
    max_hp,
    shooter_cooling,
    shooter_heat_limit,
    shooter_bullet_allowance,
    shooter_1_heat,
    shooter_2_heat,
    shooter_42mm_heat,
    chassis_power_limit,
    chassis_voltage,
    chassis_current,
    chassis_power,
    chassis_buffer_energy,
    gimbal_output_status,
    shooter_output_status,
    radar_mark_hero,
    radar_mark_engineer,
    radar_mark_infantry_3,
    radar_mark_infantry_4,
    radar_mark_infantry_5,
    radar_mark_sentry,
    radar_double_effect_chance,
    radar_double_effect_active,
    dart_remaining_time,
    dart_latest_hit_target,
    dart_hit_count,
    dart_selected_target,
    game_sync_timestamp,
    event_ally_big_energy_activation_status,
    event_ally_small_energy_activation_status,
    event_ally_fortress_occupation_status,
    shooter_42mm_bullet_allowance,
    shooter_fortress_17mm_bullet_allowance,
    chassis_output_status,
    remaining_gold_coin,
    robots_hp_red_1,
    robots_hp_red_2,
    robots_hp_red_3,
    robots_hp_red_4,
    robots_hp_red_5,
    robots_hp_red_7,
    robots_hp_red_outpost,
    robots_hp_red_base,
    robots_hp_blue_1,
    robots_hp_blue_2,
    robots_hp_blue_3,
    robots_hp_blue_4,
    robots_hp_blue_5,
    robots_hp_blue_7,
    robots_hp_blue_outpost,
    robots_hp_blue_base,
    ally_hero_position_x,
    ally_hero_position_y,
    ally_engineer_position_x,
    ally_engineer_position_y,
    ally_infantry_3_position_x,
    ally_infantry_3_position_y,
    ally_infantry_4_position_x,
    ally_infantry_4_position_y,
    ally_infantry_5_position_x,
    ally_infantry_5_position_y,
    opponent_hero_position_x,
    opponent_hero_position_y,
    opponent_engineer_position_x,
    opponent_engineer_position_y,
    opponent_infantry_3_position_x,
    opponent_infantry_3_position_y,
    opponent_infantry_4_position_x,
    opponent_infantry_4_position_y,
    opponent_uav_position_x,
    opponent_uav_position_y,
    opponent_sentry_position_x,
    opponent_sentry_position_y,
    map_command_target_position_x,
    map_command_target_position_y,
    map_command_keyboard,
    map_command_target_robot_id,
    map_command_source,
    map_command_sequence,
    sentry_can_confirm_free_revive,
    sentry_can_exchange_instant_revive,
    sentry_instant_revive_cost,
    sentry_exchanged_bullet_allowance,
    sentry_remote_bullet_exchange_count,
    sentry_exchangeable_bullet_allowance,
    sentry_mode,
    sentry_energy_mechanism_activatable,
    count,
};

constexpr std::size_t to_index(StatusField field) noexcept {
    return static_cast<std::size_t>(field);
}

inline constexpr std::array referee_robots_hp_fields{
    StatusField::robots_hp_red_1,        StatusField::robots_hp_red_2,
    StatusField::robots_hp_red_3,        StatusField::robots_hp_red_4,
    StatusField::robots_hp_red_5,        StatusField::robots_hp_red_7,
    StatusField::robots_hp_red_outpost,  StatusField::robots_hp_red_base,
    StatusField::robots_hp_blue_1,       StatusField::robots_hp_blue_2,
    StatusField::robots_hp_blue_3,       StatusField::robots_hp_blue_4,
    StatusField::robots_hp_blue_5,       StatusField::robots_hp_blue_7,
    StatusField::robots_hp_blue_outpost, StatusField::robots_hp_blue_base,
};

inline constexpr std::array referee_radar_mark_fields{
    StatusField::radar_mark_hero,       StatusField::radar_mark_engineer,
    StatusField::radar_mark_infantry_3, StatusField::radar_mark_infantry_4,
    StatusField::radar_mark_infantry_5, StatusField::radar_mark_sentry,
};

inline constexpr std::array referee_ally_robot_position_fields{
    StatusField::ally_hero_position_x,       StatusField::ally_hero_position_y,
    StatusField::ally_engineer_position_x,   StatusField::ally_engineer_position_y,
    StatusField::ally_infantry_3_position_x, StatusField::ally_infantry_3_position_y,
    StatusField::ally_infantry_4_position_x, StatusField::ally_infantry_4_position_y,
    StatusField::ally_infantry_5_position_x, StatusField::ally_infantry_5_position_y,
};

inline constexpr std::array referee_opponent_robot_position_fields{
    StatusField::opponent_hero_position_x,       StatusField::opponent_hero_position_y,
    StatusField::opponent_engineer_position_x,   StatusField::opponent_engineer_position_y,
    StatusField::opponent_infantry_3_position_x, StatusField::opponent_infantry_3_position_y,
    StatusField::opponent_infantry_4_position_x, StatusField::opponent_infantry_4_position_y,
    StatusField::opponent_uav_position_x,        StatusField::opponent_uav_position_y,
    StatusField::opponent_sentry_position_x,     StatusField::opponent_sentry_position_y,
};

namespace referee_wire {

struct [[gnu::packed]] GameStatus {
    std::uint8_t game_type     : 4;
    std::uint8_t game_progress : 4;
    std::uint16_t stage_remain_time;
    std::uint64_t sync_timestamp;
};
static_assert(sizeof(GameStatus) == 11);

struct [[gnu::packed]] GameRobotHp {
    std::uint16_t hp[16];
};
static_assert(sizeof(GameRobotHp) == 32);

struct [[gnu::packed]] EventData {
    std::uint32_t event_data;
};
static_assert(sizeof(EventData) == 4);

struct [[gnu::packed]] DartStatus {
    std::uint8_t dart_remaining_time;
    std::uint16_t dart_info;
};
static_assert(sizeof(DartStatus) == 3);

struct [[gnu::packed]] RobotStatus {
    std::uint8_t robot_id;
    std::uint8_t robot_level;
    std::uint16_t current_hp;
    std::uint16_t maximum_hp;
    std::uint16_t shooter_barrel_cooling_value;
    std::uint16_t shooter_barrel_heat_limit;
    std::uint16_t chassis_power_limit;
    std::uint8_t power_management_status;
};
static_assert(sizeof(RobotStatus) == 13);

struct [[gnu::packed]] PowerHeatData {
    std::uint16_t chassis_voltage;
    std::uint16_t chassis_current;
    float chassis_power;
    std::uint16_t chassis_buffer_energy;
    std::uint16_t shooter_1_heat;
    std::uint16_t shooter_2_heat;
};
static_assert(sizeof(PowerHeatData) == 14);

struct [[gnu::packed]] PowerHeatDataWith42mm {
    std::uint16_t chassis_voltage;
    std::uint16_t chassis_current;
    float chassis_power;
    std::uint16_t chassis_buffer_energy;
    std::uint16_t shooter_1_heat;
    std::uint16_t shooter_2_heat;
    std::uint16_t shooter_42mm_heat;
};
static_assert(sizeof(PowerHeatDataWith42mm) == 16);

struct [[gnu::packed]] ProjectileAllowance17mm {
    std::uint16_t projectile_allowance_17mm;
};
static_assert(sizeof(ProjectileAllowance17mm) == 2);

struct [[gnu::packed]] ProjectileAllowanceBase {
    std::uint16_t projectile_allowance_17mm;
    std::uint16_t projectile_allowance_42mm;
    std::uint16_t remaining_gold_coin;
};
static_assert(sizeof(ProjectileAllowanceBase) == 6);

struct [[gnu::packed]] ProjectileAllowance {
    std::uint16_t projectile_allowance_17mm;
    std::uint16_t projectile_allowance_42mm;
    std::uint16_t remaining_gold_coin;
    std::uint16_t projectile_allowance_fortress;
};
static_assert(sizeof(ProjectileAllowance) == 8);

struct [[gnu::packed]] GameRobotPosition {
    float positions[10];
};
static_assert(sizeof(GameRobotPosition) == 40);

struct [[gnu::packed]] RadarMarkProgress {
    std::uint8_t progress[6];
};
static_assert(sizeof(RadarMarkProgress) == 6);

struct [[gnu::packed]] RadarInfo {
    std::uint8_t radar_info;
};
static_assert(sizeof(RadarInfo) == 1);

struct [[gnu::packed]] SentryInfo {
    std::uint32_t sentry_info;
    std::uint16_t sentry_info_2;
};
static_assert(sizeof(SentryInfo) == 6);

struct [[gnu::packed]] MapCommand {
    float target_position_x;
    float target_position_y;
    std::uint8_t cmd_keyboard;
    std::uint8_t target_robot_id;
    std::uint16_t cmd_source;
};
static_assert(sizeof(MapCommand) == 12);

struct [[gnu::packed]] RadarMapRobotData {
    std::uint16_t positions_cm[12];
};
static_assert(sizeof(RadarMapRobotData) == 24);

} // namespace referee_wire

} // namespace rmgo_referee
