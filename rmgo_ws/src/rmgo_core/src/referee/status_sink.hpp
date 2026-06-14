#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace rmgo_core::referee {

// Indexes mirror io_state_interfaces::referee_state_interfaces. The referee
// parser writes wire-frame values into these fields instead of owning a large
// semantic "referee" object.
enum class RefereeStatusField : std::size_t {
  online = 0,
  id,
  game_stage,
  game_stage_remain_time,
  hp,
  max_hp,
  shooter_cooling,
  shooter_heat_limit,
  shooter_bullet_allowance,
  shooter_1_heat,
  shooter_2_heat,
  chassis_power_limit,
  chassis_power,
  chassis_buffer_energy,
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

constexpr std::size_t to_index(RefereeStatusField field) noexcept {
  return static_cast<std::size_t>(field);
}

class RefereeStatusSink {
public:
  virtual ~RefereeStatusSink() = default;

  virtual void set(RefereeStatusField field, double value) noexcept = 0;
  virtual double get(RefereeStatusField field) const noexcept = 0;
  virtual void mark_online(std::chrono::steady_clock::time_point time) noexcept = 0;
};

} // namespace rmgo_core::referee
