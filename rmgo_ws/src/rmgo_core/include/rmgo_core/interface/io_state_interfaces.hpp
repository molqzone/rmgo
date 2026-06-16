#pragma once

#include <array>

namespace rmgo_core::io_state_interfaces {

inline constexpr const char *remote_dr16_ch0 = "remote/dr16/ch0";
inline constexpr const char *remote_dr16_ch1 = "remote/dr16/ch1";
inline constexpr const char *remote_dr16_ch2 = "remote/dr16/ch2";
inline constexpr const char *remote_dr16_ch3 = "remote/dr16/ch3";
inline constexpr const char *remote_dr16_s1 = "remote/dr16/s1";
inline constexpr const char *remote_dr16_s2 = "remote/dr16/s2";
inline constexpr const char *remote_dr16_online = "remote/dr16/online";

inline constexpr std::array remote_state_interfaces{
    remote_dr16_ch0, remote_dr16_ch1, remote_dr16_ch2,    remote_dr16_ch3,
    remote_dr16_s1,  remote_dr16_s2,  remote_dr16_online,
};

inline constexpr const char *gimbal_imu_orientation_w =
    "gimbal/imu/orientation.w";
inline constexpr const char *gimbal_imu_orientation_x =
    "gimbal/imu/orientation.x";
inline constexpr const char *gimbal_imu_orientation_y =
    "gimbal/imu/orientation.y";
inline constexpr const char *gimbal_imu_orientation_z =
    "gimbal/imu/orientation.z";
inline constexpr const char *gimbal_imu_angular_velocity_x =
    "gimbal/imu/angular_velocity.x";
inline constexpr const char *gimbal_imu_angular_velocity_y =
    "gimbal/imu/angular_velocity.y";
inline constexpr const char *gimbal_imu_angular_velocity_z =
    "gimbal/imu/angular_velocity.z";
inline constexpr const char *gimbal_imu_linear_acceleration_x =
    "gimbal/imu/linear_acceleration.x";
inline constexpr const char *gimbal_imu_linear_acceleration_y =
    "gimbal/imu/linear_acceleration.y";
inline constexpr const char *gimbal_imu_linear_acceleration_z =
    "gimbal/imu/linear_acceleration.z";
inline constexpr const char *gimbal_yaw_velocity_imu =
    "gimbal/yaw/velocity_imu";
inline constexpr const char *gimbal_pitch_velocity_imu =
    "gimbal/pitch/velocity_imu";

// This quaternion represents the transform from PitchLink to OdomImu.
inline constexpr std::array gimbal_imu_orientation_state_interfaces{
    gimbal_imu_orientation_w,
    gimbal_imu_orientation_x,
    gimbal_imu_orientation_y,
    gimbal_imu_orientation_z,
};

inline constexpr std::array gimbal_imu_state_interfaces{
    gimbal_imu_orientation_w,         gimbal_imu_orientation_x,
    gimbal_imu_orientation_y,         gimbal_imu_orientation_z,
    gimbal_imu_angular_velocity_x,    gimbal_imu_angular_velocity_y,
    gimbal_imu_angular_velocity_z,    gimbal_imu_linear_acceleration_x,
    gimbal_imu_linear_acceleration_y, gimbal_imu_linear_acceleration_z,
    gimbal_yaw_velocity_imu,          gimbal_pitch_velocity_imu,
};

inline constexpr const char *referee_online = "referee/online";
inline constexpr const char *referee_id = "referee/id";
inline constexpr const char *referee_game_stage = "referee/game/stage";
inline constexpr const char *referee_game_stage_remain_time =
    "referee/game/stage_remain_time";
inline constexpr const char *referee_game_sync_timestamp =
    "referee/game/sync_timestamp";
inline constexpr const char *referee_event_ally_big_energy_activation_status =
    "referee/event/ally_big_energy_activation_status";
inline constexpr const char *referee_event_ally_small_energy_activation_status =
    "referee/event/ally_small_energy_activation_status";
inline constexpr const char *referee_event_ally_fortress_occupation_status =
    "referee/event/ally_fortress_occupation_status";
inline constexpr const char *referee_hp = "referee/hp";
inline constexpr const char *referee_max_hp = "referee/max_hp";
inline constexpr const char *referee_shooter_cooling =
    "referee/shooter/cooling";
inline constexpr const char *referee_shooter_heat_limit =
    "referee/shooter/heat_limit";
inline constexpr const char *referee_shooter_bullet_allowance =
    "referee/shooter/bullet_allowance";
inline constexpr const char *referee_shooter_42mm_bullet_allowance =
    "referee/shooter/42mm_bullet_allowance";
inline constexpr const char *referee_shooter_fortress_17mm_bullet_allowance =
    "referee/shooter/fortress_17mm_bullet_allowance";
inline constexpr const char *referee_shooter_1_heat = "referee/shooter/1/heat";
inline constexpr const char *referee_shooter_2_heat = "referee/shooter/2/heat";
inline constexpr const char *referee_chassis_power_limit =
    "referee/chassis/power_limit";
inline constexpr const char *referee_chassis_power = "referee/chassis/power";
inline constexpr const char *referee_chassis_buffer_energy =
    "referee/chassis/buffer_energy";
inline constexpr const char *referee_chassis_output_status =
    "referee/chassis/output_status";
inline constexpr const char *referee_remaining_gold_coin =
    "referee/remaining_gold_coin";
inline constexpr const char *referee_robots_hp_red_1 = "referee/robots/hp/red_1";
inline constexpr const char *referee_robots_hp_red_2 = "referee/robots/hp/red_2";
inline constexpr const char *referee_robots_hp_red_3 = "referee/robots/hp/red_3";
inline constexpr const char *referee_robots_hp_red_4 = "referee/robots/hp/red_4";
inline constexpr const char *referee_robots_hp_red_5 = "referee/robots/hp/red_5";
inline constexpr const char *referee_robots_hp_red_7 = "referee/robots/hp/red_7";
inline constexpr const char *referee_robots_hp_red_outpost =
    "referee/robots/hp/red_outpost";
inline constexpr const char *referee_robots_hp_red_base = "referee/robots/hp/red_base";
inline constexpr const char *referee_robots_hp_blue_1 = "referee/robots/hp/blue_1";
inline constexpr const char *referee_robots_hp_blue_2 = "referee/robots/hp/blue_2";
inline constexpr const char *referee_robots_hp_blue_3 = "referee/robots/hp/blue_3";
inline constexpr const char *referee_robots_hp_blue_4 = "referee/robots/hp/blue_4";
inline constexpr const char *referee_robots_hp_blue_5 = "referee/robots/hp/blue_5";
inline constexpr const char *referee_robots_hp_blue_7 = "referee/robots/hp/blue_7";
inline constexpr const char *referee_robots_hp_blue_outpost =
    "referee/robots/hp/blue_outpost";
inline constexpr const char *referee_robots_hp_blue_base =
    "referee/robots/hp/blue_base";
inline constexpr const char *referee_ally_hero_position_x =
    "referee/ally/hero_position_x";
inline constexpr const char *referee_ally_hero_position_y =
    "referee/ally/hero_position_y";
inline constexpr const char *referee_ally_engineer_position_x =
    "referee/ally/engineer_position_x";
inline constexpr const char *referee_ally_engineer_position_y =
    "referee/ally/engineer_position_y";
inline constexpr const char *referee_ally_infantry_3_position_x =
    "referee/ally/infantry_3_position_x";
inline constexpr const char *referee_ally_infantry_3_position_y =
    "referee/ally/infantry_3_position_y";
inline constexpr const char *referee_ally_infantry_4_position_x =
    "referee/ally/infantry_4_position_x";
inline constexpr const char *referee_ally_infantry_4_position_y =
    "referee/ally/infantry_4_position_y";
inline constexpr const char *referee_ally_infantry_5_position_x =
    "referee/ally/infantry_5_position_x";
inline constexpr const char *referee_ally_infantry_5_position_y =
    "referee/ally/infantry_5_position_y";
inline constexpr const char *referee_opponent_hero_position_x =
    "referee/opponent/hero_position_x";
inline constexpr const char *referee_opponent_hero_position_y =
    "referee/opponent/hero_position_y";
inline constexpr const char *referee_opponent_engineer_position_x =
    "referee/opponent/engineer_position_x";
inline constexpr const char *referee_opponent_engineer_position_y =
    "referee/opponent/engineer_position_y";
inline constexpr const char *referee_opponent_infantry_3_position_x =
    "referee/opponent/infantry_3_position_x";
inline constexpr const char *referee_opponent_infantry_3_position_y =
    "referee/opponent/infantry_3_position_y";
inline constexpr const char *referee_opponent_infantry_4_position_x =
    "referee/opponent/infantry_4_position_x";
inline constexpr const char *referee_opponent_infantry_4_position_y =
    "referee/opponent/infantry_4_position_y";
inline constexpr const char *referee_opponent_uav_position_x =
    "referee/opponent/uav_position_x";
inline constexpr const char *referee_opponent_uav_position_y =
    "referee/opponent/uav_position_y";
inline constexpr const char *referee_opponent_sentry_position_x =
    "referee/opponent/sentry_position_x";
inline constexpr const char *referee_opponent_sentry_position_y =
    "referee/opponent/sentry_position_y";
inline constexpr const char *referee_map_command_target_position_x =
    "referee/map_command/target_position_x";
inline constexpr const char *referee_map_command_target_position_y =
    "referee/map_command/target_position_y";
inline constexpr const char *referee_map_command_keyboard =
    "referee/map_command/keyboard";
inline constexpr const char *referee_map_command_target_robot_id =
    "referee/map_command/target_robot_id";
inline constexpr const char *referee_map_command_source =
    "referee/map_command/source";
inline constexpr const char *referee_map_command_sequence =
    "referee/map_command/sequence";
inline constexpr const char *referee_radar_mark_hero =
    "referee/radar/mark/hero";
inline constexpr const char *referee_radar_mark_engineer =
    "referee/radar/mark/engineer";
inline constexpr const char *referee_radar_mark_infantry_3 =
    "referee/radar/mark/infantry_3";
inline constexpr const char *referee_radar_mark_infantry_4 =
    "referee/radar/mark/infantry_4";
inline constexpr const char *referee_radar_mark_infantry_5 =
    "referee/radar/mark/infantry_5";
inline constexpr const char *referee_radar_mark_sentry =
    "referee/radar/mark/sentry";
inline constexpr const char *referee_radar_double_effect_chance =
    "referee/radar/double_effect/chance";
inline constexpr const char *referee_radar_double_effect_active =
    "referee/radar/double_effect/active";
inline constexpr const char *referee_dart_remaining_time =
    "referee/dart/remaining_time";
inline constexpr const char *referee_dart_latest_hit_target =
    "referee/dart/latest_hit_target";
inline constexpr const char *referee_dart_hit_count = "referee/dart/hit_count";
inline constexpr const char *referee_dart_selected_target =
    "referee/dart/selected_target";
inline constexpr const char *referee_sentry_can_confirm_free_revive =
    "referee/sentry/can_confirm_free_revive";
inline constexpr const char *referee_sentry_can_exchange_instant_revive =
    "referee/sentry/can_exchange_instant_revive";
inline constexpr const char *referee_sentry_instant_revive_cost =
    "referee/sentry/instant_revive_cost";
inline constexpr const char *referee_sentry_exchanged_bullet_allowance =
    "referee/sentry/exchanged_bullet_allowance";
inline constexpr const char *referee_sentry_remote_bullet_exchange_count =
    "referee/sentry/remote_bullet_exchange_count";
inline constexpr const char *referee_sentry_exchangeable_bullet_allowance =
    "referee/sentry/exchangeable_bullet_allowance";
inline constexpr const char *referee_sentry_mode = "referee/sentry/mode";
inline constexpr const char *referee_sentry_energy_mechanism_activatable =
    "referee/sentry/energy_mechanism_activatable";

inline constexpr std::array chassis_power_state_interfaces{
    referee_chassis_power,
    referee_chassis_buffer_energy,
    referee_chassis_power_limit,
};

inline constexpr std::array referee_state_interfaces{
    referee_online,
    referee_id,
    referee_game_stage,
    referee_game_stage_remain_time,
    referee_hp,
    referee_max_hp,
    referee_shooter_cooling,
    referee_shooter_heat_limit,
    referee_shooter_bullet_allowance,
    referee_shooter_1_heat,
    referee_shooter_2_heat,
    referee_chassis_power_limit,
    referee_chassis_power,
    referee_chassis_buffer_energy,
    referee_radar_mark_hero,
    referee_radar_mark_engineer,
    referee_radar_mark_infantry_3,
    referee_radar_mark_infantry_4,
    referee_radar_mark_infantry_5,
    referee_radar_mark_sentry,
    referee_radar_double_effect_chance,
    referee_radar_double_effect_active,
    referee_dart_remaining_time,
    referee_dart_latest_hit_target,
    referee_dart_hit_count,
    referee_dart_selected_target,
    referee_game_sync_timestamp,
    referee_event_ally_big_energy_activation_status,
    referee_event_ally_small_energy_activation_status,
    referee_event_ally_fortress_occupation_status,
    referee_shooter_42mm_bullet_allowance,
    referee_shooter_fortress_17mm_bullet_allowance,
    referee_chassis_output_status,
    referee_remaining_gold_coin,
    referee_robots_hp_red_1,
    referee_robots_hp_red_2,
    referee_robots_hp_red_3,
    referee_robots_hp_red_4,
    referee_robots_hp_red_5,
    referee_robots_hp_red_7,
    referee_robots_hp_red_outpost,
    referee_robots_hp_red_base,
    referee_robots_hp_blue_1,
    referee_robots_hp_blue_2,
    referee_robots_hp_blue_3,
    referee_robots_hp_blue_4,
    referee_robots_hp_blue_5,
    referee_robots_hp_blue_7,
    referee_robots_hp_blue_outpost,
    referee_robots_hp_blue_base,
    referee_ally_hero_position_x,
    referee_ally_hero_position_y,
    referee_ally_engineer_position_x,
    referee_ally_engineer_position_y,
    referee_ally_infantry_3_position_x,
    referee_ally_infantry_3_position_y,
    referee_ally_infantry_4_position_x,
    referee_ally_infantry_4_position_y,
    referee_ally_infantry_5_position_x,
    referee_ally_infantry_5_position_y,
    referee_opponent_hero_position_x,
    referee_opponent_hero_position_y,
    referee_opponent_engineer_position_x,
    referee_opponent_engineer_position_y,
    referee_opponent_infantry_3_position_x,
    referee_opponent_infantry_3_position_y,
    referee_opponent_infantry_4_position_x,
    referee_opponent_infantry_4_position_y,
    referee_opponent_uav_position_x,
    referee_opponent_uav_position_y,
    referee_opponent_sentry_position_x,
    referee_opponent_sentry_position_y,
    referee_map_command_target_position_x,
    referee_map_command_target_position_y,
    referee_map_command_keyboard,
    referee_map_command_target_robot_id,
    referee_map_command_source,
    referee_map_command_sequence,
    referee_sentry_can_confirm_free_revive,
    referee_sentry_can_exchange_instant_revive,
    referee_sentry_instant_revive_cost,
    referee_sentry_exchanged_bullet_allowance,
    referee_sentry_remote_bullet_exchange_count,
    referee_sentry_exchangeable_bullet_allowance,
    referee_sentry_mode,
    referee_sentry_energy_mechanism_activatable,
};

} // namespace rmgo_core::io_state_interfaces
