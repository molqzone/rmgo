#pragma once

#include <chrono>

namespace rmgo_core::referee {

struct RefereeSnapshot {
    bool online = false;
    double robot_id = 0.0;
    double game_progress = 0.0;
    double stage_remain_time = 0.0;
    double self_hp = 0.0;
    double max_hp = 0.0;
    double chassis_power_limit = 0.0;
    double chassis_power = 0.0;
    double chassis_power_buffer = 0.0;
    double shooter_heat_17mm_1 = 0.0;
    double shooter_heat_17mm_2 = 0.0;
    double shooter_heat_42mm = 0.0;
    double heat_limit_17mm = 0.0;
    double cooling_rate_17mm = 0.0;
    double projectile_allowance_17mm = 0.0;
    double projectile_allowance_42mm = 0.0;
    double remaining_gold_coin = 0.0;
    std::chrono::steady_clock::time_point last_update{};
};

} // namespace rmgo_core::referee
