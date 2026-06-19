#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace rmgo_core::controller::chassis {

enum class ChassisPowerMode : std::uint8_t {
    unknown = 0,
    normal = 1,
    burst = 2,
    charge = 3,
};

struct ChassisPowerPolicyConfig {
    double safety_power = 0.0;
    double charge_power_ratio = 0.70;
    double capacitor_threshold = 0.0;
    double buffer_threshold = 0.0;
    double burst_power = 0.0;
};

struct ChassisPowerPolicyCapacitor {
    bool online = false;
    bool resetting = false;
    double charge_ratio = 0.0;
};

struct ChassisPowerPolicyInput {
    std::optional<double> referee_power_limit;
    std::optional<double> referee_buffer_energy;
    std::optional<ChassisPowerMode> remote_mode;
    std::optional<ChassisPowerPolicyCapacitor> capacitor;
};

class ChassisPowerPolicy {
public:
    ChassisPowerPolicy() = default;
    explicit ChassisPowerPolicy(ChassisPowerPolicyConfig config)
        : config_(config) {}

    double calculate(const ChassisPowerPolicyInput& input) const {
        if (!valid_power_limit(input.referee_power_limit)) {
            return safety_power();
        }
        if (!input.remote_mode.has_value()) {
            return safety_power();
        }

        const double referee_limit = std::max(0.0, *input.referee_power_limit);
        switch (*input.remote_mode) {
        case ChassisPowerMode::normal: return referee_limit;
        case ChassisPowerMode::charge: return charge_power(referee_limit);
        case ChassisPowerMode::burst: return burst_available(input) ? burst_power() : referee_limit;
        case ChassisPowerMode::unknown: return safety_power();
        }
        return safety_power();
    }

private:
    static bool valid_power_limit(std::optional<double> value) noexcept {
        return value.has_value() && !std::isnan(*value) && *value >= 0.0;
    }

    double safety_power() const noexcept {
        return std::isfinite(config_.safety_power) && config_.safety_power > 0.0
                 ? config_.safety_power
                 : 0.0;
    }

    double charge_power(double referee_limit) const noexcept {
        const double ratio = std::isfinite(config_.charge_power_ratio)
                               ? std::clamp(config_.charge_power_ratio, 0.0, 1.0)
                               : 0.0;
        return referee_limit * ratio;
    }

    double burst_power() const noexcept {
        return std::isfinite(config_.burst_power) && config_.burst_power > 0.0 ? config_.burst_power
                                                                               : 0.0;
    }

    bool burst_available(const ChassisPowerPolicyInput& input) const noexcept {
        if (!input.capacitor.has_value() || !input.referee_buffer_energy.has_value()) {
            return false;
        }

        const auto& capacitor = *input.capacitor;
        if (!capacitor.online || capacitor.resetting || !std::isfinite(capacitor.charge_ratio)) {
            return false;
        }
        if (!std::isfinite(*input.referee_buffer_energy)) {
            return false;
        }

        return capacitor.charge_ratio >= config_.capacitor_threshold
            && *input.referee_buffer_energy >= config_.buffer_threshold && burst_power() > 0.0;
    }

    ChassisPowerPolicyConfig config_;
};

} // namespace rmgo_core::controller::chassis
