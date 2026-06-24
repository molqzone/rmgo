#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rmgo_core {
namespace pid

{
class PidCalculator {
public:
    PidCalculator() { reset(); }

    PidCalculator(double kp_value, double ki_value, double kd_value)
        : kp(kp_value)
        , ki(ki_value)
        , kd(kd_value) {
        reset();
    }

    void reset() {
        last_err_ = nan();
        err_integral_ = 0.0;
    }

    double update(double err) {
        if (!std::isfinite(err)) {
            return nan();
        }

        double control = kp * err;
        if (err < integral_split_max && err > integral_split_min) {
            control += ki * err_integral_;
            err_integral_ = std::clamp(err_integral_ + err, integral_min, integral_max);
        } else {
            err_integral_ = 0.0;
        }

        if (!std::isnan(last_err_)) {
            control += kd * (err - last_err_);
        }
        last_err_ = err;

        return std::clamp(control, output_min, output_max);
    }

    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_min = -infinity();
    double integral_max = infinity();
    double integral_split_min = -infinity();
    double integral_split_max = infinity();
    double output_min = -infinity();
    double output_max = infinity();

private:
    static constexpr double infinity() { return std::numeric_limits<double>::infinity(); }

    static constexpr double nan() { return std::numeric_limits<double>::quiet_NaN(); }

    double last_err_ = nan();
    double err_integral_ = 0.0;
};

enum class OutputLimitPolicy {
    Unbounded,
    Required,
};

template <typename NodeT>
PidCalculator
    make_pid_calculator(NodeT& node, const std::string& prefix, OutputLimitPolicy output_limits) {
    const auto required_parameter = [&node, &prefix](const char* suffix) {
        const auto name = prefix + suffix;
        if (!node.has_parameter(name)) {
            throw std::invalid_argument("Missing required PID parameter '" + name + "'");
        }
        return node.get_parameter(name).as_double();
    };

    auto calculator = PidCalculator{
        required_parameter("kp"),
        required_parameter("ki"),
        required_parameter("kd"),
    };

    if (output_limits == OutputLimitPolicy::Required) {
        calculator.output_min = required_parameter("output_min");
        calculator.output_max = required_parameter("output_max");
    }
    if (calculator.output_min > calculator.output_max) {
        throw std::invalid_argument(
            "PID parameter '" + prefix + "output_min' must not exceed '" + prefix + "output_max'");
    }
    return calculator;
}

} // namespace pid
} // namespace rmgo_core
