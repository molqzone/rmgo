#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace rmgo_core::pid
{

class PidCalculator
{
public:
  PidCalculator()
  {
    reset();
  }

  PidCalculator(double kp_value, double ki_value, double kd_value) : kp(kp_value), ki(ki_value), kd(kd_value)
  {
    reset();
  }

  void reset()
  {
    last_err_ = nan();
    err_integral_ = 0.0;
  }

  double update(double err)
  {
    if (!std::isfinite(err))
    {
      return nan();
    }

    double control = kp * err;
    if (err < integral_split_max && err > integral_split_min)
    {
      control += ki * err_integral_;
      err_integral_ = std::clamp(err_integral_ + err, integral_min, integral_max);
    }
    else
    {
      err_integral_ = 0.0;
    }

    if (!std::isnan(last_err_))
    {
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
  static constexpr double infinity()
  {
    return std::numeric_limits<double>::infinity();
  }

  static constexpr double nan()
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double last_err_ = nan();
  double err_integral_ = 0.0;
};

template <typename NodeT>
PidCalculator
make_pid_calculator(NodeT& node, const std::string& prefix, std::optional<double> kp_default = std::nullopt,
                    std::optional<double> ki_default = std::nullopt, std::optional<double> kd_default = std::nullopt)
{
  const auto declare_if_missing = [&node](const std::string& name, double default_value) {
    if (!node.has_parameter(name))
    {
      node.template declare_parameter<double>(name, default_value);
    }
  };

  const auto parameter_or_default = [&node, &declare_if_missing](const std::string& name,
                                                                 std::optional<double> default_value) {
    if (default_value.has_value())
    {
      declare_if_missing(name, *default_value);
    }
    return node.get_parameter(name).as_double();
  };

  declare_if_missing(prefix + "integral_min", std::numeric_limits<double>::lowest());
  declare_if_missing(prefix + "integral_max", std::numeric_limits<double>::max());
  declare_if_missing(prefix + "integral_split_min", std::numeric_limits<double>::lowest());
  declare_if_missing(prefix + "integral_split_max", std::numeric_limits<double>::max());
  declare_if_missing(prefix + "output_min", std::numeric_limits<double>::lowest());
  declare_if_missing(prefix + "output_max", std::numeric_limits<double>::max());

  auto calculator = PidCalculator{
    parameter_or_default(prefix + "kp", kp_default),
    parameter_or_default(prefix + "ki", ki_default),
    parameter_or_default(prefix + "kd", kd_default),
  };

  calculator.integral_min = node.get_parameter(prefix + "integral_min").as_double();
  calculator.integral_max = node.get_parameter(prefix + "integral_max").as_double();
  calculator.integral_split_min = node.get_parameter(prefix + "integral_split_min").as_double();
  calculator.integral_split_max = node.get_parameter(prefix + "integral_split_max").as_double();
  calculator.output_min = node.get_parameter(prefix + "output_min").as_double();
  calculator.output_max = node.get_parameter(prefix + "output_max").as_double();
  return calculator;
}

}  // namespace rmgo_core::pid
