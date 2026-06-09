#pragma once

#include <cmath>
#include <limits>
#include <utility>

#include <eigen3/Eigen/Geometry>

namespace rmgo_core::gimbal {

class TwoAxisGimbalSolver {
public:
    class Operation {
        friend class TwoAxisGimbalSolver;

        virtual Eigen::Vector3d
            update(TwoAxisGimbalSolver& solver, double yaw, double pitch) const = 0;
    };

    struct AngleError {
        double yaw = nan;
        double pitch = nan;
    };

    TwoAxisGimbalSolver(double upper_limit, double lower_limit)
        : upper_limit_(std::cos(upper_limit), -std::sin(upper_limit))
        , lower_limit_(std::cos(lower_limit), -std::sin(lower_limit)) {}

    class SetDisabled : public Operation {
        Eigen::Vector3d
            update(TwoAxisGimbalSolver& solver, double /*yaw*/, double /*pitch*/) const override {
            solver.control_enabled_ = false;
            return Eigen::Vector3d::UnitX();
        }
    };

    class SetToLevel : public Operation {
        Eigen::Vector3d
            update(TwoAxisGimbalSolver& solver, double yaw, double pitch) const override {
            Eigen::Vector3d base_direction = pitch_to_base(yaw, pitch) * Eigen::Vector3d::UnitX();
            if (base_direction.head<2>().norm() < 1e-6) {
                return Eigen::Vector3d::UnitX();
            }

            solver.control_enabled_ = true;
            base_direction.z() = 0.0;
            base_direction.normalize();
            return (pitch_to_base(yaw, pitch).inverse() * base_direction).normalized();
        }
    };

    class SetControlDirection : public Operation {
    public:
        explicit SetControlDirection(Eigen::Vector3d target)
            : target_(std::move(target)) {}

    private:
        Eigen::Vector3d
            update(TwoAxisGimbalSolver& solver, double yaw, double pitch) const override {
            solver.control_enabled_ = true;
            return (pitch_to_base(yaw, pitch).inverse() * target_).normalized();
        }

        Eigen::Vector3d target_;
    };

    class SetControlShift : public Operation {
    public:
        SetControlShift(double yaw, double pitch)
            : yaw_(yaw)
            , pitch_(pitch) {}

    private:
        Eigen::Vector3d
            update(TwoAxisGimbalSolver& solver, double yaw, double pitch) const override {
            Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
            if (!solver.control_enabled_) {
                solver.control_enabled_ = true;
            } else {
                direction = pitch_to_base(yaw, pitch).inverse() * solver.control_direction_;
            }

            const Eigen::AngleAxisd yaw_transform{yaw_, Eigen::Vector3d::UnitZ()};
            const Eigen::AngleAxisd pitch_transform{pitch_, Eigen::Vector3d::UnitY()};
            return (pitch_transform * (yaw_transform * direction)).normalized();
        }

        double yaw_ = 0.0;
        double pitch_ = 0.0;
    };

    AngleError update(double yaw, double pitch, const Operation& operation) {
        Eigen::Vector3d control_direction = operation.update(*this, yaw, pitch);
        if (!control_enabled_) {
            return {};
        }

        auto [direction_yaw_link, pitch_cs] = pitch_link_to_yaw_link(control_direction, pitch);
        clamp_control_direction(direction_yaw_link);
        if (!control_enabled_) {
            return {};
        }

        const Eigen::Vector3d direction_pitch_link =
            yaw_link_to_pitch_link(direction_yaw_link, pitch_cs);
        control_direction_ = pitch_to_base(yaw, pitch) * direction_pitch_link;
        return calculate_control_errors(direction_yaw_link, pitch_cs);
    }

    bool enabled() const { return control_enabled_; }

private:
    static Eigen::Isometry3d pitch_to_base(double yaw, double pitch) {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() = (Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}
                              * Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()})
                                 .matrix();
        return transform;
    }

    static std::pair<Eigen::Vector3d, Eigen::Vector2d>
        pitch_link_to_yaw_link(const Eigen::Vector3d& direction, double encoder_pitch) {
        const Eigen::Vector2d pitch_cs{std::cos(encoder_pitch), -std::sin(encoder_pitch)};
        const auto& x = direction.x();
        const auto& y = direction.y();
        const auto& z = direction.z();
        return {
            Eigen::Vector3d{
                x * pitch_cs.x() - z * pitch_cs.y(),
                y,
                x * pitch_cs.y() + z * pitch_cs.x(),
            },
            pitch_cs,
        };
    }

    static Eigen::Vector3d
        yaw_link_to_pitch_link(const Eigen::Vector3d& direction, const Eigen::Vector2d& pitch) {
        const auto& x = direction.x();
        const auto& y = direction.y();
        const auto& z = direction.z();
        const auto& c = pitch.x();
        const auto& s = pitch.y();
        return {
            x * c + z * s,
            y,
            -x * s + z * c,
        };
    }

    void clamp_control_direction(Eigen::Vector3d& direction) {
        const auto& x = direction.x();
        const auto& y = direction.y();
        const auto& z = direction.z();

        Eigen::Vector2d projection{x, y};
        const double norm = projection.norm();
        if (norm <= 0.0) {
            control_enabled_ = false;
            return;
        }
        projection /= norm;

        if (z > upper_limit_.y()) {
            direction << upper_limit_.x() * projection, upper_limit_.y();
        } else if (z < lower_limit_.y()) {
            direction << lower_limit_.x() * projection, lower_limit_.y();
        }
    }

    static AngleError
        calculate_control_errors(const Eigen::Vector3d& direction, const Eigen::Vector2d& pitch) {
        const auto& x = direction.x();
        const auto& y = direction.y();
        const auto& z = direction.z();
        const auto& c = pitch.x();
        const auto& s = pitch.y();

        const double x_projected = std::sqrt(x * x + y * y);
        return {
            std::atan2(y, x),
            -std::atan2(z * c - x_projected * s, z * s + x_projected * c),
        };
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    Eigen::Vector2d upper_limit_;
    Eigen::Vector2d lower_limit_;
    bool control_enabled_ = false;
    Eigen::Vector3d control_direction_ = Eigen::Vector3d::UnitX();
};

} // namespace rmgo_core::gimbal
