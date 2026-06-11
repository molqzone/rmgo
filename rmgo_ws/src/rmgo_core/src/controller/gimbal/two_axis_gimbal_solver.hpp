#pragma once

#include <cmath>
#include <limits>
#include <utility>

#include <eigen3/Eigen/Geometry>
#include <fast_tf/fast_tf.hpp>
#include <rmgo_description/tf_description.hpp>

namespace rmgo_core::gimbal {

class TwoAxisGimbalSolver {
    using OdomImu = rmgo_description::OdomImu;
    using PitchLink = rmgo_description::PitchLink;
    using YawLink = rmgo_description::YawLink;

public:
    class Operation {
        friend class TwoAxisGimbalSolver;

        virtual PitchLink::DirectionVector
            update(TwoAxisGimbalSolver& solver, const rmgo_description::Tf& tf) const = 0;
    };

    struct AngleError {
        double yaw = nan;
        double pitch = nan;
    };

    TwoAxisGimbalSolver(double lower_limit, double upper_limit)
        : lower_limit_(std::cos(lower_limit), -std::sin(lower_limit))
        , upper_limit_(std::cos(upper_limit), -std::sin(upper_limit)) {}

    class SetDisabled : public Operation {
        PitchLink::DirectionVector
            update(TwoAxisGimbalSolver& solver, const rmgo_description::Tf& /*tf*/) const override {
            solver.control_enabled_ = false;
            return {};
        }
    };

    class SetToLevel : public Operation {
        PitchLink::DirectionVector
            update(TwoAxisGimbalSolver& solver, const rmgo_description::Tf& tf) const override {
            auto odom_direction =
                fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, tf);
            if (std::abs(odom_direction->x()) < 1e-6 && std::abs(odom_direction->y()) < 1e-6) {
                return {};
            }

            solver.control_enabled_ = true;
            odom_direction->z() = 0.0;
            auto pitch_direction = fast_tf::cast<PitchLink>(odom_direction, tf);
            pitch_direction->normalize();
            return pitch_direction;
        }
    };

    class SetControlDirection : public Operation {
    public:
        explicit SetControlDirection(OdomImu::DirectionVector target)
            : target_(std::move(target)) {}

    private:
        PitchLink::DirectionVector
            update(TwoAxisGimbalSolver& solver, const rmgo_description::Tf& tf) const override {
            solver.control_enabled_ = true;
            return fast_tf::cast<PitchLink>(target_, tf);
        }

        OdomImu::DirectionVector target_;
    };

    class SetControlShift : public Operation {
    public:
        SetControlShift(double yaw, double pitch)
            : yaw_(yaw)
            , pitch_(pitch) {}

    private:
        PitchLink::DirectionVector
            update(TwoAxisGimbalSolver& solver, const rmgo_description::Tf& tf) const override {
            auto direction = PitchLink::DirectionVector{Eigen::Vector3d::UnitX()};
            if (!solver.control_enabled_) {
                solver.control_enabled_ = true;
            } else {
                direction = fast_tf::cast<PitchLink>(solver.control_direction_, tf);
            }

            const Eigen::AngleAxisd yaw_transform{yaw_, Eigen::Vector3d::UnitZ()};
            const Eigen::AngleAxisd pitch_transform{pitch_, Eigen::Vector3d::UnitY()};
            return PitchLink::DirectionVector{pitch_transform * (yaw_transform * (*direction))};
        }

        double yaw_ = 0.0;
        double pitch_ = 0.0;
    };

    AngleError
        update(const rmgo_description::Tf& tf, double encoder_pitch, const Operation& operation) {
        update_yaw_axis(tf);

        PitchLink::DirectionVector control_direction = operation.update(*this, tf);
        if (!control_enabled_) {
            return {};
        }

        auto [direction_yaw_link, pitch_cs] =
            pitch_link_to_yaw_link(control_direction, encoder_pitch, tf);
        clamp_control_direction(direction_yaw_link);
        if (!control_enabled_) {
            return {};
        }

        control_direction_ =
            fast_tf::cast<OdomImu>(yaw_link_to_pitch_link(direction_yaw_link, pitch_cs), tf);
        return calculate_control_errors(direction_yaw_link, pitch_cs);
    }

    bool enabled() const { return control_enabled_; }

private:
    void update_yaw_axis(const rmgo_description::Tf& tf) {
        auto yaw_axis =
            fast_tf::cast<PitchLink>(YawLink::DirectionVector{Eigen::Vector3d::UnitZ()}, tf);
        *yaw_axis_filtered_ += 0.1 * (*fast_tf::cast<OdomImu>(yaw_axis, tf));
        yaw_axis_filtered_->normalize();
    }

    std::pair<YawLink::DirectionVector, Eigen::Vector2d> pitch_link_to_yaw_link(
        const PitchLink::DirectionVector& direction, double encoder_pitch,
        const rmgo_description::Tf& tf) const {
        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, tf);
        auto pitch_cs = Eigen::Vector2d{yaw_axis->z(), yaw_axis->x()};
        if (pitch_cs.norm() <= 1e-6) {
            pitch_cs = Eigen::Vector2d{std::cos(encoder_pitch), -std::sin(encoder_pitch)};
        } else {
            pitch_cs.normalize();
        }

        const auto& x = direction->x();
        const auto& y = direction->y();
        const auto& z = direction->z();
        return {
            YawLink::DirectionVector{Eigen::Vector3d{
                x * pitch_cs.x() - z * pitch_cs.y(), y, x * pitch_cs.y() + z * pitch_cs.x()}},
            pitch_cs,
        };
    }

    static PitchLink::DirectionVector yaw_link_to_pitch_link(
        const YawLink::DirectionVector& direction, const Eigen::Vector2d& pitch) {
        const auto& x = direction->x();
        const auto& y = direction->y();
        const auto& z = direction->z();
        const auto& c = pitch.x();
        const auto& s = pitch.y();
        return PitchLink::DirectionVector{Eigen::Vector3d{x * c + z * s, y, -x * s + z * c}};
    }

    void clamp_control_direction(YawLink::DirectionVector& direction) {
        const auto& x = direction->x();
        const auto& y = direction->y();
        const auto& z = direction->z();

        Eigen::Vector2d projection{x, y};
        const double norm = projection.norm();
        if (norm <= 0.0) {
            control_enabled_ = false;
            return;
        }
        projection /= norm;

        if (z > lower_limit_.y()) {
            *direction << lower_limit_.x() * projection, lower_limit_.y();
        } else if (z < upper_limit_.y()) {
            *direction << upper_limit_.x() * projection, upper_limit_.y();
        }
    }

    static AngleError calculate_control_errors(
        const YawLink::DirectionVector& direction, const Eigen::Vector2d& pitch) {
        const auto& x = direction->x();
        const auto& y = direction->y();
        const auto& z = direction->z();
        const auto& c = pitch.x();
        const auto& s = pitch.y();

        const double x_projected = std::sqrt(x * x + y * y);
        return {
            std::atan2(y, x),
            -std::atan2(z * c - x_projected * s, z * s + x_projected * c),
        };
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    Eigen::Vector2d lower_limit_;
    Eigen::Vector2d upper_limit_;
    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};
    bool control_enabled_ = false;
    OdomImu::DirectionVector control_direction_{Eigen::Vector3d::UnitX()};
};

} // namespace rmgo_core::gimbal
