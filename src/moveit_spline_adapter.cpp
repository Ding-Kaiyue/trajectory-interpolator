#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include <algorithm>
#include <stdexcept>

namespace trajectory_interpolator 
{
bool MoveItSplineAdapter::createSplineFromTrajectory(
    const Trajectory& trajectory, 
    const SplineConfig& config) {
    try {
        // 检查轨迹是否为空
        if (trajectory.points.empty() || trajectory.joint_names.empty()) {
            return false;
        }

        config_ = config;
        joint_names_ = trajectory.joint_names;

        // 提取时间序列
        time_points_.clear();
        time_points_.reserve(trajectory.points.size());
        for (const auto& point : trajectory.points) {
            if (point.time_from_start < 0.0) {
                throw std::runtime_error("Time from start must be non-negative");
            }
            time_points_.push_back(point.time_from_start);
        }

        // 验证时间序列单调性
        for (size_t i = 1; i < time_points_.size(); ++i) {
            if (time_points_[i] <= time_points_[i-1]) {
                throw std::runtime_error("Time points must be monotonically increasing");
            }
        }

        // 创建spline
        joint_splines_.clear();
        joint_splines_.reserve(joint_names_.size());

        for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx) {
            std::vector<double> positions;
            positions.reserve(trajectory.points.size());

            for (const auto& point : trajectory.points) {
                if (joint_idx < point.positions.size()) {
                    positions.push_back(point.positions[joint_idx]);
                } else {
                    return false;
                }
            }

            // 创建spline
            auto spline = std::make_unique<tk::spline>(
                time_points_, 
                positions,
                convertSplineType(config_.spline_type),
                config_.make_monotonic,
                convertBoundaryType(config_.left_boundary),
                config_.left_value,
                convertBoundaryType(config_.right_boundary),
                config_.right_value
            );

            joint_splines_.push_back(std::move(spline));
        }
        return true;
    } catch (const std::exception& e) {
        joint_splines_.clear();
        time_points_.clear();
        joint_names_.clear();
        return false;
    }
}

#ifdef USE_ROS2_MESSAGES
bool MoveItSplineAdapter::createSplineFromTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const SplineConfig& config) {
    
    // 转换为内部轨迹格式
    Trajectory internal_trajectory;
    internal_trajectory.joint_names = trajectory.joint_names;
    
    for (const auto& point : trajectory.points) {
        TrajectoryPoint internal_point;
        internal_point.time_from_start = timeFromDuration(point.time_from_start);
        internal_point.positions = point.positions;
        internal_point.velocities = point.velocities;
        internal_point.accelerations = point.accelerations;
        internal_trajectory.points.push_back(internal_point);
    }
    
    return createSplineFromTrajectory(internal_trajectory, config);
}

bool MoveItSplineAdapter::createSplineFromTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const SplineConfig& config) {
    
    return createSplineFromTrajectory(trajectory.joint_trajectory, config);
}

trajectory_msgs::msg::JointTrajectory MoveItSplineAdapter::toRosTrajectory(double target_dt) const {
    trajectory_msgs::msg::JointTrajectory result;
    result.joint_names = joint_names_;

    if (!isLoaded()) {
        return result;
    }

    double start_time = getStartTime();
    double end_time = getEndTime();

    for (double t = start_time; t <= end_time; t += target_dt) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = durationFromTime(t);
        point.positions = interpolateAtTime(t);
        point.velocities = getVelocityAtTime(t);
        point.accelerations = getAccelerationAtTime(t);
        result.points.push_back(point);
    }

    return result;
}

double MoveItSplineAdapter::timeFromDuration(const builtin_interfaces::msg::Duration& duration) const {
    return duration.sec + duration.nanosec * 1e-9;
}

builtin_interfaces::msg::Duration MoveItSplineAdapter::durationFromTime(double time) const {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(time);
    duration.nanosec = static_cast<uint32_t>((time - static_cast<int32_t>(time)) * 1e9);
    return duration;
}
#endif

std::vector<double> MoveItSplineAdapter::interpolateAtTime(double time) const {
    if (!isLoaded()) {
        throw std::runtime_error("Spline not loaded");
    }

    if (!isTimeValid(joint_splines_, time)) {
        throw std::runtime_error("Time " + std::to_string(time) + " is outside trajectory bounds ["
                                + std::to_string(getStartTime()) + ", "
                                + std::to_string(getEndTime()) + "]");
    }

    std::vector<double> positions;
    positions.reserve(joint_splines_.size());
    try {
        for (const auto& spline : joint_splines_) {
            positions.push_back((*spline)(time));
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to interpolate at time " + std::to_string(time) + ": " + e.what());
    }
    return positions;
}

std::vector<double> MoveItSplineAdapter::getVelocityAtTime(double time) const {
    if (!isLoaded()) {
        throw std::runtime_error("Spline not loaded");
    }

    if (!isTimeValid(joint_splines_, time)) {
        throw std::runtime_error("Time " + std::to_string(time) + " is outside trajectory bounds ["
                                + std::to_string(getStartTime()) + ", "
                                + std::to_string(getEndTime()) + "]");
    }

    std::vector<double> velocities;
    velocities.reserve(joint_splines_.size());

    try {
        for (const auto& spline : joint_splines_) {
            velocities.push_back(spline->deriv(1, time));
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to get velocity at time " + std::to_string(time) + ": " + e.what());
    }
    return velocities;
}

std::vector<double> MoveItSplineAdapter::getAccelerationAtTime(double time) const {
    if (!isLoaded()) {
        throw std::runtime_error("Spline not loaded");
    }

    if (!isTimeValid(joint_splines_, time)) {
        throw std::runtime_error("Time " + std::to_string(time) + " is outside trajectory bounds ["
                                + std::to_string(getStartTime()) + ", "
                                + std::to_string(getEndTime()) + "]");
    }

    std::vector<double> accelerations;
    accelerations.reserve(joint_splines_.size());

    try {
        for (const auto& spline : joint_splines_) {
            accelerations.push_back(spline->deriv(2, time));
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to get acceleration at time " + std::to_string(time) + ": " + e.what());
    }
    return accelerations;
}

Trajectory MoveItSplineAdapter::interpolate(double target_dt) const {
    Trajectory result;
    result.joint_names = joint_names_;

    if (!isLoaded()) {
        throw std::runtime_error("Spline not loaded");
    }

    double start_time = getStartTime();
    double end_time = getEndTime();

    if (target_dt <= 0.0) {
        throw std::runtime_error("Target dt must be positive");
    }

    try {
        for (double t = start_time; t <= end_time; t += target_dt) {
            TrajectoryPoint point;
            point.time_from_start = t;

            // position
            auto positions = interpolateAtTime(t);
            point.positions = positions;

            // velocity
            auto velocities = getVelocityAtTime(t);
            point.velocities = velocities;

            // acceleration
            auto accelerations = getAccelerationAtTime(t);
            point.accelerations = accelerations;
            
            result.points.push_back(point);
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to interpolate trajectory: " + std::string(e.what()));
    }
    return result;
}

tk::spline::spline_type MoveItSplineAdapter::convertSplineType(SplineConfig::SplineType type) {
    switch (type) {
        case SplineConfig::SplineType::LINEAR:
            return tk::spline::linear;
        case SplineConfig::SplineType::CUBIC_SPLINE:
            return tk::spline::cspline;
        case SplineConfig::SplineType::CUBIC_HERMITE:
            return tk::spline::cspline_hermite;
        default:
            return tk::spline::cspline;
    }
}

tk::spline::bd_type MoveItSplineAdapter::convertBoundaryType(SplineConfig::BoundaryType type) {
    switch (type) {
        case SplineConfig::BoundaryType::FIRST_DERIVATIVE:
            return tk::spline::first_deriv;
        case SplineConfig::BoundaryType::SECOND_DERIVATIVE:
            return tk::spline::second_deriv;
        case SplineConfig::BoundaryType::NOT_A_KNOT:
            return tk::spline::not_a_knot;
        default:
            return tk::spline::second_deriv;
    }
}

double MoveItSplineAdapter::getStartTime() const {
    if (time_points_.empty()) {
        return 0.0;
    }
    return joint_splines_[0]->get_x_min();
}

double MoveItSplineAdapter::getEndTime() const {
    if (time_points_.empty()) {
        return 0.0;
    }
    return joint_splines_[0]->get_x_max();
}

bool MoveItSplineAdapter::isTimeValid(const std::vector<SplinePtr>& splines, double time) const {
    if (splines.empty()) {
        return false;
    }
    double start_t = splines[0]->get_x_min();
    double end_t = splines[0]->get_x_max();

    return time >= start_t && time <= end_t;
}

bool MoveItSplineAdapter::checkConstraints(const SplineConfig& config) const {
    if (!isLoaded()) {
        return false;
    }

    // 简单约束检查：只检查关键时间点
    double start_time = getStartTime();
    double end_time = getEndTime();
    
    // 检查起始、中间、结束时间点
    std::vector<double> check_times = {start_time, (start_time + end_time) / 2.0, end_time};
    
    for (double time : check_times) {
        for (size_t i = 0; i < joint_splines_.size(); ++i) {
            // 检查速度
            double velocity = joint_splines_[i]->deriv(1, time);
            if (std::abs(velocity) > config.max_velocity) {
                return false;
            }
            
            // 检查加速度
            double acceleration = joint_splines_[i]->deriv(2, time);
            if (std::abs(acceleration) > config.max_acceleration) {
                return false;
            }
            
            // 检查加加速度
            double jerk = joint_splines_[i]->deriv(3, time);
            if (std::abs(jerk) > config.max_jerk) {
                return false;
            }
        }
    }
    
    return true;
}


} // namespace trajectory_interpolator
