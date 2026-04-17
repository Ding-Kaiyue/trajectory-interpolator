#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace trajectory_interpolator 
{

bool MoveItSplineAdapter::createSplineFromTrajectory(const Trajectory& trajectory,
                                                     const SplineConfig& config) {
    if (trajectory.points.empty()) {
        return false;
    }

    Trajectory working = trajectory;
    if (working.points.size() == 2) {
        const auto& p0 = working.points.front();
        const auto& p1 = working.points.back();
        const size_t nj = p0.positions.size();
        if (nj == 0 || p1.positions.size() != nj) {
            return false;
        }
        const double t0 = p0.time_from_start;
        const double t1 = p1.time_from_start;
        if (!(t1 > t0)) {
            return false;
        }

        TrajectoryPoint mid;
        mid.time_from_start = 0.5 * (t0 + t1);
        mid.positions.resize(nj, 0.0);
        mid.velocities.resize(nj, 0.0);
        mid.accelerations.resize(nj, 0.0);
        const double dt = t1 - t0;
        const bool has_v0 = (p0.velocities.size() == nj);
        const bool has_v1 = (p1.velocities.size() == nj);
        const bool has_a0 = (p0.accelerations.size() == nj);
        const bool has_a1 = (p1.accelerations.size() == nj);
        for (size_t j = 0; j < nj; ++j) {
            mid.positions[j] = 0.5 * (p0.positions[j] + p1.positions[j]);
            mid.velocities[j] = (has_v0 && has_v1)
                                    ? 0.5 * (p0.velocities[j] + p1.velocities[j])
                                    : (p1.positions[j] - p0.positions[j]) / std::max(1e-9, dt);
            mid.accelerations[j] = (has_a0 && has_a1)
                                       ? 0.5 * (p0.accelerations[j] + p1.accelerations[j])
                                       : 0.0;
        }
        working.points.insert(working.points.begin() + 1, std::move(mid));
    }
    if (working.points.size() < 3) {
        return false;
    }

    config_ = config;
    joint_names_ = working.joint_names;
    joint_splines_.clear();
    time_points_.clear();

    // 提取时间序列
    time_points_.reserve(working.points.size());
    for (size_t i = 0; i < working.points.size(); ++i) {
        const auto& point = working.points[i];
        time_points_.push_back(point.time_from_start);
        if (i > 0 && !(time_points_[i] > time_points_[i - 1])) {
            return false;
        }
    }

    // 为每个关节创建spline
    size_t num_joints = working.points[0].positions.size();
    if (num_joints == 0) {
        return false;
    }
    joint_splines_.reserve(num_joints);

    for (const auto& point : working.points) {
        if (point.positions.size() != num_joints) {
            return false;
        }
        if (!point.velocities.empty() && point.velocities.size() != num_joints) {
            return false;
        }
        if (!point.accelerations.empty() && point.accelerations.size() != num_joints) {
            return false;
        }
    }

    for (size_t joint_idx = 0; joint_idx < num_joints; ++joint_idx) {
        std::vector<double> positions, velocities, accelerations;
        positions.reserve(working.points.size());
        velocities.reserve(working.points.size());
        accelerations.reserve(working.points.size());

        for (const auto& point : working.points) {
            positions.push_back(point.positions[joint_idx]);
            velocities.push_back(
                point.velocities.size() == num_joints ? point.velocities[joint_idx] : 0.0);
            accelerations.push_back(
                point.accelerations.size() == num_joints ? point.accelerations[joint_idx] : 0.0);
        }

        auto spline = std::make_unique<tk::spline>();

        // 根据配置决定边界条件类型和值
        // 对于轨迹插值，优先使用轨迹提供的端点速度/加速度
        tk::spline::bd_type left_bd_type = convertBoundaryType(config_.left_boundary);
        tk::spline::bd_type right_bd_type = convertBoundaryType(config_.right_boundary);

        double left_bd_value = (config_.left_boundary == SplineConfig::BoundaryType::FIRST_DERIVATIVE)
                               ? velocities.front()
                               : accelerations.front();
        double right_bd_value = (config_.right_boundary == SplineConfig::BoundaryType::FIRST_DERIVATIVE)
                                ? velocities.back()
                                : accelerations.back();

        spline->set_boundary(left_bd_type, left_bd_value,
                            right_bd_type, right_bd_value);

        spline->set_points(time_points_, positions, convertSplineType(config_.spline_type));

        joint_splines_.push_back(std::move(spline));
    }

    return true;
}

#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
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
    double duration = end_time - start_time;

    if (target_dt <= 0.0) {
        throw std::runtime_error("Target dt must be positive");
    }

    try {
        // 计算需要的点数
        int num_intervals = static_cast<int>(std::round(duration / target_dt));
        int num_points = num_intervals + 1;  // 包括起点和终点

        for (int i = 0; i < num_points; ++i) {
            double t;
            if (i == num_points - 1) {
                // 最后一个点确保是终点
                t = end_time;
            } else {
                // 中间的点均匀分布
                t = start_time + i * target_dt;
            }

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
