#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include <stdexcept>
#include <algorithm>

void TrajectoryInterpolator::setInterpolationConfig(const trajectory_interpolator::SplineConfig& config) {
    config_ = config;
}

bool TrajectoryInterpolator::loadTrajectory(const Trajectory& trajectory) {
    try {
        return adapter_.createSplineFromTrajectory(trajectory, config_);
    } catch (const std::exception& e) {
        // 可以在这里添加日志记录
        return false;
    }
}

Trajectory TrajectoryInterpolator::interpolate() const {
    if (!isLoaded()) {
        throw std::runtime_error("No trajectory loaded");
    }

    if (config_.target_dt <= 0.0) {
        throw std::runtime_error("Target dt must be positive");
    }

    Trajectory result;
    result.joint_names = adapter_.getJointNames();

    double start_time = adapter_.getStartTime();
    double end_time = adapter_.getEndTime();

    int num_points = static_cast<int>((end_time - start_time) / config_.target_dt) + 1;
    for (int i = 0; i < num_points; ++i) {
        double t = start_time + i * config_.target_dt;
        if (t > end_time) break;
        
        TrajectoryPoint point;
        point.time_from_start = t;
        point.positions = adapter_.interpolateAtTime(t);
        point.velocities = adapter_.getVelocityAtTime(t);
        point.accelerations = adapter_.getAccelerationAtTime(t);
        result.points.push_back(point);
    }

    return result;
}

std::vector<double> TrajectoryInterpolator::interpolateAtTime(double time) const {
    if (!isLoaded()) {
        throw std::runtime_error("No trajectory loaded");
    }
    return adapter_.interpolateAtTime(time);
}

std::vector<double> TrajectoryInterpolator::getVelocityAtTime(double time) const {
    if (!isLoaded()) {
        throw std::runtime_error("No trajectory loaded");
    }
    return adapter_.getVelocityAtTime(time);
}

std::vector<double> TrajectoryInterpolator::getAccelerationAtTime(double time) const {
    if (!isLoaded()) {
        throw std::runtime_error("No trajectory loaded");
    }
    return adapter_.getAccelerationAtTime(time);
}

bool TrajectoryInterpolator::checkConstraints() const {
    if (!isLoaded()) {
        return false;
    }
    return adapter_.checkConstraints(config_);
}

const trajectory_interpolator::SplineConfig& TrajectoryInterpolator::getConfig() const {
    return config_;
}

bool TrajectoryInterpolator::isLoaded() const {
    return adapter_.isLoaded();
}

double TrajectoryInterpolator::getStartTime() const {
    return adapter_.getStartTime();
}

double TrajectoryInterpolator::getEndTime() const {
    return adapter_.getEndTime();
}

const trajectory_interpolator::MoveItSplineAdapter& TrajectoryInterpolator::getAdapter() const {
    return adapter_;
}

trajectory_interpolator::MoveItSplineAdapter& TrajectoryInterpolator::getAdapter() {
    return adapter_;
}