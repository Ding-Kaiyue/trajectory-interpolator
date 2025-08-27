#ifndef TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_HPP
#define TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_HPP

#include "moveit_spline_adapter.hpp"
#include "config.hpp"

#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#endif

class TrajectoryInterpolator {
public:
    TrajectoryInterpolator() = default;
    ~TrajectoryInterpolator() = default;

    // 设置插值配置
    void setInterpolationConfig(const trajectory_interpolator::SplineConfig& config);

    // 加载轨迹（简化版本）
    bool loadTrajectory(const Trajectory& trajectory);

#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
    // 加载轨迹
    bool loadTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool loadTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);  

    // 转换为 ROS2 轨迹消息
    trajectory_msgs::msg::JointTrajectory toRosTrajectory(double target_dt) const;
#endif

    // 插值操作
    Trajectory interpolate() const;
    std::vector<double> interpolateAtTime(double time) const;
    std::vector<double> getVelocityAtTime(double time) const;
    std::vector<double> getAccelerationAtTime(double time) const;

    // 检查约束条件
    bool checkConstraints() const;

    // 获取配置和信息
    const trajectory_interpolator::SplineConfig& getConfig() const;
    bool isLoaded() const;
    double getStartTime() const;
    double getEndTime() const;

    // 获取适配器
    const trajectory_interpolator::MoveItSplineAdapter& getAdapter() const;
    trajectory_interpolator::MoveItSplineAdapter& getAdapter();

private:
    trajectory_interpolator::MoveItSplineAdapter adapter_;
    trajectory_interpolator::SplineConfig config_;
};

#endif // TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_HPP
