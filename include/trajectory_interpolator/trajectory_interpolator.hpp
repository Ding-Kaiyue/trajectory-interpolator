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
    bool loadTrajectory(const trajectory_interpolator::Trajectory& trajectory);

    // 加载轨迹并直接指定动力学参数
    bool loadTrajectoryWithDynamicConfig(const trajectory_interpolator::Trajectory& trajectory,
                                        double max_velocity,
                                        double max_acceleration,
                                        double max_jerk);

#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
    // 加载轨迹
    bool loadTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool loadTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);  

    // 转换为 ROS2 轨迹消息
    trajectory_msgs::msg::JointTrajectory toRosTrajectory(double target_dt) const;
#endif

    // 插值操作
    trajectory_interpolator::Trajectory interpolate() const;
    std::vector<double> interpolateAtTime(double time) const;
    std::vector<double> getVelocityAtTime(double time) const;
    std::vector<double> getAccelerationAtTime(double time) const;
    
    // 实时插值功能
    trajectory_interpolator::TrajectoryPoint getTrajectoryPointAtTime(double time) const;
    bool isFinished(double current_time) const;
    double getTotalDuration() const;

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
