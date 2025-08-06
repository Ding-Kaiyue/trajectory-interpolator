#ifndef TRAJECTORY_INTERPOLATOR_MOVEIT_SPLINE_ADAPTER_HPP
#define TRAJECTORY_INTERPOLATOR_MOVEIT_SPLINE_ADAPTER_HPP

#include "spline.h"
#include "config.hpp"
#include <string>
#include <memory>
#include <vector>

// 条件编译ROS2消息类型
#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#endif

namespace trajectory_interpolator 
{

// 简化的轨迹点结构（不依赖ROS2消息）
struct TrajectoryPoint {
    double time_from_start;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
};

// 简化的轨迹结构（不依赖ROS2消息）
struct Trajectory {
    std::vector<std::string> joint_names;
    std::vector<TrajectoryPoint> points;
};

class MoveItSplineAdapter {
public:
    using SplinePtr = std::unique_ptr<tk::spline>;

public:
    MoveItSplineAdapter() = default;
    ~MoveItSplineAdapter() = default;

    // 从简化轨迹创建spline
    bool createSplineFromTrajectory(const Trajectory& trajectory,
                                   const SplineConfig& config = SplineConfig{});
#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
    // 从 MoveIt 轨迹消息创建 spline
    bool createSplineFromTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory, 
                                    const SplineConfig& config = SplineConfig{});

    // 从 ROS2 轨迹消息创建 spline
    bool createSplineFromTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory, 
                                    const SplineConfig& config = SplineConfig{});
    
    // 转换为 ROS2 轨迹消息
    trajectory_msgs::msg::JointTrajectory toRosTrajectory(double target_dt) const;
#endif
    // 插值到指定时间点
    std::vector<double> interpolateAtTime(double time) const;

    // 插值整个轨迹
    Trajectory interpolate(double target_dt) const;

    // 获取指定时间的速度和加速度
    std::vector<double> getVelocityAtTime(double time) const;
    std::vector<double> getAccelerationAtTime(double time) const;

    // 检查约束条件
    bool checkConstraints(const SplineConfig& config) const;

    // 获取spline实例
    const std::vector<SplinePtr>& getSplines() const { return joint_splines_; }
    std::vector<SplinePtr>& getSplines() { return joint_splines_; }

    // 检查是否已加载轨迹
    bool isLoaded() const { return !joint_splines_.empty(); }

    // 清空数据
    void clear() { joint_splines_.clear(); }

    // 获取轨迹信息
    double getStartTime() const;
    double getEndTime() const;
    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    
private:
    std::vector<SplinePtr> joint_splines_;  // 每个关节一个spline
    std::vector<std::string> joint_names_;  // 关节名称
    std::vector<double> time_points_;       // 时间序列
    SplineConfig config_;                   // 配置参数
    
    // 辅助函数
    tk::spline::spline_type convertSplineType(SplineConfig::SplineType type);
    tk::spline::bd_type convertBoundaryType(SplineConfig::BoundaryType type);
    bool isTimeValid(const std::vector<SplinePtr>& splines, double time) const;
#if defined(USE_ROS2_MESSAGES) && USE_ROS2_MESSAGES
    // ROS2 辅助函数    
    double timeFromDuration(const builtin_interfaces::msg::Duration& duration) const;
    builtin_interfaces::msg::Duration durationFromTime(double time) const;
#endif
};

} // namespace trajectory_interpolator  

#endif // TRAJECTORY_INTERPOLATOR_MOVEIT_SPLINE_ADAPTER_HPP
