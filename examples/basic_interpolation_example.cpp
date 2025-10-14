#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include <iostream>
#include <vector>

int main() {
    try {
        // 创建轨迹插值器
        TrajectoryInterpolator interpolator;
        
        // 设置配置
        trajectory_interpolator::SplineConfig config;
        config.spline_type = trajectory_interpolator::SplineConfig::SplineType::CUBIC_SPLINE;
        config.target_dt = 0.01;  // 10ms插值间隔
        config.max_velocity = M_PI / 2.0;
        config.max_acceleration = M_PI;
        config.max_jerk = M_PI;
        
        interpolator.setInterpolationConfig(config);
        
        // 创建测试轨迹
        trajectory_interpolator::Trajectory trajectory;
        trajectory.joint_names = {"joint1", "joint2", "joint3"};

        // 添加轨迹点
        std::vector<double> times = {0.0, 1.0, 2.0, 3.0};
        std::vector<std::vector<double>> positions = {
            {0.0, 0.0, 0.0},    // t=0s
            {0.5, 0.3, 0.2},    // t=1s
            {1.0, 0.8, 0.6},    // t=2s
            {1.5, 1.2, 1.0}     // t=3s
        };

        for (size_t i = 0; i < times.size(); ++i) {
            trajectory_interpolator::TrajectoryPoint point;
            point.time_from_start = times[i];
            point.positions = positions[i];
            point.velocities = {0.0, 0.0, 0.0};  // 简化为零速度
            point.accelerations = {0.0, 0.0, 0.0}; // 简化为零加速度
            trajectory.points.push_back(point);
        }
        
        // 加载轨迹
        if (!interpolator.loadTrajectory(trajectory)) {
            std::cerr << "Failed to load trajectory" << std::endl;
            return -1;
        }
        
        std::cout << "Trajectory loaded successfully!" << std::endl;
        std::cout << "Start time: " << interpolator.getStartTime() << "s" << std::endl;
        std::cout << "End time: " << interpolator.getEndTime() << "s" << std::endl;
        
        // 插值轨迹
        auto interpolated_trajectory = interpolator.interpolate();  // 100ms间隔
        
        std::cout << "Interpolated trajectory has " << interpolated_trajectory.points.size() << " points" << std::endl;
        
        // 测试单点插值
        double test_time = 1.5;
        auto joint_positions = interpolator.interpolateAtTime(test_time);
        auto joint_velocities = interpolator.getVelocityAtTime(test_time);
        auto joint_accelerations = interpolator.getAccelerationAtTime(test_time);
        
        std::cout << "At time " << test_time << "s:" << std::endl;
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            std::cout << "  Joint " << i << ": pos=" << joint_positions[i] 
                      << ", vel=" << joint_velocities[i] 
                      << ", acc=" << joint_accelerations[i] << std::endl;
        }
        
        // 检查约束
        if (interpolator.checkConstraints()) {
            std::cout << "Trajectory satisfies constraints" << std::endl;
        } else {
            std::cout << "Trajectory violates constraints" << std::endl;
        }
        
        std::cout << "Example completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}