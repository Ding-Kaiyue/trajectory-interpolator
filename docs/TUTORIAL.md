# 使用教程

本教程将指导你如何使用轨迹插值库进行机器人臂轨迹插值。

## 快速开始

### 1. 基本使用

```cpp
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include <iostream>

int main() {
    // 创建插值器
    auto interpolator = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
    
    // 配置插值参数
    trajectory_interpolator::SplineConfig config;
    config.dt = 0.02;                    // 插值时间步长
    config.spline_type = SplineConfig::CSPLINE;  // 三次样条插值
    config.boundary_type = SplineConfig::NATURAL; // 自然边界条件
    config.max_velocity = 1.0;           // 最大速度约束
    config.max_acceleration = 2.0;       // 最大加速度约束
    config.max_jerk = 5.0;              // 最大加加速度约束
    
    interpolator->setInterpolationConfig(config);
    
    // 创建轨迹数据
    trajectory_interpolator::Trajectory trajectory;
    trajectory.joint_names = {"joint1", "joint2", "joint3"};
    
    // 添加轨迹点
    trajectory_interpolator::TrajectoryPoint point1;
    point1.time_from_start = 0.0;
    point1.positions = {0.0, 0.0, 0.0};
    point1.velocities = {0.0, 0.0, 0.0};
    point1.accelerations = {0.0, 0.0, 0.0};
    
    trajectory_interpolator::TrajectoryPoint point2;
    point2.time_from_start = 3.0;
    point2.positions = {1.0, 1.0, 1.0};
    point2.velocities = {0.0, 0.0, 0.0};
    point2.accelerations = {0.0, 0.0, 0.0};
    
    trajectory.points = {point1, point2};
    
    // 加载轨迹
    if (interpolator->loadTrajectory(trajectory)) {
        std::cout << "轨迹加载成功！" << std::endl;
        
        // 在指定时间点插值
        double time = 1.5;
        auto positions = interpolator->interpolateAtTime(time);
        auto velocities = interpolator->getVelocityAtTime(time);
        auto accelerations = interpolator->getAccelerationAtTime(time);
        
        std::cout << "时间 " << time << "s 的插值结果:" << std::endl;
        for (size_t i = 0; i < positions.size(); ++i) {
            std::cout << "关节 " << i << ": 位置=" << positions[i] 
                      << ", 速度=" << velocities[i] 
                      << ", 加速度=" << accelerations[i] << std::endl;
        }
        
        // 插值完整轨迹
        auto interpolated = interpolator->interpolate(0.01);
        std::cout << "插值轨迹包含 " << interpolated.points.size() << " 个点" << std::endl;
        
        // 检查约束
        if (interpolator->checkConstraints()) {
            std::cout << "轨迹满足约束条件" << std::endl;
        } else {
            std::cout << "轨迹违反约束条件" << std::endl;
        }
        
    } else {
        std::cerr << "轨迹加载失败！" << std::endl;
        return 1;
    }
    
    return 0;
}
```

### 2. MoveIt集成

```cpp
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>

// 在ROS2节点中使用
class MyPlanningNode : public rclcpp::Node {
private:
    std::unique_ptr<trajectory_interpolator::TrajectoryInterpolator> interpolator_;
    
public:
    MyPlanningNode() : Node("my_planning_node") {
        // 初始化插值器
        interpolator_ = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
        
        // 配置插值参数
        trajectory_interpolator::SplineConfig config;
        config.dt = 0.02;
        config.spline_type = SplineConfig::CSPLINE;
        interpolator_->setInterpolationConfig(config);
    }
    
    void processTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) {
        // 加载MoveIt轨迹
        if (interpolator_->loadTrajectory(trajectory)) {
            RCLCPP_INFO(this->get_logger(), "轨迹加载成功");
            
            // 实时插值
            double current_time = this->now().seconds();
            auto positions = interpolator_->interpolateAtTime(current_time);
            
            // 使用插值结果进行机器人控制
            // ...
        }
    }
};
```

### 3. 高级功能

#### 约束检查
```cpp
// 设置约束参数
trajectory_interpolator::SplineConfig config;
config.max_velocity = 1.0;        // 最大速度 (rad/s)
config.max_acceleration = 2.0;     // 最大加速度 (rad/s²)
config.max_jerk = 5.0;            // 最大加加速度 (rad/s³)

interpolator->setInterpolationConfig(config);

// 检查轨迹是否满足约束
if (interpolator->checkConstraints()) {
    std::cout << "轨迹满足约束条件" << std::endl;
} else {
    std::cout << "轨迹违反约束条件，需要重新规划" << std::endl;
}
```

#### 批量插值
```cpp
// 插值完整轨迹
auto interpolated = interpolator->interpolate(0.01);  // 0.01s时间步长

// 遍历插值结果
for (const auto& point : interpolated.points) {
    std::cout << "时间: " << point.time_from_start << "s" << std::endl;
    for (size_t i = 0; i < point.positions.size(); ++i) {
        std::cout << "  关节" << i << ": " << point.positions[i] << std::endl;
    }
}
```

## 最佳实践

1. **时间步长选择**: 通常选择0.01-0.02秒，平衡精度和性能
2. **约束设置**: 根据机器人硬件能力设置合理的约束参数
3. **错误处理**: 始终检查 `loadTrajectory` 的返回值
4. **内存管理**: 使用智能指针管理插值器生命周期

## 常见问题

**Q: 如何处理空轨迹？**
A: 检查轨迹点数量，确保至少有两个轨迹点。

**Q: 插值时间超出范围怎么办？**
A: 使用 `isTimeValid(time)` 检查时间有效性。

**Q: 如何提高插值精度？**
A: 减小时间步长，但会增加计算开销。

## 编译示例

```bash
# 使用g++编译
g++ -std=c++17 -ltrajectory_interpolator_core -lpthread your_program.cpp -o your_program

# 使用CMake
cmake_minimum_required(VERSION 3.8)
project(my_trajectory_project)
find_package(trajectory_interpolator REQUIRED)
add_executable(my_program main.cpp)
target_link_libraries(my_program trajectory_interpolator_core)
``` 