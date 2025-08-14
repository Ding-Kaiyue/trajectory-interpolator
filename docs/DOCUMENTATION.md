# 文档

本文档提供轨迹插值库的完整API参考和使用说明。

## 核心类

### TrajectoryInterpolator

主要的轨迹插值器类，提供高级接口。

```cpp
class TrajectoryInterpolator {
public:
    // 构造函数
    TrajectoryInterpolator();
    ~TrajectoryInterpolator();

    // 配置方法
    void setInterpolationConfig(const SplineConfig& config);
    SplineConfig getConfig() const;

    // 轨迹加载
    bool loadTrajectory(const Trajectory& trajectory);
    #ifdef USE_ROS2_MESSAGES
    bool loadTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool loadTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    #endif

    // 插值方法
    std::vector<double> interpolateAtTime(double time);
    std::vector<double> getVelocityAtTime(double time);
    std::vector<double> getAccelerationAtTime(double time);
    Trajectory interpolate(double dt);

    // 转换方法
    #ifdef USE_ROS2_MESSAGES
    trajectory_msgs::msg::JointTrajectory toRosTrajectory();
    #endif

    // 约束检查
    bool checkConstraints() const;

    // 状态查询
    bool isLoaded() const;
    double getStartTime() const;
    double getEndTime() const;
    std::vector<std::string> getJointNames() const;

    // 工具方法
    void clear();
    MoveItSplineAdapter* getAdapter();
};
```

### MoveItSplineAdapter

底层适配器类，直接与样条库交互。

```cpp
class MoveItSplineAdapter {
public:
    // 构造函数
    MoveItSplineAdapter();
    ~MoveItSplineAdapter();

    // 轨迹创建
    bool createSplineFromTrajectory(const Trajectory& trajectory);
    #ifdef USE_ROS2_MESSAGES
    bool createSplineFromTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool createSplineFromTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    #endif

    // 插值方法
    std::vector<double> interpolateAtTime(double time);
    std::vector<double> getVelocityAtTime(double time);
    std::vector<double> getAccelerationAtTime(double time);
    Trajectory interpolate(double dt);

    // 转换方法
    #ifdef USE_ROS2_MESSAGES
    trajectory_msgs::msg::JointTrajectory toRosTrajectory();
    #endif

    // 约束检查
    bool checkConstraints() const;

    // 状态查询
    bool isLoaded() const;
    double getStartTime() const;
    double getEndTime() const;
    std::vector<std::string> getJointNames() const;

    // 工具方法
    void clear();
    const std::vector<tk::spline>& getSplines() const;
};
```

## 数据结构

### SplineConfig

插值配置和约束参数。

```cpp
struct SplineConfig {
    // 样条类型
    enum class SplineType {
        LINEAR = 10,            // 线性插值
        CUBIC_SPLINE = 30,      // 三次样条插值 (C^2)
        CUBIC_HERMITE = 31,     // 三次Hermite插值 (C^1)
    };

    // 边界条件
    enum class BoundaryType {
        FIRST_DERIVATIVE = 1,   // 一阶导数
        SECOND_DERIVATIVE = 2,  // 二阶导数
        NOT_A_KNOT = 3,         // not a knot条件
    };
    
    SplineType spline_type = SplineType::CUBIC_SPLINE;
    BoundaryType left_boundary = BoundaryType::SECOND_DERIVATIVE;
    BoundaryType right_boundary = BoundaryType::SECOND_DERIVATIVE;
    double left_value = 0.0;         // 起始速度/加速度
    double right_value = 0.0;        // 终止速度/加速度
    bool make_monotonic = false;     // 是否强制单调性
    double target_dt = 0.02;         // 目标插值间隔

    // 机械臂约束
    double max_velocity = M_PI / 2.0;       // 最大速度(rad/s)
    double max_acceleration = M_PI;         // 最大加速度(rad/s^2)
    double max_jerk = M_PI;                 // 最大加加速度(rad/s^3)

    // 关节名称
    std::vector<std::string> joint_names;
};
```

### Trajectory

内部轨迹数据结构。

```cpp
struct Trajectory {
    std::vector<std::string> joint_names;           // 关节名称
    std::vector<TrajectoryPoint> points;            // 轨迹点
};
```

### TrajectoryPoint

轨迹点数据结构。

```cpp
struct TrajectoryPoint {
    double time_from_start;              // 从开始的时间
    std::vector<double> positions;       // 关节位置
    std::vector<double> velocities;      // 关节速度
    std::vector<double> accelerations;   // 关节加速度
};
```

## 使用示例

### 基本使用

```cpp
#include "trajectory_interpolator/trajectory_interpolator.hpp"

int main() {
    // 创建插值器
    auto interpolator = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
    
    // 配置参数
    trajectory_interpolator::SplineConfig config;
    config.dt = 0.02;
    config.spline_type = SplineConfig::CSPLINE;
    config.max_velocity = 1.0;
    config.max_acceleration = 2.0;
    interpolator->setInterpolationConfig(config);
    
    // 创建轨迹
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
        // 在指定时间点插值
        double time = 1.5;
        auto positions = interpolator->interpolateAtTime(time);
        auto velocities = interpolator->getVelocityAtTime(time);
        auto accelerations = interpolator->getAccelerationAtTime(time);
        
        // 插值完整轨迹
        auto interpolated = interpolator->interpolate();
        
        // 检查约束
        if (interpolator->checkConstraints()) {
            std::cout << "轨迹满足约束条件" << std::endl;
        }
    }
    
    return 0;
}
```

### MoveIt集成

```cpp
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>

class MyPlanningNode : public rclcpp::Node {
private:
    std::unique_ptr<trajectory_interpolator::TrajectoryInterpolator> interpolator_;
    
public:
    MyPlanningNode() : Node("my_planning_node") {
        interpolator_ = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
        
        trajectory_interpolator::SplineConfig config;
        config.dt = 0.02;
        config.spline_type = SplineConfig::CSPLINE;
        interpolator_->setInterpolationConfig(config);
    }
    
    void processTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) {
        if (interpolator_->loadTrajectory(trajectory)) {
            // 实时插值
            double current_time = this->now().seconds();
            auto positions = interpolator_->interpolateAtTime(current_time);
            
            // 转换为ROS2轨迹格式
            auto ros_trajectory = interpolator_->toRosTrajectory();
        }
    }
};
```

## 错误处理

### 常见错误

1. **轨迹加载失败**
   ```cpp
   if (!interpolator->loadTrajectory(trajectory)) {
       std::cerr << "轨迹加载失败：轨迹点数量不足或时间顺序错误" << std::endl;
       return;
   }
   ```

2. **时间超出范围**
   ```cpp
   if (!interpolator->isTimeValid(time)) {
       std::cerr << "时间超出轨迹范围" << std::endl;
       return;
   }
   ```

3. **约束检查失败**
   ```cpp
   if (!interpolator->checkConstraints()) {
       std::cerr << "轨迹违反约束条件，需要重新规划" << std::endl;
       return;
   }
   ```

## 性能优化

### 编译优化

```bash
# Release模式编译
cmake .. -DCMAKE_BUILD_TYPE=Release

# 启用优化
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3"
```

### 运行时优化

1. **合理设置时间步长**
   ```cpp
   config.dt = 0.01;  // 高精度
   config.dt = 0.05;  // 高性能
   ```

2. **预计算插值结果**
   ```cpp
   auto interpolated = interpolator->interpolate(0.01);
   // 从预计算的结果中查找，而不是实时插值
   ```

3. **使用智能指针管理内存**
   ```cpp
   std::unique_ptr<TrajectoryInterpolator> interpolator;
   ```

## 条件编译

库支持条件编译，可以选择是否包含ROS2消息支持：

```cpp
#ifdef USE_ROS2_MESSAGES
// ROS2消息相关代码
bool loadTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
trajectory_msgs::msg::JointTrajectory toRosTrajectory();
#endif
```

编译时控制：
```bash
# 启用ROS2支持
cmake .. -DUSE_ROS2_MESSAGES=ON

# 禁用ROS2支持
cmake .. -DUSE_ROS2_MESSAGES=OFF
```

## 许可证

本项目采用 MIT 许可证，但包含来自 [ttk592/spline](https://github.com/ttk592/spline) 的 GPL-2.0 组件。

---

**最后更新**: 2024-08-05 