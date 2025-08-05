#ifndef TRAJECTORY_INTERPOLATOR_CONFIG_HPP
#define TRAJECTORY_INTERPOLATOR_CONFIG_HPP

#include <string>
#include <vector>
#include <cmath>

namespace trajectory_interpolator
{

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

} // namespace trajectory_interpolator

#endif // TRAJECTORY_INTERPOLATOR_CONFIG_HPP