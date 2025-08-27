#include <gtest/gtest.h>
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include <vector>
#include <cmath>

class MoveItCompatibilityTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.spline_type = trajectory_interpolator::SplineConfig::SplineType::CUBIC_SPLINE;
        config_.target_dt = 0.02;
        config_.make_monotonic = false;
        config_.max_velocity = M_PI / 2.0;
        config_.max_acceleration = M_PI;
        config_.max_jerk = M_PI;
        
        interpolator_.setInterpolationConfig(config_);
    }

    Trajectory createMoveItStyleTrajectory() {
        Trajectory trajectory;
        trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        
        // 模拟MoveIt规划的轨迹点
        std::vector<double> times = {0.0, 0.5, 1.0, 1.5, 2.0};
        std::vector<std::vector<double>> positions = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.2, 0.1, 0.15, 0.05, 0.1, 0.2},
            {0.5, 0.3, 0.4, 0.2, 0.3, 0.5},
            {0.8, 0.6, 0.7, 0.4, 0.6, 0.8},
            {1.0, 0.8, 0.9, 0.6, 0.8, 1.0}
        };
        
        for (size_t i = 0; i < times.size(); ++i) {
            TrajectoryPoint point;
            point.time_from_start = times[i];
            point.positions = positions[i];
            point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            trajectory.points.push_back(point);
        }
        
        return trajectory;
    }

    TrajectoryInterpolator interpolator_;
    trajectory_interpolator::SplineConfig config_;
};

TEST_F(MoveItCompatibilityTest, LoadMoveItTrajectory) {
    auto trajectory = createMoveItStyleTrajectory();
    EXPECT_TRUE(interpolator_.loadTrajectory(trajectory));
    EXPECT_TRUE(interpolator_.isLoaded());
    EXPECT_EQ(interpolator_.getStartTime(), 0.0);
    EXPECT_EQ(interpolator_.getEndTime(), 2.0);
    EXPECT_EQ(interpolator_.getAdapter().getJointNames().size(), 6);
}

TEST_F(MoveItCompatibilityTest, RealTimeInterpolation) {
    auto trajectory = createMoveItStyleTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    // 模拟实时插值
    for (double t = 0.0; t <= 2.0; t += 0.1) {
        auto positions = interpolator_.interpolateAtTime(t);
        auto velocities = interpolator_.getVelocityAtTime(t);
        auto accelerations = interpolator_.getAccelerationAtTime(t);
        
        EXPECT_EQ(positions.size(), 6);
        EXPECT_EQ(velocities.size(), 6);
        EXPECT_EQ(accelerations.size(), 6);
        
        // 检查位置在合理范围内
        for (size_t i = 0; i < positions.size(); ++i) {
            EXPECT_GE(positions[i], 0.0);
            EXPECT_LE(positions[i], 1.0);
        }
    }
}

TEST_F(MoveItCompatibilityTest, ConstraintChecking) {
    auto trajectory = createMoveItStyleTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    // 设置合理的约束
    config_.max_velocity = 2.0;
    config_.max_acceleration = 5.0;
    config_.max_jerk = 10.0;
    interpolator_.setInterpolationConfig(config_);
    
    EXPECT_TRUE(interpolator_.checkConstraints());
}

TEST_F(MoveItCompatibilityTest, HighFrequencyInterpolation) {
    auto trajectory = createMoveItStyleTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    // 模拟高频率插值（如1000Hz控制循环）
    const int num_samples = 2000;  // 2秒 * 1000Hz
    for (int i = 0; i < num_samples; ++i) {
        double time = (i * 2.0) / num_samples;
        auto positions = interpolator_.interpolateAtTime(time);
        EXPECT_EQ(positions.size(), 6);
    }
}

TEST_F(MoveItCompatibilityTest, TrajectoryInterpolation) {
    auto trajectory = createMoveItStyleTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    // 插值整个轨迹
    auto interpolated_trajectory = interpolator_.interpolate();  // 10ms间隔
    
    EXPECT_EQ(interpolated_trajectory.joint_names.size(), 6);
    EXPECT_GT(interpolated_trajectory.points.size(), 100);  // 应该有200个点
    
    // 检查时间序列
    for (size_t i = 0; i < interpolated_trajectory.points.size(); ++i) {
        double expected_time = i * config_.target_dt;
        EXPECT_NEAR(interpolated_trajectory.points[i].time_from_start, expected_time, 1e-6);
    }
}

TEST_F(MoveItCompatibilityTest, ErrorHandling) {
    // 测试空轨迹
    Trajectory empty_trajectory;
    EXPECT_FALSE(interpolator_.loadTrajectory(empty_trajectory));
    
    // 测试无效时间
    auto trajectory = createMoveItStyleTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    EXPECT_THROW(interpolator_.interpolateAtTime(-0.1), std::runtime_error);
    EXPECT_THROW(interpolator_.interpolateAtTime(2.1), std::runtime_error);
}

TEST_F(MoveItCompatibilityTest, ConfigurationPersistence) {
    auto trajectory = createMoveItStyleTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    // 更改配置
    config_.target_dt = 0.005;  // 5ms间隔
    config_.make_monotonic = true;
    interpolator_.setInterpolationConfig(config_);
    
    // 验证配置已更新
    auto new_interpolated = interpolator_.interpolate();
    EXPECT_GT(new_interpolated.points.size(), 200);  // 应该有更多点
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
