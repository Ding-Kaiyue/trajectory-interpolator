#include <gtest/gtest.h>
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include <vector>
#include <cmath>

class TrajectoryInterpolatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.spline_type = trajectory_interpolator::SplineConfig::SplineType::CUBIC_SPLINE;
        config_.target_dt = 0.01;
        config_.make_monotonic = false;
        config_.max_velocity = M_PI / 2.0;
        config_.max_acceleration = M_PI;
        config_.max_jerk = M_PI;
        
        interpolator_.setInterpolationConfig(config_);
    }

    Trajectory createTestTrajectory() {
        Trajectory trajectory;
        trajectory.joint_names = {"joint1", "joint2", "joint3"};
        
        std::vector<double> times = {0.0, 1.0, 2.0, 3.0};
        std::vector<std::vector<double>> positions = {
            {0.0, 0.0, 0.0},
            {0.5, 0.3, 0.2},
            {1.0, 0.8, 0.6},
            {1.5, 1.2, 1.0}
        };
        
        for (size_t i = 0; i < times.size(); ++i) {
            TrajectoryPoint point;
            point.time_from_start = times[i];
            point.positions = positions[i];
            point.velocities = {0.0, 0.0, 0.0};
            point.accelerations = {0.0, 0.0, 0.0};
            trajectory.points.push_back(point);
        }
        
        return trajectory;
    }

    TrajectoryInterpolator interpolator_;
    trajectory_interpolator::SplineConfig config_;
};

TEST_F(TrajectoryInterpolatorTest, BasicInitialization) {
    EXPECT_FALSE(interpolator_.isLoaded());
    EXPECT_EQ(interpolator_.getStartTime(), 0.0);
    EXPECT_EQ(interpolator_.getEndTime(), 0.0);
}

TEST_F(TrajectoryInterpolatorTest, LoadTrajectory) {
    auto trajectory = createTestTrajectory();
    EXPECT_TRUE(interpolator_.loadTrajectory(trajectory));
    EXPECT_TRUE(interpolator_.isLoaded());
    EXPECT_EQ(interpolator_.getStartTime(), 0.0);
    EXPECT_EQ(interpolator_.getEndTime(), 3.0);
}

TEST_F(TrajectoryInterpolatorTest, InterpolateAtTime) {
    auto trajectory = createTestTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    auto positions_start = interpolator_.interpolateAtTime(0.0);
    EXPECT_EQ(positions_start.size(), 3);
    EXPECT_NEAR(positions_start[0], 0.0, 1e-6);
    
    auto positions_end = interpolator_.interpolateAtTime(3.0);
    EXPECT_EQ(positions_end.size(), 3);
    EXPECT_NEAR(positions_end[0], 1.5, 1e-6);
}

TEST_F(TrajectoryInterpolatorTest, GetVelocityAtTime) {
    auto trajectory = createTestTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    auto velocities = interpolator_.getVelocityAtTime(1.5);
    EXPECT_EQ(velocities.size(), 3);
}

TEST_F(TrajectoryInterpolatorTest, GetAccelerationAtTime) {
    auto trajectory = createTestTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    auto accelerations = interpolator_.getAccelerationAtTime(1.5);
    EXPECT_EQ(accelerations.size(), 3);
}

TEST_F(TrajectoryInterpolatorTest, InterpolateFullTrajectory) {
    auto trajectory = createTestTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    auto interpolated_trajectory = interpolator_.interpolate();
    EXPECT_EQ(interpolated_trajectory.joint_names.size(), 3);
    EXPECT_GT(interpolated_trajectory.points.size(), 10);
}

TEST_F(TrajectoryInterpolatorTest, CheckConstraints) {
    auto trajectory = createTestTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    config_.max_velocity = 2.0;
    config_.max_acceleration = 5.0;
    config_.max_jerk = 10.0;
    interpolator_.setInterpolationConfig(config_);
    
    EXPECT_TRUE(interpolator_.checkConstraints());
}

TEST_F(TrajectoryInterpolatorTest, InvalidTimeRequests) {
    auto trajectory = createTestTrajectory();
    interpolator_.loadTrajectory(trajectory);
    
    EXPECT_THROW(interpolator_.interpolateAtTime(-1.0), std::runtime_error);
    EXPECT_THROW(interpolator_.interpolateAtTime(4.0), std::runtime_error);
}

TEST_F(TrajectoryInterpolatorTest, EmptyTrajectory) {
    Trajectory empty_trajectory;
    EXPECT_FALSE(interpolator_.loadTrajectory(empty_trajectory));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 