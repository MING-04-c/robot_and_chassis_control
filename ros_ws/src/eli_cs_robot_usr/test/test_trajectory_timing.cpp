#include <gtest/gtest.h>
#include "trajectory_timing.hpp"

TEST(TrajectoryTiming, AcceptsTimedPathStartingAtZero) {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.points.resize(3);
  trajectory.points[1].time_from_start.nanosec = 500000000;
  trajectory.points[2].time_from_start.sec = 1;
  EXPECT_TRUE(has_valid_trajectory_timing(trajectory));
}

TEST(TrajectoryTiming, RejectsUntimedOrNonmonotonicPath) {
  trajectory_msgs::msg::JointTrajectory trajectory;
  EXPECT_FALSE(has_valid_trajectory_timing(trajectory));
  trajectory.points.resize(2);
  EXPECT_FALSE(has_valid_trajectory_timing(trajectory));
  trajectory.points[0].time_from_start.sec = 1;
  EXPECT_FALSE(has_valid_trajectory_timing(trajectory));
  trajectory.points[1].time_from_start.sec = 1;
  EXPECT_FALSE(has_valid_trajectory_timing(trajectory));
  trajectory.points[1].time_from_start.sec = 2;
  EXPECT_TRUE(has_valid_trajectory_timing(trajectory));
  trajectory.points[0].time_from_start.sec = -1;
  EXPECT_FALSE(has_valid_trajectory_timing(trajectory));
  trajectory.points[0].time_from_start.sec = 0;
  trajectory.points[0].time_from_start.nanosec = 1000000000U;
  EXPECT_FALSE(has_valid_trajectory_timing(trajectory));
}
