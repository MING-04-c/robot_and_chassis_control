#pragma once

#include <cstdint>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Modern MoveIt Cartesian services already return a scaled, time-parameterized
// path. Validate it before execution without resampling its waypoints again.
inline bool has_valid_trajectory_timing(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  std::int64_t previous = -1;
  for (const auto& point : trajectory.points) {
    const auto& stamp = point.time_from_start;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
      return false;
    }
    const auto current = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    if (current <= previous) {
      return false;
    }
    previous = current;
  }
  return previous > 0;
}
