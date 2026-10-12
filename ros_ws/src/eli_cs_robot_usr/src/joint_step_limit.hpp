#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace admittance
{
// 用同一个比例限制整个关节增量，保留 IK 解的关节运动方向。
template<std::size_t N>
double limit_joint_step(
  const std::array<double, N> & current,
  std::array<double, N> & target, double limit)
{
  double largest = 0.0;
  for (std::size_t i = 0; i < N; ++i) {
    largest = std::max(largest, std::abs(target[i] - current[i]));
  }
  const double scale = limit > 0.0 && largest > limit ? limit / largest : 1.0;
  for (std::size_t i = 0; i < N; ++i) {
    target[i] = current[i] + scale * (target[i] - current[i]);
  }
  return largest * scale;
}
}  // namespace admittance
