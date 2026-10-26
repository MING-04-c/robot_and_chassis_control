#pragma once

#include <array>
#include <string>
#include <vector>

namespace sensor_units
{
using Sample = std::array<double, 6>;
constexpr double kGravity = 9.80665;

struct Result
{
  bool success{false};
  bool after_is_newton{false};
  Sample before_mean{}, after_mean{};
  Sample before_abs_mean{}, after_abs_mean{}, before_std{}, after_std{};
  std::string candidate_after_unit{"unknown"};
  std::string candidate_basis;
  std::string candidate_message;
  double ratio{0.0};
  std::array<double, 3> axis_ratios{};
  std::array<bool, 3> axis_stable{};
  double sum_ratio{0.0};
  std::string basis;
  std::string message;
};

// 输入切换前后的原始六维数据，返回单位判断和诊断；以稳定单轴或三轴绝对值和提供证据，不混加力与力矩。
Result compare(
  const std::vector<Sample> & before, const std::vector<Sample> & after,
  double min_force, double relative_tolerance, double max_relative_noise);
}  // namespace sensor_units
