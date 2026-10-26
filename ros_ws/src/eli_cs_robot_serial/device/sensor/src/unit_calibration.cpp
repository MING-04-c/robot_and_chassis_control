#include "unit_calibration.hpp"

#include <algorithm>
#include <cmath>

namespace sensor_units
{
namespace
{
bool statistics(const std::vector<Sample> & samples, Sample & mean, Sample & deviation)
{
  if (samples.size() < 3) return false;
  for (const auto & sample : samples) {
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(sample[i])) return false;
      mean[i] += sample[i] / samples.size();
    }
  }
  for (const auto & sample : samples) {
    for (size_t i = 0; i < 6; ++i) {
      const double d = sample[i] - mean[i];
      deviation[i] += d * d / samples.size();
    }
  }
  for (double & d : deviation) d = std::sqrt(d);
  return true;
}

double force_norm(const Sample & values)
{
  return std::hypot(values[0], values[1], values[2]);
}

void diagnose_candidate(
  const std::vector<Sample> & before, const std::vector<Sample> & after,
  double min_force, Result & result)
{
  for (const auto & sample : before) {
    for (size_t i = 0; i < 6; ++i) result.before_abs_mean[i] += std::abs(sample[i]) / before.size();
  }
  for (const auto & sample : after) {
    for (size_t i = 0; i < 6; ++i) result.after_abs_mean[i] += std::abs(sample[i]) / after.size();
  }
  int direction = 0;
  bool conflict = false;
  auto vote = [&](double first, double second, double first_std, double second_std,
      const std::string & basis) {
      // 这是辅助候选，不是物理单位确认。微小均值差异或近零噪声不投票。
      if (!std::isfinite(first + second + first_std + second_std) ||
        std::min(first, second) < min_force || std::min(first_std, second_std) < 1e-6) return;
      const bool second_larger = second > first;
      if ((second_std > first_std) != second_larger) return;
      if (std::max(first, second) / std::min(first, second) < 1.25 ||
        std::max(first_std, second_std) / std::min(first_std, second_std) < 1.25) return;
      const double mean_uncertainty = std::hypot(
        first_std / std::sqrt(before.size()), second_std / std::sqrt(after.size()));
      const double std_uncertainty = std::hypot(
        first_std / std::sqrt(2.0 * (before.size() - 1)),
        second_std / std::sqrt(2.0 * (after.size() - 1)));
      if (std::abs(second - first) <= 3.0 * mean_uncertainty ||
        std::abs(second_std - first_std) <= 3.0 * std_uncertainty) return;
      const int proposed = second_larger ? 1 : -1;
      if (direction && direction != proposed) conflict = true;
      direction = proposed;
      if (!result.candidate_basis.empty()) result.candidate_basis += "+";
      result.candidate_basis += basis;
    };
  const char * names[] = {"Fx", "Fy", "Fz"};
  double first_sum = 0.0, second_sum = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    vote(result.before_abs_mean[i], result.after_abs_mean[i],
      result.before_std[i], result.after_std[i], names[i]);
    first_sum += result.before_abs_mean[i];
    second_sum += result.after_abs_mean[i];
  }
  vote(first_sum, second_sum, force_norm(result.before_std), force_norm(result.after_std),
    "sum_mean_abs(Fx,Fy,Fz)");
  if (!direction || conflict) {
    result.candidate_message = conflict ? "大小与波动证据在不同轴之间矛盾" :
      "平均绝对值与波动未同时明显增大，不能仅凭微小差异猜测单位";
    return;
  }
  result.candidate_after_unit = direction > 0 ? "N" : "kg";
  result.candidate_message = "仅按大小和波动得到的候选，需人工确认；不自动发布控制用力";
}
}  // namespace

Result compare(
  const std::vector<Sample> & before, const std::vector<Sample> & after,
  double min_force, double relative_tolerance, double max_relative_noise)
{
  Result result;
  Sample before_std{}, after_std{};
  if (before.size() != after.size() ||
    !statistics(before, result.before_mean, before_std) ||
    !statistics(after, result.after_mean, after_std))
  {
    result.message = "采样数量不符或包含无效数据";
    return result;
  }
  result.before_std = before_std;
  result.after_std = after_std;
  diagnose_candidate(before, after, min_force, result);
  double first_sum = 0.0, second_sum = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    first_sum += std::abs(result.before_mean[i]);
    second_sum += std::abs(result.after_mean[i]);
    const double low = std::min(std::abs(result.before_mean[i]), std::abs(result.after_mean[i]));
    const double high = std::max(std::abs(result.before_mean[i]), std::abs(result.after_mean[i]));
    result.axis_ratios[i] = low > 1e-12 ? high / low : 0.0;
    result.axis_stable[i] = low >= min_force &&
      before_std[i] <= max_relative_noise * std::abs(result.before_mean[i]) &&
      after_std[i] <= max_relative_noise * std::abs(result.after_mean[i]);
  }
  if (!std::isfinite(first_sum) || !std::isfinite(second_sum)) {
    result.message = "原始数据幅值无效";
    return result;
  }
  const double low_sum = std::min(first_sum, second_sum);
  result.sum_ratio = low_sum > 1e-12 ? std::max(first_sum, second_sum) / low_sum : 0.0;
  result.ratio = result.sum_ratio;
  auto ratio_matches = [relative_tolerance](double ratio) {
      return std::isfinite(ratio) && std::abs(ratio / kGravity - 1.0) <= relative_tolerance;
    };
  // 任一有足够信号且稳定的力轴即可提供证据，近零或高噪声轴不投票。
  int direction = 0;
  const char * axis_names[] = {"Fx", "Fy", "Fz"};
  for (size_t i = 0; i < 3; ++i) {
    if (!result.axis_stable[i] || !ratio_matches(result.axis_ratios[i]) ||
      std::signbit(result.before_mean[i]) != std::signbit(result.after_mean[i])) continue;
    const int vote = std::abs(result.after_mean[i]) > std::abs(result.before_mean[i]) ? 1 : -1;
    if (direction && direction != vote) {
      result.message = "不同稳定轴给出的单位方向互相矛盾，不认定单位";
      return result;
    }
    direction = vote;
    if (!result.basis.empty()) result.basis += "+";
    result.basis += axis_names[i];
    result.ratio = result.axis_ratios[i];
  }
  // 三轴相加使用绝对值，避免正负受力抵消；仍不把力矩加进来。
  const bool sum_stable = low_sum >= min_force &&
    force_norm(before_std) <= max_relative_noise * first_sum &&
    force_norm(after_std) <= max_relative_noise * second_sum;
  if (sum_stable && ratio_matches(result.sum_ratio)) {
    const int vote = second_sum > first_sum ? 1 : -1;
    if (direction && direction != vote) {
      result.message = "单轴与三轴合计给出的单位方向矛盾";
      return result;
    }
    direction = vote;
    if (!result.basis.empty()) result.basis += "+";
    result.basis += "sum_abs(Fx,Fy,Fz)";
    if (result.basis == "sum_abs(Fx,Fy,Fz)") result.ratio = result.sum_ratio;
  }
  if (!direction) {
    result.message = low_sum < min_force ?
      "原始力过小；保持稳定非零负载后重测" :
      "没有稳定单轴或三轴绝对值合计满足约 9.80665 倍；不认定单位";
    return result;
  }
  result.after_is_newton = direction > 0;
  const double scale = result.after_is_newton ? kGravity : 1.0 / kGravity;
  // 其他近零/噪声轴不阻止判定，但有明确稳定信号的轴不能与所选单位矛盾。
  for (size_t i = 0; i < 3; ++i) {
    if (!result.axis_stable[i]) continue;
    const double a = result.before_mean[i], b = result.after_mean[i];
    if (std::abs(b - scale * a) > relative_tolerance * std::abs(b)) {
      result.message = std::string("稳定的 ") + axis_names[i] + " 与单位证据不一致，请检查负载或设备单位协议";
      return result;
    }
  }
  for (size_t i = 3; i < 6; ++i) {
    const double a = result.before_mean[i], b = result.after_mean[i];
    if (std::min(std::abs(a), std::abs(b)) < 0.001 ||
      before_std[i] > max_relative_noise * std::abs(a) ||
      after_std[i] > max_relative_noise * std::abs(b)) continue;
    if (std::abs(b - scale * a) > relative_tolerance * std::abs(b)) {
      result.message = "稳定力矩通道与单位证据不一致；请检查力矩单位协议";
      return result;
    }
  }
  result.success = true;
  result.message = result.after_is_newton ? "当前设备输出 N/N·m" : "当前设备输出 kgf/kgf·m，软件乘 9.80665";
  return result;
}
}  // namespace sensor_units
