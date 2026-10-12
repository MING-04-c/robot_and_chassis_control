#include "camer_diver/line_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace camer_diver
{
void LineConfig::validate() const
{
  const auto kernel_ok = [](int k) {return k >= 1 && k <= 15 && k % 2 == 1;};
  if (processing_width < 64 || processing_width > 1920 || hue_low < 0 ||
    hue_high > 179 || hue_low > hue_high || saturation_min < 0 || saturation_min > 255 ||
    value_min < 0 || value_min > 255 || !kernel_ok(blur_kernel) ||
    weak_saturation_min < 1 || weak_saturation_min > 255 || weak_value_min < 1 ||
    weak_value_min > 255 || weak_seed_pixels < 1 || weak_seed_pixels > 1000000 ||
    !std::isfinite(blue_excess_min) || blue_excess_min < 0 || blue_excess_min > 1 ||
    !std::isfinite(strong_blue_excess_min) || strong_blue_excess_min < 0 ||
    strong_blue_excess_min > 1 || weak_support_distance < 0 || weak_support_distance > 50 ||
    !kernel_ok(open_kernel) || !kernel_ok(close_kernel) ||
    (blur_type != "median" && blur_type != "gaussian" && blur_type != "none") ||
    !std::isfinite(roi_top) || !std::isfinite(roi_bottom) || roi_top < 0 ||
    roi_bottom > 1 || roi_bottom - roi_top < 0.05 ||
    min_component_area < 0 || min_component_area > 1000000 || row_step < 1 || row_step > 50 ||
    min_line_width < 1 || min_line_width > processing_width || search_radius < 0 ||
    search_radius > processing_width || max_center_jump < 1 || max_center_jump > processing_width ||
    initial_search_radius < 0 || initial_search_radius > 1920 ||
    bottom_anchor_tolerance_rows < 0 || bottom_anchor_tolerance_rows > max_missing_rows ||
    !std::isfinite(max_width_fraction) || max_width_fraction <= 0 || max_width_fraction > 1 ||
    !std::isfinite(max_blue_fraction) || max_blue_fraction <= 0 || max_blue_fraction > 1 ||
    max_missing_rows < 0 || max_missing_rows > 100 || min_valid_rows < 1 ||
    min_valid_rows > 1000 || canny_low < 0 || canny_high > 255 || canny_low >= canny_high ||
    !std::isfinite(min_line_aspect) || min_line_aspect < 1 || min_line_aspect > 20 ||
    !std::isfinite(min_row_fill) || min_row_fill <= 0 || min_row_fill > 1 ||
    !std::isfinite(min_row_coverage) || min_row_coverage <= 0 || min_row_coverage > 1 ||
    !std::isfinite(max_shape_center_step) || max_shape_center_step <= 0 ||
    max_shape_center_step > 10 ||
    !std::isfinite(min_core_fraction) || min_core_fraction < 0 || min_core_fraction > 1 ||
    !std::isfinite(max_width_spread) || max_width_spread < 1 || max_width_spread > 20 ||
    max_occlusion_gap < 0 || max_occlusion_gap > 120 ||
    !std::isfinite(max_occlusion_offset) || max_occlusion_offset < 0 || max_occlusion_offset > 100 ||
    !std::isfinite(canny_sigma) || canny_sigma <= 0 || canny_sigma >= 1 ||
    !std::isfinite(lookahead_ratio) || lookahead_ratio < 0 || lookahead_ratio > 1)
  {
    throw std::invalid_argument("Invalid line-tracking configuration; check HSV, ROI, odd kernels, "
            "width/jump limits and scan parameters");
  }
}

namespace
{
double gap_limit(const LineConfig & config, double tape_width)
{
  return std::min(double(config.max_occlusion_gap),
    std::max(3.0 * config.row_step, 1.5 * tape_width));
}

// Median pairwise slopes resist a cable-cut tip moving just a few section centres.
cv::Vec2d robust_line(const std::vector<cv::Vec3d> & points)
{
  std::vector<double> slopes, intercepts;
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      if (points[i][0] != points[j][0]) {
        slopes.push_back((points[i][1] - points[j][1]) / (points[i][0] - points[j][0]));
      }
    }
  }
  std::sort(slopes.begin(), slopes.end());
  const double slope = slopes.empty() ? 0 : slopes[slopes.size() / 2];
  for (const auto & p : points) {intercepts.push_back(p[1] - slope * p[0]);}
  std::sort(intercepts.begin(), intercepts.end());
  return {intercepts.empty() ? 0 : intercepts[intercepts.size() / 2], slope};
}

void label(cv::Mat & image, const std::string & text)
{
  cv::rectangle(image, cv::Rect(0, 0, image.cols, std::min(28, image.rows)),
    cv::Scalar(30, 30, 30), cv::FILLED);
  cv::putText(image, text, {8, 19}, cv::FONT_HERSHEY_SIMPLEX, 0.48,
    cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
}
}  // namespace

LineResult track_blue_line(const cv::Mat & bgr, const LineConfig & config)
{
  config.validate();
  if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols < 8 || bgr.rows < 8) {
    throw std::invalid_argument("Expected a nonempty BGR8 camera image");
  }
  LineResult result;
  struct Corridor {cv::Vec3d lower, upper; double width; bool horizontal;};
  std::vector<Corridor> corridors;
  const int width = std::min(bgr.cols, config.processing_width);
  const int height = std::max(8, static_cast<int>(std::round(bgr.rows * double(width) / bgr.cols)));
  cv::resize(bgr, result.image, {width, height}, 0, 0, cv::INTER_AREA);
  cv::Mat filtered, hsv;
  if (config.blur_type == "median" && config.blur_kernel > 1) {
    cv::medianBlur(result.image, filtered, config.blur_kernel);
  } else if (config.blur_type == "gaussian" && config.blur_kernel > 1) {
    cv::GaussianBlur(result.image, filtered, {config.blur_kernel, config.blur_kernel}, 0);
  } else {
    filtered = result.image;
  }
  cv::cvtColor(filtered, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(config.hue_low, config.saturation_min, config.value_min),
    cv::Scalar(config.hue_high, 255, 255), result.strong_mask);
  // Hue alone is unstable on grey pixels; all strong seeds also need RGB blue dominance.
  for (int y = 0; y < height; ++y) {
    const auto * colors = filtered.ptr<cv::Vec3b>(y);
    auto * strong = result.strong_mask.ptr<uint8_t>(y);
    for (int x = 0; x < width; ++x) {
      const double b = colors[x][0], g = colors[x][1], r = colors[x][2];
      if (b <= g || b <= r || (2 * b - g - r) / (b + g + r + 1) <
        config.strong_blue_excess_min) {strong[x] = 0;}
    }
  }
  result.roi_top = std::clamp(static_cast<int>(height * config.roi_top), 0, height - 2);
  result.roi_bottom = std::clamp(static_cast<int>(height * config.roi_bottom),
    result.roi_top + 1, height - 1);
  const auto crop_roi = [&result, height](cv::Mat & mask) {
      mask.rowRange(0, result.roi_top).setTo(0);
      if (result.roi_bottom + 1 < height) {
        mask.rowRange(result.roi_bottom + 1, height).setTo(0);
      }
    };
  crop_roi(result.strong_mask);
  result.mask = result.strong_mask.clone();
  result.weak_mask = cv::Mat::zeros(height, width, CV_8UC1);
  if (config.illumination_support) {
    cv::inRange(hsv, cv::Scalar(config.hue_low, config.weak_saturation_min, config.weak_value_min),
      cv::Scalar(config.hue_high, 255, 255), result.weak_mask);
    // Relative blue dominance tolerates intensity scaling and mildly washed-out blue.
    // Do not accept all bright low-saturation pixels: white floor has no blue evidence.
    for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
      const auto * colors = filtered.ptr<cv::Vec3b>(y);
      auto * weak = result.weak_mask.ptr<uint8_t>(y);
      for (int x = 0; x < width; ++x) {
        const double b = colors[x][0], g = colors[x][1], r = colors[x][2];
        if (b <= g || b <= r || (2 * b - g - r) / (b + g + r + 1) < config.blue_excess_min) {
          weak[x] = 0;
        }
      }
    }
    crop_roi(result.weak_mask);
    cv::Mat non_seeds, seed_distance;
    cv::bitwise_not(result.strong_mask, non_seeds);
    cv::distanceTransform(non_seeds, seed_distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::Mat candidates, labels;
    cv::bitwise_or(result.weak_mask, result.strong_mask, candidates);
    const int count = cv::connectedComponents(candidates, labels, 8, CV_32S);
    std::vector<int> seeds(count, 0);
    for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
      const auto * strong = result.strong_mask.ptr<uint8_t>(y);
      const auto * ids = labels.ptr<int>(y);
      for (int x = 0; x < width; ++x) {if (strong[x]) {++seeds[ids[x]];}}
    }
    for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
      const auto * ids = labels.ptr<int>(y);
      auto * mask = result.mask.ptr<uint8_t>(y);
      for (int x = 0; x < width; ++x) {
        if (ids[x] && seeds[ids[x]] >= config.weak_seed_pixels &&
          seed_distance.at<float>(y, x) <= config.weak_support_distance) {mask[x] = 255;}
      }
    }
  }
  const cv::Mat color_evidence = result.mask.clone();
  if (config.open_kernel > 1) {
    cv::morphologyEx(result.mask, result.mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, {config.open_kernel, config.open_kernel}));
  }
  if (config.close_kernel > 1) {
    cv::morphologyEx(result.mask, result.mask, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, {config.close_kernel, config.close_kernel}));
  }
  // Morphology must not invent blue pixels on a grey/white interior or bridge.
  cv::bitwise_and(result.mask, color_evidence, result.mask);
  result.mask.rowRange(0, result.roi_top).setTo(0);
  if (result.roi_bottom + 1 < height) {
    result.mask.rowRange(result.roi_bottom + 1, height).setTo(0);
  }
  if (config.min_component_area > 0) {
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(result.mask, labels, stats, centroids);
    std::vector<uint8_t> keep(count, 0);
    for (int i = 1; i < count; ++i) {
      keep[i] = stats.at<int>(i, cv::CC_STAT_AREA) >= config.min_component_area ? 255 : 0;
    }
    for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
      const int * ids = labels.ptr<int>(y);
      auto * output = result.mask.ptr<uint8_t>(y);
      for (int x = 0; x < width; ++x) {output[x] = keep[ids[x]];}
    }
  }
  result.blue_fraction = double(cv::countNonZero(result.mask)) /
    (width * (result.roi_bottom - result.roi_top + 1));
  const bool blue_interference = result.blue_fraction > config.max_blue_fraction;
  bool shape_rejected = false;
  // Select one connected candidate before scanning; never jump between unrelated blobs.
  // Reward usable scan rows and saturated blue evidence, not proximity to the image centre.
  if (!blue_interference) {
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(result.mask, labels, stats, centroids);
    cv::Mat interior_distance;
    cv::distanceTransform(result.mask, interior_distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    std::vector<double> saturation_sum(count, 0);
    for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
      const auto * ids = labels.ptr<int>(y);
      const auto * colors = hsv.ptr<cv::Vec3b>(y);
      for (int x = 0; x < width; ++x) {
        if (ids[x]) {saturation_sum[ids[x]] += colors[x][1];}
      }
    }
    std::vector<int> usable(count, 0), sampled(count, 0), continuous(count, 0);
    std::vector<double> fill_sum(count, 0), previous_center(count, 0);
    std::vector<int> previous_y(count, -1), previous_width(count, 0);
    std::vector<std::vector<int>> widths(count);
    std::vector<std::vector<cv::Vec3d>> profiles(count);  // y, centre, width; bottom first
    std::vector<std::vector<double>> thicknesses(count);
    std::vector<uint8_t> horizontal(count, 0);
    std::vector<double> lengths(count, 0);
    for (int id = 1; id < count; ++id) {
      const cv::Rect bounds(stats.at<int>(id, cv::CC_STAT_LEFT),
        stats.at<int>(id, cv::CC_STAT_TOP), stats.at<int>(id, cv::CC_STAT_WIDTH),
        stats.at<int>(id, cv::CC_STAT_HEIGHT));
      cv::Mat component;
      cv::compare(labels(bounds), id, component, cv::CMP_EQ);
      std::vector<cv::Point> points;
      cv::findNonZero(component, points);
      const auto box = cv::minAreaRect(points);
      lengths[id] = std::max(box.size.width, box.size.height);
      // A small deadband keeps two halves of a 45-degree tape on the same scan
      // axis despite one-pixel bounding-box rounding differences.
      horizontal[id] = stats.at<int>(id, cv::CC_STAT_WIDTH) >
        1.1 * stats.at<int>(id, cv::CC_STAT_HEIGHT);
    }
    for (int y = result.roi_bottom; y >= result.roi_top; y -= config.row_step) {
      const auto * ids = labels.ptr<int>(y);
      std::vector<int> left(count, width), right(count, -1), pixels(count, 0);
      for (int x = 0; x < width; ++x) {
        const int id = ids[x];
        if (id) {left[id] = std::min(left[id], x); right[id] = x; ++pixels[id];}
      }
      for (int id = 1; id < count; ++id) {
        if (horizontal[id]) {continue;}
        const int top = stats.at<int>(id, cv::CC_STAT_TOP);
        if (y < top || y >= top + stats.at<int>(id, cv::CC_STAT_HEIGHT)) {continue;}
        ++sampled[id];
        if (!pixels[id]) {continue;}
        const int span = right[id] - left[id] + 1;
        const double fill = double(pixels[id]) / span;
        if (left[id] == 0 || right[id] == width - 1 || span < config.min_line_width ||
          span > width * config.max_width_fraction || fill < config.min_row_fill) {continue;}
        const double center = (left[id] + right[id]) / 2.0;
        if (previous_y[id] >= 0 && std::abs(center - previous_center[id]) >
          config.max_shape_center_step *
          ((span + previous_width[id]) / 2.0 + previous_y[id] - y)) {++continuous[id];}
        previous_y[id] = y; previous_width[id] = span; previous_center[id] = center;
        ++usable[id]; fill_sum[id] += fill; widths[id].push_back(span);
        profiles[id].push_back({double(y), center, double(span)});
        double radius = 0;
        for (int x = left[id]; x <= right[id]; ++x) {
          if (ids[x] == id) {radius = std::max(radius, double(interior_distance.at<float>(y, x)));}
        }
        thicknesses[id].push_back(2 * radius);
      }
    }
    // Horizontal tape is evaluated by column cross-sections, not by broad row chords.
    for (int x = width - 1; x >= 0; x -= config.row_step) {
      std::vector<int> top(count, height), bottom(count, -1), pixels(count, 0);
      for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
        const int id = labels.at<int>(y, x);
        if (id && horizontal[id]) {
          top[id] = std::min(top[id], y); bottom[id] = y; ++pixels[id];
        }
      }
      for (int id = 1; id < count; ++id) {
        if (!horizontal[id]) {continue;}
        const int start = stats.at<int>(id, cv::CC_STAT_LEFT);
        if (x < start || x >= start + stats.at<int>(id, cv::CC_STAT_WIDTH)) {continue;}
        ++sampled[id];
        if (!pixels[id]) {continue;}
        const int span = bottom[id] - top[id] + 1;
        const double fill = double(pixels[id]) / span;
        if (top[id] == result.roi_top || bottom[id] == result.roi_bottom ||
          span < config.min_line_width || span > width * config.max_width_fraction ||
          fill < config.min_row_fill) {continue;}
        const double center = (top[id] + bottom[id]) / 2.0;
        if (previous_y[id] >= 0 && std::abs(center - previous_center[id]) >
          config.max_shape_center_step * ((span + previous_width[id]) / 2.0 + previous_y[id] - x)) {
          ++continuous[id];
        }
        previous_y[id] = x; previous_width[id] = span; previous_center[id] = center;
        ++usable[id]; fill_sum[id] += fill; widths[id].push_back(span);
        profiles[id].push_back({double(x), center, double(span)});
        double radius = 0;
        for (int y = top[id]; y <= bottom[id]; ++y) {
          if (labels.at<int>(y, x) == id) {
            radius = std::max(radius, double(interior_distance.at<float>(y, x)));
          }
        }
        thicknesses[id].push_back(2 * radius);
      }
    }
    int selected = 0;
    double best_score = -1, best_distance = width;
    std::vector<uint8_t> qualified(count, 0);
    std::vector<uint8_t> joinable(count, 0);
    std::vector<double> medians(count, 0);
    for (int id = 1; id < count; ++id) {
      CandidateDiagnostic diagnostic;
      diagnostic.id = id; diagnostic.horizontal = horizontal[id];
      diagnostic.row_coverage = sampled[id] ? double(usable[id]) / sampled[id] : 0;
      if (usable[id] < 2) {
        diagnostic.rejection = "insufficient_sections";
        result.candidates.push_back(diagnostic); continue;
      }
      auto & row_widths = widths[id];
      std::sort(row_widths.begin(), row_widths.end());
      const double median_width = row_widths[row_widths.size() / 2];
      medians[id] = median_width;
      auto & thickness = thicknesses[id];
      std::sort(thickness.begin(), thickness.end());
      const double normal_thickness = std::max(1.0, thickness[thickness.size() / 2]);
      // Rotated tape ends (including cable-cut ends) taper naturally in row/column
      // chords. Measure width stability away from both ends, not on those triangles.
      const size_t guard = std::min(profiles[id].size() / 4,
        static_cast<size_t>(std::ceil(median_width / config.row_step)));
      std::vector<double> interior_widths;
      for (size_t k = guard; k < profiles[id].size() - guard; ++k) {
        interior_widths.push_back(profiles[id][k][2]);
      }
      std::sort(interior_widths.begin(), interior_widths.end());
      const double width_spread = interior_widths[(interior_widths.size() - 1) * 9 / 10] /
        interior_widths[(interior_widths.size() - 1) / 10];
      int core_pixels = 0;
      const cv::Rect bounds(stats.at<int>(id, cv::CC_STAT_LEFT),
        stats.at<int>(id, cv::CC_STAT_TOP), stats.at<int>(id, cv::CC_STAT_WIDTH),
        stats.at<int>(id, cv::CC_STAT_HEIGHT));
      for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
        for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
          if (labels.at<int>(y, x) == id && interior_distance.at<float>(y, x) >=
            std::max(1.5, normal_thickness * 0.20)) {++core_pixels;}
        }
      }
      // Internal thickness is rotation-tolerant; horizontal/diagonal chords alone
      // must not turn a genuine tape into a rejected "fat blob".
      diagnostic.aspect = lengths[id] / normal_thickness;
      diagnostic.core_fraction = double(core_pixels) / stats.at<int>(id, cv::CC_STAT_AREA);
      diagnostic.width_spread = width_spread;
      if (usable[id] < config.min_valid_rows) {diagnostic.rejection = "insufficient_sections";}
      else if (diagnostic.aspect < config.min_line_aspect) {diagnostic.rejection = "aspect";}
      else if (diagnostic.row_coverage < config.min_row_coverage) {diagnostic.rejection = "coverage";}
      else if (continuous[id] > 0) {diagnostic.rejection = "center_step";}
      else if (fill_sum[id] / usable[id] < config.min_row_fill) {diagnostic.rejection = "fill";}
      else if (diagnostic.core_fraction < config.min_core_fraction) {diagnostic.rejection = "core";}
      else if (width_spread > config.max_width_spread) {diagnostic.rejection = "width_spread";}
      if (!diagnostic.rejection.empty()) {
        // A short piece cut off by a cable can be too short on its own. It may
        // join an already qualified tape only with matching endpoints/widths.
        joinable[id] = (diagnostic.rejection == "aspect" ||
          diagnostic.rejection == "insufficient_sections") && diagnostic.aspect >= 0.75 &&
          diagnostic.row_coverage >= config.min_row_coverage && continuous[id] == 0 &&
          fill_sum[id] / usable[id] >= config.min_row_fill &&
          diagnostic.core_fraction >= config.min_core_fraction * 0.8 && width_spread <= config.max_width_spread;
        result.candidates.push_back(diagnostic);
        if (diagnostic.rejection != "insufficient_sections") {shape_rejected = true;}
        continue;
      }
      diagnostic.accepted = true;
      result.candidates.push_back(diagnostic);
      qualified[id] = 1;
      const double score = lengths[id] * saturation_sum[id] /
        stats.at<int>(id, cv::CC_STAT_AREA);
      const double distance = std::abs(centroids.at<double>(id, 0) - (width - 1) / 2.0);
      if (score > best_score || (score == best_score && distance < best_distance)) {
        selected = id; best_score = score; best_distance = distance;
      }
    }
    if (selected) {
      result.selected_candidate = selected;
      result.horizontal = horizontal[selected];
      std::vector<uint8_t> keep(count, 0);
      keep[selected] = 1;
      // Join only short axis-separated candidates with compatible widths and
      // matching endpoint tangents. This retains evidence, never paints over an object.
      const auto endpoint = [&profiles, &medians](int id, bool top) {
          std::vector<cv::Vec3d> points;
          for (const auto & p : profiles[id]) {
            if (p[2] >= medians[id] * 0.9) {points.push_back(p);}
          }
          if (points.empty()) {points = profiles[id];}
          const size_t guard = std::min<size_t>(2, points.size() / 4);
          const size_t n = std::min<size_t>(9, points.size() - guard);
          std::vector<cv::Vec3d> intact;
          for (size_t k = 0; k < n; ++k) {
            intact.push_back(points[top ? points.size() - 1 - guard - k : guard + k]);
          }
          const auto fit = robust_line(intact);
          const auto & p = top ? points.back() : points.front();
          return cv::Vec3d(p[0], fit[0] + fit[1] * p[0], fit[1]);
        };
      bool changed = true;
      while (changed && config.max_occlusion_gap > 0) {
        changed = false;
        for (int id = 1; id < count; ++id) {
          if ((!qualified[id] && !joinable[id]) || keep[id]) {continue;}
          for (int anchor = 1; anchor < count; ++anchor) {
            if (!keep[anchor] || !qualified[anchor]) {continue;}
            if (horizontal[id] != horizontal[anchor]) {
              if (result.joins.size() < 64) {
                result.joins.push_back({id, anchor, 0, 0, 0, 0, "scan_axis_mismatch"});
              }
              continue;
            }
            int lower = anchor, upper = id;
            if (profiles[id].back()[0] + profiles[id].front()[0] >
              profiles[anchor].back()[0] + profiles[anchor].front()[0]) {
              lower = id; upper = anchor;
            }
            const auto a = endpoint(lower, true), b = endpoint(upper, false);
            const double gap = a[0] - b[0];
            const double width_ratio = std::max(medians[id], medians[anchor]) /
              std::min(medians[id], medians[anchor]);
            const double limit = gap_limit(config, std::min(medians[id], medians[anchor]));
            const double alignment = std::max(std::abs(a[1] - a[2] * gap - b[1]),
              std::abs(b[1] + b[2] * gap - a[1]));
            std::string rejection;
            if (gap <= 0) {rejection = "overlapping_axis_ranges";}
            else if (width_ratio > 1.8) {rejection = "width_mismatch";}
            else if (alignment > config.max_occlusion_offset) {rejection = "tangent_mismatch";}
            else if (gap - config.row_step > limit) {rejection = "gap_too_long";}
            if (result.joins.size() < 64) {
              result.joins.push_back({id, anchor, gap - config.row_step, limit,
                width_ratio, alignment, rejection});
            }
            // Edge evidence is independent of permission to interpolate the gap.
            if (gap > 0 && gap <= 120 && width_ratio <= 1.8 &&
              alignment <= config.max_occlusion_offset) {
              corridors.push_back({a, b, std::min(medians[id], medians[anchor]),
                bool(horizontal[id])});
            }
            if (rejection.empty()) {
              keep[id] = 1; changed = true; break;
            }
          }
        }
      }
      for (int y = 0; y < height; ++y) {
        auto * pixels = result.mask.ptr<uint8_t>(y);
        const auto * ids = labels.ptr<int>(y);
        for (int x = 0; x < width; ++x) {pixels[x] = keep[ids[x]] ? 255 : 0;}
      }
      for (auto & diagnostic : result.candidates) {
        diagnostic.retained = keep[diagnostic.id];
        if (keep[diagnostic.id] && !qualified[diagnostic.id]) {
          diagnostic.context_supported = true; diagnostic.accepted = true; diagnostic.rejection.clear();
        }
      }
      shape_rejected = false;
    } else {
      // No fallback to the unfiltered mask: that would let rejected noise steer again.
      result.mask.setTo(0);
    }
  }
  result.target_y = result.roi_bottom - static_cast<int>(config.lookahead_ratio *
    (result.roi_bottom - result.roi_top));

  int seed = width / 2, missing = 0;
  bool have_valid_seed = false, stopped = false;
  for (int y = result.roi_bottom; y >= result.roi_top; y -= config.row_step) {
    ScanRow row;
    row.y = y;
    row.axis_coordinate = y;
    row.seed = seed;
    const auto * pixels = result.mask.ptr<uint8_t>(y);
    int start = seed;
    if (stopped || blue_interference) {
      row.reason = blue_interference ? "too much blue" : "search stopped after gap";
      row.left = 0; row.right = width - 1; row.center = (width - 1) / 2.0;
    } else {
      if (!pixels[start] && config.allow_reacquire) {
        const int radius = have_valid_seed ? config.search_radius : config.initial_search_radius;
        for (int offset = 1; offset <= radius; ++offset) {
          // Conservative nearest-blue reacquisition, not global leftmost/rightmost pooling.
          if (seed - offset >= 0 && pixels[seed - offset]) {start = seed - offset; break;}
          if (seed + offset < width && pixels[seed + offset]) {start = seed + offset; break;}
        }
        row.reacquired = start != seed;
      }
      if (!pixels[start]) {
        row.left = 0; row.right = width - 1; row.center = (width - 1) / 2.0;
        row.reason = "seed outside blue";
      } else {
        int left = start, right = start;
        while (left > 0 && pixels[left - 1]) {--left;}
        while (right < width - 1 && pixels[right + 1]) {++right;}
        row.left = left; row.right = right;
        row.left_point = {left, y}; row.right_point = {right, y};
        row.cross_width = right - left + 1;
        row.center = (left + right) / 2.0;
        row.clipped = left == 0 || right == width - 1;
        const int line_width = right - left + 1;
        if (row.clipped) {row.reason = "boundary clipped";}
        else if (line_width < config.min_line_width) {row.reason = "too narrow";}
        else if (line_width > width * config.max_width_fraction) {row.reason = "too wide / bend / junction";}
        else if (have_valid_seed && std::abs(row.center - seed) > config.max_center_jump) {
          row.reason = "center jump / bend / junction";
        } else {row.valid = true;}
      }
    }
    if (row.valid) {
      ++result.valid_rows;
      seed = std::clamp(static_cast<int>(std::round(row.center)), 0, width - 1);
      have_valid_seed = true;
      missing = 0;
    } else {
      ++result.rejected_rows;
      // Keep the last valid seed. Never steer using an edge fallback midpoint.
      // Empty rows before the first acquisition are not a tracking gap.
      const int gap_limit = std::max(config.max_missing_rows,
        (config.max_occlusion_gap + config.row_step - 1) / config.row_step);
      if (have_valid_seed && ++missing > gap_limit) {stopped = true;}
    }
    result.rows.push_back(row);
  }
  if (result.horizontal && !blue_interference) {
    result.rows.clear(); result.valid_rows = 0; result.rejected_rows = 0;
    int cross_seed = height / 2;
    bool acquired = false;
    for (int x = width - 1; x >= 0; x -= config.row_step) {
      ScanRow row;
      row.axis_coordinate = x; row.center = x; row.seed = x; row.y = cross_seed;
      int start = cross_seed;
      if (!result.mask.at<uint8_t>(start, x) && config.allow_reacquire) {
        const int radius = acquired ? config.search_radius : config.initial_search_radius;
        for (int d = 1; d <= radius; ++d) {
          if (cross_seed - d >= result.roi_top && result.mask.at<uint8_t>(cross_seed - d, x)) {
            start = cross_seed - d; break;
          }
          if (cross_seed + d <= result.roi_bottom && result.mask.at<uint8_t>(cross_seed + d, x)) {
            start = cross_seed + d; break;
          }
        }
      }
      if (!result.mask.at<uint8_t>(start, x)) {row.reason = "seed outside blue";}
      else {
        int top = start, bottom = start;
        while (top > result.roi_top && result.mask.at<uint8_t>(top - 1, x)) {--top;}
        while (bottom < result.roi_bottom && result.mask.at<uint8_t>(bottom + 1, x)) {++bottom;}
        row.y = (top + bottom) / 2;
        row.cross_width = bottom - top + 1;
        row.left = x; row.right = x;
        row.left_point = {x, top}; row.right_point = {x, bottom};
        row.clipped = top == result.roi_top || bottom == result.roi_bottom;
        if (row.clipped) {row.reason = "boundary clipped";}
        else if (row.cross_width < config.min_line_width) {row.reason = "too narrow";}
        else if (row.cross_width > width * config.max_width_fraction) {row.reason = "too wide / bend / junction";}
        else if (acquired && std::abs(row.y - cross_seed) > config.max_center_jump) {
          row.reason = "center jump / bend / junction";
        } else {row.valid = true; cross_seed = row.y; acquired = true;}
      }
      if (row.valid) {++result.valid_rows;} else {++result.rejected_rows;}
      result.rows.push_back(row);
    }
  }
  // A cable crossing one side can leave a narrow blue sliver: do not call its
  // biased midpoint a measurement of the full tape. Use nearby intact row widths.
  const auto measured = result.rows;
  for (size_t i = 0; i < result.rows.size(); ++i) {
    auto & row = result.rows[i];
    if (!row.valid && (row.reason != "too narrow" || row.cross_width == 0)) {continue;}
    std::vector<int> local_widths;
    const size_t start = i > 6 ? i - 6 : 0;
    for (size_t k = start; k < std::min(i + 7, measured.size()); ++k) {
      if (measured[k].valid) {local_widths.push_back(measured[k].cross_width);}
    }
    std::sort(local_widths.begin(), local_widths.end());
    if (local_widths.empty()) {continue;}
    if (row.cross_width < 0.70 * local_widths[local_widths.size() / 2]) {
      if (row.valid) {--result.valid_rows; ++result.rejected_rows;}
      row.valid = false; row.reason = "partial occlusion / narrowed cross-section";
    }
  }
  // Interpolate bounded gaps only when intact observations exist at BOTH ends.
  for (size_t i = 0; i < result.rows.size();) {
    if (result.rows[i].valid) {++i; continue;}
    const size_t start = i;
    while (i < result.rows.size() && !result.rows[i].valid) {++i;}
    if (!start || i == result.rows.size()) {continue;}
    const auto & lower = result.rows[start - 1];
    const auto & upper = result.rows[i];
    const int missing_height = lower.axis_coordinate - upper.axis_coordinate - config.row_step;
    const int lw = lower.cross_width, uw = upper.cross_width;
    const auto cross_position = [&result](const ScanRow & row) {
        return result.horizontal ? double(row.y) : row.center;
      };
    const auto tangent = [&result, &cross_position](size_t endpoint, int direction) {
        std::vector<cv::Vec3d> points;
        for (int step = 0; step < 9; ++step) {
          const int index = static_cast<int>(endpoint) + step * direction;
          if (index < 0 || index >= static_cast<int>(result.rows.size())) {break;}
          const auto & row = result.rows[index];
          if (!row.valid) {break;}
          points.push_back({double(row.axis_coordinate), cross_position(row), double(row.cross_width)});
        }
        if (points.size() >= 5) {points.erase(points.begin());}
        return robust_line(points);
      };
    const double axis_gap = lower.axis_coordinate - upper.axis_coordinate;
    const auto lf = tangent(start - 1, -1), uf = tangent(i, 1);
    const double lc = lf[0] + lf[1] * lower.axis_coordinate;
    const double uc = uf[0] + uf[1] * upper.axis_coordinate;
    const bool aligned = std::abs(lc - lf[1] * axis_gap - uc) <= config.max_occlusion_offset &&
      std::abs(uc + uf[1] * axis_gap - lc) <= config.max_occlusion_offset;
    bool compatible_gap = true;
    for (size_t k = start; k < i; ++k) {
      const auto & reason = result.rows[k].reason;
      if (reason != "seed outside blue" &&
        reason != "partial occlusion / narrowed cross-section") {compatible_gap = false;}
    }
    if (config.max_occlusion_gap == 0 || missing_height > gap_limit(config, std::min(lw, uw)) ||
      !compatible_gap ||
      double(std::max(lw, uw)) / std::min(lw, uw) > 1.8 ||
      !aligned) {continue;}
    for (size_t k = start; k < i; ++k) {
      auto & row = result.rows[k];
      const double t = double(lower.axis_coordinate - row.axis_coordinate) /
        (lower.axis_coordinate - upper.axis_coordinate);
      const double cross = lc + t * (uc - lc);
      row.center = result.horizontal ? row.axis_coordinate : cross;
      row.y = result.horizontal ? static_cast<int>(std::round(cross)) : row.axis_coordinate;
      row.cross_width = static_cast<int>(std::round(lw + t * (uw - lw)));
      row.inferred = true; row.reason = "occlusion estimate (not observed blue)";
      ++result.inferred_rows;
    }
  }
  cv::cvtColor(filtered, result.gray, cv::COLOR_BGR2GRAY);
  if (config.show_canny || config.use_occlusion_edges) {
    int low = config.canny_low, high = config.canny_high;
    if (config.canny_auto_threshold) {
      int histogram[256] = {};
      for (int y = result.roi_top; y <= result.roi_bottom; ++y) {
        const auto * pixels = result.gray.ptr<uint8_t>(y);
        for (int x = 0; x < width; ++x) {++histogram[pixels[x]];}
      }
      const int half = width * (result.roi_bottom - result.roi_top + 1) / 2;
      int accumulated = 0, median = 0;
      while (median < 255 && accumulated + histogram[median] <= half) {
        accumulated += histogram[median++];
      }
      low = std::clamp(static_cast<int>((1 - config.canny_sigma) * median), 0, 254);
      high = std::clamp(static_cast<int>((1 + config.canny_sigma) * median), low + 1, 255);
    }
    cv::Canny(result.gray, result.edges, low, high);
  } else {result.edges = cv::Mat::zeros(height, width, CV_8UC1);}
  // Tape ends alone also have Canny edges. Require outside continuation or dark
  // internal contrast in a geometrically supported corridor, not arbitrary edges.
  cv::Mat evidence_edges = cv::Mat::zeros(height, width, CV_8UC1);
  if (config.use_occlusion_edges) {
    const auto collect_edges = [&](const ScanRow & row) {
      const int radius = row.cross_width;
      for (int a = -config.row_step; a <= config.row_step; ++a) {
        for (int b = -radius; b <= radius; ++b) {
          const int x = static_cast<int>(std::round(row.center)) + (result.horizontal ? a : b);
          const int y = row.y + (result.horizontal ? b : a);
          if (x >= 0 && x < width && y >= 0 && y < height &&
            result.edges.at<uint8_t>(y, x) && !color_evidence.at<uint8_t>(y, x)) {
            const bool outside = std::abs(b) > row.cross_width / 2 + 2;
            // Inside-tape edges need dark contrast against BOTH side backgrounds.
            // A floor-coloured tape break alone must not count as an object.
            bool internal = false;
            if (std::abs(b) < row.cross_width / 2 - 2) {
              const int cx = cvRound(row.center), cy = row.y;
              const cv::Point p1(cx + (result.horizontal ? a : -radius),
                cy + (result.horizontal ? -radius : a));
              const cv::Point p2(cx + (result.horizontal ? a : radius),
                cy + (result.horizontal ? radius : a));
              if (p1.x >= 0 && p1.x < width && p2.x >= 0 && p2.x < width &&
                p1.y >= 0 && p1.y < height && p2.y >= 0 && p2.y < height) {
                const int background = std::min(result.gray.at<uint8_t>(p1),
                  result.gray.at<uint8_t>(p2));
                internal = result.gray.at<uint8_t>(y, x) + 30 < background;
              }
            }
            if (outside || internal) {evidence_edges.at<uint8_t>(y, x) = 255;}
          }
        }
      }
    };
    for (const auto & row : result.rows) {
      if (row.inferred) {collect_edges(row);}
    }
    for (const auto & corridor : corridors) {
      if (corridor.horizontal != result.horizontal) {continue;}
      const double gap = corridor.lower[0] - corridor.upper[0];
      for (double axis = corridor.upper[0] + config.row_step;
        axis < corridor.lower[0]; axis += config.row_step)
      {
        const double t = (axis - corridor.upper[0]) / gap;
        const double cross = corridor.upper[1] + t * (corridor.lower[1] - corridor.upper[1]);
        ScanRow row;
        row.center = result.horizontal ? axis : cross;
        row.y = cvRound(result.horizontal ? cross : axis);
        row.cross_width = cvRound(corridor.width);
        collect_edges(row);
      }
    }
  }
  result.occlusion_edge_pixels = cv::countNonZero(evidence_edges);
  result.occlusion_suspected = result.occlusion_edge_pixels >= 8;
  auto target = result.rows.end();
  for (auto it = result.rows.begin(); it != result.rows.end(); ++it) {
    if ((it->valid || it->inferred) && (target == result.rows.end() ||
      std::abs(it->y - result.target_y) < std::abs(target->y - result.target_y))) {target = it;}
  }
  if (result.horizontal && target != result.rows.end()) {
    int first = width, last = 0;
    for (const auto & row : result.rows) {
      if (row.valid) {first = std::min(first, row.axis_coordinate); last = std::max(last, row.axis_coordinate);}
    }
    const double middle = (first + last) / 2.0;
    for (auto it = result.rows.begin(); it != result.rows.end(); ++it) {
      if ((it->valid || it->inferred) && std::abs(it->center - middle) < std::abs(target->center - middle)) {
        target = it;
      }
    }
    result.target_y = target->y;
  }
  const bool target_available = target != result.rows.end() &&
    std::abs(target->y - result.target_y) <= 2 * config.row_step;
  // A finite visible strip still has a measurable centre. If the fixed lookahead
  // misses it, place the target within the observed span, never extrapolate a gap.
  if (!target_available && target != result.rows.end()) {
    int bottom_y = -1, top_y = height;
    for (const auto & row : result.rows) {
      if (row.valid) {bottom_y = std::max(bottom_y, row.y); top_y = std::min(top_y, row.y);}
    }
    const double visible_y = bottom_y - config.lookahead_ratio * (bottom_y - top_y);
    for (auto it = result.rows.begin(); it != result.rows.end(); ++it) {
      if ((it->valid || it->inferred) && std::abs(it->y - visible_y) < std::abs(target->y - visible_y)) {
        target = it;
      }
    }
    result.target_fallback = true;
  }
  bool bottom_anchor = false;
  for (const auto & row : result.rows) {
    bottom_anchor = bottom_anchor || (row.valid && row.y >=
      result.roi_bottom - config.bottom_anchor_tolerance_rows * config.row_step);
  }
  result.bottom_connected = bottom_anchor;
  result.lost = result.valid_rows < config.min_valid_rows || target == result.rows.end() ||
    (config.require_bottom_anchor && !bottom_anchor);
  if (blue_interference) {result.lost_reason = "too_much_blue";}
  else if (shape_rejected) {result.lost_reason = "shape_rejected";}
  else if (!cv::countNonZero(result.mask)) {
    result.lost_reason = result.blue_fraction > 0 ? "insufficient_valid_rows" : "no_blue_pixels";
  }
  else if (config.require_bottom_anchor && !bottom_anchor) {result.lost_reason = "no_bottom_anchor";}
  else if (result.valid_rows < config.min_valid_rows) {result.lost_reason = "insufficient_valid_rows";}
  if (!result.lost) {result.lost_reason.clear();}
  if (blue_interference) {result.state = "BLUE_INTERFERENCE";}
  else if (result.lost) {result.state = result.valid_rows ? "PARTIAL / LOST" : "LOST";}
  else if (result.occlusion_suspected) {result.state = "OCCLUDED";}
  else if (result.inferred_rows) {result.state = "GAP_ESTIMATED";}
  else if (result.horizontal) {result.state = "HORIZONTAL";}
  else {result.state = result.rejected_rows ? "PARTIAL" : "TRACKING";}
  result.confidence = result.rows.empty() ? 0 : double(result.valid_rows) / result.rows.size();
  if (!result.lost) {
    result.target_x = target->center;
    result.target_estimated = target->inferred;
    result.target_y = target->y;
    result.error_pixels = result.target_x - (width - 1) / 2.0;
    result.error_normalized = result.error_pixels / ((width - 1) / 2.0);
  }

  cv::Mat overlay = result.image.clone(), mask_bgr, evidence, edge_bgr;
  cv::cvtColor(result.mask, mask_bgr, cv::COLOR_GRAY2BGR);
  evidence = result.image.clone();
  evidence.setTo(cv::Scalar(0, 165, 255), result.weak_mask);
  evidence.setTo(cv::Scalar(0, 220, 0), result.strong_mask);
  cv::cvtColor(result.edges, edge_bgr, cv::COLOR_GRAY2BGR);
  cv::rectangle(overlay, {0, result.roi_top}, {width - 1, result.roi_bottom},
    cv::Scalar(180, 180, 180), 1);
  cv::line(overlay, {width / 2, 28}, {width / 2, height - 1}, cv::Scalar(180, 180, 180));
  const ScanRow * previous = nullptr;
  for (const auto & row : result.rows) {
    if (row.valid) {
      cv::circle(overlay, row.left_point, 2, cv::Scalar(255, 255, 0), cv::FILLED);
      cv::circle(overlay, row.right_point, 2, cv::Scalar(255, 0, 255), cv::FILLED);
      const cv::Point center(static_cast<int>(std::round(row.center)), row.y);
      cv::circle(overlay, center, 2, cv::Scalar(0, 230, 255), cv::FILLED);
      if (previous && previous->valid) {
        cv::line(overlay, {static_cast<int>(std::round(previous->center)), previous->y},
          center, cv::Scalar(0, 230, 255), 2);
      }
      if (row.reacquired) {cv::circle(overlay, {row.seed, row.y}, 3, cv::Scalar(0, 170, 255));}
    } else if (row.inferred) {
      // Orange dots are estimates, unlike yellow observed centre points.
      cv::circle(overlay, {static_cast<int>(std::round(row.center)), row.y}, 3,
        cv::Scalar(0, 140, 255), cv::FILLED);
    } else {
      cv::line(overlay, {std::max(0, row.seed - 4), row.y - 3},
        {std::min(width - 1, row.seed + 4), row.y + 3}, cv::Scalar(0, 0, 255), 2);
    }
    previous = &row;
  }
  if (!result.lost) {
    cv::line(overlay, {0, result.target_y}, {width - 1, result.target_y}, cv::Scalar(0, 160, 0));
    cv::circle(overlay, {static_cast<int>(result.target_x), result.target_y}, 7,
      cv::Scalar(0, 255, 0), 2);
    cv::line(overlay, {static_cast<int>((width - 1) / 2.0), result.target_y},
      {static_cast<int>(std::round(result.target_x)), result.target_y},
      cv::Scalar(0, 255, 255), 2);
  }
  label(overlay, "OVERLAY: yellow observed / orange estimated / cyan L / magenta R");
  label(mask_bgr, "BLUE MASK: white = selected blue; black = background");
  label(evidence, "COLOR candidates ONLY: green=strong / orange=weak (not final tape)");
  label(edge_bgr, config.show_canny ?
    (config.canny_auto_threshold ? "AUTO CANNY: crossing edges support POSSIBLE occlusion" :
    "CANNY: crossing edges support POSSIBLE occlusion") : "CANNY display off; edge evidence may still run");
  if (config.show_canny) {edge_bgr.setTo(cv::Scalar(0, 140, 255), evidence_edges);}
  else {edge_bgr.setTo(cv::Scalar(0, 0, 0));}
  cv::Mat top, bottom, grid;
  cv::hconcat(overlay, mask_bgr, top);
  cv::hconcat(evidence, edge_bgr, bottom);
  cv::vconcat(top, bottom, grid);
  result.debug = cv::Mat::zeros(grid.rows + 64, grid.cols, CV_8UC3);
  grid.copyTo(result.debug(cv::Rect(0, 64, grid.cols, grid.rows)));
  const auto color = result.lost ? cv::Scalar(80, 80, 255) : cv::Scalar(80, 230, 80);
  cv::putText(result.debug, "State: " + result.state + "  valid=" +
    std::to_string(result.valid_rows) + "/" + std::to_string(result.rows.size()) +
    "  error=" + (result.lost ? std::string("INVALID") : std::to_string(result.error_pixels) + "px"),
    {10, 23}, cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv::LINE_AA);
  cv::putText(result.debug, result.lost ? "Reason: " + result.lost_reason :
    std::string("Bottom connected: ") + (result.bottom_connected ? "yes" : "no") +
    " / target: " + (result.target_estimated ? "ESTIMATED" :
    (result.target_fallback ? "visible segment" : "lookahead")) +
    " / occlusion: " + (result.occlusion_suspected ? "POSSIBLE (edges)" :
    (result.inferred_rows ? "unconfirmed gap" : "no estimate")),
    {10, 47}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  return result;
}
}  // namespace camer_diver
