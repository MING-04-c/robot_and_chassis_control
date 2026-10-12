#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace camer_diver
{
struct LineConfig
{
  int processing_width = 640;
  int hue_low = 100, hue_high = 130, saturation_min = 70, value_min = 18;
  bool illumination_support = true;
  int weak_saturation_min = 12, weak_value_min = 8, weak_seed_pixels = 20;
  double blue_excess_min = 0.025;
  double strong_blue_excess_min = 0.12;
  int weak_support_distance = 6;
  std::string blur_type = "median";
  int blur_kernel = 5, open_kernel = 3, close_kernel = 3;
  int min_component_area = 50;
  double roi_top = 0.0, roi_bottom = 1.0;
  int row_step = 6, min_line_width = 4, search_radius = 80, max_center_jump = 50;
  int initial_search_radius = 640, bottom_anchor_tolerance_rows = 3;
  bool require_bottom_anchor = false;
  double max_width_fraction = 0.45, max_blue_fraction = 0.65;
  int max_missing_rows = 4, min_valid_rows = 4;
  double min_line_aspect = 2.0, min_row_fill = 0.75, min_row_coverage = 0.65;
  double max_shape_center_step = 2.0;
  double min_core_fraction = 0.45, max_width_spread = 2.5;
  int max_occlusion_gap = 72;
  double max_occlusion_offset = 12.0;
  bool use_occlusion_edges = true;
  bool allow_reacquire = true, show_canny = false;
  int canny_low = 50, canny_high = 150;
  bool canny_auto_threshold = true;
  double canny_sigma = 0.33;
  double lookahead_ratio = 0.60;
  void validate() const;
};

struct ScanRow
{
  int y = 0, seed = 0, left = 0, right = 0;
  double center = 0;
  bool valid = false, clipped = false, reacquired = false;
  bool inferred = false;
  int cross_width = 0, axis_coordinate = 0;
  cv::Point left_point, right_point;
  std::string reason;
};

struct CandidateDiagnostic
{
  int id = 0;
  bool accepted = false, horizontal = false, context_supported = false, retained = false;
  double aspect = 0, core_fraction = 0, row_coverage = 0, width_spread = 0;
  std::string rejection;
};

struct JoinDiagnostic
{
  int candidate = 0, anchor = 0;
  double gap = 0, gap_limit = 0, width_ratio = 0, alignment_error = 0;
  std::string rejection;
};

struct LineResult
{
  cv::Mat image, mask, strong_mask, weak_mask, gray, edges, debug;
  std::vector<ScanRow> rows;
  std::vector<CandidateDiagnostic> candidates;
  std::vector<JoinDiagnostic> joins;
  std::string state = "LOST";
  std::string lost_reason;
  bool lost = true;
  bool bottom_connected = false, target_fallback = false;
  bool target_estimated = false;
  int inferred_rows = 0;
  bool horizontal = false, occlusion_suspected = false;
  int occlusion_edge_pixels = 0;
  int selected_candidate = 0;
  int valid_rows = 0, rejected_rows = 0, roi_top = 0, roi_bottom = 0, target_y = 0;
  double target_x = 0, error_pixels = 0, error_normalized = 0, confidence = 0;
  double blue_fraction = 0;
};

// Stateless per-frame scan: no stale center is carried between camera frames.
LineResult track_blue_line(const cv::Mat & bgr, const LineConfig & config);
}  // namespace camer_diver
