#include "camer_diver/line_tracking.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <iomanip>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <stdexcept>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace camer_diver
{
// This node observes the line; it never drives the robot.
class LineTrackerNode : public rclcpp::Node
{
public:
  LineTrackerNode() : Node("line_tracker")
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto topic = declare_parameter<std::string>("input_topic", "image_raw", fixed);
    show_window_ = declare_parameter<bool>("show_window", false, fixed);
    timeout_ = declare_parameter<double>("frame_timeout_sec", 0.5, fixed);
    if (!std::isfinite(timeout_) || timeout_ < 0.1 || timeout_ > 10.0) {
      throw std::invalid_argument("frame_timeout_sec must be 0.1..10");
    }
#define DECLARE_INT(field) config_.field = static_cast<int>(declare_parameter<int>(#field, config_.field))
#define DECLARE_VALUE(field) config_.field = declare_parameter(#field, config_.field)
    DECLARE_INT(processing_width);
    DECLARE_INT(hue_low); DECLARE_INT(hue_high);
    DECLARE_INT(saturation_min); DECLARE_INT(value_min);
    DECLARE_VALUE(illumination_support);
    DECLARE_INT(weak_saturation_min); DECLARE_INT(weak_value_min); DECLARE_INT(weak_seed_pixels);
    DECLARE_VALUE(blue_excess_min);
    DECLARE_VALUE(strong_blue_excess_min); DECLARE_INT(weak_support_distance);
    DECLARE_VALUE(blur_type); DECLARE_INT(blur_kernel);
    DECLARE_INT(open_kernel); DECLARE_INT(close_kernel);
    DECLARE_INT(min_component_area);
    DECLARE_VALUE(roi_top); DECLARE_VALUE(roi_bottom);
    DECLARE_INT(row_step); DECLARE_INT(min_line_width);
    DECLARE_INT(search_radius); DECLARE_INT(max_center_jump);
    DECLARE_INT(initial_search_radius); DECLARE_INT(bottom_anchor_tolerance_rows);
    DECLARE_VALUE(require_bottom_anchor);
    DECLARE_VALUE(max_width_fraction); DECLARE_VALUE(max_blue_fraction);
    DECLARE_INT(max_missing_rows); DECLARE_INT(min_valid_rows);
    DECLARE_VALUE(min_line_aspect); DECLARE_VALUE(min_row_fill);
    DECLARE_VALUE(min_row_coverage); DECLARE_VALUE(max_shape_center_step);
    DECLARE_VALUE(min_core_fraction); DECLARE_VALUE(max_width_spread);
    DECLARE_INT(max_occlusion_gap); DECLARE_VALUE(max_occlusion_offset);
    DECLARE_VALUE(use_occlusion_edges);
    DECLARE_VALUE(allow_reacquire); DECLARE_VALUE(show_canny);
    DECLARE_INT(canny_low); DECLARE_INT(canny_high);
    DECLARE_VALUE(canny_auto_threshold); DECLARE_VALUE(canny_sigma);
    DECLARE_VALUE(lookahead_ratio);
#undef DECLARE_INT
#undef DECLARE_VALUE
    config_.validate();
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult response;
        auto candidate = config_;
        try {
          for (const auto & p : parameters) {
#define SET_INT(field) if (p.get_name() == #field) {const auto v = p.as_int(); \
  if (v < -1000000 || v > 1000000) {throw std::invalid_argument("integer out of range");} \
  candidate.field = static_cast<int>(v); continue;}
#define SET_DOUBLE(field) if (p.get_name() == #field) {candidate.field = p.as_double(); continue;}
#define SET_BOOL(field) if (p.get_name() == #field) {candidate.field = p.as_bool(); continue;}
            SET_INT(processing_width); SET_INT(hue_low); SET_INT(hue_high);
            SET_INT(saturation_min); SET_INT(value_min);
            SET_BOOL(illumination_support);
            SET_INT(weak_saturation_min); SET_INT(weak_value_min); SET_INT(weak_seed_pixels);
            SET_DOUBLE(blue_excess_min);
            SET_DOUBLE(strong_blue_excess_min); SET_INT(weak_support_distance);
            if (p.get_name() == "blur_type") {candidate.blur_type = p.as_string(); continue;}
            SET_INT(blur_kernel); SET_INT(open_kernel); SET_INT(close_kernel);
            SET_INT(min_component_area); SET_DOUBLE(roi_top); SET_DOUBLE(roi_bottom);
            SET_INT(row_step); SET_INT(min_line_width); SET_INT(search_radius);
            SET_INT(max_center_jump); SET_DOUBLE(max_width_fraction); SET_DOUBLE(max_blue_fraction);
            SET_INT(initial_search_radius); SET_INT(bottom_anchor_tolerance_rows);
            SET_BOOL(require_bottom_anchor);
            SET_INT(max_missing_rows); SET_INT(min_valid_rows);
            SET_DOUBLE(min_line_aspect); SET_DOUBLE(min_row_fill);
            SET_DOUBLE(min_row_coverage); SET_DOUBLE(max_shape_center_step);
            SET_DOUBLE(min_core_fraction); SET_DOUBLE(max_width_spread);
            SET_INT(max_occlusion_gap); SET_DOUBLE(max_occlusion_offset);
            SET_BOOL(use_occlusion_edges);
            SET_BOOL(allow_reacquire); SET_BOOL(show_canny);
            SET_INT(canny_low); SET_INT(canny_high); SET_DOUBLE(lookahead_ratio);
            SET_BOOL(canny_auto_threshold); SET_DOUBLE(canny_sigma);
#undef SET_INT
#undef SET_DOUBLE
#undef SET_BOOL
          }
          candidate.validate();
          config_ = candidate;
          response.successful = true;
        } catch (const std::exception & error) {
          response.successful = false;
          response.reason = error.what();
        }
        return response;
      });
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("line/debug_image", qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::Image>("line/mask", qos);
    edges_pub_ = create_publisher<sensor_msgs::msg::Image>("line/edges", qos);
    path_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("line/centerline", qos);
    estimated_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("line/estimated_centerline", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("line/status", 10);
    lost_pub_ = create_publisher<std_msgs::msg::Bool>("line/lost", 10);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {on_image(message);});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {watchdog();});
    if (show_window_) {
      if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
        RCLCPP_WARN(get_logger(), "No desktop display. Use line/debug_image on another computer.");
        show_window_ = false;
      } else {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, 1100, 650);
        cv::setMouseCallback(window_name_, [](int event, int x, int y, int, void * user) {
          if (event == cv::EVENT_LBUTTONDOWN) {
            static_cast<LineTrackerNode *>(user)->sample_blue(x, y);
          }
        }, this);
      }
    }
    last_receipt_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Blue-line tracker subscribed to %s. Live view: line/debug_image",
      topic.c_str());
  }

  ~LineTrackerNode() override
  {
    if (show_window_) {cv::destroyWindow(window_name_);}
  }

private:
  void publish_state(const LineResult * result, const std::string & state, double milliseconds)
  {
    const bool lost = !result || result->lost;
    std_msgs::msg::Bool flag;
    flag.data = lost;
    lost_pub_->publish(flag);
    std::ostringstream json;
    json << std::fixed << std::setprecision(4)
         << "{\"state\":\"" << state << "\",\"lost\":" << (lost ? "true" : "false")
         << ",\"valid_rows\":" << (result ? result->valid_rows : 0)
         << ",\"rejected_rows\":" << (result ? result->rejected_rows : 0)
         << ",\"inferred_rows\":" << (result ? result->inferred_rows : 0)
         << ",\"target_estimated\":" << (result && !lost && result->target_estimated ? "true" : "false")
         << ",\"orientation\":\"" << (result && result->horizontal ? "horizontal" : "vertical") << "\""
         << ",\"occlusion_suspected\":" << (result && !lost && result->occlusion_suspected ? "true" : "false")
         << ",\"occlusion_edge_pixels\":" << (result ? result->occlusion_edge_pixels : 0)
         << ",\"selected_candidate\":" << (result ? result->selected_candidate : 0)
         << ",\"processing_over_budget\":" <<
      (milliseconds > timeout_ * 1000 ? "true" : "false")
         << ",\"valid_row_fraction\":" << (result ? result->confidence : 0)
         << ",\"blue_fraction\":" << (result ? result->blue_fraction : 0)
         << ",\"bottom_connected\":" << (result && result->bottom_connected ? "true" : "false")
         << ",\"target_fallback\":" << (result && result->target_fallback ? "true" : "false")
         << ",\"lost_reason\":\"" << (result ? result->lost_reason : state) << "\""
         << ",\"first_rejection\":\"";
    if (result) {
      for (const auto & row : result->rows) {
        if (!row.valid) {json << row.reason; break;}
      }
    }
    json << "\""
         << ",\"processing_ms\":" << milliseconds
         << ",\"image_width\":" << (result ? result->image.cols : 0)
         << ",\"image_height\":" << (result ? result->image.rows : 0)
         << ",\"target_x\":";
    if (lost) {json << "null,\"target_y\":null,\"error_pixels\":null,\"error_normalized\":null";}
    else {
      json << result->target_x << ",\"target_y\":" << result->target_y
           << ",\"error_pixels\":" << result->error_pixels
           << ",\"error_normalized\":" << result->error_normalized;
    }
    json << ",\"candidates\":[";
    if (result) {
      bool first = true;
      for (const auto & candidate : result->candidates) {
        if (!first) {json << ",";}
        first = false;
        json << "{\"id\":" << candidate.id << ",\"accepted\":" <<
          (candidate.accepted ? "true" : "false") << ",\"rejection\":\"" << candidate.rejection <<
          "\",\"context_supported\":" << (candidate.context_supported ? "true" : "false") <<
          ",\"retained\":" << (candidate.retained ? "true" : "false") <<
          ",\"aspect\":" << candidate.aspect << ",\"core_fraction\":" << candidate.core_fraction <<
          ",\"section_coverage\":" << candidate.row_coverage << ",\"width_spread\":" << candidate.width_spread << "}";
      }
    }
    json << "],\"joins\":[";
    if (result) {
      bool first = true;
      for (const auto & join : result->joins) {
        if (!first) {json << ",";}
        first = false;
        json << "{\"candidate\":" << join.candidate << ",\"anchor\":" << join.anchor <<
          ",\"gap\":" << join.gap << ",\"gap_limit\":" << join.gap_limit <<
          ",\"width_ratio\":" << join.width_ratio << ",\"alignment_error\":" << join.alignment_error <<
          ",\"rejection\":\"" << join.rejection << "\"}";
      }
    }
    json << "]}";
    std_msgs::msg::String status;
    status.data = json.str();
    status_pub_->publish(status);
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const auto start = std::chrono::steady_clock::now();
    last_receipt_ = start;
    received_ = true;
    last_header_ = message->header;
    try {
      const auto bgr = cv_bridge::toCvShare(message, "bgr8");
      auto result = track_blue_line(bgr->image, config_);
      last_sample_image_ = result.image;
      const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      const bool demo = message->header.frame_id == "synthetic_demo";
      cv::putText(result.debug, demo ? "SYNTHETIC DEMO" : "CAMERA INPUT", {result.debug.cols - 180, 23},
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
      if (!sample_text_.empty()) {
        cv::rectangle(result.debug, {result.debug.cols / 2, result.debug.rows - 25,
          result.debug.cols / 2, 25}, cv::Scalar(20, 20, 20), cv::FILLED);
        cv::putText(result.debug, sample_text_, {result.debug.cols / 2 + 8, result.debug.rows - 7},
          cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
      }
      debug_pub_->publish(*cv_bridge::CvImage(message->header, "bgr8", result.debug).toImageMsg());
      mask_pub_->publish(*cv_bridge::CvImage(message->header, "mono8", result.mask).toImageMsg());
      if (config_.show_canny) {
        edges_pub_->publish(*cv_bridge::CvImage(message->header, "mono8", result.edges).toImageMsg());
      }
      geometry_msgs::msg::PolygonStamped path;
      path.header = message->header;
      geometry_msgs::msg::PolygonStamped estimates;
      estimates.header = message->header;
      // Pixel coordinates in the resized processing image, not 3D optical-frame coordinates.
      // Visible partial strips publish their measured path; never extrapolate missing rows.
      if (!result.lost) {
        for (const auto & row : result.rows) {
          if (!row.valid && !row.inferred) {continue;}
          geometry_msgs::msg::Point32 point;
          point.x = static_cast<float>(row.center);
          point.y = static_cast<float>(row.y);
          point.z = 0;
          if (row.valid) {path.polygon.points.push_back(point);}
          else {estimates.polygon.points.push_back(point);}
        }
      }
      path_pub_->publish(path);
      estimated_pub_->publish(estimates);
      publish_state(&result, result.state, milliseconds);
      last_debug_ = result.debug;
      if (show_window_) {display(result.debug);}
      last_processing_finished_ = std::chrono::steady_clock::now();
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Image processing failed: %s", error.what());
      publish_state(nullptr, "PROCESSING_ERROR", 0);
      geometry_msgs::msg::PolygonStamped empty;
      empty.header = last_header_;
      path_pub_->publish(empty);
      estimated_pub_->publish(empty);
      last_debug_ = cv::Mat::zeros(480, 900, CV_8UC3);
      last_sample_image_.release();
      cv::putText(last_debug_, "PROCESSING ERROR - LOST", {15, 40},
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(40, 40, 255), 2);
      debug_pub_->publish(*cv_bridge::CvImage(last_header_, "bgr8", last_debug_).toImageMsg());
      if (show_window_) {display(last_debug_);}
      last_processing_finished_ = std::chrono::steady_clock::now();
    }
  }

  void display(const cv::Mat & frame)
  {
    cv::imshow(window_name_, frame);
    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q') {rclcpp::shutdown();}
  }

  void sample_blue(int x, int display_y)
  {
    const int y = display_y - 64;
    if (last_sample_image_.empty() || x < 0 || x >= last_sample_image_.cols ||
      y < 28 || y >= last_sample_image_.rows ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_receipt_).count() > timeout_)
    {
      return;
    }
    const int left = std::max(0, x - 3), top = std::max(0, y - 3);
    const int right = std::min(last_sample_image_.cols, x + 4);
    const int bottom = std::min(last_sample_image_.rows, y + 4);
    cv::Mat hsv;
    cv::cvtColor(last_sample_image_(cv::Rect(left, top, right - left, bottom - top)),
      hsv, cv::COLOR_BGR2HSV);
    std::vector<int> h, s, v;
    for (int row = 0; row < hsv.rows; ++row) {
      for (int col = 0; col < hsv.cols; ++col) {
        const auto pixel = hsv.at<cv::Vec3b>(row, col);
        const auto bgr = last_sample_image_.at<cv::Vec3b>(top + row, left + col);
        const double excess = (2.0 * bgr[0] - bgr[1] - bgr[2]) /
          (double(bgr[0]) + bgr[1] + bgr[2] + 1);
        if (pixel[1] >= 70 && pixel[2] >= 8 && pixel[0] >= 90 && pixel[0] <= 140 &&
          bgr[0] > bgr[1] && bgr[0] > bgr[2] && excess >= 0.12) {
          h.push_back(pixel[0]); s.push_back(pixel[1]); v.push_back(pixel[2]);
        }
      }
    }
    if (h.size() < static_cast<size_t>((hsv.total() + 1) / 2)) {
      sample_text_ = "No usable blue color here (white glare / background / too dark)";
      RCLCPP_WARN(get_logger(), "%s", sample_text_.c_str());
      return;
    }
    const auto median = [](std::vector<int> values) {
        std::sort(values.begin(), values.end()); return values[values.size() / 2];
      };
    const int hue = median(h), saturation = median(s), value = median(v);
    const auto response = set_parameters_atomically({
      rclcpp::Parameter("hue_low", std::max(90, hue - 12)),
      rclcpp::Parameter("hue_high", std::min(140, hue + 12)),
      rclcpp::Parameter("saturation_min", std::max(70, saturation / 2)),
      rclcpp::Parameter("value_min", std::max(8, value / 3)),
    });
    sample_text_ = "Sample H=" + std::to_string(hue) + " S=" + std::to_string(saturation) +
      " V=" + std::to_string(value) + (response.successful ? " -> thresholds updated" : " -> rejected");
    RCLCPP_INFO(get_logger(), "%s", sample_text_.c_str());
  }

  void watchdog()
  {
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_processing_finished_).count();
    if (received_ && age <= timeout_) {
      if (show_window_) {cv::waitKey(1);}
      return;
    }
    publish_state(nullptr, received_ ? "STALE_FRAME" : "NO_FRAME", 0);
    geometry_msgs::msg::PolygonStamped empty;
    empty.header = last_header_;
    path_pub_->publish(empty);
    estimated_pub_->publish(empty);
    cv::Mat stale = last_debug_.empty() ? cv::Mat::zeros(480, 900, CV_8UC3) : last_debug_.clone();
    cv::rectangle(stale, {0, 0, stale.cols, 64}, cv::Scalar(20, 20, 20), cv::FILLED);
    cv::putText(stale, received_ ? "STALE FRAME - LOST" : "WAITING FOR CAMERA IMAGE - LOST",
      {10, 35}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(40, 40, 255), 2);
    // Preserve the original image timestamp: stale imagery must not look newly acquired.
    debug_pub_->publish(*cv_bridge::CvImage(last_header_, "bgr8", stale).toImageMsg());
    if (show_window_) {display(stale);}
  }

  LineConfig config_;
  bool show_window_ = false, received_ = false;
  double timeout_ = 0.5;
  const std::string window_name_ = "Blue line tracking - live ROS2";
  std::chrono::steady_clock::time_point last_receipt_;
  std::chrono::steady_clock::time_point last_processing_finished_;
  std_msgs::msg::Header last_header_;
  cv::Mat last_debug_, last_sample_image_;
  std::string sample_text_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_, mask_pub_, edges_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr estimated_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lost_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace camer_diver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {rclcpp::spin(std::make_shared<camer_diver::LineTrackerNode>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("line_tracker"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
