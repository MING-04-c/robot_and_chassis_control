#include "orbbec_camera/v54_capture.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>

namespace orbbec_camera {
namespace {
using Json = nlohmann::json;
uint64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
Json intrinsic(const OBCameraIntrinsic& k) {
  return {{"fx", k.fx}, {"fy", k.fy}, {"cx", k.cx}, {"cy", k.cy},
          {"width", k.width}, {"height", k.height}};
}
Json distortion(const OBCameraDistortion& d) {
  return {{"k1", d.k1}, {"k2", d.k2}, {"k3", d.k3}, {"k4", d.k4},
          {"k5", d.k5}, {"k6", d.k6}, {"p1", d.p1}, {"p2", d.p2}};
}
Json timestamps(const std::shared_ptr<ob::Frame>& f) {
  return {{"timeStamp", f->timeStamp()}, {"timeStampUs", f->timeStampUs()},
          {"systemTimeStamp", f->systemTimeStamp()},
          {"systemTimeStampUs", f->systemTimeStampUs()}};
}
}  // namespace

V54Capture::V54Capture(rclcpp::Node* node, std::shared_ptr<ob::Device> device,
                     bool valid_config)
    : valid_config_(valid_config), serial_(device->getDeviceInfo()->serialNumber()),
      session_(serial_ + "-" + std::to_string(nowMs())) {
  service_ = node->create_service<Capture>(
      "capture_frame", [this](const Capture::Request::SharedPtr,
                               const Capture::Response::SharedPtr response) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!valid_config_ || stopped_ || pending_) {
          response->message = "Capture unavailable: requires unflipped 640x480 MJPG/30, "
              "640x400 Y11/30 and software D2C; device must be running and idle";
          return;
        }
        pending_ = true;
        warmup_ = 5;
        previous_color_ = previous_depth_ = 0;
        request_ms_ = nowMs();
        last_error_.clear();
        result_ = Capture::Response();
        // 与原采集器等待上限相同：30 × 750 ms；SDK 回调在独立线程中响应。
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(22500);
        while (pending_ && !stopped_ && rclcpp::ok()) {
          ready_.wait_for(lock, std::chrono::milliseconds(100));
          if (std::chrono::steady_clock::now() >= deadline && pending_) {
            pending_ = false;
            result_.message = "No valid fresh SDK FrameSet within 22500 ms: " + last_error_;
          }
        }
        if (!rclcpp::ok()) {
          stopped_ = true;
          pending_ = false;
        }
        if (stopped_) {
          result_.success = false;
          result_.message = "Camera stopped";
        }
        *response = std::move(result_);
      });
}

void V54Capture::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = true;
  pending_ = false;
  ready_.notify_all();
}

void V54Capture::onFrameSet(const std::shared_ptr<ob::FrameSet>& frames,
                            ob::Pipeline& pipeline) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_ || stopped_) {
    return;
  }
  try {
    auto color = frames->colorFrame();
    auto depth = frames->depthFrame();
    if (!color || !depth) {
      return;
    }
    auto ct = color->systemTimeStamp();
    auto dt = depth->systemTimeStamp();
    auto delta = ct > dt ? ct - dt : dt - ct;
    if (ct < request_ms_ || dt < request_ms_ || ct <= previous_color_ ||
        dt <= previous_depth_) {
      last_error_ = "Duplicate or stale timestamps";
      return;
    }
    previous_color_ = ct;
    previous_depth_ = dt;
    if (warmup_ > 0) {
      --warmup_;
      return;
    }
    if (delta > 100 || nowMs() > ct + 1000 || nowMs() > dt + 1000) {
      last_error_ = "Timestamp diagnostic/freshness limit exceeded";
      return;
    }
    if (color->width() != 640 || color->height() != 480 ||
        depth->width() != 640 || depth->height() != 480 ||
        color->format() != OB_FORMAT_MJPG || depth->dataSize() != 640 * 480 * 2) {
      throw std::runtime_error("Actual FrameSet profile or SW-D2C output differs from V5.4");
    }
    auto scale = depth->getValueScale();
    if (!std::isfinite(scale) || scale <= 0 || scale >= 100) {
      throw std::runtime_error("SDK depth value scale is invalid");
    }
    // 必须在原始 SDK 数据乘比例、翻转或转换为 ROS 毫米图之前复制。
    Capture::Response response;
    response.color.header.stamp = rclcpp::Time(
        static_cast<int64_t>(color->systemTimeStampUs()) * 1000);
    response.color.header.frame_id = "camera_color_optical_frame";
    response.color.format = "jpeg";
    const auto* bytes = static_cast<const uint8_t*>(color->data());
    response.color.data.assign(bytes, bytes + color->dataSize());
    response.depth_raw.header = response.color.header;
    response.depth_raw.header.stamp = rclcpp::Time(
        static_cast<int64_t>(depth->systemTimeStampUs()) * 1000);
    response.depth_raw.width = 640;
    response.depth_raw.height = 480;
    response.depth_raw.encoding = "16UC1";
    response.depth_raw.is_bigendian = false;
    response.depth_raw.step = 640 * 2;
    bytes = static_cast<const uint8_t*>(depth->data());
    response.depth_raw.data.assign(bytes, bytes + depth->dataSize());
    auto p = pipeline.getCameraParam();
    Json raw = {{"rgbIntrinsic", intrinsic(p.rgbIntrinsic)},
                {"depthIntrinsic", intrinsic(p.depthIntrinsic)},
                {"rgbDistortion", distortion(p.rgbDistortion)},
                {"depthDistortion", distortion(p.depthDistortion)},
                {"transform", {{"rot", std::vector<float>(p.transform.rot, p.transform.rot + 9)},
                               {"trans", std::vector<float>(p.transform.trans, p.transform.trans + 3)}}},
                {"isMirrored", p.isMirrored}};
    const auto version = std::to_string(ob::Version::getMajor()) + "." +
        std::to_string(ob::Version::getMinor()) + "." +
        std::to_string(ob::Version::getPatch());
    Json metadata = {{"sdk_raw_camera_param", raw}, {"depth_scale_mm_per_raw", scale},
        {"sdk_api", "Orbbec C++ SDK v1"}, {"sdk_version", version},
        {"device_serial", serial_}, {"session_id", session_},
        {"actual_align_mode", "ALIGN_D2C_SW_MODE"},
        {"sdk_color_timestamp_fields_raw", timestamps(color)},
        {"sdk_depth_timestamp_fields_raw", timestamps(depth)},
        {"rgb_system_timestamp_raw", ct}, {"depth_system_timestamp_raw", dt},
        {"system_timestamp_delta_raw", delta}, {"system_timestamp_unit", "ms"},
        {"color_format_id", static_cast<int>(color->format())},
        {"depth_format_id", static_cast<int>(depth->format())},
        {"color_frame_index", color->index()}, {"depth_frame_index", depth->index()},
        {"warmup_complete_framesets", 5}};
    response.metadata_json = metadata.dump();
    response.success = true;
    response.message = "One fresh SDK FrameSet, original SW-D2C uint16 and SDK parameters";
    result_ = std::move(response);
    pending_ = false;
    ready_.notify_all();
  } catch (const std::exception& error) {
    // 参数/配置错误不能通过改变输出格式掩盖，直接返回失败。
    result_.success = false;
    result_.message = error.what();
    pending_ = false;
    ready_.notify_all();
  }
}
}  // namespace orbbec_camera
