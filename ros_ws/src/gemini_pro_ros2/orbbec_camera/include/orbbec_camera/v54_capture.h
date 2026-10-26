#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <libobsensor/ObSensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <orbbec_camera_msgs/srv/get_capture_frame.hpp>

namespace orbbec_camera {
// 按需截取同一 FrameSet 的原始数据；平时不复制、解码或发布额外图像。
class V54Capture {
 public:
  using Capture = orbbec_camera_msgs::srv::GetCaptureFrame;
  V54Capture(rclcpp::Node* node, std::shared_ptr<ob::Device> device, bool valid_config);
  void onFrameSet(const std::shared_ptr<ob::FrameSet>& frames, ob::Pipeline& pipeline);
  void cancel();

 private:
  std::mutex mutex_;
  std::condition_variable ready_;
  bool pending_ = false;
  bool stopped_ = false;
  bool valid_config_;
  int warmup_ = 0;
  uint64_t previous_color_ = 0;
  uint64_t previous_depth_ = 0;
  uint64_t request_ms_ = 0;
  std::string serial_;
  std::string session_;
  std::string last_error_;
  Capture::Response result_;
  rclcpp::Service<Capture>::SharedPtr service_;
};
}  // namespace orbbec_camera
