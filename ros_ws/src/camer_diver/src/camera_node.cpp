#include "camer_diver/frame_conversion.hpp"
#include "camer_diver/v4l2_capture.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <linux/videodev2.h>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>

namespace camer_diver
{
class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    fixed.description = "Startup configuration; restart the node to change this parameter";
    const auto device = declare_parameter<std::string>("video_device", "/dev/video0", fixed);
    const auto format = declare_parameter<std::string>("pixel_format", "MJPG", fixed);
    const auto width = declare_parameter<int>("image_width", 640, fixed);
    const auto height = declare_parameter<int>("image_height", 480, fixed);
    const auto fps = declare_parameter<int>("framerate", 30, fixed);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame", fixed);
    const auto name = declare_parameter<std::string>("camera_name", "usb_camera", fixed);
    const auto url = declare_parameter<std::string>("camera_info_url", "", fixed);
    publish_raw_ = declare_parameter<bool>("publish_raw", true, fixed);
    publish_compressed_ = declare_parameter<bool>("publish_compressed", true, fixed);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85, fixed);
    if (width < 1 || width > 4096 || height < 1 || height > 4096 || fps < 1 || fps > 240 ||
      jpeg_quality_ < 1 || jpeg_quality_ > 100 || frame_id_.empty() ||
      (!publish_raw_ && !publish_compressed_))
    {
      throw std::invalid_argument("Invalid camera parameters: dimensions 1..4096, "
              "framerate 1..240, jpeg_quality 1..100, nonempty frame_id and an enabled output required");
    }

    capture_.open_device(device, width, height, fps, format);
    RCLCPP_INFO(get_logger(), "Camera: %s (%s)", capture_.description().c_str(), device.c_str());
    if (capture_.width() != static_cast<uint32_t>(width) ||
      capture_.height() != static_cast<uint32_t>(height))
    {
      RCLCPP_WARN(get_logger(), "Requested %dx%d; publishing negotiated %ux%u",
        static_cast<int>(width), static_cast<int>(height), capture_.width(), capture_.height());
    }
    info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, name);
    if (!url.empty() && !info_manager_->loadCameraInfo(url)) {
      throw std::runtime_error("Failed to load camera_info_url=" + url);
    }
    if (!info_manager_->isCalibrated()) {
      RCLCPP_WARN(get_logger(), "Camera is uncalibrated; CameraInfo K[0] will be zero");
    }
    const auto qos = rclcpp::SensorDataQoS();
    if (publish_raw_) {
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    }
    if (publish_compressed_) {
      compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", qos);
    }
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    last_frame_ = std::chrono::steady_clock::now();
    // Nonblocking poll: the executor remains responsive to SIGINT and services.
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {capture_and_publish();});
  }

private:
  void capture_and_publish()
  {
    if (!capture_.read_frame(frame_)) {
      if (std::chrono::steady_clock::now() - last_frame_ > std::chrono::seconds(5)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No valid camera frame for 5 seconds; check USB connection and format");
      }
      return;
    }
    last_frame_ = std::chrono::steady_clock::now();
    std_msgs::msg::Header header;
    // ROS receive time, not the V4L2 monotonic clock. Use the same stamp for both outputs.
    header.stamp = now();
    header.frame_id = frame_id_;

    cv::Mat bgr;
    try {
      if (publish_raw_ || (publish_compressed_ && frame_.pixel_format != V4L2_PIX_FMT_MJPEG)) {
        bgr = to_bgr(frame_);
      }
      if (publish_raw_) {
        raw_pub_->publish(*cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg());
      }
      if (publish_compressed_) {
        sensor_msgs::msg::CompressedImage compressed;
        compressed.header = header;
        if (frame_.pixel_format == V4L2_PIX_FMT_MJPEG) {
          compressed.format = "jpeg";
          compressed.data = frame_.data;
        } else {
          compressed.format = "bgr8; jpeg compressed bgr8";
          if (!cv::imencode(".jpg", bgr, compressed.data,
            {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_}))
          {
            throw std::runtime_error("JPEG encoding failed");
          }
        }
        compressed_pub_->publish(compressed);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Dropping invalid frame: %s", error.what());
      return;
    }
    auto info = info_manager_->getCameraInfo();
    if (info.k[0] != 0.0 && (info.width != frame_.width || info.height != frame_.height)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Calibration resolution does not match frame; publishing uncalibrated CameraInfo");
      info = sensor_msgs::msg::CameraInfo();
    }
    info.header = header;
    info.width = frame_.width;
    info.height = frame_.height;
    info_pub_->publish(info);
  }

  V4L2Capture capture_;
  Frame frame_;
  std::string frame_id_;
  bool publish_raw_;
  bool publish_compressed_;
  int jpeg_quality_;
  std::chrono::steady_clock::time_point last_frame_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace camer_diver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<camer_diver::CameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
