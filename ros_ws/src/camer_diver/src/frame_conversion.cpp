#include "camer_diver/frame_conversion.hpp"

#include <linux/videodev2.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace camer_diver
{
cv::Mat to_bgr(const Frame & frame)
{
  if (!frame.width || !frame.height || frame.width > 4096 || frame.height > 4096 ||
    frame.data.empty())
  {
    throw std::runtime_error("Empty frame or invalid dimensions");
  }
  cv::Mat bgr;
  if (frame.pixel_format == V4L2_PIX_FMT_MJPEG) {
    bgr = cv::imdecode(frame.data, cv::IMREAD_COLOR);
    if (bgr.empty() || bgr.cols != static_cast<int>(frame.width) ||
      bgr.rows != static_cast<int>(frame.height))
    {
      throw std::runtime_error("Invalid JPEG or decoded dimensions differ from camera format");
    }
  } else if (frame.pixel_format == V4L2_PIX_FMT_YUYV ||
    frame.pixel_format == V4L2_PIX_FMT_UYVY)
  {
    const size_t row_bytes = static_cast<size_t>(frame.width) * 2;
    const size_t required = static_cast<size_t>(frame.height - 1) * frame.stride + row_bytes;
    if (frame.width % 2 || frame.stride < row_bytes || frame.data.size() < required) {
      throw std::runtime_error("Truncated YUV frame or invalid row stride");
    }
    cv::Mat yuv(static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC2,
      const_cast<uint8_t *>(frame.data.data()), frame.stride);
    cv::cvtColor(yuv, bgr, frame.pixel_format == V4L2_PIX_FMT_YUYV ?
      cv::COLOR_YUV2BGR_YUY2 : cv::COLOR_YUV2BGR_UYVY);
  } else {
    throw std::runtime_error("Unsupported pixel format");
  }
  return bgr;
}
}  // namespace camer_diver
