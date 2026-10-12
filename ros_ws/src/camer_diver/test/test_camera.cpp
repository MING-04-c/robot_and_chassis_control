#include "camer_diver/frame_conversion.hpp"
#include <gtest/gtest.h>
#include <linux/videodev2.h>
#include <opencv2/imgcodecs.hpp>

using camer_diver::Frame;

TEST(FrameConversion, YuyvRespectsPadding)
{
  Frame frame;
  frame.width = 2;
  frame.height = 2;
  frame.stride = 8;
  frame.pixel_format = V4L2_PIX_FMT_YUYV;
  // White first row, black second row; poison padding must not become pixels.
  frame.data = {235, 128, 235, 128, 255, 0, 255, 0, 16, 128, 16, 128};
  const auto bgr = camer_diver::to_bgr(frame);
  EXPECT_EQ(bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 255, 255));
  EXPECT_EQ(bgr.at<cv::Vec3b>(1, 0), cv::Vec3b(0, 0, 0));
  frame.data.resize(11);
  EXPECT_THROW(camer_diver::to_bgr(frame), std::runtime_error);
}

TEST(FrameConversion, UyvyWhite)
{
  Frame frame;
  frame.width = 2;
  frame.height = 1;
  frame.stride = 4;
  frame.pixel_format = V4L2_PIX_FMT_UYVY;
  frame.data = {128, 235, 128, 235};
  EXPECT_EQ(camer_diver::to_bgr(frame).at<cv::Vec3b>(0, 1), cv::Vec3b(255, 255, 255));
}

TEST(FrameConversion, MjpegDecodeAndDimensions)
{
  Frame frame;
  frame.width = 16;
  frame.height = 8;
  frame.pixel_format = V4L2_PIX_FMT_MJPEG;
  cv::Mat original(8, 16, CV_8UC3, cv::Scalar(20, 80, 180));
  ASSERT_TRUE(cv::imencode(".jpg", original, frame.data));
  const auto decoded = camer_diver::to_bgr(frame);
  EXPECT_EQ(decoded.size(), original.size());
  EXPECT_NEAR(decoded.at<cv::Vec3b>(0, 0)[2], 180, 3);
  frame.width = 32;
  EXPECT_THROW(camer_diver::to_bgr(frame), std::runtime_error);
  frame.data = {0, 1, 2, 3};
  EXPECT_THROW(camer_diver::to_bgr(frame), std::runtime_error);
}

TEST(FrameConversion, RejectsOddWidthAndInvalidStride)
{
  Frame frame;
  frame.width = 3;
  frame.height = 1;
  frame.stride = 6;
  frame.pixel_format = V4L2_PIX_FMT_YUYV;
  frame.data.resize(6);
  EXPECT_THROW(camer_diver::to_bgr(frame), std::runtime_error);
  frame.width = 2;
  frame.stride = 2;
  EXPECT_THROW(camer_diver::to_bgr(frame), std::runtime_error);
}

TEST(Capture, RejectsInvalidConfigAndMissingDevice)
{
  camer_diver::V4L2Capture capture;
  EXPECT_THROW(capture.open_device("/dev/null", 640, 480, 30, "H264"), std::invalid_argument);
  EXPECT_THROW(capture.open_device("/dev/null", 0, 480, 30, "MJPG"), std::invalid_argument);
  EXPECT_THROW(capture.open_device("/dev/camer_diver_missing_test_device", 640, 480, 30,
    "MJPG"), std::runtime_error);
  // /dev/null is a character device but not a V4L2 camera; cleanup must be safe.
  EXPECT_THROW(capture.open_device("/dev/null", 640, 480, 30, "MJPG"), std::runtime_error);
  Frame frame;
  EXPECT_THROW(capture.read_frame(frame), std::runtime_error);
  capture.close_device();
  capture.close_device();
}
