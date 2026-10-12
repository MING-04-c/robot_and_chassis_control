#pragma once

#include "camer_diver/v4l2_capture.hpp"
#include <opencv2/core.hpp>

namespace camer_diver
{
// Validates byte counts/row padding and returns an owned BGR image.
cv::Mat to_bgr(const Frame & frame);
}  // namespace camer_diver
