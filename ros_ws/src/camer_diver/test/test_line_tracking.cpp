#include "camer_diver/line_tracking.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
cv::Mat stripe(cv::Scalar floor = {240, 240, 240}, int x = 320)
{
  cv::Mat image(480, 640, CV_8UC3, floor);
  cv::rectangle(image, {x - 15, 0, 31, 480}, cv::Scalar(220, 80, 20), cv::FILLED);
  return image;
}
}  // namespace

TEST(BlueLineTracking, WhiteGrayAndWarmFloors)
{
  camer_diver::LineConfig config;
  for (const auto & floor : {cv::Scalar(240, 240, 240), cv::Scalar(90, 90, 90),
    cv::Scalar(150, 190, 215)})
  {
    const auto result = camer_diver::track_blue_line(stripe(floor), config);
    EXPECT_FALSE(result.lost);
    EXPECT_NEAR(result.target_x, 320, 1);
    EXPECT_GT(result.valid_rows, 40);
    EXPECT_EQ(result.debug.cols, 1280);
    EXPECT_EQ(result.debug.rows, 1024);
    EXPECT_NEAR(result.error_normalized, 0, 0.01);
  }
}

TEST(BlueLineTracking, RightwardOffsetAndReacquisition)
{
  camer_diver::LineConfig config;
  const auto result = camer_diver::track_blue_line(stripe({240, 240, 240}, 360), config);
  EXPECT_FALSE(result.lost);
  EXPECT_GT(result.error_normalized, 0.10);
  EXPECT_TRUE(result.rows.front().reacquired);
  config.allow_reacquire = false;
  EXPECT_TRUE(camer_diver::track_blue_line(stripe({240, 240, 240}, 360), config).lost);
}

TEST(BlueLineTracking, EmptyBlueFloorAndClippedLineAreLost)
{
  camer_diver::LineConfig config;
  cv::Mat empty(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  EXPECT_TRUE(camer_diver::track_blue_line(empty, config).lost);
  cv::Mat blue(480, 640, CV_8UC3, cv::Scalar(220, 80, 20));
  auto interference = camer_diver::track_blue_line(blue, config);
  EXPECT_TRUE(interference.lost);
  EXPECT_EQ(interference.state, "BLUE_INTERFERENCE");
  cv::rectangle(empty, {310, 0, 330, 480}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto clipped = camer_diver::track_blue_line(empty, config);
  EXPECT_TRUE(clipped.lost);
  EXPECT_EQ(cv::countNonZero(clipped.mask), 0);  // No usable full cross-section.
  EXPECT_FALSE(clipped.rows.front().valid);
}

TEST(BlueLineTracking, LongGapSelectsOneVisibleSegmentWithoutBridging)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {0, 300, 640, 65}, cv::Scalar(240, 240, 240), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(result.lost);
  for (const auto & row : result.rows) {
    if (row.y >= 300) {EXPECT_FALSE(row.valid);}
  }
  EXPECT_FALSE(result.bottom_connected);
}

TEST(BlueLineTracking, DistinctSegmentsAreNotPooled)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {520, 0, 25, 480}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(result.lost);
  EXPECT_NEAR(result.target_x, 320, 1);
  EXPECT_LE(result.rows.front().right, 335);
}

TEST(BlueLineTracking, NoiseAndCannyDoNotDefineCenterline)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  for (int y = 170; y < 440; y += 20) {
    cv::rectangle(image, {100, y, 4, 4}, cv::Scalar(220, 80, 20), cv::FILLED);
  }
  const auto without = camer_diver::track_blue_line(image, config);
  config.show_canny = true;
  const auto with = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(with.lost);
  EXPECT_EQ(cv::countNonZero(with.mask != without.mask), 0);
  EXPECT_GT(cv::countNonZero(with.edges), 0);
  EXPECT_EQ(with.mask.at<uint8_t>(170, 101), 0);
}

TEST(BlueLineTracking, WideCornerIsRejected)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {305, 270, 305, 40}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  bool saw_rejected_corner = false;
  for (const auto & row : result.rows) {
    if (row.y >= 273 && row.y <= 306) {
      EXPECT_FALSE(row.valid);
      saw_rejected_corner = saw_rejected_corner || row.reason == "too wide / bend / junction";
    }
  }
  EXPECT_TRUE(saw_rejected_corner);
  EXPECT_EQ(result.inferred_rows, 0);  // A broad junction must not become an occlusion estimate.
}

TEST(BlueLineTracking, InvalidConfigurationAndImageRejected)
{
  camer_diver::LineConfig config;
  config.hue_low = 150; config.hue_high = 100;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = camer_diver::LineConfig();
  config.blur_kernel = 4;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = camer_diver::LineConfig();
  EXPECT_THROW(camer_diver::track_blue_line(cv::Mat(), config), std::invalid_argument);
}

TEST(BlueLineTracking, DarkVisibleBlueSurvivesShadow)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  // V=28 is visible but was below the previous default V=40.
  cv::rectangle(image, {305, 240, 31, 140}, cv::Scalar(28, 10, 3), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(result.lost);
  EXPECT_EQ(result.mask.at<uint8_t>(300, 320), 255);
  EXPECT_NEAR(result.target_x, 320, 1);
}

TEST(BlueLineTracking, ConnectedWashedOutBlueRecoveredButIsolatedTintRejected)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  // Low S ~18, still measurably blue rather than clipped white.
  cv::rectangle(image, {305, 240, 31, 10}, cv::Scalar(250, 240, 232), cv::FILLED);
  cv::rectangle(image, {100, 200, 25, 100}, cv::Scalar(250, 240, 232), cv::FILLED);
  auto result = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(result.lost);
  EXPECT_EQ(result.strong_mask.at<uint8_t>(245, 320), 0);
  EXPECT_EQ(result.weak_mask.at<uint8_t>(245, 320), 255);
  EXPECT_EQ(result.mask.at<uint8_t>(245, 320), 255);
  EXPECT_EQ(result.mask.at<uint8_t>(240, 110), 0);
  config.illumination_support = false;
  const auto partial = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(partial.lost);
  EXPECT_EQ(partial.mask.at<uint8_t>(245, 320), 0);
}

TEST(BlueLineTracking, GreyTintRejectedEvenWithRelaxedHsv)
{
  camer_diver::LineConfig config;
  config.hue_low = 90; config.hue_high = 140; config.saturation_min = 12;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {100, 90, 100, 200}, cv::Scalar(130, 126, 120), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(result.lost);
  EXPECT_EQ(cv::countNonZero(result.strong_mask), 0);
  EXPECT_EQ(cv::countNonZero(result.mask), 0);
}

TEST(BlueLineTracking, WeakBackgroundCannotFloodFromSmallBlueSeed)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {100, 70, 180, 300}, cv::Scalar(250, 240, 232), cv::FILLED);
  cv::rectangle(image, {100, 70, 25, 100}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_EQ(result.weak_mask.at<uint8_t>(250, 200), 255);
  EXPECT_EQ(result.mask.at<uint8_t>(250, 200), 0);
  EXPECT_LT(cv::countNonZero(result.mask), 4000);
  for (const auto & row : result.rows) {
    if (row.valid) {EXPECT_LT(row.y, 180);}
  }
}

TEST(BlueLineTracking, MorphologyCannotCreateBlueInsideWhiteHole)
{
  camer_diver::LineConfig config;
  config.blur_type = "none";
  config.close_kernel = 7;
  auto image = stripe();
  cv::rectangle(image, {318, 100, 3, 300}, cv::Scalar(235, 235, 235), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_EQ(result.mask.at<uint8_t>(200, 319), 0);
  for (const auto & row : result.rows) {
    if (row.valid && row.y > 100 && row.y < 400) {
      EXPECT_TRUE(row.right < 318 || row.left > 320);
    }
  }
}

TEST(BlueLineTracking, FullWhiteGlareIsNotInventedAsBlue)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {305, 240, 31, 140}, cv::Scalar(255, 255, 255), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(result.lost);  // A visible finite piece still has a measured offset.
  EXPECT_EQ(result.mask.at<uint8_t>(300, 320), 0);
  EXPECT_EQ(result.weak_mask.at<uint8_t>(300, 320), 0);
}

TEST(BlueLineTracking, LargerInitialSearchFindsLineAwayFromCenter)
{
  camer_diver::LineConfig config;
  auto image = stripe({240, 240, 240}, 510);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(result.lost);
  EXPECT_NEAR(result.target_x, 510, 1);
  config.initial_search_radius = 80;
  const auto rejected = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(rejected.lost);
  EXPECT_EQ(rejected.lost_reason, "insufficient_valid_rows");
}

TEST(BlueLineTracking, BottomGapAllowedUnlessExplicitlyRequired)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {0, 450, 640, 30}, cv::Scalar(240, 240, 240), cv::FILLED);
  const auto short_gap = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(short_gap.rows.front().valid);
  EXPECT_FALSE(short_gap.lost);
  cv::rectangle(image, {0, 410, 640, 70}, cv::Scalar(240, 240, 240), cv::FILLED);
  const auto visible = camer_diver::track_blue_line(image, config);
  EXPECT_FALSE(visible.lost);
  EXPECT_FALSE(visible.bottom_connected);
  config.require_bottom_anchor = true;
  const auto strict = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(strict.lost);
  EXPECT_EQ(strict.lost_reason, "no_bottom_anchor");
}

TEST(BlueLineTracking, FiniteTapeAnywhereHasSignedOffset)
{
  camer_diver::LineConfig config;
  for (int x : {35, 150, 320, 510, 605}) {
    // Missing top, bottom, or both; also a strip entirely in the former excluded ROI.
    for (const auto & span : {cv::Range(0, 160), cv::Range(320, 480),
      cv::Range(170, 290), cv::Range(35, 110)})
    {
      cv::Mat image(480, 640, CV_8UC3, cv::Scalar(240, 240, 240));
      cv::rectangle(image, {x - 10, span.start, 21, span.end - span.start},
        cv::Scalar(220, 80, 20), cv::FILLED);
      const auto result = camer_diver::track_blue_line(image, config);
      ASSERT_FALSE(result.lost) << "x=" << x << " top=" << span.start;
      EXPECT_NEAR(result.target_x, x, 1);
      EXPECT_NEAR(result.error_pixels, x - 319.5, 1);
      EXPECT_GE(result.target_y, span.start);
      EXPECT_LT(result.target_y, span.end);
      EXPECT_TRUE(result.lost_reason.empty());
      for (const auto & row : result.rows) {
        if (row.valid) {EXPECT_GE(row.y, span.start); EXPECT_LT(row.y, span.end);}
      }
    }
  }
}

TEST(BlueLineTracking, SaturatedTapePreferredOverPaleBackground)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(240, 240, 240));
  cv::rectangle(image, {100, 70, 25, 180}, cv::Scalar(180, 35, 15), cv::FILLED);
  // Pale blue background satisfies the strong HSV cutoff but is not the tape.
  cv::rectangle(image, {390, 260, 100, 180}, cv::Scalar(230, 185, 185), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_NEAR(result.target_x, 112, 1);
  EXPECT_LT(result.error_pixels, 0);
  EXPECT_EQ(result.mask.at<uint8_t>(300, 440), 0);
}

TEST(BlueLineTracking, TooShortVisibleTapeDoesNotProduceOffset)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(240, 240, 240));
  cv::rectangle(image, {100, 210, 21, 10}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(result.lost);
  EXPECT_EQ(result.lost_reason, "insufficient_valid_rows");
}

TEST(BlueLineTracking, CompactBluePatchRejectedAsNotTape)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
  cv::rectangle(image, {295, 270, 50, 55}, cv::Scalar(60, 20, 5), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_GT(cv::countNonZero(result.strong_mask), 100);
  EXPECT_TRUE(result.lost);
  EXPECT_EQ(result.lost_reason, "shape_rejected");
  EXPECT_EQ(result.valid_rows, 0);
  EXPECT_EQ(cv::countNonZero(result.mask), 0);
}

TEST(BlueLineTracking, ConnectedPorousNoiseDoesNotDefineLine)
{
  camer_diver::LineConfig config;
  config.blur_type = "none"; config.close_kernel = 1;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
  // Connected mesh has enough area and height, but its interior is mainly background.
  for (int x = 290; x <= 350; x += 8) {
    cv::line(image, {x, 210}, {x, 310}, cv::Scalar(60, 20, 5), 2);
  }
  for (int y = 210; y <= 310; y += 8) {
    cv::line(image, {290, y}, {346, y}, cv::Scalar(60, 20, 5), 2);
  }
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(result.lost);
  EXPECT_EQ(result.valid_rows, 0);
  EXPECT_EQ(cv::countNonZero(result.mask), 0);
}

TEST(BlueLineTracking, PartialCurvedTapeRetainsMeasuredCenter)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  const std::vector<cv::Point> curve{{100, 50}, {130, 130}, {220, 220},
    {260, 300}, {230, 400}};
  cv::polylines(image, curve, false, cv::Scalar(220, 80, 20), 20);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_FALSE(result.bottom_connected);
  EXPECT_LT(result.error_pixels, 0);
  EXPECT_GT(result.valid_rows, 35);
}

TEST(BlueLineTracking, NoiseRejectedBeforeChoosingRealTape)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
  cv::rectangle(image, {280, 260, 70, 70}, cv::Scalar(100, 25, 5), cv::FILLED);
  cv::rectangle(image, {100, 120, 21, 160}, cv::Scalar(100, 25, 5), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_NEAR(result.target_x, 110, 1);
  EXPECT_EQ(result.mask.at<uint8_t>(290, 300), 0);
}

TEST(BlueLineTracking, ThinSerratedCombNoiseRejected)
{
  camer_diver::LineConfig config;
  config.blur_type = "none";
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(35, 35, 35));
  // A connected long noisy ribbon with thin branches, not a solid tape interior.
  cv::line(image, {320, 140}, {320, 320}, cv::Scalar(70, 20, 5), 3);
  for (int y = 140; y < 320; y += 6) {
    cv::line(image, {310, y}, {333, y}, cv::Scalar(70, 20, 5), 2);
  }
  const auto result = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(result.lost);
  EXPECT_EQ(cv::countNonZero(result.mask), 0);
}

TEST(BlueLineTracking, ShortFullOcclusionInferredWithoutPaintingMask)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {290, 180, 60, 26}, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_EQ(result.state, "OCCLUDED");
  EXPECT_GT(result.inferred_rows, 0);
  EXPECT_TRUE(result.target_estimated);
  EXPECT_NEAR(result.target_x, 320, 1);
  EXPECT_EQ(result.mask.at<uint8_t>(191, 320), 0);
  bool above = false, below = false;
  for (const auto & row : result.rows) {
    above = above || (row.valid && row.y < 180);
    below = below || (row.valid && row.y >= 206);
    if (row.inferred) {EXPECT_FALSE(row.valid); EXPECT_NEAR(row.center, 320, 1);}
  }
  EXPECT_TRUE(above); EXPECT_TRUE(below);
}

TEST(BlueLineTracking, PartialSideOcclusionDoesNotBiasMidpoint)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {305, 180, 16, 24}, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_GT(result.inferred_rows, 0);
  EXPECT_TRUE(result.target_estimated);
  EXPECT_NEAR(result.target_x, 320, 1);
  EXPECT_EQ(result.mask.at<uint8_t>(191, 312), 0);
}

TEST(BlueLineTracking, LargeOrMisalignedGapIsNotInferred)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {290, 160, 60, 80}, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto long_gap = camer_diver::track_blue_line(image, config);
  EXPECT_EQ(long_gap.inferred_rows, 0);
  image.setTo(cv::Scalar(235, 235, 235));
  cv::rectangle(image, {305, 206, 31, 160}, cv::Scalar(220, 80, 20), cv::FILLED);
  cv::rectangle(image, {365, 30, 31, 150}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto misaligned = camer_diver::track_blue_line(image, config);
  EXPECT_EQ(misaligned.inferred_rows, 0);
}

TEST(BlueLineTracking, OcclusionNeedsBothEndpointsAndCanBeDisabled)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {290, 0, 60, 206}, cv::Scalar(15, 15, 15), cv::FILLED);
  EXPECT_EQ(camer_diver::track_blue_line(image, config).inferred_rows, 0);
  image = stripe();
  cv::rectangle(image, {290, 180, 60, 26}, cv::Scalar(15, 15, 15), cv::FILLED);
  config.max_occlusion_gap = 0;
  EXPECT_EQ(camer_diver::track_blue_line(image, config).inferred_rows, 0);
}

TEST(BlueLineTracking, AutomaticAndFixedCannyRemainDiagnostic)
{
  camer_diver::LineConfig config;
  const auto image = stripe();
  const auto baseline = camer_diver::track_blue_line(image, config);
  config.show_canny = true;
  const auto automatic = camer_diver::track_blue_line(image, config);
  config.canny_auto_threshold = false;
  const auto fixed = camer_diver::track_blue_line(image, config);
  EXPECT_GT(cv::countNonZero(automatic.edges), 0);
  EXPECT_EQ(cv::countNonZero(automatic.mask != baseline.mask), 0);
  EXPECT_EQ(cv::countNonZero(fixed.mask != baseline.mask), 0);
  EXPECT_NEAR(automatic.target_x, fixed.target_x, 0.01);
  config.canny_auto_threshold = true;
  cv::Mat black(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  EXPECT_TRUE(camer_diver::track_blue_line(black, config).lost);
  config.canny_sigma = 1.0;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(BlueLineTracking, RotatedTapeIsNotRejectedByHorizontalChordWidth)
{
  camer_diver::LineConfig config;
  for (float angle : {0.0F, 25.0F, 45.0F, 65.0F, 90.0F}) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    const cv::RotatedRect tape({310, 240}, {260, 28}, angle);
    cv::Point2f corners[4]; tape.points(corners);
    std::vector<cv::Point> polygon;
    for (const auto & corner : corners) {polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));}
    cv::fillConvexPoly(image, polygon, cv::Scalar(220, 80, 20));
    const auto result = camer_diver::track_blue_line(image, config);
    ASSERT_FALSE(result.lost) << "angle=" << angle;
    EXPECT_GT(result.valid_rows, 25);
    EXPECT_GT(result.selected_candidate, 0);
  }
}

TEST(BlueLineTracking, HorizontalTapeSelectedInsteadOfBlueTintedEdge)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {100, 290, 340, 25}, cv::Scalar(220, 80, 20), cv::FILLED);
  cv::rectangle(image, {590, 45, 8, 150}, cv::Scalar(140, 85, 80), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_TRUE(result.horizontal);
  EXPECT_EQ(result.state, "HORIZONTAL");
  EXPECT_NEAR(result.target_x, 270, config.row_step);
  EXPECT_NEAR(result.target_y, 302, 1);
  EXPECT_EQ(result.mask.at<uint8_t>(100, 594), 0);
}

TEST(BlueLineTracking, CrossingObjectHasEdgeEvidenceButWhiteBreakDoesNot)
{
  camer_diver::LineConfig config;
  auto image = stripe();
  cv::rectangle(image, {290, 180, 60, 26}, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto object = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(object.lost);
  EXPECT_TRUE(object.occlusion_suspected);
  EXPECT_GE(object.occlusion_edge_pixels, 8);
  image = stripe();
  cv::rectangle(image, {305, 180, 31, 26}, cv::Scalar(240, 240, 240), cv::FILLED);
  const auto break_only = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(break_only.lost);
  EXPECT_GT(break_only.inferred_rows, 0);
  EXPECT_FALSE(break_only.occlusion_suspected);
  EXPECT_EQ(break_only.state, "GAP_ESTIMATED");
  config.use_occlusion_edges = false;
  EXPECT_FALSE(camer_diver::track_blue_line(image, config).occlusion_suspected);
}

TEST(BlueLineTracking, HorizontalCableGapHasSeparateEstimatedPoints)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {100, 290, 340, 25}, cv::Scalar(220, 80, 20), cv::FILLED);
  cv::rectangle(image, {255, 275, 24, 60}, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_TRUE(result.horizontal);
  EXPECT_GT(result.inferred_rows, 0);
  EXPECT_TRUE(result.occlusion_suspected);
  EXPECT_EQ(result.mask.at<uint8_t>(302, 267), 0);
  for (const auto & row : result.rows) {
    if (row.inferred) {EXPECT_NEAR(row.y, 302, 1); EXPECT_FALSE(row.valid);}
  }
}

TEST(BlueLineTracking, ShapeRejectionsReportNumericDiagnostics)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
  cv::rectangle(image, {295, 270, 50, 55}, cv::Scalar(60, 20, 5), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_FALSE(result.candidates.front().accepted);
  EXPECT_EQ(result.candidates.front().rejection, "aspect");
  EXPECT_GT(result.candidates.front().aspect, 0);
  EXPECT_GT(result.candidates.front().core_fraction, 0);
}

TEST(BlueLineTracking, DiagonalTapeCrossingObjectUsesLocalTangents)
{
  camer_diver::LineConfig config;
  for (float angle : {25.0F, 45.0F, 65.0F}) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    const auto rectangle = [&image](cv::Size2f size, float direction, cv::Scalar color) {
        cv::Point2f corners[4]; cv::RotatedRect({310, 240}, size, direction).points(corners);
        std::vector<cv::Point> polygon;
        for (const auto & p : corners) {polygon.emplace_back(cvRound(p.x), cvRound(p.y));}
        cv::fillConvexPoly(image, polygon, color);
      };
    rectangle({260, 28}, angle, cv::Scalar(220, 80, 20));
    rectangle({75, 20}, angle + 90, cv::Scalar(15, 15, 15));
    const auto result = camer_diver::track_blue_line(image, config);
    ASSERT_FALSE(result.lost) << "angle=" << angle;
    EXPECT_GT(result.inferred_rows, 0) << "angle=" << angle;
    EXPECT_TRUE(result.occlusion_suspected) << "angle=" << angle;
  }
}

TEST(BlueLineTracking, ShortCableCutFragmentCanUseQualifiedTapeContext)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {305, 110, 31, 240}, cv::Scalar(220, 80, 20), cv::FILLED);
  cv::rectangle(image, {290, 160, 60, 20}, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_GT(result.inferred_rows, 0);
  bool supported = false;
  for (const auto & candidate : result.candidates) {supported = supported || candidate.context_supported;}
  EXPECT_TRUE(supported);
}

TEST(BlueLineTracking, SlantedCablesRetainMultipleTapePieces)
{
  camer_diver::LineConfig config;
  for (int shift : {-3, 0, 3}) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
    cv::line(image, {370, 100}, {305, 350}, cv::Scalar(220, 80, 20), 32);
    cv::line(image, {285, 190 + shift}, {400, 250 + shift}, cv::Scalar(15, 15, 15), 10);
    cv::line(image, {280, 297 + shift}, {395, 325 + shift}, cv::Scalar(15, 15, 15), 9);
    const auto result = camer_diver::track_blue_line(image, config);
    std::string diagnostics;
    for (const auto & c : result.candidates) {
      diagnostics += " candidate=" + std::to_string(c.id) + ":" + c.rejection +
        " aspect=" + std::to_string(c.aspect) + " retained=" + std::to_string(c.retained);
    }
    for (const auto & j : result.joins) {
      diagnostics += " join=" + std::to_string(j.candidate) + "/" + std::to_string(j.anchor) +
        ":" + j.rejection + " gap=" + std::to_string(j.gap) +
        " alignment=" + std::to_string(j.alignment_error);
    }
    SCOPED_TRACE(diagnostics);
    ASSERT_FALSE(result.lost);
    EXPECT_NE(result.mask.at<uint8_t>(135, 361), 0) << shift;
    EXPECT_NE(result.mask.at<uint8_t>(340, 308), 0) << shift;
    EXPECT_GT(result.inferred_rows, 0) << shift;
    EXPECT_TRUE(result.occlusion_suspected) << shift;
  }
}

TEST(BlueLineTracking, InternalRoundObjectHasIndependentEdgeEvidence)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {295, 100, 51, 280}, cv::Scalar(220, 80, 20), cv::FILLED);
  cv::circle(image, {320, 220}, 25, cv::Scalar(15, 15, 15), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.lost);
  EXPECT_GT(result.inferred_rows, 0);
  EXPECT_TRUE(result.occlusion_suspected);
  EXPECT_NE(result.mask.at<uint8_t>(150, 320), 0);
  EXPECT_NE(result.mask.at<uint8_t>(300, 320), 0);
  EXPECT_EQ(result.mask.at<uint8_t>(220, 320), 0);
  config.max_occlusion_gap = 18;
  const auto bounded = camer_diver::track_blue_line(image, config);
  EXPECT_EQ(bounded.inferred_rows, 0);
  EXPECT_TRUE(bounded.occlusion_suspected);
}

TEST(BlueLineTracking, JoinFailuresAndFinalRetentionAreExplicit)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {290, 260, 31, 140}, cv::Scalar(220, 80, 20), cv::FILLED);
  cv::rectangle(image, {350, 110, 31, 130}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto result = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(result.joins.empty());
  EXPECT_EQ(result.joins.front().rejection, "tangent_mismatch");
  EXPECT_GT(result.joins.front().alignment_error, config.max_occlusion_offset);
  int retained = 0;
  for (const auto & candidate : result.candidates) {retained += candidate.retained;}
  EXPECT_EQ(retained, 1);
  EXPECT_EQ(result.inferred_rows, 0);
}

TEST(BlueLineTracking, FewSectionFragmentNeedsQualifiedAlignedContext)
{
  camer_diver::LineConfig config;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(235, 235, 235));
  cv::rectangle(image, {310, 100, 21, 20}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto alone = camer_diver::track_blue_line(image, config);
  EXPECT_TRUE(alone.lost);
  cv::rectangle(image, {310, 140, 21, 190}, cv::Scalar(220, 80, 20), cv::FILLED);
  const auto joined = camer_diver::track_blue_line(image, config);
  ASSERT_FALSE(joined.lost);
  EXPECT_NE(joined.mask.at<uint8_t>(108, 320), 0);
  EXPECT_GT(joined.inferred_rows, 0);
  bool contextual = false;
  for (const auto & c : joined.candidates) {contextual = contextual || (c.context_supported && c.retained);}
  EXPECT_TRUE(contextual);
}
