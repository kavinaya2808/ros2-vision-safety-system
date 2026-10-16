// Copyright 2026 Kavinaya. MIT License.
//
// ColorDetector tests on synthetic images: no camera, no Gazebo, fully deterministic.

#include <gtest/gtest.h>

#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vision_safety_core/color_detector.hpp"

using vision_safety::ColorDetector;
using vision_safety::ColorDetectorConfig;
using vision_safety::HsvRange;

namespace
{

constexpr int kW = 640;
constexpr int kH = 480;

cv::Mat grey_image()
{
  return cv::Mat(kH, kW, CV_8UC3, cv::Scalar(128, 128, 128));
}

// OpenCV HSV (H 0..179) -> BGR colour for drawing.
cv::Scalar hsv_colour(int h, int s, int v)
{
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto px = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(px[0], px[1], px[2]);
}

const cv::Scalar kRed(0, 0, 220);     // BGR
const cv::Scalar kBlue(220, 30, 0);
const cv::Scalar kGreen(0, 200, 0);

void fill(cv::Mat & img, const cv::Rect & r, const cv::Scalar & colour)
{
  cv::rectangle(img, r, colour, cv::FILLED);
}

}  // namespace

TEST(ColorDetector, UniformGreyImageHasNoDetection)
{
  ColorDetector d{ColorDetectorConfig{}};
  const auto det = d.detect(grey_image());
  EXPECT_FALSE(det.detected);
  EXPECT_DOUBLE_EQ(det.area_ratio, 0.0);
}

TEST(ColorDetector, RedRectangleIsDetectedWithAccurateBox)
{
  auto img = grey_image();
  const cv::Rect drawn(200, 150, 120, 100);
  fill(img, drawn, kRed);

  ColorDetector d{ColorDetectorConfig{}};
  const auto det = d.detect(img);

  ASSERT_TRUE(det.detected);
  EXPECT_NEAR(det.bbox.x, drawn.x, 1);
  EXPECT_NEAR(det.bbox.y, drawn.y, 1);
  EXPECT_NEAR(det.bbox.width, drawn.width, 2);
  EXPECT_NEAR(det.bbox.height, drawn.height, 2);
  EXPECT_NEAR(det.area_ratio, 120.0 * 100.0 / (kW * kH), 0.002);
}

TEST(ColorDetector, DetectsRedOnBothSidesOfTheHueWrapAround)
{
  ColorDetector d{ColorDetectorConfig{}};
  for (int hue : {2, 176}) {
    auto img = grey_image();
    fill(img, cv::Rect(250, 200, 100, 80), hsv_colour(hue, 230, 200));
    EXPECT_TRUE(d.detect(img).detected) << "hue " << hue << " not detected";
  }
}

TEST(ColorDetector, SingleHueBandMissesTheOtherSideOfRed)
{
  // Documents WHY two bands are needed: with only 0..10, hue 176 is missed.
  ColorDetectorConfig cfg;
  cfg.ranges = {HsvRange{0, 10, 120, 255, 70, 255}};
  ColorDetector d{cfg};
  auto img = grey_image();
  fill(img, cv::Rect(250, 200, 100, 80), hsv_colour(176, 230, 200));
  EXPECT_FALSE(d.detect(img).detected);
}

TEST(ColorDetector, IgnoresBlueAndGreenObjects)
{
  auto img = grey_image();
  fill(img, cv::Rect(50, 100, 150, 150), kBlue);
  fill(img, cv::Rect(400, 100, 150, 150), kGreen);
  ColorDetector d{ColorDetectorConfig{}};
  EXPECT_FALSE(d.detect(img).detected);
}

TEST(ColorDetector, IgnoresLowSaturationPinkAndWhite)
{
  auto img = grey_image();
  fill(img, cv::Rect(50, 100, 150, 150), hsv_colour(0, 60, 230));    // pale pink
  fill(img, cv::Rect(400, 100, 150, 150), cv::Scalar(250, 250, 250)); // white
  ColorDetector d{ColorDetectorConfig{}};
  EXPECT_FALSE(d.detect(img).detected);
}

TEST(ColorDetector, ShadedRedIsDetectedButNearBlackIsNot)
{
  ColorDetector d{ColorDetectorConfig{}};

  auto shaded = grey_image();
  fill(shaded, cv::Rect(250, 200, 100, 80), hsv_colour(0, 220, 90));   // side face in shadow
  EXPECT_TRUE(d.detect(shaded).detected);

  auto dark = grey_image();
  fill(dark, cv::Rect(250, 200, 100, 80), hsv_colour(0, 220, 40));     // below v_min
  EXPECT_FALSE(d.detect(dark).detected);
}

TEST(ColorDetector, T4_SmallRedBlobIsFilteredAsNoise)
{
  auto img = grey_image();
  fill(img, cv::Rect(300, 300, 15, 8), kRed);  // 120 px << 614 px threshold
  ColorDetector d{ColorDetectorConfig{}};
  const auto det = d.detect(img);
  EXPECT_FALSE(det.detected);
  EXPECT_GT(det.area_ratio, 0.0);  // still reported, for tuning/debugging
}

TEST(ColorDetector, T4_BlobJustAboveThresholdIsKept)
{
  auto img = grey_image();
  fill(img, cv::Rect(300, 300, 32, 24), kRed);  // 768 px > 614 px
  ColorDetector d{ColorDetectorConfig{}};
  EXPECT_TRUE(d.detect(img).detected);
}

TEST(ColorDetector, SpeckleNoiseIsRemovedAndDoesNotInflateTheBox)
{
  auto img = grey_image();
  const cv::Rect drawn(250, 180, 140, 110);
  fill(img, drawn, kRed);

  cv::RNG rng(42);  // fixed seed: deterministic test
  for (int i = 0; i < 3000; ++i) {
    img.at<cv::Vec3b>(rng.uniform(0, kH), rng.uniform(0, kW)) = cv::Vec3b(0, 0, 230);
  }

  ColorDetector d{ColorDetectorConfig{}};
  const auto det = d.detect(img);
  ASSERT_TRUE(det.detected);
  EXPECT_NEAR(det.bbox.x, drawn.x, 2);
  EXPECT_NEAR(det.bbox.y, drawn.y, 2);
  EXPECT_NEAR(det.bbox.width, drawn.width, 3);
  EXPECT_NEAR(det.bbox.height, drawn.height, 3);
}

TEST(ColorDetector, WithoutMorphologySpeckleTouchingTheObjectGrowsTheBox)
{
  // Shows what the open/close step buys us: same image, morphology disabled.
  auto img = grey_image();
  const cv::Rect drawn(250, 180, 140, 110);
  fill(img, drawn, kRed);
  cv::RNG rng(42);
  for (int i = 0; i < 3000; ++i) {
    img.at<cv::Vec3b>(rng.uniform(0, kH), rng.uniform(0, kW)) = cv::Vec3b(0, 0, 230);
  }

  ColorDetectorConfig cfg;
  cfg.morph_kernel_size = 0;
  ColorDetector d{cfg};
  const auto det = d.detect(img);
  ASSERT_TRUE(det.detected);
  const int grown = (det.bbox.width - drawn.width) + (det.bbox.height - drawn.height);
  EXPECT_GT(grown, 0) << "expected stray pixels adjacent to the box to enlarge it";
}

TEST(ColorDetector, PicksTheLargestOfSeveralBlobs)
{
  auto img = grey_image();
  fill(img, cv::Rect(20, 20, 60, 60), kRed);
  const cv::Rect big(300, 200, 200, 150);
  fill(img, big, kRed);
  fill(img, cv::Rect(550, 400, 50, 50), kRed);

  ColorDetector d{ColorDetectorConfig{}};
  const auto det = d.detect(img);
  ASSERT_TRUE(det.detected);
  EXPECT_NEAR(det.bbox.x, big.x, 1);
  EXPECT_NEAR(det.bbox.width, big.width, 2);
}

TEST(ColorDetector, MaskOutputMatchesTheImage)
{
  auto img = grey_image();
  fill(img, cv::Rect(200, 150, 120, 100), kRed);
  ColorDetector d{ColorDetectorConfig{}};
  cv::Mat mask;
  d.detect(img, &mask);
  ASSERT_EQ(mask.type(), CV_8UC1);
  EXPECT_EQ(mask.size(), img.size());
  EXPECT_NEAR(cv::countNonZero(mask), 120 * 100, 200);
}

TEST(ColorDetector, RejectsInvalidImages)
{
  ColorDetector d{ColorDetectorConfig{}};
  EXPECT_THROW(d.detect(cv::Mat{}), std::invalid_argument);
  EXPECT_THROW(d.detect(cv::Mat(kH, kW, CV_8UC1, cv::Scalar(0))), std::invalid_argument);
}

TEST(ColorDetectorConfigValidation, RejectsBadValues)
{
  auto expect_invalid = [](ColorDetectorConfig c) {
      EXPECT_THROW(ColorDetector{c}, std::invalid_argument);
    };

  ColorDetectorConfig c;
  c.ranges.clear();
  expect_invalid(c);

  c = ColorDetectorConfig{};
  c.ranges[0].h_max = 200;            // hue only goes to 179
  expect_invalid(c);

  c = ColorDetectorConfig{};
  c.ranges[0].s_min = 200;            // min > max
  c.ranges[0].s_max = 100;
  expect_invalid(c);

  c = ColorDetectorConfig{};
  c.morph_kernel_size = 4;            // must be odd
  expect_invalid(c);

  c = ColorDetectorConfig{};
  c.min_area_ratio = 1.5;
  expect_invalid(c);
}

TEST(ColorDetectorConfigValidation, AcceptsDefaultsAndDisabledMorphology)
{
  EXPECT_NO_THROW(ColorDetector{ColorDetectorConfig{}});
  ColorDetectorConfig c;
  c.morph_kernel_size = 1;
  EXPECT_NO_THROW(ColorDetector{c});
}
