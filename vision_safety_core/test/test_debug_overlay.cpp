// Copyright 2026 Kavinaya. MIT License.

#include <gtest/gtest.h>

#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vision_safety_core/color_detector.hpp"
#include "vision_safety_core/danger_zone.hpp"
#include "vision_safety_core/debug_overlay.hpp"

using vision_safety::BlobDetection;
using vision_safety::BoundingBox;
using vision_safety::CameraGeometry;
using vision_safety::DangerZoneConfig;
using vision_safety::ZoneAssessment;
using vision_safety::ground_to_pixel;
using vision_safety::render_debug_overlay;

namespace
{

CameraGeometry cam() {return CameraGeometry::from_fov(640, 480, 1.047, 0.135, 0.15);}

cv::Mat grey() {return cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));}

// Pixel in the middle of the danger-zone trapezoid on the floor.
cv::Point zone_centre(const DangerZoneConfig & z)
{
  const double mid = z.camera_to_front_m + z.stop_gap_m * 0.5;
  const auto p = ground_to_pixel(cam(), mid, 0.0);
  return cv::Point(static_cast<int>(p->u), static_cast<int>(p->v));
}

BlobDetection detection(int x, int y, int w, int h, bool detected = true)
{
  BlobDetection d;
  d.detected = detected;
  d.bbox = BoundingBox{x, y, w, h};
  d.area_ratio = double(w * h) / (640.0 * 480.0);
  return d;
}

}  // namespace

TEST(DebugOverlay, KeepsSizeAndTypeAndDoesNotModifyInput)
{
  const auto in = grey();
  const auto copy = in.clone();
  const auto out = render_debug_overlay(
    in, cv::Mat{}, BlobDetection{}, ZoneAssessment{}, cam(), DangerZoneConfig{});
  EXPECT_EQ(out.size(), in.size());
  EXPECT_EQ(out.type(), CV_8UC3);
  EXPECT_EQ(cv::norm(in, copy, cv::NORM_INF), 0.0);
}

TEST(DebugOverlay, ZoneIsGreenWhenClear)
{
  const DangerZoneConfig z;
  const auto out = render_debug_overlay(
    grey(), cv::Mat{}, BlobDetection{}, ZoneAssessment{}, cam(), z);
  const auto px = out.at<cv::Vec3b>(zone_centre(z));
  EXPECT_GT(px[1], px[2] + 20) << "green should dominate red";
}

TEST(DebugOverlay, ZoneIsRedWhenBlocked)
{
  const DangerZoneConfig z;
  ZoneAssessment blocked;
  blocked.in_danger_zone = true;
  const auto out = render_debug_overlay(
    grey(), cv::Mat{}, detection(250, 250, 140, 140), blocked, cam(), z);
  const auto px = out.at<cv::Vec3b>(zone_centre(z));
  EXPECT_GT(px[2], px[1] + 20) << "red should dominate green";
}

TEST(DebugOverlay, BoxColourEncodesTheDecision)
{
  ZoneAssessment blocked;
  blocked.in_danger_zone = true;
  // Top edge of the box, away from banner and zone.
  const cv::Point top_edge(400, 120);

  auto out = render_debug_overlay(
    grey(), cv::Mat{}, detection(300, 120, 200, 100), blocked, cam(), DangerZoneConfig{});
  EXPECT_EQ(out.at<cv::Vec3b>(top_edge), cv::Vec3b(0, 0, 255));      // red

  out = render_debug_overlay(
    grey(), cv::Mat{}, detection(300, 120, 200, 100), ZoneAssessment{}, cam(), DangerZoneConfig{});
  EXPECT_EQ(out.at<cv::Vec3b>(top_edge), cv::Vec3b(0, 255, 255));    // yellow

  out = render_debug_overlay(
    grey(), cv::Mat{}, detection(300, 120, 200, 100, false), ZoneAssessment{}, cam(),
    DangerZoneConfig{});
  EXPECT_EQ(out.at<cv::Vec3b>(top_edge), cv::Vec3b(170, 170, 170));  // grey: below threshold
}

TEST(DebugOverlay, MaskIsOutlinedAndInteriorKeepsItsColour)
{
  auto img = grey();
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
  cv::rectangle(mask, cv::Rect(500, 150, 60, 40), cv::Scalar(255), cv::FILLED);
  const auto out = render_debug_overlay(
    img, mask, BlobDetection{}, ZoneAssessment{}, cam(), DangerZoneConfig{});
  EXPECT_EQ(out.at<cv::Vec3b>(170, 500), cv::Vec3b(255, 255, 0));      // on the outline
  EXPECT_EQ(out.at<cv::Vec3b>(170, 530), img.at<cv::Vec3b>(170, 530));  // interior untouched
  EXPECT_EQ(out.at<cv::Vec3b>(170, 620), img.at<cv::Vec3b>(170, 620));  // outside untouched
}

TEST(DebugOverlay, IgnoresMaskOfWrongSize)
{
  cv::Mat small_mask(10, 10, CV_8UC1, cv::Scalar(255));
  EXPECT_NO_THROW(render_debug_overlay(
      grey(), small_mask, BlobDetection{}, ZoneAssessment{}, cam(), DangerZoneConfig{}));
}

TEST(DebugOverlay, RejectsInvalidImage)
{
  EXPECT_THROW(
    render_debug_overlay(
      cv::Mat{}, cv::Mat{}, BlobDetection{}, ZoneAssessment{}, cam(), DangerZoneConfig{}),
    std::invalid_argument);
}
