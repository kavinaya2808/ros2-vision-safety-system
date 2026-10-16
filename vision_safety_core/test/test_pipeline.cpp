// Copyright 2026 Kavinaya. MIT License.
//
// End-to-end tests of the perception pipeline (ColorDetector -> DangerZone) on
// synthetic camera images rendered from 3D scenes. These catch geometry bugs
// that hand-built detections hide - e.g. a box's top face making an obstacle
// beside the path look like it is in the path.

#include <gtest/gtest.h>

#include <cmath>

#include "synthetic_scene.hpp"
#include "vision_safety_core/color_detector.hpp"
#include "vision_safety_core/danger_zone.hpp"

using vision_safety::BlobDetection;
using vision_safety::CameraGeometry;
using vision_safety::ColorDetector;
using vision_safety::ColorDetectorConfig;
using vision_safety::DangerZone;
using vision_safety::DangerZoneConfig;
using vision_safety::ZoneAssessment;

namespace
{

class Pipeline : public ::testing::Test
{
protected:
  CameraGeometry cam = CameraGeometry::from_fov(640, 480, 1.047, 0.135, 0.15);
  DangerZoneConfig zone_cfg;
  ColorDetector detector{ColorDetectorConfig{}};
  DangerZone zone{cam, zone_cfg};

  // Front face `gap` metres ahead of the robot's front edge.
  double front_at_gap(double gap) const {return gap + zone_cfg.camera_to_front_m;}

  struct Result
  {
    BlobDetection det;
    ZoneAssessment zone;
  };

  Result run(const std::vector<synthetic::Box> & boxes, bool red_patch = false) const
  {
    const auto img = synthetic::render(cam, boxes, red_patch);
    Result r;
    r.det = detector.detect(img);
    r.zone = zone.assess(r.det);
    return r;
  }
};

}  // namespace

TEST_F(Pipeline, T1_FarCentredBoxIsDetectedButClear)
{
  const auto r = run({synthetic::red_cube(front_at_gap(1.6), 0.0)});
  ASSERT_TRUE(r.det.detected);
  EXPECT_TRUE(r.zone.in_path);
  EXPECT_FALSE(r.zone.in_danger_zone);
  EXPECT_NEAR(r.zone.gap_m, 1.6, 0.06);
}

TEST_F(Pipeline, T2_CloseCentredBoxIsBlocked)
{
  const auto r = run({synthetic::red_cube(front_at_gap(0.45), 0.0)});
  ASSERT_TRUE(r.det.detected);
  EXPECT_TRUE(r.zone.in_danger_zone);
  EXPECT_FALSE(r.zone.triggered_by_area);
  EXPECT_NEAR(r.zone.gap_m, 0.45, 0.02);
}

TEST_F(Pipeline, CloseBoxBesideThePathIsNotBlocked)
{
  // Regression test: spans lateral 0.27..0.57 m, path is +-0.24 m.
  // Its top face projects towards the image centre; a bbox-based footprint
  // wrongly put it in the path.
  for (double lat : {0.42, -0.42}) {
    const auto r = run({synthetic::red_cube(front_at_gap(0.5), lat)});
    ASSERT_TRUE(r.det.detected) << "lat " << lat;
    EXPECT_FALSE(r.zone.in_path) << "lat " << lat;
    EXPECT_FALSE(r.zone.in_danger_zone) << "lat " << lat;
  }
}

TEST_F(Pipeline, CloseBoxStraddlingThePathEdgeIsBlocked)
{
  // Spans lateral 0.18..0.48 m: its inner edge is inside the +-0.24 m path.
  for (double lat : {0.33, -0.33}) {
    const auto r = run({synthetic::red_cube(front_at_gap(0.45), lat)});
    ASSERT_TRUE(r.det.detected) << "lat " << lat;
    EXPECT_TRUE(r.zone.in_danger_zone) << "lat " << lat;
  }
}

TEST_F(Pipeline, GapEstimateIsAccurateOverTheWorkingRange)
{
  for (double gap : {0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 2.5}) {
    const auto r = run({synthetic::red_cube(front_at_gap(gap), 0.0)});
    ASSERT_TRUE(r.det.detected) << "gap " << gap;
    EXPECT_NEAR(r.zone.gap_m, gap, 0.02 + 0.03 * gap * gap) << "gap " << gap;
  }
}

TEST_F(Pipeline, T2_ApproachingBoxTriggersNearTheConfiguredStopGapAndStaysBlocked)
{
  // Move the box towards the robot in 1 cm steps, as if driving at it.
  double first_blocked_gap = -1.0;
  for (int cm = 200; cm >= 25; --cm) {
    const double gap = cm / 100.0;
    const auto r = run({synthetic::red_cube(front_at_gap(gap), 0.0)});
    if (r.zone.in_danger_zone && first_blocked_gap < 0.0) {
      first_blocked_gap = gap;
    }
    if (first_blocked_gap > 0.0) {
      ASSERT_TRUE(r.zone.in_danger_zone) << "zone released again at gap " << gap;
    }
  }
  ASSERT_GT(first_blocked_gap, 0.0) << "never blocked";
  EXPECT_NEAR(first_blocked_gap, zone_cfg.stop_gap_m, 0.03);
}

TEST_F(Pipeline, T4_SmallRedFloorPatchAloneIsNotAnObstacle)
{
  const auto r = run({}, true);
  EXPECT_FALSE(r.det.detected);
  EXPECT_FALSE(r.zone.in_danger_zone);
}

TEST_F(Pipeline, T4_RedPatchDoesNotHideTheRealObstacle)
{
  const auto r = run({synthetic::red_cube(front_at_gap(0.45), 0.0)}, true);
  EXPECT_TRUE(r.zone.in_danger_zone);
}

TEST_F(Pipeline, BlueBoxInThePathIsIgnoredByTheRedDetector)
{
  // Documents the colour-specific scope: this is a known limitation, not a bug.
  const auto r = run({synthetic::blue_cube(front_at_gap(0.4), 0.0)});
  EXPECT_FALSE(r.det.detected);
  EXPECT_FALSE(r.zone.in_danger_zone);
}

TEST_F(Pipeline, ContactOutlineFollowsTheFloorNotTheTopFace)
{
  const auto r = run({synthetic::red_cube(front_at_gap(0.5), 0.42)});
  ASSERT_FALSE(r.det.contact.empty());
  // Every contact point must project to a lateral offset outside the path.
  for (const auto & c : r.det.contact) {
    const auto g = vision_safety::project_to_ground(cam, c.u, c.v);
    ASSERT_TRUE(g.has_value());
    EXPECT_GT(g->lateral_m, zone_cfg.path_half_width_m - 0.01) << "u=" << c.u;
  }
}
