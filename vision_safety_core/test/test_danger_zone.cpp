// Copyright 2026 Kavinaya. MIT License.
//
// DangerZone tests. Obstacles are placed in 3D (metres) and projected into the
// image with an independent forward model, so the tests check the real geometry:
// "a 0.3 m box 0.4 m in front of the robot" rather than hand-picked pixels.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "vision_safety_core/danger_zone.hpp"

using vision_safety::BlobDetection;
using vision_safety::BoundingBox;
using vision_safety::CameraGeometry;
using vision_safety::DangerZone;
using vision_safety::DangerZoneConfig;
using vision_safety::project_to_ground;

namespace
{

// Same camera as the simulated robot: 640x480, 60 deg HFOV, 0.135 m high, pitched 0.15 rad.
CameraGeometry sim_camera()
{
  return CameraGeometry::from_fov(640, 480, 1.047, 0.135, 0.15);
}

struct Pixel
{
  double u;
  double v;
};

// Forward model, written independently of the code under test:
// world point relative to the floor spot under the camera -> pixel.
Pixel world_to_pixel(const CameraGeometry & c, double forward, double lateral, double z)
{
  const double dz = z - c.mount_height_m;
  const double x_cam = forward * std::cos(c.pitch_rad) - dz * std::sin(c.pitch_rad);
  const double up = forward * std::sin(c.pitch_rad) + dz * std::cos(c.pitch_rad);
  return Pixel{c.cx - c.fx * lateral / x_cam, c.cy - c.fy * up / x_cam};
}

// Detection of the front face of a cube (side `size`) whose front face is
// `gap` metres ahead of the robot front and centred at `lateral`.
BlobDetection cube_detection(
  const CameraGeometry & c, const DangerZoneConfig & z,
  double gap, double lateral, double size = 0.3)
{
  const double forward = gap + z.camera_to_front_m;
  double u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
  for (double l : {lateral - size / 2, lateral + size / 2}) {
    for (double h : {0.0, size}) {
      const auto p = world_to_pixel(c, forward, l, h);
      u0 = std::min(u0, p.u);
      u1 = std::max(u1, p.u);
      v0 = std::min(v0, p.v);
      v1 = std::max(v1, p.v);
    }
  }
  // Clip to the image like a real detector would see it.
  u0 = std::clamp(u0, 0.0, double(c.width));
  u1 = std::clamp(u1, 0.0, double(c.width));
  v0 = std::clamp(v0, 0.0, double(c.height));
  v1 = std::clamp(v1, 0.0, double(c.height));

  BlobDetection d;
  d.detected = (u1 - u0) > 1.0 && (v1 - v0) > 1.0;
  d.bbox = BoundingBox{
    static_cast<int>(std::lround(u0)), static_cast<int>(std::lround(v0)),
    static_cast<int>(std::lround(u1 - u0)), static_cast<int>(std::lround(v1 - v0))};
  d.area_ratio = (u1 - u0) * (v1 - v0) / (c.width * c.height);
  return d;
}

BlobDetection manual_detection(int x, int y, int w, int h, double area_ratio)
{
  BlobDetection d;
  d.detected = true;
  d.bbox = BoundingBox{x, y, w, h};
  d.area_ratio = area_ratio;
  return d;
}

}  // namespace

// ---------------------------------------------------------------------------
// Ground projection
// ---------------------------------------------------------------------------

TEST(GroundProjection, ImageCentreHitsTheFloorAtHeightOverTanPitch)
{
  const auto c = sim_camera();
  const auto p = project_to_ground(c, c.cx, c.cy);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->forward_m, c.mount_height_m / std::tan(c.pitch_rad), 1e-9);
  EXPECT_NEAR(p->lateral_m, 0.0, 1e-12);
}

TEST(GroundProjection, RoundTripsWithTheForwardModel)
{
  const auto c = sim_camera();
  for (double fwd : {0.3, 0.6, 1.0, 2.0, 4.0}) {
    for (double lat : {-0.5, -0.1, 0.0, 0.2, 0.6}) {
      const auto px = world_to_pixel(c, fwd, lat, 0.0);
      const auto g = project_to_ground(c, px.u, px.v);
      ASSERT_TRUE(g.has_value()) << "fwd=" << fwd << " lat=" << lat;
      EXPECT_NEAR(g->forward_m, fwd, 1e-9);
      EXPECT_NEAR(g->lateral_m, lat, 1e-9);
    }
  }
}

TEST(GroundProjection, LeftOfImageCentreIsPositiveLateral)
{
  const auto c = sim_camera();
  const auto left = project_to_ground(c, c.cx - 100, c.height - 10);
  const auto right = project_to_ground(c, c.cx + 100, c.height - 10);
  ASSERT_TRUE(left && right);
  EXPECT_GT(left->lateral_m, 0.0);
  EXPECT_LT(right->lateral_m, 0.0);
  EXPECT_NEAR(left->lateral_m, -right->lateral_m, 1e-12);
}

TEST(GroundProjection, RowsAtOrAboveTheHorizonNeverHitTheFloor)
{
  const auto c = sim_camera();
  const double horizon_v = c.cy - c.fy * std::tan(c.pitch_rad);
  EXPECT_FALSE(project_to_ground(c, c.cx, 0.0).has_value());
  EXPECT_FALSE(project_to_ground(c, c.cx, horizon_v).has_value());
  EXPECT_TRUE(project_to_ground(c, c.cx, horizon_v + 5.0).has_value());
}

TEST(GroundProjection, LowerRowsAreCloser)
{
  const auto c = sim_camera();
  double previous = std::numeric_limits<double>::infinity();
  for (int v = 200; v <= 480; v += 20) {
    const auto g = project_to_ground(c, c.cx, v);
    ASSERT_TRUE(g.has_value());
    EXPECT_LT(g->forward_m, previous);
    previous = g->forward_m;
  }
}

TEST(GroundProjection, GroundToPixelIsTheInverseOfProjectToGround)
{
  const auto c = sim_camera();
  for (double fwd : {0.3, 0.8, 2.0}) {
    for (double lat : {-0.4, 0.0, 0.3}) {
      const auto px = vision_safety::ground_to_pixel(c, fwd, lat);
      ASSERT_TRUE(px.has_value());
      const auto g = project_to_ground(c, px->u, px->v);
      ASSERT_TRUE(g.has_value());
      EXPECT_NEAR(g->forward_m, fwd, 1e-9);
      EXPECT_NEAR(g->lateral_m, lat, 1e-9);
    }
  }
}

TEST(GroundProjection, GroundToPixelMatchesIndependentModelForRaisedPoints)
{
  const auto c = sim_camera();
  const auto px = vision_safety::ground_to_pixel(c, 1.0, 0.2, 0.3);
  const auto ref = world_to_pixel(c, 1.0, 0.2, 0.3);
  ASSERT_TRUE(px.has_value());
  EXPECT_NEAR(px->u, ref.u, 1e-9);
  EXPECT_NEAR(px->v, ref.v, 1e-9);
}

TEST(GroundProjection, PointsBehindTheCameraHaveNoPixel)
{
  EXPECT_FALSE(vision_safety::ground_to_pixel(sim_camera(), -1.0, 0.0).has_value());
}

TEST(GroundProjection, HorizonRowSeparatesFloorFromSky)
{
  const auto c = sim_camera();
  const double hv = vision_safety::horizon_row(c);
  EXPECT_FALSE(project_to_ground(c, c.cx, hv - 1.0).has_value());
  EXPECT_TRUE(project_to_ground(c, c.cx, hv + 1.0).has_value());
}

// ---------------------------------------------------------------------------
// Danger zone decisions
// ---------------------------------------------------------------------------

TEST(DangerZone, NoDetectionIsNeverInTheZone)
{
  DangerZone zone(sim_camera(), DangerZoneConfig{});
  const auto a = zone.assess(BlobDetection{});
  EXPECT_FALSE(a.in_danger_zone);
  EXPECT_TRUE(std::isnan(a.gap_m));
}

TEST(DangerZone, T2_CentredObstacleInsideStopGapIsInZone)
{
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  const auto a = zone.assess(cube_detection(c, z, 0.4, 0.0));
  EXPECT_TRUE(a.in_danger_zone);
  EXPECT_TRUE(a.in_path);
  EXPECT_FALSE(a.triggered_by_area);
  EXPECT_NEAR(a.gap_m, 0.4, 0.01);   // pixel rounding only
}

TEST(DangerZone, T1_CentredObstacleBeyondStopGapIsNotInZone)
{
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  const auto a = zone.assess(cube_detection(c, z, 1.5, 0.0));
  EXPECT_FALSE(a.in_danger_zone);
  EXPECT_TRUE(a.in_path);
  EXPECT_NEAR(a.gap_m, 1.5, 0.03);
}

TEST(DangerZone, GapEstimateIsAccurateAcrossDistances)
{
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  for (double gap : {0.3, 0.5, 0.8, 1.2, 2.0, 3.0}) {
    const auto a = zone.assess(cube_detection(c, z, gap, 0.0));
    // Error grows with distance because one pixel covers more floor.
    EXPECT_NEAR(a.gap_m, gap, 0.01 + 0.02 * gap * gap) << "gap " << gap;
  }
}

TEST(DangerZone, ObjectSizeDoesNotChangeTheDecision)
{
  // The weakness of an area-ratio rule: a small object close by looks "far".
  // The ground-plane rule judges by where the object touches the floor.
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  const auto small_close_det = cube_detection(c, z, 0.4, 0.0, 0.08);
  const auto big_far_det = cube_detection(c, z, 1.2, 0.0, 0.6);

  // An area rule would get both wrong: the close one looks SMALLER.
  ASSERT_LT(small_close_det.area_ratio, big_far_det.area_ratio);

  EXPECT_TRUE(zone.assess(small_close_det).in_danger_zone);
  EXPECT_FALSE(zone.assess(big_far_det).in_danger_zone);
}

TEST(DangerZone, CloseObstacleBesideThePathIsNotInZone)
{
  // Box spans lateral 0.27..0.57 m, path is +-0.24 m: robot can pass it.
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  const auto det = cube_detection(c, z, 0.55, 0.42);
  ASSERT_TRUE(det.detected) << "test setup: box should be partly in view";
  const auto a = zone.assess(det);
  EXPECT_FALSE(a.in_path);
  EXPECT_FALSE(a.in_danger_zone);
}

TEST(DangerZone, ObstaclePartlyOverlappingThePathIsInZone)
{
  // Box spans lateral 0.15..0.45 m: its inner edge enters the +-0.24 m path.
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  const auto a = zone.assess(cube_detection(c, z, 0.4, 0.30));
  EXPECT_TRUE(a.in_path);
  EXPECT_TRUE(a.in_danger_zone);
}

TEST(DangerZone, PathWidthIsSymmetric)
{
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  EXPECT_EQ(
    zone.assess(cube_detection(c, z, 0.4, 0.30)).in_danger_zone,
    zone.assess(cube_detection(c, z, 0.4, -0.30)).in_danger_zone);
}

TEST(DangerZone, BaseBelowTheImageIsTreatedAsClose)
{
  // Bottom edge touches the image bottom: the base is out of view, so the
  // obstacle is at least as close as the nearest visible floor.
  const auto c = sim_camera();
  DangerZone zone(c, DangerZoneConfig{});
  const auto a = zone.assess(manual_detection(220, 300, 200, 180, 0.12));
  EXPECT_TRUE(a.bottom_clipped);
  EXPECT_TRUE(a.in_danger_zone);
  EXPECT_LT(a.gap_m, 0.6);   // upper bound
}

TEST(DangerZone, SmallBlobAboveTheHorizonIsNotInZone)
{
  // Something red high up (e.g. a sign on a far wall): no floor contact visible.
  const auto c = sim_camera();
  DangerZone zone(c, DangerZoneConfig{});
  const auto a = zone.assess(manual_detection(280, 40, 80, 60, 0.016));
  EXPECT_FALSE(a.in_danger_zone);
  EXPECT_TRUE(std::isnan(a.gap_m));
}

TEST(DangerZone, LargeBlobTriggersTheAreaSafetyNetEvenAboveTheHorizon)
{
  // e.g. an overhanging red object filling the view: base never visible.
  const auto c = sim_camera();
  DangerZoneConfig z;
  z.large_blob_area_ratio = 0.30;
  DangerZone zone(c, z);
  const auto a = zone.assess(manual_detection(0, 0, 640, 150, 0.31));
  EXPECT_TRUE(a.in_danger_zone);
  EXPECT_TRUE(a.triggered_by_area);
}

TEST(DangerZone, AreaFlagIsOnlySetWhenAreaAloneTriggers)
{
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  // Very close centred box: huge area AND inside the stop gap.
  const auto det = cube_detection(c, z, 0.3, 0.0);
  ASSERT_GE(det.area_ratio, z.large_blob_area_ratio);
  const auto a = zone.assess(det);
  EXPECT_TRUE(a.in_danger_zone);
  EXPECT_FALSE(a.triggered_by_area);  // the ground model caught it first
}

TEST(DangerZone, DefaultAreaNetDoesNotPreemptTheGroundModel)
{
  // At the stop distance and beyond, the sim box must be judged by the
  // ground model, not by its apparent size.
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  for (double gap : {0.65, 0.8, 1.0, 1.5}) {
    const auto det = cube_detection(c, z, gap, 0.0);
    EXPECT_LT(det.area_ratio, z.large_blob_area_ratio) << "gap " << gap;
    EXPECT_FALSE(zone.assess(det).in_danger_zone) << "gap " << gap;
  }
}

TEST(DangerZone, UpdatedIntrinsicsAreUsed)
{
  const auto c = sim_camera();
  const DangerZoneConfig z;
  DangerZone zone(c, z);
  const auto det = cube_detection(c, z, 0.5, 0.0);
  ASSERT_TRUE(zone.assess(det).in_danger_zone);

  // Same pixels, but now the camera claims to be mounted 3x higher:
  // the same bottom row maps to a floor point 3x further away.
  auto higher = c;
  higher.mount_height_m = 3 * c.mount_height_m;
  zone.set_camera(higher);
  EXPECT_FALSE(zone.assess(det).in_danger_zone);
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

TEST(DangerZoneValidation, RejectsBadCameraGeometry)
{
  auto bad = sim_camera();
  bad.fx = 0.0;
  EXPECT_THROW(DangerZone(bad, DangerZoneConfig{}), std::invalid_argument);

  bad = sim_camera();
  bad.mount_height_m = -0.1;
  EXPECT_THROW(DangerZone(bad, DangerZoneConfig{}), std::invalid_argument);

  bad = sim_camera();
  bad.pitch_rad = 1.6;   // beyond straight down
  EXPECT_THROW(DangerZone(bad, DangerZoneConfig{}), std::invalid_argument);

  bad = sim_camera();
  bad.width = 0;
  EXPECT_THROW(DangerZone(bad, DangerZoneConfig{}), std::invalid_argument);

  DangerZone zone(sim_camera(), DangerZoneConfig{});
  bad = sim_camera();
  bad.fy = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(zone.set_camera(bad), std::invalid_argument);
}

TEST(DangerZoneValidation, RejectsBadZoneConfig)
{
  DangerZoneConfig z;
  z.stop_gap_m = -0.1;
  EXPECT_THROW(DangerZone(sim_camera(), z), std::invalid_argument);

  z = DangerZoneConfig{};
  z.path_half_width_m = 0.0;
  EXPECT_THROW(DangerZone(sim_camera(), z), std::invalid_argument);

  z = DangerZoneConfig{};
  z.large_blob_area_ratio = 1.5;
  EXPECT_THROW(DangerZone(sim_camera(), z), std::invalid_argument);

  z = DangerZoneConfig{};
  z.clip_margin_px = -1;
  EXPECT_THROW(DangerZone(sim_camera(), z), std::invalid_argument);
}

TEST(DangerZoneValidation, FromFovProducesAValidCamera)
{
  const auto c = sim_camera();
  EXPECT_NO_THROW(vision_safety::validate(c));
  EXPECT_NEAR(c.fx, 320.0 / std::tan(1.047 / 2.0), 1e-9);
  EXPECT_DOUBLE_EQ(c.cx, 320.0);
  EXPECT_DOUBLE_EQ(c.cy, 240.0);
}
