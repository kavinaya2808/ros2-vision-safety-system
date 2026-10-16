// Copyright 2026 Kavinaya. MIT License.
//
// DangerZone: decides whether a detected blob is an obstacle the robot must
// not drive towards, using a GROUND-PLANE model instead of apparent size.
//
// Idea: for an object standing on a flat floor, the lowest blob pixel in each
// image column is where the object touches the floor. Knowing the camera
// intrinsics (fx, fy, cx, cy), its mounting height and its downward pitch, each
// of those pixels can be projected onto the floor, giving the object's
// footprint in metres - independent of how big the object looks. Each footprint
// point is then tested against the zone (inside the robot's path width and
// closer than the stop gap).
//
// Why per column and not the bounding box: a box's TOP face projects towards
// the image centre, so a bbox-based footprint would smear an obstacle beside
// the path into the path. The lower outline follows the real floor contact.
//
// Assumptions (documented limitations): flat floor, object resting on it,
// camera roll ~ 0. Overhanging objects (e.g. a table top) have a lower outline
// that is not on the floor and look further away than they are. A large-blob
// area check is kept as a safety net for such cases.

#pragma once

#include <limits>
#include <optional>

#include "vision_safety_core/types.hpp"

namespace vision_safety
{

struct CameraGeometry
{
  // Intrinsics in pixels (from sensor_msgs/CameraInfo K matrix at runtime).
  double fx{554.26};
  double fy{554.26};
  double cx{320.0};
  double cy{240.0};
  int width{640};
  int height{480};
  // Extrinsics.
  double mount_height_m{0.135};  // optical centre above the floor
  double pitch_rad{0.15};        // + = tilted down

  // Ideal pinhole camera with square pixels and centred principal point.
  static CameraGeometry from_fov(
    int width, int height, double horizontal_fov_rad,
    double mount_height_m, double pitch_rad);
};

// Throws std::invalid_argument if any value is out of range.
void validate(const CameraGeometry & camera);

// A point on the floor, relative to the spot directly below the camera.
struct GroundPoint
{
  double forward_m{0.0};
  double lateral_m{0.0};  // + = left
};

// Project pixel (u, v) onto the floor. std::nullopt if the ray points at or
// above the horizon (it never hits the floor).
std::optional<GroundPoint> project_to_ground(const CameraGeometry & camera, double u, double v);

struct PixelPoint
{
  double u{0.0};
  double v{0.0};
};

// Inverse of project_to_ground: where a world point (relative to the floor spot
// under the camera, at height z_m above the floor) appears in the image.
// std::nullopt if the point is behind the camera. The result may lie outside
// the image bounds.
std::optional<PixelPoint> ground_to_pixel(
  const CameraGeometry & camera, double forward_m, double lateral_m, double z_m = 0.0);

// Image row of the horizon (where the floor plane vanishes), assuming zero roll.
double horizon_row(const CameraGeometry & camera);

struct DangerZoneConfig
{
  // Stop if the free gap between robot front and obstacle base is <= this.
  double stop_gap_m{0.6};
  // Half-width of the corridor the robot sweeps (robot half-width + margin).
  double path_half_width_m{0.24};
  // Distance from the camera to the robot's front edge along +x
  // (+ = front edge is ahead of the camera).
  double camera_to_front_m{0.01};
  // Safety net: a blob covering at least this fraction of the image is in the
  // zone regardless of the ground-plane estimate (for bases that are hidden or
  // objects that overhang). Kept high so it does not pre-empt the ground model:
  // the sim's 0.3 m box already covers ~26% of the view at a 0.6 m gap.
  double large_blob_area_ratio{0.5};
  // A box whose bottom edge is within this many pixels of the image bottom is
  // treated as "base not visible" (bottom_clipped).
  int clip_margin_px{2};
};

// Throws std::invalid_argument if any value is out of range.
void validate(const DangerZoneConfig & config);

struct ZoneAssessment
{
  bool in_danger_zone{false};
  // Estimated gap robot front -> obstacle base [m]. NaN if unknown.
  // If bottom_clipped, this is an upper bound (the base is even closer).
  double gap_m{std::numeric_limits<double>::quiet_NaN()};
  bool bottom_clipped{false};
  bool in_path{false};             // lateral extent overlaps the robot's path
  bool triggered_by_area{false};   // in zone ONLY because of the large-blob safety net
};

class DangerZone
{
public:
  DangerZone(CameraGeometry camera, DangerZoneConfig config);

  ZoneAssessment assess(const BlobDetection & detection) const;

  // Intrinsics can change at runtime (new CameraInfo). Validates, throws on bad input.
  void set_camera(const CameraGeometry & camera);

  const CameraGeometry & camera() const {return camera_;}
  const DangerZoneConfig & config() const {return config_;}

private:
  CameraGeometry camera_;
  DangerZoneConfig config_;
};

}  // namespace vision_safety
