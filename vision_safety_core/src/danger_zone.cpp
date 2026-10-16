// Copyright 2026 Kavinaya. MIT License.

#include "vision_safety_core/danger_zone.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace vision_safety
{

namespace
{

constexpr double kHalfPi = 1.5707963267948966;

void require_positive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be a finite value > 0");
  }
}

void require_finite(double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

}  // namespace

CameraGeometry CameraGeometry::from_fov(
  int width, int height, double horizontal_fov_rad, double mount_height_m, double pitch_rad)
{
  CameraGeometry c;
  c.width = width;
  c.height = height;
  c.fx = (static_cast<double>(width) / 2.0) / std::tan(horizontal_fov_rad / 2.0);
  c.fy = c.fx;  // square pixels
  c.cx = static_cast<double>(width) / 2.0;
  c.cy = static_cast<double>(height) / 2.0;
  c.mount_height_m = mount_height_m;
  c.pitch_rad = pitch_rad;
  return c;
}

void validate(const CameraGeometry & c)
{
  require_positive(c.fx, "fx");
  require_positive(c.fy, "fy");
  require_finite(c.cx, "cx");
  require_finite(c.cy, "cy");
  if (c.width <= 0 || c.height <= 0) {
    throw std::invalid_argument("image width and height must be > 0");
  }
  require_positive(c.mount_height_m, "mount_height_m");
  require_finite(c.pitch_rad, "pitch_rad");
  if (c.pitch_rad <= -kHalfPi || c.pitch_rad >= kHalfPi) {
    throw std::invalid_argument("pitch_rad must be in (-pi/2, pi/2)");
  }
}

std::optional<GroundPoint> project_to_ground(const CameraGeometry & c, double u, double v)
{
  // Normalised image coordinates (pinhole model): ray = (x_n, y_n, 1) in the
  // optical frame (x right, y down, z forward).
  const double x_n = (u - c.cx) / c.fx;
  const double y_n = (v - c.cy) / c.fy;

  // Rotate the ray into a level frame (forward, left, up) using the pitch.
  //   forward =  cos(p) - y_n * sin(p)
  //   up      = -sin(p) - y_n * cos(p)
  // The ray hits the floor (up = -h) at t = h / (sin(p) + y_n cos(p)).
  const double s = std::sin(c.pitch_rad);
  const double k = std::cos(c.pitch_rad);
  const double descent = s + y_n * k;
  if (descent <= 1e-9) {
    return std::nullopt;  // at or above the horizon
  }
  const double t = c.mount_height_m / descent;
  return GroundPoint{t * (k - y_n * s), -t * x_n};
}

std::optional<PixelPoint> ground_to_pixel(
  const CameraGeometry & c, double forward_m, double lateral_m, double z_m)
{
  // Level frame -> camera frame (rotate by pitch), then pinhole projection.
  const double dz = z_m - c.mount_height_m;
  const double s = std::sin(c.pitch_rad);
  const double k = std::cos(c.pitch_rad);
  const double depth = forward_m * k - dz * s;   // along the optical axis
  if (depth <= 1e-9) {
    return std::nullopt;  // behind the camera
  }
  const double up = forward_m * s + dz * k;
  return PixelPoint{c.cx - c.fx * lateral_m / depth, c.cy - c.fy * up / depth};
}

double horizon_row(const CameraGeometry & c)
{
  return c.cy - c.fy * std::tan(c.pitch_rad);
}

void validate(const DangerZoneConfig & z)
{
  require_positive(z.stop_gap_m, "stop_gap_m");
  require_positive(z.path_half_width_m, "path_half_width_m");
  require_finite(z.camera_to_front_m, "camera_to_front_m");
  if (!std::isfinite(z.large_blob_area_ratio) ||
    z.large_blob_area_ratio <= 0.0 || z.large_blob_area_ratio > 1.0)
  {
    throw std::invalid_argument("large_blob_area_ratio must be in (0, 1]");
  }
  if (z.clip_margin_px < 0) {
    throw std::invalid_argument("clip_margin_px must be >= 0");
  }
}

DangerZone::DangerZone(CameraGeometry camera, DangerZoneConfig config)
: camera_(camera), config_(config)
{
  validate(camera_);
  validate(config_);
}

void DangerZone::set_camera(const CameraGeometry & camera)
{
  validate(camera);
  camera_ = camera;
}

ZoneAssessment DangerZone::assess(const BlobDetection & det) const
{
  ZoneAssessment a;
  if (!det.detected) {
    return a;
  }

  // Floor-contact samples. Preferred: the blob's lower outline (one pixel per
  // column). Fallback when it is missing: points along the bbox bottom edge.
  std::vector<ContactPixel> samples = det.contact;
  if (samples.empty()) {
    const int bottom = det.bbox.bottom();
    const int step = std::max(1, det.bbox.width / 32);
    for (int u = det.bbox.x; u < det.bbox.right(); u += step) {
      samples.push_back(ContactPixel{u, bottom});
    }
    samples.push_back(ContactPixel{det.bbox.right(), bottom});
  }

  // Project every sample onto the floor and test it against the zone:
  //   in path  : |lateral| <= path half width
  //   in zone  : in path AND gap <= stop gap
  // A sample whose bottom touches the image edge has its true contact out of
  // view; using the image edge instead gives an upper bound on its gap.
  const double hw = config_.path_half_width_m;
  const double clip_row = static_cast<double>(camera_.height - config_.clip_margin_px);
  double min_gap_all = std::numeric_limits<double>::infinity();
  double min_gap_path = std::numeric_limits<double>::infinity();
  bool clipped_all = false;
  bool clipped_path = false;

  for (const auto & s : samples) {
    double v = static_cast<double>(s.v);
    const bool clipped = v >= clip_row;
    if (clipped) {
      v = static_cast<double>(camera_.height);
    }
    const auto g = project_to_ground(camera_, static_cast<double>(s.u), v);
    if (!g) {
      continue;  // at or above the horizon: no floor contact information
    }
    const double gap = g->forward_m - config_.camera_to_front_m;
    if (gap < min_gap_all) {
      min_gap_all = gap;
      clipped_all = clipped;
    }
    if (std::abs(g->lateral_m) <= hw) {
      a.in_path = true;
      if (gap < min_gap_path) {
        min_gap_path = gap;
        clipped_path = clipped;
      }
    }
  }

  // Report the closest contact in the path if there is one, otherwise the
  // closest contact overall (useful for display and logging).
  if (a.in_path) {
    a.gap_m = min_gap_path;
    a.bottom_clipped = clipped_path;
  } else if (std::isfinite(min_gap_all)) {
    a.gap_m = min_gap_all;
    a.bottom_clipped = clipped_all;
  }
  const bool close_and_in_path = a.in_path && min_gap_path <= config_.stop_gap_m;

  // Safety net: a blob this large is treated as close no matter what.
  // triggered_by_area is set only when the area rule ALONE caused the stop,
  // so logs show which mechanism actually fired.
  const bool large = det.area_ratio >= config_.large_blob_area_ratio;
  a.triggered_by_area = large && !close_and_in_path;

  a.in_danger_zone = close_and_in_path || large;
  return a;
}

}  // namespace vision_safety
