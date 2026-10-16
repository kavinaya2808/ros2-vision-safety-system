// Copyright 2026 Kavinaya. MIT License.
//
// Debug overlay: draws what the perception step "thinks" on top of the camera
// image, so tuning can be done by eye. Pure OpenCV, no ROS.
//
//   - cyan outline around detected-colour regions (the cleaned mask)
//   - danger zone drawn ON THE FLOOR as a trapezoid (green clear / red blocked)
//   - robot path corridor continued further ahead as thin lines
//   - horizon line (rows above it can never touch the floor)
//   - bounding box (red in zone, yellow detected, grey below size threshold)
//   - magenta line = the blob's lower outline, i.e. the assumed floor contact
//   - status banner: decision, gap estimate, area

#pragma once

#include <opencv2/core.hpp>

#include "vision_safety_core/danger_zone.hpp"
#include "vision_safety_core/types.hpp"

namespace vision_safety
{

// Returns a new image; the input is not modified.
// bgr: CV_8UC3 (throws std::invalid_argument otherwise).
// mask: CV_8UC1 of the same size, or empty to skip the tint.
cv::Mat render_debug_overlay(
  const cv::Mat & bgr,
  const cv::Mat & mask,
  const BlobDetection & detection,
  const ZoneAssessment & zone,
  const CameraGeometry & camera,
  const DangerZoneConfig & config);

}  // namespace vision_safety
