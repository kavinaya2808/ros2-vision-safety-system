// Copyright 2026 Kavinaya. MIT License.

#include "vision_safety_core/debug_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace vision_safety
{

namespace
{

// BGR colours
const cv::Scalar kZoneClear(60, 200, 60);
const cv::Scalar kZoneBlocked(40, 40, 230);
const cv::Scalar kBoxBlocked(0, 0, 255);
const cv::Scalar kBoxDetected(0, 255, 255);
const cv::Scalar kBoxIgnored(170, 170, 170);
const cv::Scalar kMaskOutline(255, 255, 0);
const cv::Scalar kContact(255, 0, 255);
const cv::Scalar kPath(220, 220, 220);
const cv::Scalar kHorizon(200, 160, 60);
const cv::Scalar kText(255, 255, 255);
const cv::Scalar kBanner(30, 30, 30);

cv::Point to_cv(const PixelPoint & p)
{
  return cv::Point(
    static_cast<int>(std::lround(p.u)), static_cast<int>(std::lround(p.v)));
}

void blend_fill_poly(cv::Mat & img, const std::vector<cv::Point> & poly, const cv::Scalar & c, double alpha)
{
  cv::Mat layer = img.clone();
  cv::fillPoly(layer, std::vector<std::vector<cv::Point>>{poly}, c, cv::LINE_AA);
  cv::addWeighted(layer, alpha, img, 1.0 - alpha, 0.0, img);
}

std::string format(const char * fmt, double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt, value);
  return buf;
}

}  // namespace

cv::Mat render_debug_overlay(
  const cv::Mat & bgr,
  const cv::Mat & mask,
  const BlobDetection & det,
  const ZoneAssessment & zone,
  const CameraGeometry & cam,
  const DangerZoneConfig & cfg)
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("render_debug_overlay: expected a non-empty CV_8UC3 image");
  }
  cv::Mat out = bgr.clone();

  // 1. Mask outline (an outline, not a fill, so the object's real colour stays visible)
  if (!mask.empty() && mask.size() == out.size() && mask.type() == CV_8UC1) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(out, contours, -1, kMaskOutline, 1, cv::LINE_8);
  }

  // 2. Danger zone on the floor: forward from the robot front to front + stop_gap,
  //    lateral +- path half width.
  const double near_f = std::max(cfg.camera_to_front_m, 0.01);
  const double far_f = cfg.camera_to_front_m + cfg.stop_gap_m;
  const double hw = cfg.path_half_width_m;
  const auto a = ground_to_pixel(cam, near_f, hw);
  const auto b = ground_to_pixel(cam, far_f, hw);
  const auto c = ground_to_pixel(cam, far_f, -hw);
  const auto d = ground_to_pixel(cam, near_f, -hw);
  if (a && b && c && d) {
    const std::vector<cv::Point> poly{to_cv(*a), to_cv(*b), to_cv(*c), to_cv(*d)};
    const cv::Scalar zc = zone.in_danger_zone ? kZoneBlocked : kZoneClear;
    blend_fill_poly(out, poly, zc, 0.30);
    cv::polylines(out, std::vector<std::vector<cv::Point>>{poly}, true, zc, 2, cv::LINE_AA);

    // 3. Path corridor continued ahead (thin), so you can see what "in path" means.
    const double path_end = far_f + 3.0;
    const auto b2 = ground_to_pixel(cam, path_end, hw);
    const auto c2 = ground_to_pixel(cam, path_end, -hw);
    if (b2 && c2) {
      cv::line(out, to_cv(*b), to_cv(*b2), kPath, 1, cv::LINE_AA);
      cv::line(out, to_cv(*c), to_cv(*c2), kPath, 1, cv::LINE_AA);
    }
  }

  // 4. Horizon
  const int hv = static_cast<int>(std::lround(horizon_row(cam)));
  if (hv > 0 && hv < out.rows) {
    for (int x = 0; x < out.cols; x += 16) {
      cv::line(out, cv::Point(x, hv), cv::Point(std::min(x + 8, out.cols - 1), hv), kHorizon, 1);
    }
  }

  // 5. Bounding box + assumed floor contact (the lower outline used for the decision)
  const auto & bb = det.bbox;
  if (bb.width > 0 && bb.height > 0) {
    const cv::Scalar bc = zone.in_danger_zone ? kBoxBlocked :
      (det.detected ? kBoxDetected : kBoxIgnored);
    cv::rectangle(out, cv::Rect(bb.x, bb.y, bb.width, bb.height), bc, 2);
    if (det.detected) {
      if (det.contact.empty()) {
        const int y = std::min(bb.bottom(), out.rows - 1);
        cv::line(out, cv::Point(bb.x, y), cv::Point(bb.right(), y), kContact, 3);
      } else {
        std::vector<cv::Point> outline;
        outline.reserve(det.contact.size());
        for (const auto & cp : det.contact) {
          outline.emplace_back(cp.u, std::min(cp.v, out.rows - 1));
        }
        cv::polylines(out, outline, false, kContact, 3, cv::LINE_AA);
      }
    }
  }

  // 6. Status banner
  std::string status;
  if (!det.detected) {
    status = "CLEAR - no obstacle";
  } else if (zone.in_danger_zone) {
    status = zone.triggered_by_area ? "BLOCKED - large blob (area net)" :
      "BLOCKED - obstacle in zone";
  } else {
    status = zone.in_path ? "CLEAR - obstacle ahead, outside zone" :
      "CLEAR - obstacle beside path";
  }

  std::string gap = "gap: n/a";
  if (std::isfinite(zone.gap_m)) {
    gap = zone.bottom_clipped ? format("gap: <= %.2f m (base hidden)", zone.gap_m) :
      format("gap: %.2f m", zone.gap_m);
  }
  const std::string area = format("area: %.1f %%", det.area_ratio * 100.0);

  const int line_h = 20;
  cv::rectangle(out, cv::Rect(0, 0, std::min(out.cols, 330), 3 * line_h + 10), kBanner, cv::FILLED);
  int y = line_h;
  for (const auto & s : {status, gap, area}) {
    cv::putText(out, s, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, kText, 1, cv::LINE_AA);
    y += line_h;
  }
  return out;
}

}  // namespace vision_safety
