// Copyright 2026 Kavinaya. MIT License.
//
// Tiny synthetic renderer for tests: draws a floor, corridor walls and boxes
// as seen by a CameraGeometry, using ground_to_pixel for projection.
// Only faces turned towards the camera are drawn (back-face culling), so the
// image looks like a real camera view of convex boxes - including visible
// side and top faces, which is what tripped up a bbox-based footprint.

#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vision_safety_core/danger_zone.hpp"

namespace synthetic
{

struct Box
{
  double front_m;   // forward distance of the face nearest the camera (from the camera's floor spot)
  double lateral_m; // centre, + = left
  double size_m;    // cube side
  cv::Scalar front_bgr;
  cv::Scalar side_bgr;
  cv::Scalar top_bgr;
};

inline Box red_cube(double front_m, double lateral_m, double size_m = 0.3)
{
  return Box{front_m, lateral_m, size_m,
    cv::Scalar(0, 0, 200), cv::Scalar(0, 0, 150), cv::Scalar(40, 40, 235)};
}

inline Box blue_cube(double front_m, double lateral_m, double size_m = 0.3)
{
  return Box{front_m, lateral_m, size_m,
    cv::Scalar(200, 30, 0), cv::Scalar(150, 20, 0), cv::Scalar(235, 60, 30)};
}

using V3 = std::array<double, 3>;  // forward, lateral, z

inline void fill_face(
  cv::Mat & img, const vision_safety::CameraGeometry & cam,
  const std::vector<V3> & pts, const cv::Scalar & colour)
{
  std::vector<cv::Point> poly;
  for (const auto & p : pts) {
    const auto px = vision_safety::ground_to_pixel(cam, p[0], p[1], p[2]);
    if (!px) {
      return;
    }
    poly.emplace_back(static_cast<int>(px->u), static_cast<int>(px->v));
  }
  cv::fillConvexPoly(img, poly, colour, cv::LINE_8);
}

inline void draw_box(cv::Mat & img, const vision_safety::CameraGeometry & cam, const Box & b)
{
  const double f0 = b.front_m, f1 = b.front_m + b.size_m;
  const double l0 = b.lateral_m - b.size_m / 2, l1 = b.lateral_m + b.size_m / 2;
  const double s = b.size_m, h = cam.mount_height_m;

  // Back-face culling: the camera is at (0, 0, h).
  if (f0 > 0.0) {   // front face (normal -forward)
    fill_face(img, cam, {{f0, l0, 0}, {f0, l1, 0}, {f0, l1, s}, {f0, l0, s}}, b.front_bgr);
  }
  if (l1 < 0.0) {   // left face (normal +lateral) visible when box is right of camera
    fill_face(img, cam, {{f0, l1, 0}, {f1, l1, 0}, {f1, l1, s}, {f0, l1, s}}, b.side_bgr);
  }
  if (l0 > 0.0) {   // right face (normal -lateral) visible when box is left of camera
    fill_face(img, cam, {{f0, l0, 0}, {f1, l0, 0}, {f1, l0, s}, {f0, l0, s}}, b.side_bgr);
  }
  if (s < h) {      // top face visible when below the camera
    fill_face(img, cam, {{f0, l0, s}, {f1, l0, s}, {f1, l1, s}, {f0, l1, s}}, b.top_bgr);
  }
}

// Floor + corridor walls (2 m wide) + boxes (drawn far to near) + optional red floor patch.
inline cv::Mat render(
  const vision_safety::CameraGeometry & cam, std::vector<Box> boxes, bool red_patch = false)
{
  cv::Mat img(cam.height, cam.width, CV_8UC3, cv::Scalar(230, 215, 205));  // sky
  fill_face(img, cam, {{0.05, -1, 0}, {30, -1, 0}, {30, 1, 0}, {0.05, 1, 0}}, cv::Scalar(150, 150, 150));
  fill_face(img, cam, {{0.05, 1, 0}, {8, 1, 0}, {8, 1, 0.5}, {0.05, 1, 0.5}}, cv::Scalar(205, 210, 210));
  fill_face(img, cam, {{0.05, -1, 0}, {8, -1, 0}, {8, -1, 0.5}, {0.05, -1, 0.5}}, cv::Scalar(190, 195, 195));

  if (red_patch) {  // 6 cm disc on the floor, like the sim world's noise patch
    const auto c = vision_safety::ground_to_pixel(cam, 1.36, 0.45, 0.0);
    cv::ellipse(img, cv::Point(static_cast<int>(c->u), static_cast<int>(c->v)),
      cv::Size(12, 3), 0, 0, 360, cv::Scalar(0, 0, 220), cv::FILLED);
  }

  std::sort(boxes.begin(), boxes.end(), [](const Box & a, const Box & b) {
      return a.front_m > b.front_m;
    });
  for (const auto & b : boxes) {
    draw_box(img, cam, b);
  }
  return img;
}

}  // namespace synthetic
