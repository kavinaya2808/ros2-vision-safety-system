// Copyright 2026 Kavinaya. MIT License.
//
// ColorDetector: finds the largest blob of a target colour in a BGR image.
//
// Pipeline: BGR -> HSV -> inRange (one or more hue bands, OR-ed) ->
//           morphological open (remove speckle) -> close (fill holes) ->
//           connected components -> largest component -> size filter.
//
// Red needs TWO hue bands because hue is circular: in OpenCV's 0..179 scale,
// red sits at both ends (about 0..10 and 170..179).

#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "vision_safety_core/types.hpp"

namespace vision_safety
{

// Inclusive HSV bounds in OpenCV units: H 0..179, S 0..255, V 0..255.
struct HsvRange
{
  int h_min{0};
  int h_max{179};
  int s_min{0};
  int s_max{255};
  int v_min{0};
  int v_max{255};
};

struct ColorDetectorConfig
{
  // Default: saturated red, both sides of the hue wrap-around.
  // s_min 120 rejects pink/white highlights; v_min 70 rejects near-black shadow.
  std::vector<HsvRange> ranges{
    HsvRange{0, 10, 120, 255, 70, 255},
    HsvRange{170, 179, 120, 255, 70, 255}};

  // Blobs smaller than this fraction of the image are noise (0.002 = 614 px at 640x480).
  double min_area_ratio{0.002};

  // Elliptical kernel size for open/close. Odd number; 0 or 1 disables morphology.
  int morph_kernel_size{5};
};

// Throws std::invalid_argument if any value is out of range.
void validate(const ColorDetectorConfig & config);

class ColorDetector
{
public:
  explicit ColorDetector(ColorDetectorConfig config);

  // bgr must be a non-empty CV_8UC3 image (throws std::invalid_argument otherwise).
  // If mask_out is given, it receives the cleaned binary mask (CV_8UC1) for debugging.
  BlobDetection detect(const cv::Mat & bgr, cv::Mat * mask_out = nullptr) const;

  const ColorDetectorConfig & config() const {return config_;}

private:
  ColorDetectorConfig config_;
  cv::Mat kernel_;  // empty when morphology is disabled
};

}  // namespace vision_safety
