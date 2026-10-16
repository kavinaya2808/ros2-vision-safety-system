// Copyright 2026 Kavinaya. MIT License.

#include "vision_safety_core/color_detector.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace vision_safety
{

namespace
{

void require_range(int value, int lo, int hi, const std::string & name)
{
  if (value < lo || value > hi) {
    throw std::invalid_argument(
            name + " must be in [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
  }
}

}  // namespace

void validate(const ColorDetectorConfig & config)
{
  if (config.ranges.empty()) {
    throw std::invalid_argument("at least one HSV range is required");
  }
  for (std::size_t i = 0; i < config.ranges.size(); ++i) {
    const auto & r = config.ranges[i];
    const std::string p = "ranges[" + std::to_string(i) + "].";
    require_range(r.h_min, 0, 179, p + "h_min");
    require_range(r.h_max, 0, 179, p + "h_max");
    require_range(r.s_min, 0, 255, p + "s_min");
    require_range(r.s_max, 0, 255, p + "s_max");
    require_range(r.v_min, 0, 255, p + "v_min");
    require_range(r.v_max, 0, 255, p + "v_max");
    if (r.h_min > r.h_max || r.s_min > r.s_max || r.v_min > r.v_max) {
      throw std::invalid_argument(p + "min must be <= max (use two ranges for hue wrap-around)");
    }
  }
  if (!std::isfinite(config.min_area_ratio) ||
    config.min_area_ratio < 0.0 || config.min_area_ratio >= 1.0)
  {
    throw std::invalid_argument("min_area_ratio must be in [0, 1)");
  }
  if (config.morph_kernel_size < 0 ||
    (config.morph_kernel_size > 1 && config.morph_kernel_size % 2 == 0))
  {
    throw std::invalid_argument("morph_kernel_size must be 0, 1 or an odd number");
  }
}

ColorDetector::ColorDetector(ColorDetectorConfig config)
: config_(std::move(config))
{
  validate(config_);
  if (config_.morph_kernel_size > 1) {
    kernel_ = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(config_.morph_kernel_size, config_.morph_kernel_size));
  }
}

BlobDetection ColorDetector::detect(const cv::Mat & bgr, cv::Mat * mask_out) const
{
  if (bgr.empty()) {
    throw std::invalid_argument("ColorDetector::detect: empty image");
  }
  if (bgr.type() != CV_8UC3) {
    throw std::invalid_argument("ColorDetector::detect: expected an 8-bit 3-channel BGR image");
  }

  // 1. Colour segmentation in HSV: hue is separated from brightness, so shading
  //    on the obstacle's side faces changes V but keeps H roughly constant.
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
  cv::Mat band;
  for (const auto & r : config_.ranges) {
    cv::inRange(
      hsv, cv::Scalar(r.h_min, r.s_min, r.v_min), cv::Scalar(r.h_max, r.s_max, r.v_max), band);
    cv::bitwise_or(mask, band, mask);
  }

  // 2. Clean-up: opening removes isolated speckles, closing fills small holes.
  if (!kernel_.empty()) {
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);
  }

  // 3. Connected components give each blob's exact pixel count and bounding box.
  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

  int best = -1;
  int best_area = 0;
  for (int i = 1; i < n; ++i) {  // label 0 is the background
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area > best_area) {
      best_area = area;
      best = i;
    }
  }

  BlobDetection result;
  if (best > 0) {
    result.area_ratio = static_cast<double>(best_area) / static_cast<double>(bgr.total());
    result.bbox = BoundingBox{
      stats.at<int>(best, cv::CC_STAT_LEFT),
      stats.at<int>(best, cv::CC_STAT_TOP),
      stats.at<int>(best, cv::CC_STAT_WIDTH),
      stats.at<int>(best, cv::CC_STAT_HEIGHT)};
    // 4. Size filter: tiny blobs are treated as noise (T4).
    result.detected = result.area_ratio >= config_.min_area_ratio;

    // 5. Lower outline: the lowest pixel of the blob in every column. For an
    //    object on the floor this traces its floor contact, including visible
    //    side faces, while ignoring top faces that project towards the centre.
    const auto & bb = result.bbox;
    result.contact.reserve(static_cast<std::size_t>(bb.width));
    for (int x = bb.x; x < bb.right(); ++x) {
      for (int y = bb.bottom() - 1; y >= bb.y; --y) {
        if (labels.at<int>(y, x) == best) {
          result.contact.push_back(ContactPixel{x, y + 1});
          break;
        }
      }
    }
  }

  if (mask_out != nullptr) {
    *mask_out = mask;
  }
  return result;
}

}  // namespace vision_safety
