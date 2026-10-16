// Copyright 2026 Kavinaya. MIT License.

#pragma once

#include <vector>

namespace vision_safety
{

// Axis-aligned box in pixel coordinates. (x, y) is the top-left corner;
// right() and bottom() are the exclusive edges.
struct BoundingBox
{
  int x{0};
  int y{0};
  int width{0};
  int height{0};

  int right() const {return x + width;}
  int bottom() const {return y + height;}
};

// Lowest blob pixel in one image column. If the object rests on the floor,
// this is where it touches the floor. v is the pixel's bottom EDGE (row + 1),
// matching BoundingBox::bottom().
struct ContactPixel
{
  int u{0};
  int v{0};
};

// Output of the colour detector for one image.
struct BlobDetection
{
  bool detected{false};     // largest blob passed the size filter
  BoundingBox bbox;         // of the largest blob
  double area_ratio{0.0};   // largest blob pixel count / image pixel count
  // One entry per column of the largest blob (its lower outline).
  // May be empty (e.g. hand-built detections); consumers then fall back to the bbox.
  std::vector<ContactPixel> contact;
};

}  // namespace vision_safety
