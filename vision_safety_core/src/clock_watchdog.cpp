// Copyright 2026 Kavinaya. MIT License.

#include "vision_safety_core/clock_watchdog.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace vision_safety
{

ClockWatchdog::ClockWatchdog(double stall_timeout_s)
: stall_timeout_s_(stall_timeout_s)
{
  if (!std::isfinite(stall_timeout_s) || stall_timeout_s <= 0.0) {
    throw std::invalid_argument("clock stall timeout must be a finite value > 0");
  }
}

bool ClockWatchdog::update(double ros_now_s, double wall_now_s)
{
  if (!std::isfinite(ros_now_s) || ros_now_s <= 0.0) {
    healthy_ = false;
    reason_ = "ROS clock not running (no /clock received)";
    return healthy_;
  }

  // Any change counts as movement: forward, or backward after a simulation reset.
  if (!last_ros_ || ros_now_s != *last_ros_) {
    last_ros_ = ros_now_s;
    last_advance_wall_ = wall_now_s;
    healthy_ = true;
    reason_.clear();
    return healthy_;
  }

  const double frozen_for = wall_now_s - *last_advance_wall_;
  if (frozen_for > stall_timeout_s_) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "ROS clock stalled for %.2f s", frozen_for);
    healthy_ = false;
    reason_ = buf;
  }
  return healthy_;
}

}  // namespace vision_safety
