// Copyright 2026 Kavinaya. MIT License.
//
// ClockWatchdog: detects that ROS time (simulation time) is not running.
//
// In simulation, ROS time comes from /clock. If /clock never arrives or stops
// (bridge crash, simulator hang), ROS time stays at zero or freezes. Everything
// that judges freshness with ROS time would then see "no time has passed" and
// could keep trusting old data. The watchdog compares ROS time against a steady
// wall clock: if ROS time has not advanced for longer than the timeout, the
// clock is unhealthy and the safety layer must command zero.
//
// A backward jump (e.g. a simulation reset) is NOT a stall: the clock is moving.

#pragma once

#include <optional>
#include <string>

namespace vision_safety
{

class ClockWatchdog
{
public:
  // stall_timeout_s: wall time ROS time may stand still before it counts as stalled.
  // Throws std::invalid_argument if not a finite value > 0.
  explicit ClockWatchdog(double stall_timeout_s);

  // ros_now_s: current ROS time; wall_now_s: current steady wall time.
  // Returns true if the clock is healthy.
  bool update(double ros_now_s, double wall_now_s);

  bool healthy() const {return healthy_;}
  // Human-readable explanation when unhealthy (empty when healthy).
  const std::string & reason() const {return reason_;}

private:
  double stall_timeout_s_;
  std::optional<double> last_ros_;
  std::optional<double> last_advance_wall_;
  bool healthy_{false};
  std::string reason_{"ROS clock not running yet"};
};

}  // namespace vision_safety
