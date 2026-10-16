// Copyright 2026 Kavinaya. MIT License.

#include "vision_safety_core/safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace vision_safety
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

void require_positive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be a finite value > 0");
  }
}

void require_non_negative(double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be a finite value >= 0");
  }
}

std::string format_seconds(double s)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << s << " s";
  return os.str();
}

}  // namespace

const char * to_string(SafetyState state)
{
  switch (state) {
    case SafetyState::kInit: return "INIT";
    case SafetyState::kClear: return "CLEAR";
    case SafetyState::kStopObstacle: return "STOP_OBSTACLE";
    case SafetyState::kStopStale: return "STOP_STALE";
  }
  return "UNKNOWN";
}

void validate(const SafetyConfig & config)
{
  require_positive(config.max_detection_age_s, "max_detection_age_s");
  require_positive(config.command_timeout_s, "command_timeout_s");
  require_positive(config.clear_hold_time_s, "clear_hold_time_s");
  require_non_negative(config.max_linear_speed, "max_linear_speed");
  require_non_negative(config.max_reverse_speed, "max_reverse_speed");
  require_non_negative(config.max_angular_speed, "max_angular_speed");
}

SafetyMonitor::SafetyMonitor(SafetyConfig config)
: config_(config)
{
  validate(config_);
}

void SafetyMonitor::set_config(const SafetyConfig & config)
{
  validate(config);  // throws before anything is changed
  config_ = config;
}

bool SafetyMonitor::on_detection(double stamp_s, bool in_danger_zone)
{
  if (!std::isfinite(stamp_s)) {
    return false;
  }
  // Late or duplicated results must never overwrite fresher information.
  if (last_detection_stamp_ && stamp_s <= *last_detection_stamp_) {
    return false;
  }

  last_detection_stamp_ = stamp_s;
  last_in_danger_zone_ = in_danger_zone;

  if (in_danger_zone) {
    clear_since_.reset();           // any sighting restarts the hold timer
  } else if (!clear_since_) {
    clear_since_ = stamp_s;         // first clear frame of a new clear run
  }
  return true;
}

bool SafetyMonitor::on_command(double receive_time_s, const Velocity & command)
{
  last_command_time_ = receive_time_s;
  if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) {
    last_command_ = Velocity{};     // corrupt input: treat as "stop"
    return false;
  }
  last_command_ = command;
  return true;
}

Velocity SafetyMonitor::clamp(const Velocity & v) const
{
  return Velocity{
    std::clamp(v.linear, -config_.max_reverse_speed, config_.max_linear_speed),
    std::clamp(v.angular, -config_.max_angular_speed, config_.max_angular_speed)};
}

SafetyOutput SafetyMonitor::update(double now_s)
{
  SafetyOutput out;

  // ---- 1. Perception state -------------------------------------------------
  if (!last_detection_stamp_) {
    state_ = SafetyState::kInit;
    out.detection_age_s = kInf;
    out.reason = "waiting for first detection";
  } else {
    // A stamp slightly in the future (clock jitter) counts as age 0.
    const double age = std::max(0.0, now_s - *last_detection_stamp_);
    out.detection_age_s = age;

    if (age > config_.max_detection_age_s) {
      state_ = SafetyState::kStopStale;
      clear_since_.reset();         // after a gap, the hold time starts over
      out.reason = "perception stale: last detection " + format_seconds(age) + " old";
    } else if (last_in_danger_zone_) {
      state_ = SafetyState::kStopObstacle;
      out.reason = "obstacle in danger zone";
    } else {
      if (!clear_since_) {
        clear_since_ = *last_detection_stamp_;
      }
      const double held = now_s - *clear_since_;
      if (state_ == SafetyState::kClear || held >= config_.clear_hold_time_s) {
        state_ = SafetyState::kClear;
        out.reason = "path clear";
      } else {
        // Hysteresis: keep the previous stop state until the path has been
        // clear for long enough, so a flickering detection cannot release it.
        out.reason = "path clear, holding stop (" + format_seconds(held) + " of " +
          format_seconds(config_.clear_hold_time_s) + ")";
      }
    }
  }
  out.state = state_;

  // ---- 2. Command gating ---------------------------------------------------
  out.command_timed_out = !last_command_time_ ||
    (now_s - *last_command_time_) > config_.command_timeout_s;

  const Velocity requested = out.command_timed_out ? Velocity{} : clamp(last_command_);

  switch (state_) {
    case SafetyState::kClear:
      out.command = requested;
      break;
    case SafetyState::kStopObstacle:
      if (config_.block_forward_only) {
        out.command = Velocity{std::min(requested.linear, 0.0), requested.angular};
      } else {
        out.command = Velocity{};
      }
      break;
    case SafetyState::kInit:
    case SafetyState::kStopStale:
      out.command = Velocity{};
      break;
  }

  if (out.command_timed_out && state_ == SafetyState::kClear) {
    out.reason += "; no recent command";
  }
  return out;
}

}  // namespace vision_safety
