// Copyright 2026 Kavinaya. MIT License.
//
// SafetyMonitor: the decision logic of the safety layer, with no ROS dependency.
//
// Time is passed in explicitly (seconds, as double) instead of being read from
// a clock. That makes every behaviour deterministic and unit-testable: a test
// can say "detection at t=1.0, then update at t=1.4" without sleeping.
// The ROS node supplies sim-time / wall-time values.

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace vision_safety
{

enum class SafetyState : std::uint8_t
{
  kInit = 0,         // no detection received yet: full stop (fail-safe default)
  kClear = 1,        // fresh perception, path clear: commands pass through (clamped)
  kStopObstacle = 2, // obstacle in danger zone: forward motion blocked
  kStopStale = 3,    // perception missing or too old: full stop
};

const char * to_string(SafetyState state);

struct Velocity
{
  double linear{0.0};   // m/s, + = forward
  double angular{0.0};  // rad/s, + = counter-clockwise
};

struct SafetyConfig
{
  // Detection older than this (now - image stamp) counts as stale.
  double max_detection_age_s{0.3};
  // Upstream command older than this is ignored: output zero.
  double command_timeout_s{0.5};
  // Path must stay clear this long before motion resumes (hysteresis).
  double clear_hold_time_s{0.5};
  // true: when blocked, still allow reversing and turning in place.
  // false: when blocked, stop completely.
  bool block_forward_only{true};
  // Output limits applied in every state.
  double max_linear_speed{0.5};
  double max_reverse_speed{0.3};
  double max_angular_speed{1.5};
};

// Throws std::invalid_argument if any value is out of range.
void validate(const SafetyConfig & config);

struct SafetyOutput
{
  Velocity command;                    // what goes to the robot
  SafetyState state{SafetyState::kInit};
  bool command_timed_out{false};
  double detection_age_s{0.0};         // +infinity if no detection yet
  std::string reason;
};

class SafetyMonitor
{
public:
  explicit SafetyMonitor(SafetyConfig config);

  // A detection result for the image captured at stamp_s.
  // Returns false (and ignores it) if the stamp is not finite or is not newer
  // than the newest detection already seen: late or duplicated messages must
  // never overwrite fresher information.
  bool on_detection(double stamp_s, bool in_danger_zone);

  // An upstream velocity request received at receive_time_s.
  // Returns false if the command contains NaN/inf; it is then replaced by zero.
  bool on_command(double receive_time_s, const Velocity & command);

  // Evaluate the state at time now_s and compute the output command.
  // Call this from a fixed-rate timer, NOT only when messages arrive.
  SafetyOutput update(double now_s);

  // Change limits/timeouts at runtime (live parameter tuning). State, hold
  // timers and history are kept. Throws std::invalid_argument and keeps the
  // old config if the new one is invalid.
  void set_config(const SafetyConfig & config);

  SafetyState state() const {return state_;}
  const SafetyConfig & config() const {return config_;}

private:
  Velocity clamp(const Velocity & v) const;

  SafetyConfig config_;
  SafetyState state_{SafetyState::kInit};

  std::optional<double> last_detection_stamp_;
  bool last_in_danger_zone_{true};
  std::optional<double> clear_since_;  // stamp of the first clear frame in the current clear run

  std::optional<double> last_command_time_;
  Velocity last_command_;
};

}  // namespace vision_safety
