// Copyright 2026 Kavinaya. MIT License.
//
// safety_monitor: the ONLY node allowed to command the robot.
//
//   subscribes  perception/obstacle  (ObstacleDetection, stamped with the image time)
//               cmd_vel_raw          (geometry_msgs/Twist from teleop / planner / cruise_driver)
//   publishes   cmd_vel              (geometry_msgs/Twist -> robot), from a fixed-rate TIMER
//               safety/status        (SafetyStatus, every cycle)
//               /diagnostics         (via diagnostic_updater, 1 Hz)
//
// Why a timer and not "forward each command": the simulated DiffDrive (like
// many motor drivers) keeps executing the LAST command it received. If we only
// published when a command arrived, a stop decision made between commands would
// never reach the robot, and silence upstream would mean "keep driving".
//
// The timer runs on WALL time, not ROS time. In simulation, ROS time comes from
// /clock; if /clock stopped, a ROS-time timer would stop too and the robot would
// keep its last command. A ClockWatchdog checks that ROS time is advancing and
// forces a zero command when it is not.
//
// All decisions are made by vision_safety::SafetyMonitor (unit-tested in
// vision_safety_core). This node only moves data between ROS and that class.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "vision_safety_core/clock_watchdog.hpp"
#include "vision_safety_core/safety_monitor.hpp"
#include "vision_safety_interfaces/msg/obstacle_detection.hpp"
#include "vision_safety_interfaces/msg/safety_status.hpp"

namespace vision_safety_ros
{

using geometry_msgs::msg::Twist;
using vision_safety::SafetyConfig;
using vision_safety::SafetyMonitor;
using vision_safety::SafetyOutput;
using vision_safety::SafetyState;
using vision_safety::Velocity;
using vision_safety_interfaces::msg::ObstacleDetection;
using vision_safety_interfaces::msg::SafetyStatus;

// The message constants and the core enum must never drift apart.
static_assert(SafetyStatus::STATE_INIT == static_cast<uint8_t>(SafetyState::kInit));
static_assert(SafetyStatus::STATE_CLEAR == static_cast<uint8_t>(SafetyState::kClear));
static_assert(SafetyStatus::STATE_STOP_OBSTACLE == static_cast<uint8_t>(SafetyState::kStopObstacle));
static_assert(SafetyStatus::STATE_STOP_STALE == static_cast<uint8_t>(SafetyState::kStopStale));

namespace
{

using ParamGetter = std::function<rclcpp::Parameter(const std::string &)>;

SafetyConfig read_config(const ParamGetter & get)
{
  SafetyConfig c;
  c.max_detection_age_s = get("max_detection_age_s").as_double();
  c.command_timeout_s = get("command_timeout_s").as_double();
  c.clear_hold_time_s = get("clear_hold_time_s").as_double();
  c.block_forward_only = get("block_forward_only").as_bool();
  c.max_linear_speed = get("max_linear_speed").as_double();
  c.max_reverse_speed = get("max_reverse_speed").as_double();
  c.max_angular_speed = get("max_angular_speed").as_double();
  vision_safety::validate(c);
  return c;
}

// A backwards jump larger than this (e.g. Gazebo world reset) resets the monitor.
constexpr double kTimeJumpResetS = 1.0;

}  // namespace

class SafetyMonitorNode : public rclcpp::Node
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions & options)
  : Node("safety_monitor", options), diagnostics_(this)
  {
    declare_all_parameters();

    const auto config = read_config([this](const std::string & n) {return get_parameter(n);});
    monitor_ = std::make_unique<SafetyMonitor>(config);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!std::isfinite(rate_hz) || rate_hz < 1.0 || rate_hz > 1000.0) {
      throw std::invalid_argument("rate_hz must be in [1, 1000]");
    }
    clock_watchdog_ = std::make_unique<vision_safety::ClockWatchdog>(
      get_parameter("clock_stall_timeout_s").as_double());

    on_set_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changes) {return validate_changes(changes);});
    post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {apply_parameters();});

    cmd_pub_ = create_publisher<Twist>("cmd_vel", rclcpp::QoS(10));
    status_pub_ = create_publisher<SafetyStatus>("safety/status", rclcpp::QoS(10));

    detection_sub_ = create_subscription<ObstacleDetection>(
      "perception/obstacle", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](ObstacleDetection::ConstSharedPtr msg) {on_detection(*msg);});
    cmd_sub_ = create_subscription<Twist>(
      "cmd_vel_raw", rclcpp::QoS(10),
      [this](Twist::ConstSharedPtr msg) {on_command(*msg);});

    // WALL timer: the heartbeat must not depend on /clock (see header comment).
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() {on_timer();});

    diagnostics_.setHardwareID("vision_bot");
    diagnostics_.add("safety_monitor", this, &SafetyMonitorNode::produce_diagnostics);
    diag_window_start_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(), "safety_monitor ready: %.0f Hz, stale after %.2f s, hold %.2f s, %s",
      rate_hz, config.max_detection_age_s, config.clear_hold_time_s,
      config.block_forward_only ? "blocks forward motion only" : "full stop when blocked");
  }

  ~SafetyMonitorNode() override
  {
    // Best effort: leave the robot with a zero command, not the last one.
    if (rclcpp::ok() && cmd_pub_) {
      cmd_pub_->publish(Twist{});
    }
  }

private:
  // ------------------------------------------------------------------ params
  template<typename T>
  void declare(
    const std::string & name, const T & default_value, const std::string & description,
    bool read_only = false)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    d.read_only = read_only;
    declare_parameter<T>(name, default_value, d);
  }

  void declare_all_parameters()
  {
    declare<double>("rate_hz", 20.0, "output rate of cmd_vel and safety/status (wall time)", true);
    declare<double>(
      "clock_stall_timeout_s", 0.5,
      "ROS time standing still this long (wall time) forces a stop", true);
    declare<double>("max_detection_age_s", 0.3, "older perception counts as stale -> full stop");
    declare<double>("command_timeout_s", 0.5, "older cmd_vel_raw is ignored -> zero output");
    declare<double>("clear_hold_time_s", 0.5, "path must stay clear this long before resuming");
    declare<bool>("block_forward_only", true, "when blocked, still allow reverse and turning");
    declare<double>("max_linear_speed", 0.5, "forward speed limit [m/s]");
    declare<double>("max_reverse_speed", 0.3, "reverse speed limit [m/s]");
    declare<double>("max_angular_speed", 1.5, "turn rate limit [rad/s]");
  }

  rcl_interfaces::msg::SetParametersResult validate_changes(
    const std::vector<rclcpp::Parameter> & changes)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    const auto get = [&](const std::string & name) -> rclcpp::Parameter {
        for (const auto & c : changes) {
          if (c.get_name() == name) {
            return c;
          }
        }
        return get_parameter(name);
      };
    try {
      read_config(get);
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  }

  void apply_parameters()
  {
    try {
      const auto config = read_config([this](const std::string & n) {return get_parameter(n);});
      std::lock_guard<std::mutex> lock(mutex_);
      monitor_->set_config(config);
      RCLCPP_INFO(get_logger(), "safety parameters updated");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "keeping previous safety parameters: %s", e.what());
    }
  }

  // --------------------------------------------------------------- callbacks
  void on_detection(const ObstacleDetection & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()).seconds();
    std::lock_guard<std::mutex> lock(mutex_);

    // Simulation reset: time jumped backwards. Without this, every new
    // detection would look "older than the last one" and be rejected forever.
    if (last_detection_stamp_ && stamp + kTimeJumpResetS < *last_detection_stamp_) {
      RCLCPP_WARN(
        get_logger(), "time jumped backwards (%.2f -> %.2f s): resetting safety state",
        *last_detection_stamp_, stamp);
      monitor_ = std::make_unique<SafetyMonitor>(monitor_->config());
      last_detection_stamp_.reset();
    }

    if (monitor_->on_detection(stamp, msg.in_danger_zone)) {
      last_detection_stamp_ = stamp;
      ++detections_in_window_;
    } else {
      ++rejected_detections_;
      RCLCPP_DEBUG(get_logger(), "rejected out-of-order detection stamped %.3f s", stamp);
    }
  }

  void on_command(const Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_requested_ = msg;
    if (!monitor_->on_command(now().seconds(), Velocity{msg.linear.x, msg.angular.z})) {
      ++rejected_commands_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "non-finite cmd_vel_raw received, treated as stop");
    }
  }

  void on_timer()
  {
    const rclcpp::Time stamp = now();
    const double wall_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    SafetyOutput out;
    Twist requested;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool clock_ok = clock_watchdog_->update(stamp.seconds(), wall_s);
      out = monitor_->update(stamp.seconds());
      if (!clock_ok) {
        // Without a running clock, freshness cannot be judged: never move.
        if (out.state != SafetyState::kInit) {
          out.state = SafetyState::kStopStale;
        }
        out.command = Velocity{};
        out.reason = clock_watchdog_->reason();
      }
      clock_ok_ = clock_ok;
      requested = last_requested_;
      last_output_ = out;
    }

    Twist cmd;
    cmd.linear.x = out.command.linear;
    cmd.angular.z = out.command.angular;
    cmd_pub_->publish(cmd);

    if (out.state != last_state_) {
      log_transition(last_state_, out);
      if (out.state == SafetyState::kStopObstacle || out.state == SafetyState::kStopStale) {
        ++stop_events_;
      }
      last_state_ = out.state;
    }

    SafetyStatus status;
    status.header.stamp = stamp;
    status.state = static_cast<uint8_t>(out.state);
    status.reason = out.reason;
    status.detection_age_s = out.detection_age_s;
    status.command_timed_out = out.command_timed_out;
    status.requested = requested;
    status.commanded = cmd;
    status_pub_->publish(status);
  }

  void log_transition(SafetyState from, const SafetyOutput & out)
  {
    const char * from_s = vision_safety::to_string(from);
    const char * to_s = vision_safety::to_string(out.state);
    switch (out.state) {
      case SafetyState::kStopStale:
        RCLCPP_ERROR(get_logger(), "%s -> %s: %s", from_s, to_s, out.reason.c_str());
        break;
      case SafetyState::kStopObstacle:
        RCLCPP_WARN(get_logger(), "%s -> %s: %s", from_s, to_s, out.reason.c_str());
        break;
      default:
        RCLCPP_INFO(get_logger(), "%s -> %s: %s", from_s, to_s, out.reason.c_str());
        break;
    }
  }

  // ------------------------------------------------------------- diagnostics
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    SafetyOutput out;
    uint64_t detections = 0;
    bool clock_ok = false;
    std::string clock_reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      out = last_output_;
      clock_ok = clock_ok_;
      clock_reason = clock_watchdog_->reason();
      detections = detections_in_window_;
      detections_in_window_ = 0;
    }
    const auto t = std::chrono::steady_clock::now();
    const double window_s = std::chrono::duration<double>(t - diag_window_start_).count();
    diag_window_start_ = t;
    const double detection_rate = window_s > 0.0 ? static_cast<double>(detections) / window_s : 0.0;

    switch (out.state) {
      case SafetyState::kClear:
        stat.summary(DiagnosticStatus::OK, "path clear");
        break;
      case SafetyState::kStopObstacle:
        // Working as intended: healthy system, currently blocking.
        stat.summary(DiagnosticStatus::OK, "blocking forward motion: obstacle in zone");
        break;
      case SafetyState::kInit:
        stat.summary(DiagnosticStatus::WARN, "waiting for perception");
        break;
      case SafetyState::kStopStale:
        stat.summary(DiagnosticStatus::ERROR, out.reason);
        break;
    }

    if (!clock_ok) {
      stat.mergeSummary(DiagnosticStatus::ERROR, clock_reason);
    }

    // T7 at runtime: any other publisher on cmd_vel can bypass this node.
    const std::size_t publishers = count_publishers("cmd_vel");
    if (publishers > 1) {
      stat.mergeSummaryf(
        DiagnosticStatus::ERROR,
        "%zu publishers on cmd_vel: safety layer can be bypassed (remap them to cmd_vel_raw)",
        publishers);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%zu publishers on cmd_vel! Only safety_monitor may publish there; "
        "remap other nodes to cmd_vel_raw", publishers);
    }

    stat.add("state", vision_safety::to_string(out.state));
    stat.add("ros_clock", clock_ok ? std::string("running") : clock_reason);
    stat.add("reason", out.reason);
    stat.addf("detection_age_s", "%.3f", out.detection_age_s);
    stat.addf("detection_rate_hz", "%.1f", detection_rate);
    stat.add("command_timed_out", out.command_timed_out);
    stat.add("stop_events", stop_events_);
    stat.add("rejected_detections", rejected_detections_);
    stat.add("rejected_commands", rejected_commands_);
    stat.add("cmd_vel_publishers", publishers);
  }

  // ------------------------------------------------------------------- state
  std::mutex mutex_;
  std::unique_ptr<SafetyMonitor> monitor_;
  std::unique_ptr<vision_safety::ClockWatchdog> clock_watchdog_;
  bool clock_ok_{false};
  std::optional<double> last_detection_stamp_;
  Twist last_requested_;
  SafetyOutput last_output_;
  SafetyState last_state_{SafetyState::kInit};

  uint64_t stop_events_{0};
  uint64_t rejected_detections_{0};
  uint64_t rejected_commands_{0};
  uint64_t detections_in_window_{0};
  std::chrono::steady_clock::time_point diag_window_start_;

  diagnostic_updater::Updater diagnostics_;

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<ObstacleDetection>::SharedPtr detection_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

}  // namespace vision_safety_ros

RCLCPP_COMPONENTS_REGISTER_NODE(vision_safety_ros::SafetyMonitorNode)
