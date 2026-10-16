// Copyright 2026 Kavinaya. MIT License.
//
// Unit tests for SafetyMonitor. No ROS, no Gazebo: time is injected.
// Test names reference the project scenarios T1-T7 where they apply.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>

#include "vision_safety_core/safety_monitor.hpp"

using vision_safety::SafetyConfig;
using vision_safety::SafetyMonitor;
using vision_safety::SafetyOutput;
using vision_safety::SafetyState;
using vision_safety::Velocity;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

SafetyConfig default_config()
{
  SafetyConfig c;
  c.max_detection_age_s = 0.3;
  c.command_timeout_s = 0.5;
  c.clear_hold_time_s = 0.5;
  c.block_forward_only = true;
  c.max_linear_speed = 0.5;
  c.max_reverse_speed = 0.3;
  c.max_angular_speed = 1.5;
  return c;
}

const Velocity kForward{0.4, 0.0};

// Feed clear detections + a forward command every `dt` from t0 to t1 (inclusive),
// calling update each step. Returns the last output.
SafetyOutput drive_clear(SafetyMonitor & m, double t0, double t1, double dt = 0.05)
{
  SafetyOutput out;
  for (double t = t0; t <= t1 + 1e-9; t += dt) {
    m.on_detection(t, false);
    m.on_command(t, kForward);
    out = m.update(t);
  }
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Start-up behaviour
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, StartsInInitAndStopsBeforeAnyDetection)
{
  SafetyMonitor m(default_config());
  m.on_command(0.0, kForward);
  const auto out = m.update(0.0);
  EXPECT_EQ(out.state, SafetyState::kInit);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
  EXPECT_TRUE(std::isinf(out.detection_age_s));
}

TEST(SafetyMonitor, FirstClearDetectionStillRequiresHoldTime)
{
  SafetyMonitor m(default_config());
  m.on_detection(0.0, false);
  m.on_command(0.0, kForward);
  const auto out = m.update(0.0);
  EXPECT_NE(out.state, SafetyState::kClear);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
}

// ---------------------------------------------------------------------------
// T1: no obstacle -> robot keeps moving
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, T1_ClearPathPassesCommandAfterHoldTime)
{
  SafetyMonitor m(default_config());
  const auto out = drive_clear(m, 0.0, 0.6);
  EXPECT_EQ(out.state, SafetyState::kClear);
  EXPECT_DOUBLE_EQ(out.command.linear, kForward.linear);
}

// ---------------------------------------------------------------------------
// T2: obstacle enters danger zone -> forward motion stops
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, T2_ObstacleInZoneBlocksForwardImmediately)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);

  m.on_detection(1.05, true);
  m.on_command(1.05, kForward);
  const auto out = m.update(1.05);

  EXPECT_EQ(out.state, SafetyState::kStopObstacle);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
}

TEST(SafetyMonitor, BlockForwardOnlyPolicyAllowsReverseAndTurning)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  m.on_detection(1.05, true);

  m.on_command(1.05, Velocity{-0.2, 0.8});
  const auto out = m.update(1.05);

  EXPECT_EQ(out.state, SafetyState::kStopObstacle);
  EXPECT_DOUBLE_EQ(out.command.linear, -0.2);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.8);
}

TEST(SafetyMonitor, FullStopPolicyBlocksEverything)
{
  auto cfg = default_config();
  cfg.block_forward_only = false;
  SafetyMonitor m(cfg);
  drive_clear(m, 0.0, 1.0);
  m.on_detection(1.05, true);

  m.on_command(1.05, Velocity{-0.2, 0.8});
  const auto out = m.update(1.05);

  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
}

// ---------------------------------------------------------------------------
// T3: obstacle leaves zone -> motion resumes, but only after the hold time
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, T3_ResumesOnlyAfterPathStaysClearForHoldTime)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  m.on_detection(1.05, true);
  m.update(1.05);

  // Clear again from t=1.1. Hold time is 0.5 s -> still blocked at t=1.5.
  SafetyOutput out;
  for (double t = 1.1; t <= 1.5 + 1e-9; t += 0.05) {
    m.on_detection(t, false);
    m.on_command(t, kForward);
    out = m.update(t);
  }
  EXPECT_EQ(out.state, SafetyState::kStopObstacle);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);

  // ...and released once 0.5 s of continuous clear frames have passed.
  out = drive_clear(m, 1.55, 1.65);
  EXPECT_EQ(out.state, SafetyState::kClear);
  EXPECT_DOUBLE_EQ(out.command.linear, kForward.linear);
}

TEST(SafetyMonitor, FlickeringDetectionNeverReleasesTheStop)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);

  // Obstacle visible every 4th frame: each sighting must restart the hold timer.
  SafetyOutput out;
  int frame = 0;
  for (double t = 1.05; t <= 3.0; t += 0.05, ++frame) {
    m.on_detection(t, frame % 4 == 0);
    m.on_command(t, kForward);
    out = m.update(t);
    ASSERT_DOUBLE_EQ(out.command.linear, 0.0) << "robot moved at t=" << t;
  }
}

// ---------------------------------------------------------------------------
// T5: camera / detections stop arriving -> robot stops
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, T5_MissingDetectionsCauseStaleStop)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);

  // Commands keep coming, detections stop after t=1.0. Max age 0.3 s.
  m.on_command(1.25, kForward);
  EXPECT_EQ(m.update(1.25).state, SafetyState::kClear);  // age 0.25: still fresh

  m.on_command(1.35, kForward);
  const auto out = m.update(1.35);                        // age 0.35: stale
  EXPECT_EQ(out.state, SafetyState::kStopStale);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
  EXPECT_NEAR(out.detection_age_s, 0.35, 1e-9);
}

TEST(SafetyMonitor, RecoveryFromStaleRequiresFreshHoldTime)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  m.update(2.0);  // long gap: stale

  m.on_detection(2.0, false);
  m.on_command(2.0, kForward);
  EXPECT_NE(m.update(2.0).state, SafetyState::kClear);

  const auto out = drive_clear(m, 2.05, 2.55);
  EXPECT_EQ(out.state, SafetyState::kClear);
}

// ---------------------------------------------------------------------------
// T6: late / out-of-order detections are rejected
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, T6_OlderDetectionCannotOverwriteNewerOne)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  ASSERT_TRUE(m.on_detection(1.05, true));   // obstacle seen in frame 1.05

  // A delayed "clear" result for an older frame arrives afterwards.
  EXPECT_FALSE(m.on_detection(1.02, false));
  EXPECT_FALSE(m.on_detection(1.05, false));  // duplicate stamp also rejected

  m.on_command(1.06, kForward);
  EXPECT_EQ(m.update(1.06).state, SafetyState::kStopObstacle);
}

TEST(SafetyMonitor, T6_DelayedDetectionThatIsAlreadyTooOldIsStale)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  // Detection for an image taken at 1.05 only arrives at t=1.5 (0.45 s late).
  m.on_detection(1.05, false);
  m.on_command(1.5, kForward);
  EXPECT_EQ(m.update(1.5).state, SafetyState::kStopStale);
}

TEST(SafetyMonitor, NonFiniteDetectionStampIsRejected)
{
  SafetyMonitor m(default_config());
  EXPECT_FALSE(m.on_detection(kNaN, false));
  EXPECT_FALSE(m.on_detection(kInf, false));
  EXPECT_EQ(m.update(0.0).state, SafetyState::kInit);
}

// ---------------------------------------------------------------------------
// T7: forward commands while blocked cannot bypass the safety layer
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, T7_RepeatedForwardCommandsWhileBlockedAreOverridden)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);

  for (double t = 1.05; t <= 2.0; t += 0.01) {
    m.on_detection(t, true);
    m.on_command(t, Velocity{10.0, 0.0});  // even absurdly large requests
    const auto out = m.update(t);
    ASSERT_LE(out.command.linear, 0.0) << "forward motion leaked at t=" << t;
  }
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, CommandTimeoutForcesZeroEvenWhenClear)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);  // last command at t=1.0

  for (double t = 1.05; t <= 1.6 + 1e-9; t += 0.05) {
    m.on_detection(t, false);  // perception still healthy
  }
  const auto out = m.update(1.6);  // command age 0.6 > timeout 0.5
  EXPECT_EQ(out.state, SafetyState::kClear);
  EXPECT_TRUE(out.command_timed_out);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
}

TEST(SafetyMonitor, NoCommandEverReceivedMeansZeroOutput)
{
  SafetyMonitor m(default_config());
  for (double t = 0.0; t <= 1.0; t += 0.05) {
    m.on_detection(t, false);
  }
  const auto out = m.update(1.0);
  EXPECT_TRUE(out.command_timed_out);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
}

TEST(SafetyMonitor, NonFiniteCommandIsReplacedByZero)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  EXPECT_FALSE(m.on_command(1.05, Velocity{kNaN, 0.5}));
  m.on_detection(1.05, false);
  const auto out = m.update(1.05);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
}

TEST(SafetyMonitor, CommandsAreClampedToConfiguredLimits)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);

  m.on_detection(1.05, false);
  m.on_command(1.05, Velocity{5.0, -9.0});
  auto out = m.update(1.05);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.5);
  EXPECT_DOUBLE_EQ(out.command.angular, -1.5);

  m.on_detection(1.1, false);
  m.on_command(1.1, Velocity{-5.0, 9.0});
  out = m.update(1.1);
  EXPECT_DOUBLE_EQ(out.command.linear, -0.3);
  EXPECT_DOUBLE_EQ(out.command.angular, 1.5);
}

TEST(SafetyMonitor, EveryOutputHasAReason)
{
  SafetyMonitor m(default_config());
  EXPECT_FALSE(m.update(0.0).reason.empty());
  EXPECT_FALSE(drive_clear(m, 0.0, 1.0).reason.empty());
  m.on_detection(1.05, true);
  EXPECT_FALSE(m.update(1.05).reason.empty());
  EXPECT_FALSE(m.update(5.0).reason.empty());
}

// ---------------------------------------------------------------------------
// Decision table: every combination of perception state x command x policy
// ---------------------------------------------------------------------------

enum class Perception { kNone, kStale, kFreshClear, kFreshBlocked };

struct DecisionCase
{
  const char * name;   // readable test name in gtest output
  Perception perception;
  Velocity requested;
  bool block_forward_only;
  double expected_linear;
  double expected_angular;
  SafetyState expected_state;
};

// Print the case name instead of raw bytes when a parameterised test fails.
void PrintTo(const DecisionCase & c, std::ostream * os) {*os << c.name;}

class SafetyDecisionTable : public ::testing::TestWithParam<DecisionCase> {};

TEST_P(SafetyDecisionTable, OutputMatchesTable)
{
  const auto & c = GetParam();
  auto cfg = default_config();
  cfg.block_forward_only = c.block_forward_only;
  SafetyMonitor m(cfg);

  double now = 0.0;
  switch (c.perception) {
    case Perception::kNone:
      break;
    case Perception::kStale:
      drive_clear(m, 0.0, 1.0);
      now = 2.0;
      break;
    case Perception::kFreshClear:
      drive_clear(m, 0.0, 1.0);
      now = 1.05;
      m.on_detection(now, false);
      break;
    case Perception::kFreshBlocked:
      drive_clear(m, 0.0, 1.0);
      now = 1.05;
      m.on_detection(now, true);
      break;
  }
  m.on_command(now, c.requested);
  const auto out = m.update(now);

  EXPECT_EQ(out.state, c.expected_state);
  EXPECT_DOUBLE_EQ(out.command.linear, c.expected_linear);
  EXPECT_DOUBLE_EQ(out.command.angular, c.expected_angular);
}

INSTANTIATE_TEST_SUITE_P(
  AllCombinations, SafetyDecisionTable,
  ::testing::Values(
    // name, perception, request {lin, ang}, block_forward_only, expected lin, ang, state
    DecisionCase{"NoPerception_Forward", Perception::kNone, {0.4, 0.5}, true,
      0.0, 0.0, SafetyState::kInit},
    DecisionCase{"NoPerception_Reverse", Perception::kNone, {-0.2, 0.5}, true,
      0.0, 0.0, SafetyState::kInit},
    DecisionCase{"Stale_Forward", Perception::kStale, {0.4, 0.5}, true,
      0.0, 0.0, SafetyState::kStopStale},
    DecisionCase{"Stale_Reverse", Perception::kStale, {-0.2, 0.5}, true,
      0.0, 0.0, SafetyState::kStopStale},
    DecisionCase{"Clear_Forward", Perception::kFreshClear, {0.4, 0.5}, true,
      0.4, 0.5, SafetyState::kClear},
    DecisionCase{"Clear_Reverse", Perception::kFreshClear, {-0.2, 0.5}, true,
      -0.2, 0.5, SafetyState::kClear},
    DecisionCase{"Blocked_Forward_ForwardOnlyPolicy", Perception::kFreshBlocked, {0.4, 0.5},
      true, 0.0, 0.5, SafetyState::kStopObstacle},
    DecisionCase{"Blocked_Reverse_ForwardOnlyPolicy", Perception::kFreshBlocked, {-0.2, 0.5},
      true, -0.2, 0.5, SafetyState::kStopObstacle},
    DecisionCase{"Blocked_Forward_FullStopPolicy", Perception::kFreshBlocked, {0.4, 0.5},
      false, 0.0, 0.0, SafetyState::kStopObstacle},
    DecisionCase{"Blocked_Reverse_FullStopPolicy", Perception::kFreshBlocked, {-0.2, 0.5},
      false, 0.0, 0.0, SafetyState::kStopObstacle}),
  [](const ::testing::TestParamInfo<DecisionCase> & case_info) {
    return std::string(case_info.param.name);
  });

// ---------------------------------------------------------------------------
// Live reconfiguration
// ---------------------------------------------------------------------------

TEST(SafetyMonitor, SetConfigKeepsStateAndAppliesNewLimits)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  ASSERT_EQ(m.state(), SafetyState::kClear);

  auto slower = default_config();
  slower.max_linear_speed = 0.2;
  m.set_config(slower);

  m.on_detection(1.05, false);
  m.on_command(1.05, kForward);
  const auto out = m.update(1.05);
  EXPECT_EQ(out.state, SafetyState::kClear);   // no new hold time needed
  EXPECT_DOUBLE_EQ(out.command.linear, 0.2);
}

TEST(SafetyMonitor, InvalidSetConfigIsRejectedAndOldConfigKept)
{
  SafetyMonitor m(default_config());
  auto bad = default_config();
  bad.max_detection_age_s = -1.0;
  EXPECT_THROW(m.set_config(bad), std::invalid_argument);
  EXPECT_DOUBLE_EQ(m.config().max_detection_age_s, 0.3);
}

TEST(SafetyMonitor, ShorterStaleTimeoutTakesEffectImmediately)
{
  SafetyMonitor m(default_config());
  drive_clear(m, 0.0, 1.0);
  m.on_command(1.2, kForward);
  EXPECT_EQ(m.update(1.2).state, SafetyState::kClear);   // age 0.2 < 0.3

  auto strict = default_config();
  strict.max_detection_age_s = 0.1;
  m.set_config(strict);
  EXPECT_EQ(m.update(1.2).state, SafetyState::kStopStale);  // age 0.2 > 0.1
}

// ---------------------------------------------------------------------------
// Configuration validation
// ---------------------------------------------------------------------------

TEST(SafetyConfigValidation, RejectsNonPositiveDurations)
{
  auto c = default_config();
  c.max_detection_age_s = 0.0;
  EXPECT_THROW(SafetyMonitor{c}, std::invalid_argument);

  c = default_config();
  c.command_timeout_s = -1.0;
  EXPECT_THROW(SafetyMonitor{c}, std::invalid_argument);

  c = default_config();
  c.clear_hold_time_s = kNaN;
  EXPECT_THROW(SafetyMonitor{c}, std::invalid_argument);
}

TEST(SafetyConfigValidation, RejectsNegativeSpeedLimits)
{
  auto c = default_config();
  c.max_linear_speed = -0.1;
  EXPECT_THROW(SafetyMonitor{c}, std::invalid_argument);

  c = default_config();
  c.max_angular_speed = kInf;
  EXPECT_THROW(SafetyMonitor{c}, std::invalid_argument);
}

TEST(SafetyConfigValidation, AcceptsDefaults)
{
  EXPECT_NO_THROW(SafetyMonitor{SafetyConfig{}});
}

TEST(SafetyStateNames, AllStatesHaveNames)
{
  EXPECT_STREQ(vision_safety::to_string(SafetyState::kInit), "INIT");
  EXPECT_STREQ(vision_safety::to_string(SafetyState::kClear), "CLEAR");
  EXPECT_STREQ(vision_safety::to_string(SafetyState::kStopObstacle), "STOP_OBSTACLE");
  EXPECT_STREQ(vision_safety::to_string(SafetyState::kStopStale), "STOP_STALE");
}
