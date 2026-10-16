// Copyright 2026 Kavinaya. MIT License.

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "vision_safety_core/clock_watchdog.hpp"

using vision_safety::ClockWatchdog;

TEST(ClockWatchdog, UnhealthyUntilRosTimeIsRunning)
{
  ClockWatchdog w(0.5);
  EXPECT_FALSE(w.healthy());
  EXPECT_FALSE(w.update(0.0, 100.0));       // ROS time 0: /clock never arrived
  EXPECT_FALSE(w.update(0.0, 105.0));
  EXPECT_NE(w.reason().find("not running"), std::string::npos);
}

TEST(ClockWatchdog, HealthyWhileRosTimeAdvances)
{
  ClockWatchdog w(0.5);
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(w.update(10.0 + 0.05 * i, 100.0 + 0.05 * i));
  }
  EXPECT_TRUE(w.reason().empty());
}

TEST(ClockWatchdog, SlowSimulationIsNotAStall)
{
  // Simulation at 30 % real-time: ROS time advances slowly but steadily.
  ClockWatchdog w(0.5);
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(w.update(10.0 + 0.015 * i, 100.0 + 0.05 * i));
  }
}

TEST(ClockWatchdog, FrozenRosTimeIsDetectedAfterTheTimeout)
{
  ClockWatchdog w(0.5);
  w.update(10.0, 100.0);
  w.update(10.05, 100.05);                  // last advance at wall 100.05
  EXPECT_TRUE(w.update(10.05, 100.5));      // frozen for 0.45 s: still within timeout
  EXPECT_FALSE(w.update(10.05, 100.6));     // frozen for 0.55 s: stalled
  EXPECT_NE(w.reason().find("stalled"), std::string::npos);
}

TEST(ClockWatchdog, RecoversWhenRosTimeMovesAgain)
{
  ClockWatchdog w(0.5);
  w.update(10.0, 100.0);
  EXPECT_FALSE(w.update(10.0, 101.0));
  EXPECT_TRUE(w.update(10.1, 101.05));
  EXPECT_TRUE(w.reason().empty());
}

TEST(ClockWatchdog, BackwardJumpIsMovementNotAStall)
{
  ClockWatchdog w(0.5);
  w.update(50.0, 100.0);
  EXPECT_TRUE(w.update(0.2, 100.1));        // simulation reset
  EXPECT_TRUE(w.update(0.25, 100.15));
}

TEST(ClockWatchdog, RejectsInvalidTimeout)
{
  EXPECT_THROW(ClockWatchdog(0.0), std::invalid_argument);
  EXPECT_THROW(ClockWatchdog(-1.0), std::invalid_argument);
  EXPECT_THROW(ClockWatchdog(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
