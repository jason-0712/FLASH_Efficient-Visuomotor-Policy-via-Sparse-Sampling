#include <gtest/gtest.h>
#include <limits>
#include "flash_fr3_pilot_controller/pilot_core.hpp"

using namespace flash_fr3_pilot_controller;
namespace {
const Joints q{-.1611163, -.7827237, .1119249, -2.3837485, .0714358, 1.5608085, .6637760};
constexpr Joints zero{};
PilotCore armed() {
  PilotCore c;
  EXPECT_TRUE(c.arm(q, q, zero, q, .005, 0., 0.));
  return c;
}
}

TEST(Pilot, FullRoundTripIsFiniteBoundedAndNeverRestarts) {
  auto c = armed();
  double peak = 0.;
  for (int i = 0; i <= 8000; ++i) {
    const double t = i*.001;
    ASSERT_TRUE(c.step(t, t, 0., q, zero)) << i;
    for (std::size_t j = 0; j < 7; ++j) {
      peak = std::max(peak, std::abs(c.reference().q[j] - q[j]));
      if (j != 1) { EXPECT_NEAR(c.reference().q[j], q[j], 1e-14); }
    }
  }
  EXPECT_EQ(c.phase(), Phase::Complete);
  EXPECT_NEAR(peak, .005, 1e-13);
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(c.reference().q[j], q[j], 1e-13);
    EXPECT_DOUBLE_EQ(c.reference().dq[j], 0.);
    EXPECT_DOUBLE_EQ(c.reference().ddq[j], 0.);
  }
}

TEST(Pilot, QuinticPhysicalDerivativesAndBothEndpointConditions) {
  Reference start;
  start.q = q;
  Joints end = q;
  end[1] += .005;
  Segment segment;
  segment.set(start, end, 3.);
  for (double t : {.2, 1.3, 2.5}) {
    const auto r = segment.at(t), plus = segment.at(t+1e-4), minus = segment.at(t-1e-4);
    EXPECT_NEAR((plus.q[1] - minus.q[1])/2e-4, r.dq[1], 1e-10);
    EXPECT_NEAR((plus.dq[1] - minus.dq[1])/2e-4, r.ddq[1], 1e-10);
    EXPECT_NEAR((plus.ddq[1] - minus.ddq[1])/2e-4, r.jerk[1], 1e-10);
  }
  for (double t : {0., 3.}) {
    EXPECT_NEAR(segment.at(t).dq[1], 0., 1e-14);
    EXPECT_NEAR(segment.at(t).ddq[1], 0., 1e-14);
  }
  EXPECT_NEAR(segment.at(3.).q[1], end[1], 1e-14);
}

TEST(Pilot, MissingHeartbeatBeforeStartLatchesWithoutMotion) {
  auto c = armed();
  for (int i = 0; i <= 2200; ++i) ASSERT_TRUE(c.step(i*.001, i*.001, -1., q, zero));
  EXPECT_EQ(c.phase(), Phase::LatchedHold);
  EXPECT_EQ(c.reason(), Reason::StartTimeout);
  ASSERT_TRUE(c.step(2.201, 2.201, 0., q, zero));
  EXPECT_EQ(c.phase(), Phase::LatchedHold);
  EXPECT_EQ(c.reference().q, q);
}

TEST(Pilot, HeartbeatTimeoutBrakesFromEveryStageAndCannotResume) {
  // 120 different stop phases cover both halves and the turning point.
  for (int stop_tick = 25; stop_tick < 6000; stop_tick += 50) {
    auto c = armed();
    for (int i = 0; i <= stop_tick; ++i) ASSERT_TRUE(c.step(i*.001, i*.001, 0., q, zero));
    const auto before = c.reference();
    // Same physical instant isolates the handoff from normal trajectory motion.
    ASSERT_TRUE(c.step(stop_tick*.001, stop_tick*.001, .201, q, zero));
    EXPECT_EQ(c.phase(), Phase::Braking);
    for (std::size_t j = 0; j < 7; ++j) {
      EXPECT_NEAR(c.reference().q[j], before.q[j], 1e-14);
      EXPECT_NEAR(c.reference().dq[j], before.dq[j], 1e-14);
      EXPECT_NEAR(c.reference().ddq[j], before.ddq[j], 1e-14);
    }
    for (int i = 1; i <= 500; ++i) {
      const double t = (stop_tick+i)*.001;
      ASSERT_TRUE(c.step(t, t, 0., q, zero)) << stop_tick << "," << i;
    }
    EXPECT_EQ(c.phase(), Phase::LatchedHold);
    EXPECT_EQ(c.reason(), Reason::HeartbeatTimeout);
    for (std::size_t j = 0; j < 7; ++j) {
      EXPECT_DOUBLE_EQ(c.reference().dq[j], 0.);
      EXPECT_DOUBLE_EQ(c.reference().ddq[j], 0.);
    }
  }
}

TEST(Pilot, RobotAndWallClockFailuresLatchErrors) {
  for (const auto clocks : {std::array<double, 2>{.006,.001}, {.001,.011}, {-.001,.001}, {.001,-.001}}) {
    auto c = armed();
    EXPECT_FALSE(c.step(clocks[0], clocks[1], 0., q, zero));
    EXPECT_EQ(c.reason(), Reason::ClockOrDeadline);
    EXPECT_FALSE(c.step(.001, .001, 0., q, zero));
  }
  auto c = armed();
  for (int i = 1; i <= 5; ++i) ASSERT_TRUE(c.step(0., i*.001, 0., q, zero));
  EXPECT_FALSE(c.step(0., .006, 0., q, zero));
  EXPECT_EQ(c.reason(), Reason::ClockOrDeadline);
}

TEST(Pilot, QueuedReplayedForeignOrFutureHeartbeatsDoNotRefreshWatchdog) {
  constexpr int64_t activation = 1000000000, now = 1200000000, previous = 1100000000;
  EXPECT_TRUE(valid_heartbeat(1190000000, now, activation, previous, true));
  EXPECT_FALSE(valid_heartbeat(900000000, now, activation, previous, true));
  EXPECT_FALSE(valid_heartbeat(previous, now, activation, previous, true));
  EXPECT_FALSE(valid_heartbeat(now+1, now, activation, previous, true));
  EXPECT_FALSE(valid_heartbeat(1190000000, now, activation, previous, false));
  EXPECT_FALSE(valid_heartbeat(1190000000, now, 0, previous, true));
}

TEST(Pilot, StaleBaselineMovingArmAndNonfiniteStateAreRejected) {
  PilotCore c;
  Joints changed = q, velocity{};
  changed[0] += .003;
  EXPECT_FALSE(c.arm(q, q, zero, changed, .005, 0., 0.));
  velocity[0] = .011;
  EXPECT_FALSE(c.arm(q, q, velocity, q, .005, 0., 0.));
  EXPECT_FALSE(c.arm(q, q, zero, q, .006, 0., 0.));
  c = armed();
  changed[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(c.step(.001, .001, 0., changed, zero));
  EXPECT_EQ(c.reason(), Reason::StateLimit);
}

TEST(Pilot, MeasuredTrackingAndVelocityLimitsAreEnforced) {
  auto c = armed();
  Joints changed = q;
  changed[1] += .015;
  EXPECT_FALSE(c.step(.001,.001,0.,changed,zero));
  EXPECT_EQ(c.reason(), Reason::TrackingError);
  c = armed();
  Joints velocity{};
  velocity[1] = .051;
  EXPECT_FALSE(c.step(.001,.001,0.,q,velocity));
  EXPECT_EQ(c.reason(), Reason::StateLimit);
}
