#include <gtest/gtest.h>
#include "flash_fr3_model_controller/continuous_reference.hpp"
using namespace flash_fr3_model_controller;
PhysicalPacket packet(int sequence) {
  PhysicalPacket p;p.sequence=sequence;p.observation=100.+sequence*.25;p.handoff=p.observation+.2;return p;
}
TEST(PhysicalCurve, AnalyticLegendreDerivativesHavePhysicalUnits) {
  PhysicalCurve curve;curve.legendre[2][0]=.3;curve.legendre[3][1]=.2;
  for(double t:{0.,.2,1.,32./15.}) {
    Reference r;ASSERT_TRUE(curve.at(t,r));double x=2.*(curve.anchor+t/curve.duration)-1.,scale=2./curve.duration;
    EXPECT_NEAR(r.q[0],.3*(3*x*x-1)/2,1e-12);EXPECT_NEAR(r.dq[0],.9*x*scale,1e-12);
    EXPECT_NEAR(r.ddq[0],.9*scale*scale,1e-12);
    EXPECT_NEAR(r.q[1],.2*(5*x*x*x-3*x)/2,1e-12);
    EXPECT_NEAR(r.dq[1],.2*(15*x*x-3)/2*scale,1e-12);
    EXPECT_NEAR(r.jerk[1],3.*scale*scale*scale,1e-12);
  }
}
TEST(PhysicalCurve, RejectsNonfiniteWrongPhysicalClockAndExtrapolation) {
  Reference r;PhysicalCurve curve;
  EXPECT_FALSE(curve.at(-.001,r));EXPECT_FALSE(curve.at(3.,r));
  curve.legendre[6][6]=NAN;EXPECT_FALSE(curve.at(.1,r));
  curve={};curve.duration=3.;EXPECT_FALSE(curve.at(.1,r));
}
TEST(RawTimeline, HandlesTwentyHandoffsWithoutPerSegmentReset) {
  RawTimeline timeline;auto first=packet(0);ASSERT_TRUE(timeline.initialize(first,100.1));
  ASSERT_TRUE(timeline.step(first.handoff));
  for(int i=1;i<=20;++i) {
    const auto next=packet(i);const auto ready=next.observation+.1;
    ASSERT_TRUE(timeline.step(ready));ASSERT_TRUE(timeline.stage(next,ready));
    ASSERT_TRUE(timeline.step(next.handoff));EXPECT_EQ(timeline.sequence(),i);
  }
  EXPECT_EQ(timeline.handoffs(),20);EXPECT_EQ(timeline.phase(),TimelinePhase::Running);
}
TEST(RawTimeline, RawPositionJumpRejectedWithoutCorrectingCoefficients) {
  RawTimeline timeline;ASSERT_TRUE(timeline.initialize(packet(0),100.1));ASSERT_TRUE(timeline.step(100.3));
  auto next=packet(1);next.curve.legendre[0][4]=.03;
  EXPECT_FALSE(timeline.stage(next,100.35));EXPECT_EQ(timeline.rejection(),TimelineRejection::Position);
  EXPECT_NEAR(timeline.position_gap(),.03,1e-12);EXPECT_EQ(timeline.sequence(),0);
  ASSERT_TRUE(timeline.step(100.5));EXPECT_DOUBLE_EQ(timeline.reference().q[4],0.);
  // Dropping one inference must not require a reused sequence number.
  ASSERT_TRUE(timeline.stage(packet(2),100.6));ASSERT_TRUE(timeline.step(100.7));
  EXPECT_EQ(timeline.sequence(),2);
}
TEST(RawTimeline, VelocityJumpRejectedEvenWithMatchedPosition) {
  RawTimeline timeline;ASSERT_TRUE(timeline.initialize(packet(0),100.1));ASSERT_TRUE(timeline.step(100.3));
  auto next=packet(1);const double x=2.*(next.curve.anchor+.2/next.curve.duration)-1.;
  next.curve.legendre[1][0]=.02;next.curve.legendre[0][0]=-.02*x;
  EXPECT_FALSE(timeline.stage(next,100.35));EXPECT_EQ(timeline.rejection(),TimelineRejection::Velocity);
  EXPECT_LT(timeline.position_gap(),1e-10);EXPECT_GT(timeline.velocity_gap(),.005);
}
TEST(RawTimeline, MissingPredictionExpiresAndNeverExtrapolates) {
  RawTimeline timeline;ASSERT_TRUE(timeline.initialize(packet(0),100.1));
  EXPECT_FALSE(timeline.step(102.2));EXPECT_EQ(timeline.phase(),TimelinePhase::Expired);
  EXPECT_FALSE(timeline.stage(packet(10),102.6));EXPECT_FALSE(timeline.step(102.7));
}
TEST(RawTimeline, RejectsReplayLateAndPendingOverwrite) {
  RawTimeline timeline;ASSERT_TRUE(timeline.initialize(packet(0),100.1));ASSERT_TRUE(timeline.step(100.3));
  EXPECT_FALSE(timeline.stage(packet(1),100.44));EXPECT_EQ(timeline.rejection(),TimelineRejection::Invalid);
  ASSERT_TRUE(timeline.stage(packet(2),100.6));
  EXPECT_FALSE(timeline.stage(packet(3),100.85));EXPECT_EQ(timeline.rejection(),TimelineRejection::Sequence);
  ASSERT_TRUE(timeline.step(100.7));EXPECT_FALSE(timeline.stage(packet(2),100.71));
  EXPECT_FALSE(timeline.step(100.69));EXPECT_EQ(timeline.phase(),TimelinePhase::Error);
}
