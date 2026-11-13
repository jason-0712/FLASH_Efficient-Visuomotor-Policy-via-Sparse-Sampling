#include <gtest/gtest.h>
#include "flash_fr3_model_controller/stream_core.hpp"
using namespace flash_fr3_model_controller;

Plan first_plan() {
  Plan p; p[0].duration=.5; p[0].c[3][1]=.0002;
  p[1]=stopping_polynomial(p[0].at(.5),.5);return p;
}
StreamPacket next_packet(double time=.5) {
  StreamPacket packet;packet.sequence=1;packet.handoff_robot_time=time;
  const auto r=first_plan()[0].at(.5);
  auto& p=packet.plan;p[0].duration=.5;
  p[0].c[0]=r.q;
  for (int j=0;j<7;++j) {p[0].c[1][j]=.5*r.dq[j];p[0].c[2][j]=.125*r.ddq[j];}
  p[0].c[3][1]=-.0001;p[1]=stopping_polynomial(p[0].at(.5),.5);return packet;
}
Joints home(){return {-.16,-.78,.11,-2.38,.07,1.56,.66};}
void tick(StreamCore& c,int begin,int end,double age=.01) {
  for (int i=begin;i<=end;++i) ASSERT_TRUE(c.step(i*.001,i*.001,age,home(),{}));
}
StreamCore armed(int max_replacements=1) {
  StreamCore c;const auto q=home();
  EXPECT_TRUE(c.arm(q,q,{},q,first_plan(),0.,0.,max_replacements));
  EXPECT_TRUE(c.step(0.,0.,.01,q,{}));return c;
}
TEST(StreamCore, FutureReplacementPreservesC2AndKeepsNonzeroVelocity) {
  auto c=armed();tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));
  tick(c,101,499);ASSERT_EQ(c.sequence(),0);
  tick(c,500,500);ASSERT_EQ(c.sequence(),1);
  const auto expected=first_plan()[0].at(.5);
  for (int j=0;j<7;++j) {
    EXPECT_NEAR(c.reference().q[j]-home()[j],expected.q[j],1e-12);
    EXPECT_NEAR(c.reference().dq[j],expected.dq[j],1e-12);
    EXPECT_NEAR(c.reference().ddq[j],expected.ddq[j],1e-12);
  }
  EXPECT_GT(c.reference().dq[1],.001);
  tick(c,501,1501);EXPECT_EQ(c.phase(),Phase::Complete);
  EXPECT_DOUBLE_EQ(c.reference().dq[1],0.);
}
TEST(StreamCore, MissingAndLatePredictionsUseExistingStop) {
  auto c=armed();tick(c,1,490);
  EXPECT_FALSE(c.stage(next_packet(),.49));EXPECT_EQ(c.rejection(),Rejection::LateOrWrongTime);
  tick(c,491,1001);EXPECT_EQ(c.sequence(),0);EXPECT_EQ(c.phase(),Phase::Complete);
  EXPECT_NEAR(c.reference().q[1]-home()[1],first_plan()[1].at(.5).q[1],1e-12);
}
TEST(StreamCore, RejectWrongTimeAndDiscontinuousReplacement) {
  auto c=armed();tick(c,1,100);EXPECT_FALSE(c.stage(next_packet(.6),.1));
  auto d=armed();tick(d,1,100);auto packet=next_packet();packet.plan[0].c[0][1]+=.0001;
  EXPECT_FALSE(d.stage(packet,.1));EXPECT_EQ(d.rejection(),Rejection::InvalidPlan);
}
TEST(StreamCore, RejectWholePlanAndWatchdogBounds) {
  auto c=armed();tick(c,1,100);auto packet=next_packet();
  packet.plan[0].c[3][1]=.1;packet.plan[1]=stopping_polynomial(packet.plan[0].at(.5),.5);
  EXPECT_FALSE(c.stage(packet,.1));EXPECT_EQ(c.rejection(),Rejection::InvalidPlan);
}
TEST(StreamCore, HeartbeatStopsAfterHandoffAndCannotRestart) {
  auto c=armed();tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,600);
  tick(c,601,601,.201);EXPECT_EQ(c.phase(),Phase::Braking);
  tick(c,602,852,.3);EXPECT_EQ(c.phase(),Phase::LatchedHold);
  tick(c,853,860,.01);EXPECT_EQ(c.phase(),Phase::LatchedHold);
}
TEST(StreamCore, SecondExtensionIsRejectedAndClockFaultStops) {
  auto c=armed();tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,600);
  auto extra=next_packet(1.);extra.sequence=2;EXPECT_FALSE(c.stage(extra,.6));
  EXPECT_EQ(c.rejection(),Rejection::WrongSequence);
  EXPECT_FALSE(c.step(.62,.62,.01,home(),{}));EXPECT_EQ(c.reason(),Reason::ClockOrDeadline);
}

StreamPacket second_packet() {
  StreamPacket packet;packet.sequence=2;packet.handoff_robot_time=1.;
  const auto r=next_packet().plan[0].at(.5);
  packet.plan[0].duration=.5;packet.plan[0].c[0]=r.q;
  for (int j=0;j<7;++j) {
    packet.plan[0].c[1][j]=.5*r.dq[j];packet.plan[0].c[2][j]=.125*r.ddq[j];
  }
  packet.plan[0].c[3][1]=-.0001;
  packet.plan[1]=stopping_polynomial(packet.plan[0].at(.5),.5);return packet;
}
TEST(StreamCore, TwoReplacementsPreserveMovingBoundaryAndStopAtTwoSeconds) {
  auto c=armed(2);tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,600);
  ASSERT_TRUE(c.stage(second_packet(),.6));EXPECT_EQ(c.pending_sequence(),2);
  tick(c,601,1000);ASSERT_EQ(c.sequence(),2);
  const auto boundary=next_packet().plan[0].at(.5);
  for (int j=0;j<7;++j) {
    EXPECT_NEAR(c.reference().q[j]-home()[j],boundary.q[j],1e-12);
    EXPECT_NEAR(c.reference().dq[j],boundary.dq[j],1e-12);
    EXPECT_NEAR(c.reference().ddq[j],boundary.ddq[j],1e-12);
  }
  EXPECT_GT(c.reference().dq[1],.001);
  tick(c,1001,1999);EXPECT_EQ(c.phase(),Phase::Moving);
  tick(c,2000,2010);EXPECT_EQ(c.phase(),Phase::Complete);
  EXPECT_NEAR(c.reference().q[1]-home()[1],second_packet().plan[1].at(.5).q[1],1e-12);
}
TEST(StreamCore, MissingOrLateSecondPacketUsesFirstReplacementStop) {
  for (bool late:{false,true}) {
    auto c=armed(2);tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,990);
    if(late) { EXPECT_FALSE(c.stage(second_packet(),.99)); }
    tick(c,991,1500);EXPECT_EQ(c.sequence(),1);EXPECT_EQ(c.phase(),Phase::Complete);
    EXPECT_NEAR(c.reference().q[1]-home()[1],next_packet().plan[1].at(.5).q[1],1e-12);
  }
}
TEST(StreamCore, SecondPacketCannotResetGlobalPositionEnvelope) {
  auto c=armed(2);tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,600);
  auto p=second_packet();p.plan[0].c[3][1]=.009;
  p.plan[1]=stopping_polynomial(p.plan[0].at(.5),.5);
  EXPECT_FALSE(c.stage(p,.6));EXPECT_EQ(c.rejection(),Rejection::InvalidPlan);
  tick(c,601,1500);EXPECT_EQ(c.phase(),Phase::Complete);EXPECT_EQ(c.sequence(),1);
}
TEST(StreamCore, WatchdogAfterSecondHandoffLatchesAndCannotExtend) {
  auto c=armed(2);tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,600);
  ASSERT_TRUE(c.stage(second_packet(),.6));tick(c,601,1100);
  tick(c,1101,1101,.201);EXPECT_EQ(c.phase(),Phase::Braking);
  tick(c,1102,1360,.3);EXPECT_EQ(c.phase(),Phase::LatchedHold);
  auto p=second_packet();p.sequence=3;p.handoff_robot_time=1.5;
  EXPECT_FALSE(c.stage(p,1.36));tick(c,1361,1370,.01);EXPECT_EQ(c.phase(),Phase::LatchedHold);
}
TEST(StreamCore, HardReplacementCapRejectsThirdPacketAndInvalidConfiguration) {
  auto c=armed(2);tick(c,1,100);ASSERT_TRUE(c.stage(next_packet(),.1));tick(c,101,600);
  ASSERT_TRUE(c.stage(second_packet(),.6));tick(c,601,1100);
  auto p=second_packet();p.sequence=3;p.handoff_robot_time=1.5;
  EXPECT_FALSE(c.stage(p,1.1));EXPECT_EQ(c.rejection(),Rejection::WrongSequence);
  tick(c,1101,2000);EXPECT_EQ(c.sequence(),2);EXPECT_EQ(c.phase(),Phase::Complete);
  const auto q=home();StreamCore invalid;
  EXPECT_FALSE(invalid.arm(q,q,{},q,first_plan(),0.,0.,0));
  EXPECT_FALSE(invalid.arm(q,q,{},q,first_plan(),0.,0.,3));
}
