#include <gtest/gtest.h>
#include "flash_fr3_model_controller/torque_core.hpp"
using namespace flash_fr3_model_controller;

TorqueSettings torque_settings() {
  TorqueSettings s;s.kp.fill(100.);s.kd.fill(10.);s.magnitude.fill(4.);s.rate.fill(100.);return s;
}
Joints torque_home() {return {-.16,-.77,.11,-2.38,.07,1.57,.66};}
TEST(TorqueLaw, PositionVelocityFeedforwardAndCoriolisAreIndependent) {
  TorqueLaw law;ASSERT_TRUE(law.configure(torque_settings()));
  Reference r;r.q=torque_home();r.q[0]+=.001;r.dq[0]=.001;
  Joints c{},last{};c[0]=.01;last[0]=.05;TorqueOutput out;
  ASSERT_TRUE(law.evaluate(r,torque_home(),{},c,last,.001,out));
  EXPECT_NEAR(out.raw[0],.12,1e-12);EXPECT_NEAR(out.command[0],.12,1e-12);
  EXPECT_FALSE(out.limited[0]);
  r.dq[0]=0.;ASSERT_TRUE(law.evaluate(r,torque_home(),{},c,last,.001,out));
  EXPECT_NEAR(out.raw[0],.11,1e-12);
}
TEST(TorqueLaw, MeasuredVelocityHasDampingSign) {
  TorqueLaw law;ASSERT_TRUE(law.configure(torque_settings()));Reference r;r.q=torque_home();
  Joints dq{};dq[3]=.005;TorqueOutput out;
  ASSERT_TRUE(law.evaluate(r,torque_home(),dq,{},{},.001,out));
  EXPECT_NEAR(out.raw[3],-.05,1e-12);
}
TEST(TorqueLaw, RateUsesNativePreviousDesiredTorqueAndPhysicalPeriod) {
  TorqueLaw law;ASSERT_TRUE(law.configure(torque_settings()));Reference r;r.q=torque_home();
  Joints previous{};previous[0]=.5;previous[1]=-.5;TorqueOutput out;
  ASSERT_TRUE(law.evaluate(r,torque_home(),{},{},previous,.001,out));
  EXPECT_NEAR(out.command[0],.4,1e-12);EXPECT_NEAR(out.command[1],-.4,1e-12);
  EXPECT_TRUE(out.limited[0]);EXPECT_TRUE(out.limited[1]);
  ASSERT_TRUE(law.evaluate(r,torque_home(),{},{},previous,.002,out));
  EXPECT_NEAR(out.command[0],.3,1e-12);
}
TEST(TorqueLaw, InvalidInputsLatchWithoutPartiallyWritingOutput) {
  for (int scenario=0;scenario<6;++scenario) {
    TorqueLaw law;ASSERT_TRUE(law.configure(torque_settings()));Reference r;r.q=torque_home();
    Joints q=torque_home(),dq{},c{},previous{};double dt=.001;TorqueOutput out;out.command.fill(123.);
    if(scenario==0)c[6]=NAN;
    if(scenario==1)dt=.003;
    if(scenario==2)r.dq[0]=.006;
    if(scenario==3)c[6]=5.;
    if(scenario==4)previous[0]=5.;
    if(scenario==5)dq[0]=.06;
    EXPECT_FALSE(law.evaluate(r,q,dq,c,previous,dt,out));
    EXPECT_NE(law.fault(),TorqueFault::None);
    for(double v:out.command)EXPECT_DOUBLE_EQ(v,123.);
    EXPECT_FALSE(law.evaluate(Reference{torque_home(),{},{},{}},torque_home(),{},{},{},.001,out));
  }
}
TEST(TorqueLaw, InvalidConfigurationHasNoDefaultActuation) {
  TorqueLaw law;TorqueOutput out;Reference r;r.q=torque_home();
  EXPECT_FALSE(law.evaluate(r,torque_home(),{},{},{},.001,out));
  auto s=torque_settings();s.kp[0]=NAN;EXPECT_FALSE(law.configure(s));
  s=torque_settings();s.rate[0]=101.;EXPECT_FALSE(law.configure(s));
  s=torque_settings();s.magnitude[6]=9.;EXPECT_FALSE(law.configure(s));
}
TEST(TorqueReference, SamePolynomialDerivativeReachesVelocityTerm) {
  Polynomial p;p.duration=2.;p.c[3][1]=.001;p.c[4][1]=-.0015;p.c[5][1]=.0006;
  auto r=p.at(.8);const double epsilon=1e-5;
  EXPECT_NEAR((p.at(.8+epsilon).q[1]-p.at(.8-epsilon).q[1])/(2*epsilon),r.dq[1],1e-10);
  const double velocity=r.dq[1];r.q=torque_home();
  TorqueLaw law;ASSERT_TRUE(law.configure(torque_settings()));TorqueOutput out;
  ASSERT_TRUE(law.evaluate(r,torque_home(),{},{},{},.001,out));
  EXPECT_NEAR(out.raw[1],10.*velocity,1e-12);
  Plan hold;hold[0].duration=2.;hold[1].duration=.75;
  EXPECT_TRUE(validate_torque_commissioning_plan(hold,torque_home()));
  hold[0].c[3][1]=.01;hold[1]=stopping_polynomial(hold[0].at(2.),.75);
  EXPECT_FALSE(validate_torque_commissioning_plan(hold,torque_home()));
}
TEST(TorqueReference, NativeHeartbeatBrakeCompletesAndRemainsLatched) {
  TaskCore reference;Plan plan;plan[0].duration=2.;plan[1].duration=.75;
  const auto q=torque_home();ASSERT_TRUE(reference.arm(q,q,{},q,plan,0.,0.));
  for(int i=0;i<=900;++i)ASSERT_TRUE(reference.step(i*.001,i*.001,i<300?.01:.21,q,{}));
  EXPECT_EQ(reference.phase(),Phase::LatchedHold);
  ASSERT_TRUE(reference.step(.901,.901,.01,q,{}));EXPECT_EQ(reference.phase(),Phase::LatchedHold);
}
TEST(TorqueReference, PreholdUsesExistingWaitingDeadlineAndUnchangedCurve) {
  Plan plan;plan[0].duration=3.;plan[1].duration=.75;
  plan[0].c[3][3]=.010;plan[0].c[4][3]=-.015;plan[0].c[5][3]=.006;
  plan[1].c[0][3]=.001;
  TaskCore reference;const auto q=torque_home();
  ASSERT_TRUE(reference.arm(q,q,{},q,plan,0.,0.));
  for(int i=0;i<=5251;++i) {
    ASSERT_TRUE(reference.step(i*.001,i*.001,i<1500?-1.:.01,q,{}));
    if(i<1500) {EXPECT_EQ(reference.phase(),Phase::Waiting);EXPECT_DOUBLE_EQ(reference.reference().q[3],q[3]);}
  }
  EXPECT_EQ(reference.phase(),Phase::Complete);
  EXPECT_NEAR(reference.reference().q[3]-q[3],.001,1e-12);
  TaskCore no_start;ASSERT_TRUE(no_start.arm(q,q,{},q,plan,0.,0.));
  for(int i=0;i<=2001;++i)ASSERT_TRUE(no_start.step(i*.001,i*.001,-1.,q,{}));
  EXPECT_EQ(no_start.phase(),Phase::LatchedHold);
  EXPECT_EQ(no_start.reason(),Reason::StartTimeout);
}
