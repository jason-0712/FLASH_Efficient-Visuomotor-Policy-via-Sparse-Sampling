#include <gtest/gtest.h>
#include "flash_fr3_model_controller/model_core.hpp"
using namespace flash_fr3_model_controller;
namespace {
Joints q{-.16,-.78,.11,-2.38,.07,1.56,.66};
Plan test_plan() {
  Plan p;
  p[0].duration = 1.; p[0].c[3][1] = .001;
  p[1] = stopping_polynomial(p[0].at(1.),.75);
  return p;
}
}
TEST(ModelCore, DerivativesAndC2Stop) {
  auto p = test_plan();
  ASSERT_TRUE(validate_plan(p));
  const auto join = p[1].at(0.), before = p[0].at(1.);
  EXPECT_NEAR(join.q[1],before.q[1],1e-14);
  EXPECT_NEAR(join.dq[1],before.dq[1],1e-14);
  EXPECT_NEAR(join.ddq[1],before.ddq[1],1e-14);
  const auto end = p[1].at(.75);
  EXPECT_NEAR(end.dq[1],0.,1e-14); EXPECT_NEAR(end.ddq[1],0.,1e-14);
  const double h = 1e-5;
  for (const auto& poly : p) for (double t : {.1,.4}) {
    const auto r=poly.at(t), plus=poly.at(t+h), minus=poly.at(t-h);
    EXPECT_NEAR((plus.q[1]-minus.q[1])/(2*h),r.dq[1],1e-10);
    EXPECT_NEAR((plus.dq[1]-minus.dq[1])/(2*h),r.ddq[1],1e-10);
    EXPECT_NEAR((plus.ddq[1]-minus.ddq[1])/(2*h),r.jerk[1],1e-10);
  }
}
TEST(ModelCore, RejectInvalidDiscontinuousOrBrakeUnsafePlans) {
  auto p=test_plan(); p[0].c[0][0]=.001; EXPECT_FALSE(validate_plan(p));
  p=test_plan(); p[1].c[0][1]+=.0001; EXPECT_FALSE(validate_plan(p));
  p=test_plan(); p[0].duration=NAN; EXPECT_FALSE(validate_plan(p));
  p=test_plan(); p[0].c[6][0]=INFINITY; EXPECT_FALSE(validate_plan(p));
  p=test_plan(); p[1].c[3][2]=.01; EXPECT_FALSE(validate_plan(p));
  p=test_plan(); p[0].c[3][1]=.02; p[1]=stopping_polynomial(p[0].at(1.),.75);
  EXPECT_FALSE(validate_plan(p));
}
TEST(ModelCore, NativeOriginAndNormalCompletion) {
  ModelCore core;
  auto desired=q; desired[0]+=.0001;
  ASSERT_TRUE(core.arm(desired,q,{},q,test_plan(),0.,0.));
  EXPECT_DOUBLE_EQ(core.reference().q[0],desired[0]);
  for (int i=0; i<=1800; ++i) {
    ASSERT_TRUE(core.step(i*.001,i*.001,0.,core.reference().q,core.reference().dq));
  }
  EXPECT_EQ(core.phase(),Phase::Complete);
  EXPECT_NEAR(core.reference().q[1]-desired[1],test_plan()[1].at(.75).q[1],1e-14);
  EXPECT_EQ(core.reference().dq,Joints{});
}
TEST(ModelCore, EveryCutoffBrakesAndLatchesWithinBounds) {
  for (int cut=10; cut<1750; cut+=17) {
    ModelCore core;
    ASSERT_TRUE(core.arm(q,q,{},q,test_plan(),0.,0.));
    for (int i=0; i<=cut+260; ++i)
      ASSERT_TRUE(core.step(i*.001,i*.001,i>=cut?.21:0.,core.reference().q,core.reference().dq));
    ASSERT_EQ(core.phase(),Phase::LatchedHold);
    ASSERT_EQ(core.reason(),Reason::HeartbeatTimeout);
    ASSERT_EQ(core.reference().dq,Joints{});
  }
}
TEST(ModelCore, FaultsNeverResumeAndNoHeartbeatNeverStarts) {
  ModelCore core;
  ASSERT_TRUE(core.arm(q,q,{},q,test_plan(),0.,0.));
  for (int i=0;i<=2100;++i) ASSERT_TRUE(core.step(i*.001,i*.001,-1.,q,{}));
  EXPECT_EQ(core.phase(),Phase::LatchedHold); EXPECT_EQ(core.reason(),Reason::StartTimeout);
  EXPECT_EQ(core.reference().q,q);
  ASSERT_TRUE(core.arm(q,q,{},q,test_plan(),0.,0.));
  EXPECT_FALSE(core.step(.02,.02,0.,q,{})); EXPECT_EQ(core.reason(),Reason::ClockOrDeadline);
  EXPECT_FALSE(core.step(.021,.021,0.,q,{}));
  ASSERT_TRUE(core.arm(q,q,{},q,test_plan(),0.,0.));
  auto wrong=q; wrong[0]+=.011;
  EXPECT_FALSE(core.step(.001,.001,0.,wrong,{})); EXPECT_EQ(core.reason(),Reason::TrackingError);
  wrong=q; wrong[0]+=.003;
  EXPECT_FALSE(core.arm(q,q,{},wrong,test_plan(),0.,0.));
}
