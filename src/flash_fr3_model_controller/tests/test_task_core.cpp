#include <gtest/gtest.h>
#include "flash_fr3_model_controller/task_core.hpp"
using namespace flash_fr3_model_controller;
Joints task_home(){return {-.16,-.77,.11,-2.38,.07,1.57,.66};}
Plan task_plan(){Plan p;p[0].duration=2.;p[0].c[3][1]=.01;
  p[1]=stopping_polynomial(p[0].at(2.),.75);return p;}
TEST(TaskCore, CompletesPhysicalUnitsAndStaticEndpoint){
  TaskCore c;const auto q=task_home();ASSERT_TRUE(c.arm(q,q,{},q,task_plan(),0.,0.));
  for(int i=0;i<=2760;++i){const auto r=c.reference();
    ASSERT_TRUE(c.step(i*.001,i*.001,.01,r.q,r.dq));}
  EXPECT_EQ(c.phase(),Phase::Complete);EXPECT_GT(c.reference().q[1]-q[1],.01);
  EXPECT_DOUBLE_EQ(c.reference().dq[1],0.);
}
TEST(TaskCore, HeartbeatBrakesAndLatches){
  TaskCore c;const auto q=task_home();ASSERT_TRUE(c.arm(q,q,{},q,task_plan(),0.,0.));
  for(int i=0;i<=1510;++i){const auto r=c.reference();
    ASSERT_TRUE(c.step(i*.001,i*.001,i<1000?.01:.21,r.q,r.dq));}
  EXPECT_EQ(c.phase(),Phase::LatchedHold);
  const auto end=c.reference().q;
  ASSERT_TRUE(c.step(1.511,1.511,.01,end,{}));EXPECT_EQ(c.phase(),Phase::LatchedHold);
}
TEST(TaskCore, RejectsInitialGapAndJointEnvelope){
  const auto q=task_home();auto stale=q;stale[0]+=.003;TaskCore c;
  EXPECT_FALSE(c.arm(q,q,{},stale,task_plan(),0.,0.));
  auto p=task_plan();p[0].c[3][4]=.1;p[1]=stopping_polynomial(p[0].at(2.),.75);
  EXPECT_FALSE(validate_task_plan(p,q));
}
TEST(TaskCore, RejectsWholeCurveBrakeBoundsAndDiscontinuity){
  auto p=task_plan();const auto q=task_home();p[0].c[3][1]=.3;
  p[1]=stopping_polynomial(p[0].at(2.),.75);EXPECT_FALSE(validate_task_plan(p,q));
  p=task_plan();p[1].c[0][1]+=.001;EXPECT_FALSE(validate_task_plan(p,q));
  p=task_plan();p[0].c[6][0]=NAN;EXPECT_FALSE(validate_task_plan(p,q));
}
TEST(TaskCore, ClockOrTrackingFaultIsNotRestarted){
  const auto q=task_home();TaskCore c;ASSERT_TRUE(c.arm(q,q,{},q,task_plan(),0.,0.));
  EXPECT_FALSE(c.step(.02,.02,.01,q,{}));EXPECT_EQ(c.reason(),Reason::ClockOrDeadline);
  TaskCore d;ASSERT_TRUE(d.arm(q,q,{},q,task_plan(),0.,0.));auto bad=q;bad[0]+=.03;
  EXPECT_FALSE(d.step(.001,.001,.01,bad,{}));EXPECT_EQ(d.reason(),Reason::TrackingError);
}
