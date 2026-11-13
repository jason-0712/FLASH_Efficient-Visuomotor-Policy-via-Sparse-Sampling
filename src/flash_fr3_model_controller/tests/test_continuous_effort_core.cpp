#include <gtest/gtest.h>
#include "flash_fr3_model_controller/continuous_effort_core.hpp"
#include "flash_fr3_model_controller/rollout_torque_law.hpp"
using namespace flash_fr3_model_controller;
namespace {
Joints origin(){return {-.16,-.77,.11,-2.38,.07,1.57,.66};}
PhysicalPacket curve_packet(int sequence,double observation=100.,double velocity=0.) {
  PhysicalPacket p;p.sequence=sequence;p.observation=observation;p.handoff=observation+.1;
  p.curve.legendre[0]=origin();
  p.curve.legendre[1][3]=velocity*p.curve.duration/2.;
  p.curve.legendre[0][3]+=velocity*(observation-100.)-
                            p.curve.legendre[1][3]*(2*p.curve.anchor-1.);
  return p;
}
PreparedEffortPacket prepare(const PhysicalPacket& raw) {
  PreparedEffortPacket packet;EXPECT_TRUE(prepare_effort_packet(raw,packet));return packet;
}
bool tick(ContinuousEffortCore& core,int ms,double heartbeat=.01,bool finish=false) {
  return core.step(10.+ms*.001,100.+ms*.001,heartbeat,origin(),{},finish);
}
}
TEST(ContinuousEffortBounds, PowerConversionPreservesRawPolynomialAndPhysicalDerivatives) {
  auto raw=curve_packet(0);raw.curve.legendre[2][1]=.0002;raw.curve.legendre[6][5]=.00001;
  Polynomial power;ASSERT_TRUE(physical_power_interval(raw.curve,.15,1.9,power));
  for(double t:{0.,.127,.8,1.75}) {
    Reference leg;ASSERT_TRUE(raw.curve.at(t+.15,leg));const auto poly=power.at(t);
    for(int j=0;j<7;++j) {
      EXPECT_NEAR(poly.q[j],leg.q[j],1e-12);EXPECT_NEAR(poly.dq[j],leg.dq[j],1e-12);
      EXPECT_NEAR(poly.ddq[j],leg.ddq[j],1e-12);EXPECT_NEAR(poly.jerk[j],leg.jerk[j],1e-12);
    }
  }
}
TEST(ContinuousEffortBounds, EntireFutureAndEveryBrakeStartAreBounded) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
  auto raw=curve_packet(0);raw.curve.legendre[6][0]=.01;
  ASSERT_FALSE(core.admit(prepare(raw),100.02));EXPECT_EQ(core.rejection(),EffortStreamRejection::Envelope);
  raw.curve.legendre[6][0]=NAN;PreparedEffortPacket result;
  EXPECT_FALSE(prepare_effort_packet(raw,result));EXPECT_FALSE(result.available);
  raw=curve_packet(0);raw.handoff=102.;EXPECT_FALSE(prepare_effort_packet(raw,result));
}
TEST(ContinuousEffortCore, TwentyUpdatesPreservePhysicalClockNonzeroVelocityAndOneActivation) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
  ASSERT_TRUE(core.admit(prepare(curve_packet(0,100.,.00005)),100.02));
  int next=1;
  for(int ms=1;ms<=6000;++ms) {
    ASSERT_TRUE(tick(core,ms));
    if(next<=20&&ms>=std::ceil((next*4./15.+.02)*1000)) {
      ASSERT_TRUE(core.admit(prepare(curve_packet(next,100.+next*4./15.,.00005)),100.+ms*.001));++next;
    }
  }
  EXPECT_EQ(core.handoffs(),20);EXPECT_EQ(core.sequence(),20);EXPECT_EQ(core.phase(),Phase::Moving);
  EXPECT_NEAR(core.reference().dq[3],.00005,1e-12);
  EXPECT_NEAR(core.reference().q[3],origin()[3]+6.*.00005,1e-12);
  ASSERT_TRUE(tick(core,6001,.01,true));EXPECT_EQ(core.phase(),Phase::Braking);
  for(int ms=6002;ms<=6502;++ms) ASSERT_TRUE(tick(core,ms));
  EXPECT_EQ(core.phase(),Phase::Complete);EXPECT_EQ(core.reason(),EffortStreamReason::Finish);
  EXPECT_DOUBLE_EQ(core.reference().dq[3],0.);
  EXPECT_FALSE(core.admit(prepare(curve_packet(21,106.5)),106.52));
  EXPECT_FALSE(core.arm(origin(),{},origin(),20.,200.,8.));
}
TEST(ContinuousEffortCore, HeartbeatLossBrakesWithoutAcceptingMorePredictions) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
  ASSERT_TRUE(core.admit(prepare(curve_packet(0,100.,.00005)),100.02));
  for(int ms=1;ms<=201;++ms) ASSERT_TRUE(tick(core,ms,ms<201?.01:.201));
  EXPECT_EQ(core.phase(),Phase::Braking);EXPECT_EQ(core.reason(),EffortStreamReason::Heartbeat);
  EXPECT_FALSE(core.admit(prepare(curve_packet(1,100.2)),100.22));
  for(int ms=202;ms<=703;++ms) ASSERT_TRUE(tick(core,ms,.5));
  EXPECT_EQ(core.phase(),Phase::LatchedHold);EXPECT_DOUBLE_EQ(core.reference().dq[3],0.);
}
TEST(ContinuousEffortCore, MissingPredictionBrakesBeforeRawCurveExpiresWithoutExtrapolation) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
  ASSERT_TRUE(core.admit(prepare(curve_packet(0)),100.02));
  for(int ms=1;ms<=2135;++ms) ASSERT_TRUE(tick(core,ms));
  EXPECT_EQ(core.reason(),EffortStreamReason::PredictionExpired);EXPECT_EQ(core.phase(),Phase::LatchedHold);
  EXPECT_FALSE(core.admit(prepare(curve_packet(1,102.1)),102.12));
}
TEST(ContinuousEffortCore, RejectsPositionAndVelocityJumpsWithoutRepairOrOriginShift) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
  auto raw=curve_packet(0);raw.curve.legendre[0][2]+=.001;
  EXPECT_FALSE(core.admit(prepare(raw),100.02));EXPECT_EQ(core.rejection(),EffortStreamRejection::Position);
  EXPECT_EQ(core.reference().q,origin());
  ContinuousEffortCore velocity;ASSERT_TRUE(velocity.arm(origin(),{},origin(),10.,100.,8.));
  raw=curve_packet(0,100.,.0004);raw.curve.legendre[0][3]-=.0004*.1;
  EXPECT_FALSE(velocity.admit(prepare(raw),100.02));EXPECT_EQ(velocity.rejection(),EffortStreamRejection::Velocity);
  EXPECT_EQ(velocity.reference().q,origin());
}
TEST(ContinuousEffortCore, RejectsStaleLateReplayAndPendingOverwrite) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
  auto first=prepare(curve_packet(0));EXPECT_FALSE(core.admit(first,100.09));
  EXPECT_EQ(core.rejection(),EffortStreamRejection::Clock);
  ContinuousEffortCore active;ASSERT_TRUE(active.arm(origin(),{},origin(),10.,100.,8.));
  ASSERT_TRUE(active.admit(first,100.02));
  EXPECT_FALSE(active.admit(first,100.03));
  EXPECT_FALSE(active.admit(prepare(curve_packet(1,100.01)),100.04));
  EXPECT_EQ(active.rejection(),EffortStreamRejection::Sequence);
  for(int ms=1;ms<=120;++ms) ASSERT_TRUE(tick(active,ms));
  ASSERT_TRUE(active.admit(prepare(curve_packet(2,100.12)),100.14));
  EXPECT_FALSE(active.admit(prepare(curve_packet(3,100.15)),100.17));
  EXPECT_EQ(active.rejection(),EffortStreamRejection::Busy);
}
TEST(ContinuousEffortCore, DeadlinesStateAndClockErrorsLatch) {
  for(int mode=0;mode<4;++mode) {
    ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,8.));
    auto q=origin();if(mode==2)q[1]+=.005;if(mode==3)q[6]=NAN;
    EXPECT_FALSE(core.step(mode==0?10.003:10.001,mode==1?99.:100.001,.01,q,{}));
    EXPECT_EQ(core.phase(),Phase::Error);EXPECT_FALSE(tick(core,2));
  }
}
TEST(ContinuousEffortClock, IntegerSdkTicksAcceptExactTwoMillisecondsAtArchivedUptime) {
  constexpr uint64_t previous_ms=10491237,current_ms=10491239;
  const double old_period=current_ms/1000.-previous_ms/1000.;
  EXPECT_GT(old_period,.002);  // Reproduce the erroneous seconds subtraction.
  ContinuousEffortCore core;
  ASSERT_TRUE(core.arm_milliseconds(origin(),{},origin(),previous_ms,100.,3.,true));
  ASSERT_TRUE(core.step_milliseconds(current_ms,100.002,.01,origin(),{}));
  EXPECT_DOUBLE_EQ(core.robot_period_seconds(),.002);
  EXPECT_DOUBLE_EQ(core.clock_sample()[1],old_period);
  TorqueSettings settings;settings.kp.fill(100.);settings.kd.fill(10.);
  settings.magnitude.fill(4.);settings.rate.fill(100.);
  RolloutTorqueLaw law;ASSERT_TRUE(law.configure(settings,true));
  Reference ref;ref.q=origin();TorqueOutput output;
  EXPECT_TRUE(law.evaluate(ref,origin(),{},Joints{},Joints{},core.robot_period_seconds(),output));
}
TEST(ContinuousEffortClock, IntegerSdkTicksRetainOverrunReplayHostGapAndDriftRejection) {
  for(int mode=0;mode<5;++mode) {
    ContinuousEffortCore core;constexpr uint64_t start=10491237;
    ASSERT_TRUE(core.arm_milliseconds(origin(),{},origin(),start,100.,3.,true));
    const uint64_t next=mode==0?start+3:mode==1?start:mode==2?start-1:start+1;
    const double now=mode==3?100.011:mode==4?100.03:100.001;
    EXPECT_FALSE(core.step_milliseconds(next,now,.01,origin(),{}));
    EXPECT_EQ(core.reason(),EffortStreamReason::Clock);
    EXPECT_FALSE(core.step_milliseconds(start+4,100.004,.01,origin(),{}));
    if(mode==0)EXPECT_DOUBLE_EQ(core.robot_period_seconds(),.003);
  }
  // Accumulated drift still rejects even when every individual period is valid.
  ContinuousEffortCore drift;constexpr uint64_t start=10491237;
  ASSERT_TRUE(drift.arm_milliseconds(origin(),{},origin(),start,100.,3.,true));
  int ms=1;
  for(;ms<=19;++ms)ASSERT_TRUE(drift.step_milliseconds(start+ms,100.+ms*.002,.01,origin(),{}));
  bool rejected=false;
  for(;ms<=22;++ms)if(!drift.step_milliseconds(start+ms,100.+ms*.002,.01,origin(),{})) {
    rejected=true;break;
  }
  EXPECT_TRUE(rejected);EXPECT_EQ(drift.reason(),EffortStreamReason::Clock);
}
TEST(ContinuousEffortCore, BoundedSessionStopsAndWaitingTimesOut) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,1.));
  ASSERT_TRUE(core.admit(prepare(curve_packet(0)),100.02));
  for(int ms=1;ms<=1002;++ms) ASSERT_TRUE(tick(core,ms));
  EXPECT_EQ(core.phase(),Phase::Complete);EXPECT_EQ(core.reason(),EffortStreamReason::SessionLimit);
  ContinuousEffortCore waiting;ASSERT_TRUE(waiting.arm(origin(),{},origin(),10.,100.,1.));
  for(int ms=1;ms<=501;++ms) ASSERT_TRUE(tick(waiting,ms,-1.));
  EXPECT_EQ(waiting.phase(),Phase::LatchedHold);EXPECT_EQ(waiting.reason(),EffortStreamReason::StartTimeout);
}
TEST(ContinuousEffortWire, FixedRawCoefficientSchemaRejectsMalformedFields) {
  std::array<double,57> data{};data[0]=1.;data[2]=1700000000.;data[3]=100.;data[4]=100.1;
  data[5]=2./11.;data[6]=44./15.;data[7]=32./15.;
  const auto q=origin();for(int j=0;j<7;++j)data[8+j]=q[j];
  PhysicalPacket result;ASSERT_TRUE(decode_effort_packet(data.data(),data.size(),result));
  EXPECT_EQ(result.curve.legendre[0],q);EXPECT_FALSE(decode_effort_packet(data.data(),56,result));
  data[1]=.5;EXPECT_FALSE(decode_effort_packet(data.data(),57,result));
  data[1]=0.;data[56]=NAN;EXPECT_FALSE(decode_effort_packet(data.data(),57,result));
}
TEST(RawPolicyRollout, ConsumesUnmodifiedNonMicroCurveAndExpiresWithNativeBrake) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,3.,true));
  auto raw=curve_packet(0,100.,.01);const auto original=raw.curve.legendre;
  PreparedEffortPacket packet;ASSERT_TRUE(prepare_effort_packet(raw,packet,true));
  ASSERT_TRUE(core.admit(packet,100.02));
  for(int ms=1;ms<=700;++ms) ASSERT_TRUE(tick(core,ms));
  Reference exact;ASSERT_TRUE(raw.curve.at(.7,exact));
  EXPECT_NEAR(core.reference().q[3],exact.q[3],1e-12);
  EXPECT_NEAR(core.reference().dq[3],.01,1e-12);
  EXPECT_EQ(raw.curve.legendre,original);
  // With no further prediction the bounded prefix brakes; no polynomial
  // extrapolation or repeated activation is used as the normal execution.
  for(int ms=701;ms<=2202;++ms) ASSERT_TRUE(tick(core,ms));
  EXPECT_EQ(core.phase(),Phase::LatchedHold);EXPECT_EQ(core.reason(),EffortStreamReason::PredictionExpired);
}
TEST(RawPolicyRollout, RejectsHardwareEnvelopeButRecordsInitialPositionGap) {
  for(int mode=0;mode<2;++mode) {
    ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,3.,true));
    auto raw=curve_packet(0);raw.curve.legendre[0][mode==0?0:3]+=mode==0?4.:.06;
    PreparedEffortPacket packet;ASSERT_TRUE(prepare_effort_packet(raw,packet,true));
    if(mode==0) {
      EXPECT_FALSE(core.admit(packet,100.02));EXPECT_EQ(core.rejection(),EffortStreamRejection::Envelope);
    } else {
      EXPECT_TRUE(core.admit(packet,100.02));EXPECT_NEAR(core.position_gap(),.06,1e-12);
      EXPECT_EQ(core.rejection(),EffortStreamRejection::None);
    }
  }
}
TEST(RawPolicyRollout, TorqueUsesAnalyticVelocityAndRetainsActualMagnitudeLimit) {
  TorqueSettings s;s.kp.fill(100.);s.kd.fill(10.);s.magnitude.fill(4.);s.rate.fill(100.);
  RolloutTorqueLaw law;ASSERT_TRUE(law.configure(s,true));
  Reference r;r.q=origin();r.dq[3]=.02;TorqueOutput out;
  Joints previous{};previous[3]=.2;
  ASSERT_TRUE(law.evaluate(r,origin(),{},Joints{},previous,.001,out));EXPECT_NEAR(out.command[3],.2,1e-12);
  r.q[3]+=.06;EXPECT_FALSE(law.evaluate(r,origin(),{},Joints{},previous,.001,out));
  EXPECT_EQ(law.fault(),TorqueFault::Magnitude);
  RolloutTorqueLaw magnitude;ASSERT_TRUE(magnitude.configure(s,true));r.q=origin();r.dq[3]=.8;
  EXPECT_FALSE(magnitude.evaluate(r,origin(),{},Joints{},previous,.001,out));
  EXPECT_EQ(magnitude.fault(),TorqueFault::Magnitude);
}
TEST(RawPolicyRollout, ManufacturerProfileAcceptsObservedJointTwoFeedbackDemand) {
  TorqueSettings s;s.kp={150.,150.,150.,150.,50.,30.,30.};
  for(std::size_t j=0;j<7;++j)s.kd[j]=2.*std::sqrt(s.kp[j]);
  s.magnitude=kRolloutTorqueMagnitude;s.rate.fill(kRolloutTorqueRate);
  RolloutTorqueLaw law;ASSERT_TRUE(law.configure(s,true));
  Reference r;r.q=origin();Joints dq{},previous{};TorqueOutput out;
  // Observed 15:50 failure's joint-2 position/velocity errors. Coriolis is set
  // to zero here: archived torque columns retain a previous successful cycle.
  r.q[1]-=.08224547254467718;
  r.dq[1]=.356710805831278;dq[1]=.3526995778083801;
  previous[1]=.40790863069094213;
  ASSERT_TRUE(law.evaluate(r,origin(),dq,Joints{},previous,.001,out));
  EXPECT_NEAR(out.raw[1],-12.238566262720128,1e-10);
  EXPECT_TRUE(out.limited[1]);
  EXPECT_NEAR(out.command[1],previous[1]-kRolloutTorqueRate*.001,1e-12);
  // Full-task settings must not silently become commissioning settings.
  TorqueLaw commissioning;EXPECT_FALSE(commissioning.configure(s));
}
TEST(RawPolicyRollout, ManufacturerTorqueBoundsAndIllegalValuesRemainRejected) {
  TorqueSettings s;s.kp.fill(100.);s.kd.fill(10.);
  s.magnitude=kRolloutTorqueMagnitude;s.rate.fill(kRolloutTorqueRate);
  for(const auto j:{0,4}) {
    RolloutTorqueLaw law;ASSERT_TRUE(law.configure(s,true));
    Reference r;r.q=origin();r.q[j]+=(s.magnitude[j]+1.)/s.kp[j];TorqueOutput out;
    EXPECT_FALSE(law.evaluate(r,origin(),{},Joints{},Joints{},.001,out));
    EXPECT_EQ(law.fault(),TorqueFault::Magnitude);
  }
  RolloutTorqueLaw invalid;auto excessive=s;excessive.magnitude[0]=88.;
  EXPECT_FALSE(invalid.configure(excessive,true));
  excessive=s;excessive.rate.fill(1000.);EXPECT_FALSE(invalid.configure(excessive,true));
  ASSERT_TRUE(invalid.configure(s,true));Reference r;r.q=origin();r.dq[0]=NAN;TorqueOutput out;
  EXPECT_FALSE(invalid.evaluate(r,origin(),{},Joints{},Joints{},.001,out));
  EXPECT_EQ(invalid.fault(),TorqueFault::Nonfinite);
}
TEST(RawPolicyRollout, ObservedUpdateGapIsRecordedAndExecutedWithoutPositionErrorCutoff) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,120.,true));
  auto first=curve_packet(0);PreparedEffortPacket prepared;
  ASSERT_TRUE(prepare_effort_packet(first,prepared,true));ASSERT_TRUE(core.admit(prepared,100.02));
  for(int ms=1;ms<=120;++ms)ASSERT_TRUE(tick(core,ms));
  auto next=curve_packet(1,100.12);constexpr double gap=.14790583482325975;
  next.curve.legendre[0][3]+=gap;const auto unmodified=next.curve.legendre;
  ASSERT_TRUE(prepare_effort_packet(next,prepared,true));ASSERT_TRUE(core.admit(prepared,100.14));
  EXPECT_NEAR(core.position_gap(),gap,1e-12);
  for(int ms=121;ms<=250;++ms)ASSERT_TRUE(tick(core,ms));
  EXPECT_EQ(core.sequence(),1);EXPECT_EQ(core.handoffs(),1);
  EXPECT_NEAR(core.reference().q[3],origin()[3]+gap,1e-12);EXPECT_EQ(next.curve.legendre,unmodified);
  TorqueSettings s;s.kp.fill(10.);s.kd.fill(10.);s.magnitude.fill(4.);s.rate.fill(100.);
  RolloutTorqueLaw law;ASSERT_TRUE(law.configure(s,true));TorqueOutput output;
  ASSERT_TRUE(law.evaluate(core.reference(),origin(),{},Joints{},Joints{},.001,output));
  EXPECT_NEAR(output.raw[3],10.*gap,1e-12);
}
TEST(RawPolicyRollout, FirstAnchorHasSeparateExecutionClockWithoutHidingStaleSource) {
  auto raw=curve_packet(0,100.,.12);raw.handoff=100.5333333333;raw.execution_origin=raw.handoff;
  PreparedEffortPacket packet;ASSERT_TRUE(prepare_effort_packet(raw,packet,true));
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,3.,true));
  ASSERT_TRUE(core.admit(packet,100.33));
  for(int ms=1;ms<=600;++ms) ASSERT_TRUE(tick(core,ms));
  EXPECT_NEAR(core.reference().q[3],origin()[3]+.12*(100.6-raw.handoff),1e-12);
  EXPECT_NEAR(core.reference().dq[3],.12,1e-12);
  EXPECT_DOUBLE_EQ(core.observation_time(),100.);
  raw.observation=99.;ASSERT_TRUE(prepare_effort_packet(raw,packet,true));
  ContinuousEffortCore stale;ASSERT_TRUE(stale.arm(origin(),{},origin(),10.,100.,3.,true));
  EXPECT_FALSE(stale.admit(packet,100.33));EXPECT_EQ(stale.rejection(),EffortStreamRejection::Clock);
  raw.sequence=1;EXPECT_FALSE(prepare_effort_packet(raw,packet,true));
}

TEST(RawPolicyRollout, UsesPhysicalHorizonAndAcceptsUpdatePastOldMicroPrefix) {
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(origin(),{},origin(),10.,100.,120.,true));
  auto first=curve_packet(0);first.handoff=100.15;first.execution_origin=100.15;
  PreparedEffortPacket prepared;ASSERT_TRUE(prepare_effort_packet(first,prepared,true));
  ASSERT_TRUE(core.admit(prepared,100.02));
  for(int ms=1;ms<=660;++ms)ASSERT_TRUE(tick(core,ms));
  auto next=curve_packet(1,100.4);next.handoff=100.97;
  ASSERT_TRUE(prepare_effort_packet(next,prepared,true));
  ASSERT_TRUE(core.admit(prepared,100.66));
  for(int ms=661;ms<=1000;++ms)ASSERT_TRUE(tick(core,ms));
  EXPECT_EQ(core.sequence(),1);EXPECT_EQ(core.handoffs(),1);
}
TEST(RawPolicyRollout, HardwareJointLimitsReplaceDemonstrationRange) {
  auto q=origin();q[0]=.4;
  ContinuousEffortCore core;ASSERT_TRUE(core.arm(q,{},q,10.,100.,120.,true));
  auto raw=curve_packet(0);raw.curve.legendre[0]=q;
  PreparedEffortPacket prepared;ASSERT_TRUE(prepare_effort_packet(raw,prepared,true));
  EXPECT_TRUE(core.admit(prepared,100.02));
  q[0]=kRolloutUpper[0]+.01;
  ContinuousEffortCore invalid;EXPECT_FALSE(invalid.arm(q,{},q,10.,100.,120.,true));
}
