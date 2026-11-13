#pragma once
#include "flash_fr3_model_controller/continuous_reference.hpp"
#include "flash_fr3_model_controller/rollout_torque_law.hpp"
#include <cstdint>

namespace flash_fr3_model_controller {
// Bounded commissioning, NOT the task envelope or hardware limits. Raw model
// coefficients are never repaired, shifted to measured q, or time-scaled.
constexpr std::array<double,4> kContinuousEffortLimits{.002,.005,.02,.2};
constexpr double kContinuousBrake=.5;
// Local FR3 rollout guards, not author-reported parameters. The task profile
// consumes an unmodified raw prefix while a fresh inference is being prepared.
// These bounds replace micro-probe admission; they do not rescale the curve.
constexpr std::array<double,4> kRolloutDerivativeLimits{0.,.8,10.,100.};
enum class EffortStreamReason {None=0,Heartbeat=1,StartTimeout=2,Clock=3,State=4,
  Tracking=5,Reference=6,Initial=7,PredictionExpired=8,SessionLimit=9,Finish=10};
enum class EffortStreamRejection {None=0,Format=1,Sequence=2,Clock=3,Busy=4,
  Envelope=5,Position=6,Velocity=7,Stopped=8};

struct PreparedEffortPacket {
  bool available{false};PhysicalPacket raw{};
  double proof_end_seconds{};
  Joints lower{},upper{},brake_drift{};
  std::array<Joints,4> nominal{},brake{};
};

// Algebraic basis conversion over [begin,end], only for Bernstein bounds.
// Actual execution still evaluates the original Legendre curve directly.
inline bool physical_power_interval(const PhysicalCurve& curve,double begin,double end,Polynomial& out) {
  if(!curve.valid()||!std::isfinite(begin)||!std::isfinite(end)||begin<0.||
     end>curve.horizon||end-begin<.02) return false;
  Polynomial p;p.duration=end-begin;
  const double a=2.*(curve.anchor+begin/curve.duration)-1.,b=2.*p.duration/curve.duration;
  std::array<std::array<double,7>,7> basis{};basis[0][0]=1.;basis[1][0]=a;basis[1][1]=b;
  for(int n=2;n<=6;++n) for(int k=0;k<=n;++k)
    basis[n][k]=((2*n-1)*(a*basis[n-1][k]+(k?b*basis[n-1][k-1]:0.))-
                  (n-1)*basis[n-2][k])/n;
  for(int k=0;k<=6;++k) for(std::size_t j=0;j<7;++j) {
    for(int n=0;n<=6;++n) p.c[k][j]+=curve.legendre[n][j]*basis[n][k];
    if(!std::isfinite(p.c[k][j])) return false;
  }
  out=p;return true;
}

// Run in the non-RT subscription callback; immutable proof data crosses the
// RT buffer with the original coefficients. Covers every possible brake start
// in the admitted future interval by triangle/convex-hull bounds, not a grid.
inline bool prepare_effort_packet(const PhysicalPacket& raw,PreparedEffortPacket& result,bool task_rollout=false) {
  if(!std::isfinite(raw.observation)||!std::isfinite(raw.handoff)||raw.observation<0.||
     raw.sequence<0||raw.sequence>1000000||!raw.curve.valid()) return false;
  const double begin=raw.handoff-raw.curve_origin();
  if(!std::isfinite(raw.curve_origin())||raw.curve_origin()<raw.observation||
     (std::isfinite(raw.execution_origin)&&(raw.sequence!=0||std::abs(raw.execution_origin-raw.handoff)>1e-8)))
    return false;
  if(begin<0.||begin>raw.curve.horizon-kContinuousBrake-.02) return false;
  const double end=raw.curve.horizon;
  PreparedEffortPacket candidate;candidate.available=true;candidate.raw=raw;
  candidate.proof_end_seconds=end;
  candidate.lower.fill(INFINITY);candidate.upper.fill(-INFINITY);
  // Same Bernstein bounds over smaller intervals; no refitting or repair.
  // Subdivision avoids loose whole-horizon bounds rejecting a valid curve.
  const int parts=task_rollout?8:1;
  for(int i=0;i<parts;++i) {
    Polynomial p;
    if(!physical_power_interval(raw.curve,begin+(end-begin)*i/parts,
                               begin+(end-begin)*(i+1)/parts,p)) return false;
    const auto range=signed_position_bounds(p);
    for(std::size_t j=0;j<7;++j) {
      candidate.lower[j]=std::min(candidate.lower[j],range[0][j]);
      candidate.upper[j]=std::max(candidate.upper[j],range[1][j]);
    }
    for(int order=1;order<=3;++order) {
      const auto bounds=absolute_bound(p,order);
      for(std::size_t j=0;j<7;++j)candidate.nominal[order][j]=std::max(candidate.nominal[order][j],bounds[j]);
    }
  }
  Reference v{},a{};v.dq.fill(1.);a.ddq.fill(1.);
  const auto pv=stopping_polynomial(v,kContinuousBrake),pa=stopping_polynomial(a,kContinuousBrake);
  for(int order=0;order<=3;++order) {
    const auto bv=absolute_bound(pv,order),ba=absolute_bound(pa,order);
    for(std::size_t j=0;j<7;++j) {
      const double bound=bv[j]*candidate.nominal[1][j]+ba[j]*candidate.nominal[2][j];
      if(!std::isfinite(bound)||!std::isfinite(candidate.lower[j])||!std::isfinite(candidate.upper[j])) return false;
      if(order==0) candidate.brake_drift[j]=bound;
      else candidate.brake[order][j]=bound;
    }
  }
  result=candidate;return true;
}

// Wire data: version, sequence, sent ROS-wall seconds, observation steady
// seconds, handoff steady seconds, anchor, duration, horizon, 49 coefficients.
// v2 appends the explicit first-curve execution origin. Source time is never
// overwritten. v1 remains compatible for latency-compensated updates/probes.
// Session identity is the single layout dimension label, checked by the node.
inline bool decode_effort_packet(const double* data,std::size_t count,PhysicalPacket& result) {
  if(count!=57&&count!=58) return false;
  for(std::size_t i=0;i<count;++i) if(!std::isfinite(data[i])) return false;
  if((count==57?data[0]!=1.:data[0]!=2.)||data[1]<0.||data[1]>1000000.||std::floor(data[1])!=data[1]||data[2]<0.||data[2]>9e9)
    return false;
  PhysicalPacket candidate;candidate.sequence=static_cast<int>(data[1]);
  candidate.observation=data[3];candidate.handoff=data[4];
  candidate.curve.anchor=data[5];candidate.curve.duration=data[6];candidate.curve.horizon=data[7];
  for(int k=0;k<7;++k) for(int j=0;j<7;++j) candidate.curve.legendre[k][j]=data[8+k*7+j];
  if(count==58) {
    candidate.execution_origin=data[57];
    if(candidate.sequence!=0||candidate.execution_origin<candidate.observation||
       std::abs(candidate.execution_origin-candidate.handoff)>1e-8) return false;
  }
  if(!candidate.curve.valid()) return false;
  result=candidate;return true;
}

class ContinuousEffortCore {
 public:
  bool arm(const Joints& q,const Joints& dq,const Joints& expected,double robot,double now,double session,
           bool task_rollout=false) {
    phase_=Phase::Error;reason_=EffortStreamReason::Initial;
    if(armed_||!finite(q)||!finite(dq)||!finite(expected)||!std::isfinite(robot)||robot<0.||
       !std::isfinite(now)||now<0.||!std::isfinite(session)||session<1.) return false;
    for(std::size_t j=0;j<7;++j)
      if(std::abs(q[j]-expected[j])>.002||std::abs(dq[j])>.01||
         q[j]-.004<(task_rollout?kRolloutLower[j]:kTaskLower[j])||q[j]+.004>(task_rollout?kRolloutUpper[j]:kTaskUpper[j])) return false;
    origin_=q;reference_={};reference_.q=q;session_=session;task_rollout_=task_rollout;
    previous_robot_=start_robot_=robot;previous_steady_=start_steady_=now;
    phase_=Phase::Waiting;reason_=EffortStreamReason::None;armed_=true;return true;
  }
  // libfranka supplies an integer millisecond clock. Subtract those ticks
  // before conversion: subtracting toSec() values can turn exactly 2 ms into
  // 0.0020000000004 s and spuriously trip the unchanged 2 ms upper bound.
  bool arm_milliseconds(const Joints& q,const Joints& dq,const Joints& expected,
                        uint64_t robot_ms,double now,double session,bool task_rollout=false) {
    if(!arm(q,dq,expected,robot_ms/1000.,now,session,task_rollout)) return false;
    previous_robot_ms_=robot_ms;return true;
  }
  bool admit(const PreparedEffortPacket& prepared,double now) {
    position_gap_=velocity_gap_=0.;
    if(!armed_||phase_==Phase::Braking||phase_==Phase::LatchedHold||phase_==Phase::Complete||phase_==Phase::Error)
      return reject(EffortStreamRejection::Stopped);
    const auto& p=prepared.raw;
    if(!prepared.available||!std::isfinite(now)||now<previous_steady_||
       p.sequence<=last_candidate_||(!have_active_&&p.sequence!=0)) return reject(EffortStreamRejection::Sequence);
    last_candidate_=p.sequence;
    if(pending_) return reject(EffortStreamRejection::Busy);
    if(std::isfinite(p.execution_origin)&&(!task_rollout_||have_active_||p.sequence!=0))
      return reject(EffortStreamRejection::Clock);
    if(!std::isfinite(p.observation)||!std::isfinite(p.handoff)||p.observation>now||now-p.observation>.7||
       p.handoff-now<.02||p.handoff>start_steady_+session_-kContinuousBrake||
       p.handoff-p.curve_origin()>p.curve.horizon-kContinuousBrake-.02)
      return reject(EffortStreamRejection::Clock);
    if(have_active_&&(p.observation<=active_.observation||p.handoff<=active_.handoff||
       p.handoff>=active_deadline()))
      return reject(EffortStreamRejection::Clock);
    const double required_end=p.curve.horizon;
    if(prepared.proof_end_seconds+1e-8<required_end) return reject(EffortStreamRejection::Envelope);
    for(std::size_t j=0;j<7;++j) {
      const double reach=std::max(std::abs(prepared.lower[j]-origin_[j]),std::abs(prepared.upper[j]-origin_[j]))+
                         prepared.brake_drift[j];
      if(!std::isfinite(reach)||
         (!task_rollout_&&reach>kContinuousEffortLimits[0]-1e-10)||
         (task_rollout_&&(prepared.lower[j]-prepared.brake_drift[j]<kRolloutLower[j]||
                          prepared.upper[j]+prepared.brake_drift[j]>kRolloutUpper[j])))
        return reject(EffortStreamRejection::Envelope);
      for(int order=1;order<=3;++order)
        if(!std::isfinite(prepared.nominal[order][j])||!std::isfinite(prepared.brake[order][j])||
           std::max(prepared.nominal[order][j],prepared.brake[order][j])>
             (task_rollout_?kRolloutDerivativeLimits[order]:kContinuousEffortLimits[order])-1e-10)
          return reject(EffortStreamRejection::Envelope);
    }
    Reference left=reference_,right{};
    if((have_active_&&!active_.curve.at(p.handoff-active_.curve_origin(),left))||
       !p.curve.at(p.handoff-p.curve_origin(),right)) return reject(EffortStreamRejection::Clock);
    for(std::size_t j=0;j<7;++j) {
      position_gap_=std::max(position_gap_,std::abs(left.q[j]-right.q[j]));
      velocity_gap_=std::max(velocity_gap_,std::abs(left.dq[j]-right.dq[j]));
    }
    // Task execution records the raw jump; it does not impose the local
    // commissioning continuity threshold on the learned FLASH polynomial.
    if(!task_rollout_&&position_gap_>1e-4) return reject(EffortStreamRejection::Position);
    if(!task_rollout_&&velocity_gap_>1e-4) return reject(EffortStreamRejection::Velocity);
    next_=p;pending_=true;rejection_=EffortStreamRejection::None;return true;
  }
  bool step(double robot,double now,double heartbeat_age,const Joints& q,const Joints& dq,bool finish=false) {
    return step_with_period(robot,now,heartbeat_age,q,dq,finish,robot-previous_robot_);
  }
  bool step_milliseconds(uint64_t robot_ms,double now,double heartbeat_age,
                         const Joints& q,const Joints& dq,bool finish=false) {
    const double period=robot_ms>=previous_robot_ms_?(robot_ms-previous_robot_ms_)*.001:
                                                     -(previous_robot_ms_-robot_ms)*.001;
    if(!step_with_period(robot_ms/1000.,now,heartbeat_age,q,dq,finish,period)) return false;
    previous_robot_ms_=robot_ms;return true;
  }
 private:
  bool step_with_period(double robot,double now,double heartbeat_age,const Joints& q,
                        const Joints& dq,bool finish,double dr) {
    if(!armed_||phase_==Phase::Error) return false;
    const double ds=now-previous_steady_;
    clock_sample_={dr,robot-previous_robot_,ds,(robot-start_robot_)-(now-start_steady_)};
    if(!std::isfinite(robot)||!std::isfinite(now)||!std::isfinite(heartbeat_age)||
       dr<.0005||dr>.002||ds<0.||ds>.010||
       std::abs((robot-start_robot_)-(now-start_steady_))>.02) return fail(EffortStreamReason::Clock);
    previous_robot_=robot;previous_steady_=now;
    if(!finite(q)||!finite(dq)) return fail(EffortStreamReason::State);
    for(std::size_t j=0;j<7;++j) {
      if(q[j]<(task_rollout_?kRolloutLower[j]:kTaskLower[j])||q[j]>(task_rollout_?kRolloutUpper[j]:kTaskUpper[j])||
         (!task_rollout_&&std::abs(q[j]-origin_[j])>.004)||std::abs(dq[j])>(task_rollout_?1.:.05))
        return fail(EffortStreamReason::State);
      if(!task_rollout_&&std::abs(q[j]-reference_.q[j])>.01) return fail(EffortStreamReason::Tracking);
    }
    const bool heartbeat_ok=heartbeat_age>=0.&&heartbeat_age<=kHeartbeatSeconds;
    if(phase_==Phase::Waiting) {
      if(finish) {reason_=EffortStreamReason::Finish;phase_=Phase::Complete;pending_=false;}
      else if(now-start_steady_>=std::min(2.,session_-kContinuousBrake)) {
        reason_=EffortStreamReason::StartTimeout;phase_=Phase::LatchedHold;pending_=false;
      }
      else if(pending_&&now>=next_.handoff) {
        if(!heartbeat_ok) {reason_=EffortStreamReason::Heartbeat;phase_=Phase::LatchedHold;pending_=false;}
        else {active_=next_;have_active_=true;pending_=false;phase_=Phase::Moving;}
      }
    }
    if(phase_==Phase::Moving) {
      const auto stop=finish?EffortStreamReason::Finish:!heartbeat_ok?EffortStreamReason::Heartbeat:
        now-start_steady_>=session_-kContinuousBrake?EffortStreamReason::SessionLimit:
        now>=active_deadline()?EffortStreamReason::PredictionExpired:
        EffortStreamReason::None;
      if(stop==EffortStreamReason::None&&pending_&&now>=next_.handoff) {
        active_=next_;pending_=false;++handoffs_;
      }
      if(!active_.curve.at(now-active_.curve_origin(),reference_)) return fail(EffortStreamReason::Reference);
      if(stop!=EffortStreamReason::None) {
        brake_=stopping_polynomial(reference_,kContinuousBrake);brake_start_=now;
        phase_=Phase::Braking;reason_=stop;pending_=false;
      }
    }
    if(phase_==Phase::Braking) {
      reference_=brake_.at(now-brake_start_);
      if(now-brake_start_>=kContinuousBrake) {
        reference_.dq=reference_.ddq=reference_.jerk={};
        phase_=reason_==EffortStreamReason::Finish||reason_==EffortStreamReason::SessionLimit?Phase::Complete:Phase::LatchedHold;
      }
    }
    const Joints* values[]{&reference_.q,&reference_.dq,&reference_.ddq,&reference_.jerk};
    for(int order=0;order<=3;++order) for(std::size_t j=0;j<7;++j)
      if(!std::isfinite((*values[order])[j])||
         (!task_rollout_&&std::abs((*values[order])[j]-(order==0?origin_[j]:0.))>kContinuousEffortLimits[order])||
         (task_rollout_&&(order==0?((*values[order])[j]<kRolloutLower[j]||(*values[order])[j]>kRolloutUpper[j]):
             std::abs((*values[order])[j])>kRolloutDerivativeLimits[order])))
        return fail(EffortStreamReason::Reference);
    return true;
  }
 public:
  const std::array<double,4>& clock_sample()const{return clock_sample_;}
  double robot_period_seconds()const{return clock_sample_[0];}
  const Reference& reference()const{return reference_;}
  Phase phase()const{return phase_;}
  EffortStreamReason reason()const{return reason_;}
  EffortStreamRejection rejection()const{return rejection_;}
  int sequence()const{return have_active_?active_.sequence:-1;}
  int pending_sequence()const{return pending_?next_.sequence:-1;}
  int last_candidate()const{return last_candidate_;}
  int handoffs()const{return handoffs_;}
  double position_gap()const{return position_gap_;}
  double velocity_gap()const{return velocity_gap_;}
  double observation_time()const{return have_active_?active_.observation:0.;}
  double remaining(double now)const{return have_active_?active_.curve_origin()+active_.curve.horizon-now:0.;}
 private:
  double active_deadline()const {
    const double raw=active_.curve_origin()+active_.curve.horizon-kContinuousBrake;
    return raw;
  }
  bool fail(EffortStreamReason reason){phase_=Phase::Error;reason_=reason;pending_=false;return false;}
  bool reject(EffortStreamRejection reason){rejection_=reason;return false;}
  bool armed_{false},have_active_{false},pending_{false};
  bool task_rollout_{false};
  PhysicalPacket active_{},next_{};Joints origin_{};Reference reference_{};Polynomial brake_{};
  Phase phase_{Phase::Error};EffortStreamReason reason_{EffortStreamReason::Initial};
  EffortStreamRejection rejection_{EffortStreamRejection::None};
  int last_candidate_{-1},handoffs_{0};double position_gap_{},velocity_gap_{};
  uint64_t previous_robot_ms_{};std::array<double,4> clock_sample_{};
  double previous_robot_{},previous_steady_{},start_robot_{},start_steady_{},brake_start_{},session_{};
};
}  // namespace flash_fr3_model_controller
