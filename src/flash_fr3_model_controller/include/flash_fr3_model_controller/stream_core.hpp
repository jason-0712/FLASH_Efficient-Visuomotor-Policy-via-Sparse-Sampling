#pragma once
#include "flash_fr3_model_controller/model_core.hpp"

namespace flash_fr3_model_controller {
// Bounded commissioning: 0.5s prefixes and a 0.5s nominal stop. At most two
// replacements (2s total). The position envelope always uses the run origin.
inline bool validate_stream_plan(const Plan& plan, const Reference& initial,
                                 std::array<Joints,4>* nominal = nullptr,
                                 std::array<Joints,4>* brake = nullptr) {
  if (std::abs(plan[0].duration-.5)>1e-12 || std::abs(plan[1].duration-.5)>1e-12)
    return false;
  const auto start = plan[0].at(0.);
  const Joints* a[]{&start.q,&start.dq,&start.ddq};
  const Joints* b[]{&initial.q,&initial.dq,&initial.ddq};
  for (int k=0;k<3;++k) for (std::size_t j=0;j<7;++j)
    if (!std::isfinite((*b[k])[j]) || std::abs((*a[k])[j]-(*b[k])[j])>1e-9) return false;
  // Match the existing nominal/brake validator, allowing a moving C2 start.
  std::array<Joints,4> bounds{}, brake_bounds{};
  for (const auto& p:plan) {
    if (!std::isfinite(p.duration)) return false;
    for (const auto& c:p.c) if (!finite(c)) return false;
    for (int k=0;k<4;++k) {
      const auto v=absolute_bound(p,k);
      for (std::size_t j=0;j<7;++j) bounds[k][j]=std::max(bounds[k][j],v[j]);
    }
  }
  const auto left=plan[0].at(plan[0].duration), right=plan[1].at(0.);
  const auto end=plan[1].at(plan[1].duration);
  for (std::size_t j=0;j<7;++j)
    if (std::abs(left.q[j]-right.q[j])>1e-9 || std::abs(left.dq[j]-right.dq[j])>1e-9 ||
        std::abs(left.ddq[j]-right.ddq[j])>1e-9 || std::abs(end.dq[j])>1e-9 ||
        std::abs(end.ddq[j])>1e-9) return false;
  Reference unit_v{},unit_a{}; unit_v.dq.fill(1.); unit_a.ddq.fill(1.);
  const auto rv=stopping_polynomial(unit_v,kBrakeSeconds), ra=stopping_polynomial(unit_a,kBrakeSeconds);
  for (int k=0;k<4;++k) {
    const auto bv=absolute_bound(rv,k), ba=absolute_bound(ra,k);
    for (std::size_t j=0;j<7;++j) {
      brake_bounds[k][j]=(k==0?bounds[0][j]:0.)+bv[j]*bounds[1][j]+ba[j]*bounds[2][j];
      if (bounds[k][j]>kReferenceLimits[k]-1e-10 || brake_bounds[k][j]>kReferenceLimits[k]-1e-10)
        return false;
    }
  }
  if (nominal) *nominal=bounds;
  if (brake) *brake=brake_bounds;
  return true;
}

struct StreamPacket {
  int sequence{0};
  double handoff_robot_time{0.};
  Plan plan{};
};
enum class Rejection { None=0, WrongSequence=1, LateOrWrongTime=2, InvalidPlan=3 };

class StreamCore {
 public:
  bool arm(const Joints& desired,const Joints& q,const Joints& dq,const Joints& expected,
           const Plan& plan,double robot_time,double steady_time,int max_replacements=1) {
    phase_=Phase::Error; reason_=Reason::InitialState;
    ModelCore initial_check;
    if (max_replacements<1 || max_replacements>2 || !validate_stream_plan(plan,{}) ||
        !initial_check.arm(desired,q,dq,expected,plan,robot_time,steady_time)) return false;
    plan_=plan; origin_=desired; reference_={}; reference_.q=desired;
    previous_robot_=robot_time;
    previous_steady_=activated_steady_=last_progress_steady_=steady_time;
    sequence_=0; last_candidate_=0; pending_=false; rejection_=Rejection::None;
    max_replacements_=max_replacements;
    phase_=Phase::Waiting; reason_=Reason::None;
    return true;
  }
  bool stage(const StreamPacket& packet,double robot_time) {
    if (packet.sequence<=last_candidate_) return false;
    last_candidate_=packet.sequence;
    if (phase_!=Phase::Moving || sequence_>=max_replacements_ ||
        packet.sequence!=sequence_+1 || pending_) {
      rejection_=Rejection::WrongSequence; return false;
    }
    const double end=prefix_start_+plan_[0].duration;
    if (!std::isfinite(packet.handoff_robot_time) || !std::isfinite(robot_time) ||
        std::abs(packet.handoff_robot_time-end)>1e-6 || end-robot_time<.02) {
      rejection_=Rejection::LateOrWrongTime; return false;
    }
    std::array<Joints,4> bounds{},brake{};
    if (!validate_stream_plan(packet.plan,plan_[0].at(plan_[0].duration),&bounds,&brake)) {
      rejection_=Rejection::InvalidPlan; return false;
    }
    for (std::size_t j=0;j<7;++j) {
      const double reach=std::max(bounds[0][j],brake[0][j]);
      if (origin_[j]-reach<kLower[j]+.02 || origin_[j]+reach>kUpper[j]-.02) {
        rejection_=Rejection::InvalidPlan; return false;
      }
    }
    pending_plan_=packet.plan; pending_=true; rejection_=Rejection::None;
    return true;
  }
  bool step(double robot_time,double steady_time,double heartbeat_age,const Joints& q,const Joints& dq) {
    if (phase_==Phase::Error) return false;
    const double dr=robot_time-previous_robot_,ds=steady_time-previous_steady_;
    if (!std::isfinite(robot_time)||!std::isfinite(steady_time)||!std::isfinite(heartbeat_age)||
        dr<0||ds<0||dr>.005||ds>.010) return fail(Reason::ClockOrDeadline);
    if (dr>0) last_progress_steady_=steady_time;
    if (steady_time-last_progress_steady_>.005) return fail(Reason::ClockOrDeadline);
    previous_robot_=robot_time; previous_steady_=steady_time;
    if (!finite(q)||!finite(dq)) return fail(Reason::StateLimit);
    for (std::size_t j=0;j<7;++j) {
      if (q[j]<kLower[j]||q[j]>kUpper[j]||std::abs(q[j]-origin_[j])>.02||std::abs(dq[j])>.05)
        return fail(Reason::StateLimit);
      if (std::abs(q[j]-reference_.q[j])>.01) return fail(Reason::TrackingError);
    }
    if (phase_==Phase::Waiting) {
      if (steady_time-activated_steady_>2.) {phase_=Phase::LatchedHold;reason_=Reason::StartTimeout;}
      else if (heartbeat_age>=0&&heartbeat_age<=kHeartbeatSeconds) {
        prefix_start_=robot_time;phase_=Phase::Moving;
      }
    }
    if (phase_==Phase::Moving) {
      if (pending_ && robot_time>=prefix_start_+plan_[0].duration) {
        prefix_start_+=plan_[0].duration;plan_=pending_plan_;pending_=false;++sequence_;
      }
      const double t=robot_time-prefix_start_;
      if (t>=plan_[0].duration+plan_[1].duration) {
        reference_=plan_[1].at(plan_[1].duration);
        reference_.dq=reference_.ddq=reference_.jerk={};
        for (std::size_t j=0;j<7;++j) reference_.q[j]+=origin_[j];
        phase_=Phase::Complete;
      } else {
        reference_=t<=plan_[0].duration?plan_[0].at(t):plan_[1].at(t-plan_[0].duration);
        for (std::size_t j=0;j<7;++j) reference_.q[j]+=origin_[j];
        if (heartbeat_age<0||heartbeat_age>kHeartbeatSeconds) {
          brake_=stopping_polynomial(reference_,kBrakeSeconds);brake_start_=robot_time;
          pending_=false;phase_=Phase::Braking;reason_=Reason::HeartbeatTimeout;
        }
      }
    }
    if (phase_==Phase::Braking) {
      reference_=brake_.at(robot_time-brake_start_);
      if (robot_time-brake_start_>=kBrakeSeconds) {
        reference_.dq=reference_.ddq=reference_.jerk={};phase_=Phase::LatchedHold;
      }
    }
    const Joints* values[]{&reference_.q,&reference_.dq,&reference_.ddq,&reference_.jerk};
    for (int k=0;k<4;++k) for (std::size_t j=0;j<7;++j)
      if (!std::isfinite((*values[k])[j]) ||
          std::abs((*values[k])[j]-(k==0?origin_[j]:0.))>kReferenceLimits[k]) return fail(Reason::ReferenceLimit);
    return true;
  }
  const Reference& reference() const{return reference_;}
  const Joints& origin() const{return origin_;}
  Phase phase() const{return phase_;} Reason reason() const{return reason_;}
  int sequence() const{return sequence_;} int pending_sequence() const{return pending_?sequence_+1:-1;}
  int last_candidate() const{return last_candidate_;} Rejection rejection() const{return rejection_;}
  double prefix_start() const{return prefix_start_;}
  double prefix_end() const{return prefix_start_+plan_[0].duration;}
 private:
  bool fail(Reason r){phase_=Phase::Error;reason_=r;return false;}
  Joints origin_{}; Reference reference_{}; Plan plan_{},pending_plan_{}; Polynomial brake_{};
  Phase phase_{Phase::Error}; Reason reason_{Reason::InitialState};
  int sequence_{0},last_candidate_{0},max_replacements_{1};
  bool pending_{false}; Rejection rejection_{Rejection::None};
  double previous_robot_{},previous_steady_{},activated_steady_{},last_progress_steady_{};
  double prefix_start_{},brake_start_{};
};
}  // namespace flash_fr3_model_controller
