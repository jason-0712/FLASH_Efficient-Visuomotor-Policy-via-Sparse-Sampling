#pragma once
#include "flash_fr3_model_controller/model_core.hpp"
namespace flash_fr3_model_controller {
// Dataset-specific commissioning envelope, not a collision model or new hardware limits.
constexpr Joints kTaskLower{-.40,-.86,-.31,-2.57,-.08,1.48,.22};
constexpr Joints kTaskUpper{.21,.58,.23,-1.83,.11,2.90,1.00};
constexpr std::array<double,4> kTaskReferenceLimits{.20,.20,1.5,30.};
constexpr double kTaskBrakeSeconds=.5;
inline std::array<Joints,2> signed_position_bounds(const Polynomial& p) {
  std::array<Joints,2> out;out[0].fill(INFINITY);out[1].fill(-INFINITY);
  for(std::size_t j=0;j<7;++j) for(int i=0;i<=6;++i) {
    double b=0.;for(int k=0;k<=i;++k)b+=choose(i,k)/choose(6,k)*p.c[k][j];
    out[0][j]=std::min(out[0][j],b);out[1][j]=std::max(out[1][j],b);
  }
  return out;
}
inline bool validate_task_plan(const Plan& plan,const Joints& origin,
                              std::array<Joints,4>* nominal=nullptr,
                              std::array<Joints,4>* brake=nullptr) {
  if(!finite(origin)||!std::isfinite(plan[0].duration)||plan[0].duration<.8||
     plan[0].duration>4.8+1e-9||std::abs(plan[1].duration-.75)>1e-12)return false;
  std::array<Joints,4> bounds{},brake_bounds{};
  for(const auto& p:plan) {
    for(const auto& c:p.c)if(!finite(c))return false;
    for(int k=0;k<4;++k){const auto b=absolute_bound(p,k);
      for(std::size_t j=0;j<7;++j)bounds[k][j]=std::max(bounds[k][j],b[j]);}
  }
  const auto start=plan[0].at(0.),left=plan[0].at(plan[0].duration),right=plan[1].at(0.),end=plan[1].at(.75);
  for(std::size_t j=0;j<7;++j)
    if(std::abs(start.q[j])>1e-10||std::abs(start.dq[j])>1e-10||std::abs(start.ddq[j])>1e-10||
       std::abs(left.q[j]-right.q[j])>1e-9||std::abs(left.dq[j]-right.dq[j])>1e-9||
       std::abs(left.ddq[j]-right.ddq[j])>1e-9||std::abs(end.dq[j])>1e-9||std::abs(end.ddq[j])>1e-9)return false;
  Reference v{},a{};v.dq.fill(1.);a.ddq.fill(1.);
  const auto rv=stopping_polynomial(v,kTaskBrakeSeconds),ra=stopping_polynomial(a,kTaskBrakeSeconds);
  for(int k=0;k<4;++k){const auto bv=absolute_bound(rv,k),ba=absolute_bound(ra,k);
    for(std::size_t j=0;j<7;++j){
      brake_bounds[k][j]=(k==0?bounds[0][j]:0.)+bv[j]*bounds[1][j]+ba[j]*bounds[2][j];
      if(bounds[k][j]>kTaskReferenceLimits[k]-1e-10||brake_bounds[k][j]>kTaskReferenceLimits[k]-1e-10)return false;
    }
  }
  const auto bv=absolute_bound(rv,0),ba=absolute_bound(ra,0);
  for(const auto& p:plan){const auto range=signed_position_bounds(p);
    for(std::size_t j=0;j<7;++j){
      const double drift=bv[j]*bounds[1][j]+ba[j]*bounds[2][j];
      if(origin[j]+range[0][j]-drift<kTaskLower[j]||origin[j]+range[1][j]+drift>kTaskUpper[j])return false;
    }
  }
  if(nominal)*nominal=bounds;
  if(brake)*brake=brake_bounds;
  return true;
}
class TaskCore {
 public:
  bool arm(const Joints& desired, const Joints& q, const Joints& dq, const Joints& expected,
           const Plan& plan, double robot_time, double steady_time) {
    phase_ = Phase::Error; reason_ = Reason::InitialState;
    if (!finite(desired) || !finite(q) || !finite(dq) || !finite(expected) ||
        !std::isfinite(robot_time) || !std::isfinite(steady_time) || !validate_task_plan(plan,desired)) return false;
    std::array<Joints,4> bounds{}, brake_bounds{};
    validate_task_plan(plan,desired,&bounds,&brake_bounds);
    for (std::size_t j = 0; j < 7; ++j) {
      const double reach = std::max(bounds[0][j],brake_bounds[0][j]);
      if (std::abs(q[j]-expected[j]) > .002 || std::abs(desired[j]-q[j]) > .002 ||
          std::abs(dq[j]) > .01 || desired[j]-reach < kLower[j]+.02 ||
          desired[j]+reach > kUpper[j]-.02) return false;
    }
    plan_ = plan; origin_ = desired;
    reference_ = {}; reference_.q = desired;
    previous_robot_ = robot_time;
    previous_steady_ = activated_steady_ = last_progress_steady_ = steady_time;
    phase_ = Phase::Waiting; reason_ = Reason::None;
    return true;
  }
  bool step(double robot_time, double steady_time, double heartbeat_age,
            const Joints& q, const Joints& dq) {
    if (phase_ == Phase::Error) return false;
    const double dr = robot_time-previous_robot_, ds = steady_time-previous_steady_;
    if (!std::isfinite(robot_time) || !std::isfinite(steady_time) || !std::isfinite(heartbeat_age) ||
        dr < 0 || ds < 0 || dr > .005 || ds > .010) return fail(Reason::ClockOrDeadline);
    if (dr > 0) last_progress_steady_ = steady_time;
    if (steady_time-last_progress_steady_ > .005) return fail(Reason::ClockOrDeadline);
    previous_robot_ = robot_time; previous_steady_ = steady_time;
    if (!finite(q) || !finite(dq)) return fail(Reason::StateLimit);
    for (std::size_t j = 0; j < 7; ++j) {
      if (q[j] < kTaskLower[j] || q[j] > kTaskUpper[j] || std::abs(q[j]-origin_[j]) > .22 ||
          std::abs(dq[j]) > .30) return fail(Reason::StateLimit);
      if (std::abs(q[j]-reference_.q[j]) > .02) return fail(Reason::TrackingError);
    }
    if (phase_ == Phase::Waiting) {
      if (steady_time-activated_steady_ > 2.) {
        phase_ = Phase::LatchedHold; reason_ = Reason::StartTimeout;
      } else if (heartbeat_age >= 0 && heartbeat_age <= kHeartbeatSeconds) {
        started_robot_ = robot_time; phase_ = Phase::Moving;
      }
    }
    if (phase_ == Phase::Moving) {
      const double t = robot_time-started_robot_;
      if (t >= plan_[0].duration+plan_[1].duration) {
        reference_ = plan_[1].at(plan_[1].duration);
        reference_.dq = reference_.ddq = reference_.jerk = {};
        for (std::size_t j = 0; j < 7; ++j) reference_.q[j] += origin_[j];
        phase_ = Phase::Complete;
      } else {
        reference_ = t <= plan_[0].duration ? plan_[0].at(t) : plan_[1].at(t-plan_[0].duration);
        for (std::size_t j = 0; j < 7; ++j) reference_.q[j] += origin_[j];
        if (heartbeat_age < 0 || heartbeat_age > kHeartbeatSeconds) {
          brake_ = stopping_polynomial(reference_,kTaskBrakeSeconds);
          started_brake_robot_ = robot_time;
          phase_ = Phase::Braking; reason_ = Reason::HeartbeatTimeout;
        }
      }
    }
    if (phase_ == Phase::Braking) {
      const double t = robot_time-started_brake_robot_;
      reference_ = brake_.at(t);
      if (t >= kTaskBrakeSeconds) {
        reference_.dq = reference_.ddq = reference_.jerk = {};
        phase_ = Phase::LatchedHold;
      }
    }
    const Joints* values[]{&reference_.q,&reference_.dq,&reference_.ddq,&reference_.jerk};
    for (int order = 0; order <= 3; ++order)
      for (std::size_t j = 0; j < 7; ++j)
        if (!std::isfinite((*values[order])[j]) ||
            std::abs((*values[order])[j]-(order == 0 ? origin_[j] : 0.)) > kTaskReferenceLimits[order])
          return fail(Reason::ReferenceLimit);
    return true;
  }
  const Reference& reference() const { return reference_; }
  Phase phase() const { return phase_; }
  Reason reason() const { return reason_; }
 private:
  bool fail(Reason reason) { phase_ = Phase::Error; reason_ = reason; return false; }
  Joints origin_{};
  Reference reference_{};
  Plan plan_{};
  Polynomial brake_{};
  Phase phase_{Phase::Error};
  Reason reason_{Reason::InitialState};
  double previous_robot_{}, previous_steady_{}, activated_steady_{}, last_progress_steady_{};
  double started_robot_{}, started_brake_robot_{};
};
}  // namespace flash_fr3_model_controller
