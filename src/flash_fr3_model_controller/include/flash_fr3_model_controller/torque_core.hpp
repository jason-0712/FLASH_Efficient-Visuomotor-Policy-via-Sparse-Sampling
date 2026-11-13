#pragma once
#include "flash_fr3_model_controller/task_core.hpp"

namespace flash_fr3_model_controller {
// Initial effort-interface commissioning envelope. These are application
// limits, not the robot's hardware limits or the paper's unpublished gains.
struct TorqueSettings {
  Joints kp{}, kd{}, magnitude{}, rate{};
  bool valid() const {
    if (!finite(kp) || !finite(kd) || !finite(magnitude) || !finite(rate)) return false;
    for (std::size_t j=0; j<7; ++j)
      if (kp[j]<1. || kp[j]>300. || kd[j]<1. || kd[j]>60. ||
          magnitude[j]<.5 || magnitude[j]>(j<4?20.:8.) || rate[j]<1. || rate[j]>100.) return false;
    return true;
  }
};
enum class TorqueFault { None=0, Configuration=1, Nonfinite=2, Period=3,
                         ReferenceOrState=4, Magnitude=5, PreviousTorque=6, Model=7 };
struct TorqueOutput { Joints raw{}, command{}; std::array<bool,7> limited{}; };

class TorqueLaw {
 public:
  bool configure(const TorqueSettings& settings) {
    settings_=settings; fault_=settings.valid()?TorqueFault::None:TorqueFault::Configuration;
    return fault_==TorqueFault::None;
  }
  bool evaluate(const Reference& desired, const Joints& q, const Joints& dq,
                const Joints& coriolis, const Joints& previous_desired_torque,
                double robot_period, TorqueOutput& result) {
    // Failure is latched; never publish a partially validated 7-vector.
    if (fault_!=TorqueFault::None) return false;
    if (!finite(desired.q)||!finite(desired.dq)||!finite(q)||!finite(dq)||
        !finite(coriolis)||!finite(previous_desired_torque)) return fail(TorqueFault::Nonfinite);
    if (!std::isfinite(robot_period)||robot_period<.0005||robot_period>.002)
      return fail(TorqueFault::Period);
    TorqueOutput candidate;
    for (std::size_t j=0;j<7;++j) {
      if (q[j]<kTaskLower[j]||q[j]>kTaskUpper[j]||std::abs(dq[j])>.05||
          std::abs(desired.q[j]-q[j])>.01||std::abs(desired.dq[j])>.005)
        return fail(TorqueFault::ReferenceOrState);
      // libfranka torques exclude gravity. Do not add a second gravity term.
      candidate.raw[j]=settings_.kp[j]*(desired.q[j]-q[j])+
          settings_.kd[j]*(desired.dq[j]-dq[j])+coriolis[j];
      if (!std::isfinite(candidate.raw[j])||std::abs(candidate.raw[j])>settings_.magnitude[j])
        return fail(TorqueFault::Magnitude);
      if (std::abs(previous_desired_torque[j])>settings_.magnitude[j])
        return fail(TorqueFault::PreviousTorque);
      const double delta=candidate.raw[j]-previous_desired_torque[j];
      const double bound=settings_.rate[j]*robot_period;
      candidate.command[j]=previous_desired_torque[j]+std::clamp(delta,-bound,bound);
      candidate.limited[j]=std::abs(delta)>bound;
    }
    result=candidate;
    return true;
  }
  TorqueFault fault() const { return fault_; }
 private:
  bool fail(TorqueFault fault) { fault_=fault; return false; }
  TorqueSettings settings_{};
  TorqueFault fault_{TorqueFault::Configuration};
};

inline bool validate_torque_commissioning_plan(const Plan& plan, const Joints& origin) {
  std::array<Joints,4> nominal{}, brake{};
  if (!validate_task_plan(plan,origin,&nominal,&brake)) return false;
  constexpr std::array<double,4> limits{.002,.005,.02,.2};
  for (int k=0;k<4;++k) for (std::size_t j=0;j<7;++j)
    if (nominal[k][j]>limits[k]||brake[k][j]>limits[k]) return false;
  return plan[0].duration+plan[1].duration<=4.;
}
}  // namespace flash_fr3_model_controller
