#pragma once
#include "flash_fr3_model_controller/torque_core.hpp"
namespace flash_fr3_model_controller {
// Active FR3 URDF limits; manufacturer robot_specifications.html, not demo extrema.
constexpr Joints kRolloutLower{-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,.4398,-3.0508};
constexpr Joints kRolloutUpper{2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508};
// FR3 manufacturer torque-amplitude recommendation and libfranka's torque-rate
// ceiling (1000 Nm/s minus its published 1e-3 epsilon). The hardware driver
// also applies franka::limitRate against the last robot-desired torque.
constexpr Joints kRolloutTorqueMagnitude{87.,87.,87.,87.,12.,12.,12.};
constexpr double kRolloutTorqueRate=1000.-1e-3;
inline bool valid_rollout_torque_settings(const TorqueSettings& settings) {
  if(!finite(settings.kp)||!finite(settings.kd)||!finite(settings.magnitude)||!finite(settings.rate)) return false;
  for(std::size_t j=0;j<7;++j)
    if(settings.kp[j]<1.||settings.kp[j]>300.||settings.kd[j]<1.||settings.kd[j]>60.||
       settings.magnitude[j]<.5||settings.magnitude[j]>kRolloutTorqueMagnitude[j]||
       settings.rate[j]<1.||settings.rate[j]>kRolloutTorqueRate) return false;
  return true;
}
// Same PD + analytic desired velocity + Coriolis law. Separate type keeps the
// installed commissioning plugin ABI and its small-motion settings untouched.
// Numerical gains are local FR3 settings, not parameters published by FLASH.
class RolloutTorqueLaw {
 public:
  bool configure(const TorqueSettings& settings,bool rollout) {
    settings_=settings;rollout_=rollout;
    fault_=(rollout?valid_rollout_torque_settings(settings):settings.valid())?
        TorqueFault::None:TorqueFault::Configuration;
    return rollout?fault_==TorqueFault::None:commissioning_.configure(settings);
  }
  bool evaluate(const Reference& ref,const Joints& q,const Joints& dq,const Joints& coriolis,
                const Joints& previous,double period,TorqueOutput& result) {
    if(!rollout_) return commissioning_.evaluate(ref,q,dq,coriolis,previous,period,result);
    if(fault_!=TorqueFault::None) return false;
    if(!finite(ref.q)||!finite(ref.dq)||!finite(q)||!finite(dq)||!finite(coriolis)||!finite(previous))
      return fail(TorqueFault::Nonfinite);
    if(!std::isfinite(period)||period<.0005||period>.002) return fail(TorqueFault::Period);
    TorqueOutput candidate;
    for(std::size_t j=0;j<7;++j) {
      if(q[j]<kRolloutLower[j]||q[j]>kRolloutUpper[j]||ref.q[j]<kRolloutLower[j]||ref.q[j]>kRolloutUpper[j]||std::abs(dq[j])>1.||
         std::abs(ref.dq[j])>.8)
        return fail(TorqueFault::ReferenceOrState);
      candidate.raw[j]=settings_.kp[j]*(ref.q[j]-q[j])+settings_.kd[j]*(ref.dq[j]-dq[j])+coriolis[j];
      if(!std::isfinite(candidate.raw[j])||std::abs(candidate.raw[j])>=settings_.magnitude[j])
        return fail(TorqueFault::Magnitude);
      if(std::abs(previous[j])>=settings_.magnitude[j]) return fail(TorqueFault::PreviousTorque);
      const auto change=candidate.raw[j]-previous[j],limit=settings_.rate[j]*period;
      candidate.command[j]=previous[j]+std::clamp(change,-limit,limit);
      candidate.limited[j]=std::abs(change)>limit;
    }
    result=candidate;return true;
  }
  TorqueFault fault()const{return rollout_?fault_:commissioning_.fault();}
 private:
  bool fail(TorqueFault f){fault_=f;return false;}
  bool rollout_{false};TorqueSettings settings_{};TorqueLaw commissioning_;
  TorqueFault fault_{TorqueFault::Configuration};
};
}  // namespace flash_fr3_model_controller
