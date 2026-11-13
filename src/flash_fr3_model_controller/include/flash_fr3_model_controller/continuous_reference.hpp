#pragma once
#include <limits>
#include "flash_fr3_model_controller/bounded_types.hpp"

namespace flash_fr3_model_controller {
// Raw coefficient/time consumer for numerical replay. No command interfaces,
// torque law, collision model or automatic correction. Not an actuator gate.
struct PhysicalCurve {
  std::array<Joints,7> legendre{};
  double anchor{2./11.},duration{44./15.},horizon{32./15.};
  bool valid() const {
    if(!std::isfinite(anchor)||!std::isfinite(duration)||!std::isfinite(horizon)||
       std::abs(anchor-2./11.)>1e-7||std::abs(duration-44./15.)>1e-7||
       std::abs(horizon-32./15.)>1e-7) return false;
    for(const auto& row:legendre) if(!finite(row)) return false;
    return true;
  }
  bool at(double seconds,Reference& result) const {
    if(!valid()||!std::isfinite(seconds)||seconds<0.||seconds>horizon+1e-8) return false;
    const double x=2.*(anchor+seconds/duration)-1.;
    // Recurrences for P_n and its first three analytic derivatives.
    std::array<std::array<double,7>,4> p{};p[0][0]=1.;p[0][1]=x;p[1][1]=1.;
    for(int n=2;n<=6;++n) for(int order=0;order<=3;++order)
      p[order][n]=((2*n-1)*(x*p[order][n-1]+(order?order*p[order-1][n-1]:0.))-
                    (n-1)*p[order][n-2])/n;
    Reference candidate{};Joints* fields[]{&candidate.q,&candidate.dq,&candidate.ddq,&candidate.jerk};
    for(int order=0;order<=3;++order) for(std::size_t j=0;j<7;++j) {
      for(int n=0;n<=6;++n) (*fields[order])[j]+=legendre[n][j]*p[order][n];
      (*fields[order])[j]*=std::pow(2./duration,order);
      if(!std::isfinite((*fields[order])[j])) return false;
    }
    result=candidate;return true;
  }
};
struct PhysicalPacket {
  int sequence{0};double observation{},handoff{};PhysicalCurve curve{};
  // Separate real sensor time from the first curve's execution start. NaN is
  // the legacy v1 convention: evaluate relative to the observation timestamp.
  double execution_origin{std::numeric_limits<double>::quiet_NaN()};
  double curve_origin()const{return std::isnan(execution_origin)?observation:execution_origin;}
};
enum class TimelineRejection {None=0,Sequence=1,Invalid=2,Late=3,Overlap=4,Position=5,Velocity=6,Unavailable=7};
enum class TimelinePhase {Waiting=0,Running=1,Expired=2,Error=3};

class RawTimeline {
 public:
  bool initialize(const PhysicalPacket& packet,double ready) {
    if(initialized_||packet.sequence!=0||!packet_valid(packet,ready)) return false;
    active_=packet;previous_=ready;initialized_=true;phase_=TimelinePhase::Waiting;return true;
  }
  bool stage(const PhysicalPacket& packet,double ready) {
    position_gap_=velocity_gap_=0.;
    if(!initialized_||phase_==TimelinePhase::Expired||phase_==TimelinePhase::Error)
      return reject(TimelineRejection::Unavailable);
    if(pending_||packet.sequence<=active_.sequence||packet.sequence<=last_candidate_)
      return reject(TimelineRejection::Sequence);
    last_candidate_=packet.sequence;
    if(!packet_valid(packet,ready)) return reject(TimelineRejection::Invalid);
    if(ready<previous_||packet.handoff-ready<.02) return reject(TimelineRejection::Late);
    if(packet.observation<=active_.observation||packet.handoff<=active_.handoff)
      return reject(TimelineRejection::Sequence);
    Reference left{},right{};
    if(!active_.curve.at(packet.handoff-active_.observation,left)||
       !packet.curve.at(packet.handoff-packet.observation,right)) return reject(TimelineRejection::Overlap);
    for(std::size_t j=0;j<7;++j) {
      position_gap_=std::max(position_gap_,std::abs(left.q[j]-right.q[j]));
      velocity_gap_=std::max(velocity_gap_,std::abs(left.dq[j]-right.dq[j]));
    }
    // Diagnostic C1 tolerance only. It is not a hardware safety approval.
    if(position_gap_>.002) return reject(TimelineRejection::Position);
    if(velocity_gap_>.005) return reject(TimelineRejection::Velocity);
    next_=packet;pending_=true;rejection_=TimelineRejection::None;return true;
  }
  bool step(double now) {
    if(!initialized_||phase_==TimelinePhase::Expired||phase_==TimelinePhase::Error) return false;
    if(!std::isfinite(now)||now<previous_) {phase_=TimelinePhase::Error;return false;}
    previous_=now;
    if(pending_&&now>=next_.handoff) {active_=next_;pending_=false;++handoffs_;}
    if(now<active_.handoff) {phase_=TimelinePhase::Waiting;return true;}
    if(!active_.curve.at(now-active_.observation,reference_)) {phase_=TimelinePhase::Expired;return false;}
    phase_=TimelinePhase::Running;return true;
  }
  const Reference& reference()const{return reference_;}
  TimelinePhase phase()const{return phase_;}
  TimelineRejection rejection()const{return rejection_;}
  int sequence()const{return active_.sequence;}
  int handoffs()const{return handoffs_;}
  double position_gap()const{return position_gap_;}
  double velocity_gap()const{return velocity_gap_;}
 private:
  bool packet_valid(const PhysicalPacket& p,double ready)const {
    return p.sequence>=0&&p.sequence<=1000000&&std::isfinite(ready)&&ready>=0.&&
      std::isfinite(p.observation)&&p.observation>=0.&&p.observation<=ready&&
      std::isfinite(p.handoff)&&p.handoff>=ready+.02&&p.handoff-p.observation<=p.curve.horizon&&p.curve.valid();
  }
  bool reject(TimelineRejection reason){rejection_=reason;return false;}
  PhysicalPacket active_{},next_{};Reference reference_{};
  bool initialized_{false},pending_{false};int last_candidate_{0},handoffs_{0};
  double previous_{0.},position_gap_{0.},velocity_gap_{0.};
  TimelinePhase phase_{TimelinePhase::Waiting};TimelineRejection rejection_{TimelineRejection::None};
};
}  // namespace flash_fr3_model_controller
