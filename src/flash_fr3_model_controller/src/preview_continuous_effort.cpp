// Offline admission/reference replay only. Prescribed measured q stays fixed;
// no hardware, ROS node, dynamics, actuator or wall-time performance model.
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include "flash_fr3_model_controller/continuous_effort_core.hpp"
using namespace flash_fr3_model_controller;
int main(int argc,char** argv) {
  const bool rollout=argc==2&&std::string(argv[1])=="--task-rollout";
  if(argc!=1&&!rollout) return 2;
  Joints measured{};
  for(auto& value:measured) if(!(std::cin>>value)) return 2;
  std::size_t count=0;double session=0.;
  if(!(std::cin>>count>>session)||count<1||count>512) return 2;
  struct Input {double ready;PhysicalPacket packet;};
  std::vector<Input> inputs;
  for(std::size_t i=0;i<count;++i) {
    Input in;std::vector<double> wire(57);
    if(!(std::cin>>in.ready)||!std::isfinite(in.ready)) return 2;
    for(auto& value:wire) if(!(std::cin>>value)) return 2;
    if(wire[0]==2.) {double origin;if(!(std::cin>>origin)) return 2;wire.push_back(origin);}
    if(!decode_effort_packet(wire.data(),wire.size(),in.packet)) return 2;
    if(!inputs.empty()&&in.ready<inputs.back().ready) return 2;
    inputs.push_back(in);
  }
  std::string extra;if(std::cin>>extra) return 2;
  const double start=inputs.front().ready;
  ContinuousEffortCore core;
  if(!core.arm(measured,Joints{},measured,0.,start,session,rollout)) return 3;
  double elapsed=0.;int samples=0;bool healthy=true;
  auto advance=[&](double target,bool finish=false) {
    if(!std::isfinite(target)||target<start+elapsed||target-start>180.) return false;
    // Every prescribed robot period is 1 ms. Event readiness is rounded up to
    // the next tick; actual replayed admission time is emitted in the report.
    const int ticks=static_cast<int>(std::ceil((target-start)*1000.-1e-7));
    while(samples<ticks&&healthy) {
      elapsed=(++samples)*.001;
      healthy=core.step(elapsed,start+elapsed,0.,measured,Joints{},finish);
    }
    return true;
  };
  std::ostringstream events;events<<std::setprecision(17);int accepted_count=0;
  for(std::size_t i=0;i<inputs.size();++i) {
    const auto& in=inputs[i];
    if(!advance(std::max(in.ready,start+elapsed))) return 2;
    PreparedEffortPacket prepared;const bool prepared_ok=prepare_effort_packet(in.packet,prepared,rollout);
    const bool accepted=core.admit(prepared,start+elapsed);accepted_count+=accepted;
    double reach=0.,v=0.,a=0.,jerk=0.,handoff_gap=0.;Reference right;
    if(prepared_ok) for(std::size_t j=0;j<7;++j) {
      reach=std::max(reach,std::max(std::abs(prepared.lower[j]-measured[j]),
                                  std::abs(prepared.upper[j]-measured[j]))+prepared.brake_drift[j]);
      v=std::max(v,std::max(prepared.nominal[1][j],prepared.brake[1][j]));
      a=std::max(a,std::max(prepared.nominal[2][j],prepared.brake[2][j]));
      jerk=std::max(jerk,std::max(prepared.nominal[3][j],prepared.brake[3][j]));
    }
    if(!in.packet.curve.at(in.packet.handoff-in.packet.curve_origin(),right)) return 2;
    for(std::size_t j=0;j<7;++j) handoff_gap=std::max(handoff_gap,std::abs(right.q[j]-measured[j]));
    if(i) events<<',';
    events<<"{\"sequence\":"<<in.packet.sequence<<",\"prepared\":"<<(prepared_ok?"true":"false")
      <<",\"admission_monotonic\":"<<start+elapsed<<",\"accepted\":"<<(accepted?"true":"false")
      <<",\"rejection\":"<<static_cast<int>(core.rejection())
      <<",\"raw_handoff_gap_to_prescribed_q_rad\":"<<handoff_gap
      <<",\"conservative_bounds_with_braking\":["<<reach<<','<<v<<','<<a<<','<<jerk<<"]}";
  }
  const double finish_at=std::min(start+session-kContinuousBrake,
                                  inputs.back().packet.handoff+4./15.);
  if(!advance(std::max(start+elapsed,finish_at))) return 2;
  if(!advance(std::max(start+elapsed,start+session+.1),true)) return 2;
  std::cout<<std::setprecision(17)<<"{\"status\":\"CONTINUOUS_EFFORT_OFFLINE_ADMISSION_COMPLETE\","
    <<"\"robot_motion_commanded\":false,\"gripper_motion_commanded\":false,"
    <<"\"prescribed_measured_state_fixed\":true,\"hardware_ready\":false,"
    <<"\"samples\":"<<samples<<",\"accepted_packets\":"<<accepted_count
    <<",\"handoffs\":"<<core.handoffs()<<",\"final_phase\":"<<static_cast<int>(core.phase())
    <<",\"final_reason\":"<<static_cast<int>(core.reason())
    <<",\"micro_limits_q_v_a_jerk\":[0.002,0.005,0.02,0.2],\"events\":["<<events.str()<<"]}\n";
}
