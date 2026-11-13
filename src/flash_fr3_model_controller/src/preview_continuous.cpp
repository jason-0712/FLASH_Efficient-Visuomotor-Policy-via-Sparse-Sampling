// Numerical replay only: stdin physical packets, stdout diagnostic summary.
#include <iomanip>
#include <iostream>
#include <sstream>
#include "flash_fr3_model_controller/continuous_reference.hpp"
using namespace flash_fr3_model_controller;
int main() {
  std::size_t count=0;double finish=0.;
  if(!(std::cin>>count>>finish)||count<1||count>512||!std::isfinite(finish)) return 2;
  RawTimeline timeline;double now=0.,first_ready=0.;std::size_t samples=0;
  std::ostringstream events;events<<std::setprecision(17);
  auto advance=[&](double target) {
    if(!std::isfinite(target)||target<now||target-first_ready>180.) return false;
    while(now+.001<target) {now+=.001;timeline.step(now);++samples;}
    now=target;timeline.step(now);++samples;return true;
  };
  for(std::size_t i=0;i<count;++i) {
    PhysicalPacket packet;double ready=0.;
    if(!(std::cin>>packet.sequence>>packet.observation>>packet.handoff>>ready)) return 2;
    for(auto& row:packet.curve.legendre) for(auto& value:row) if(!(std::cin>>value)) return 2;
    bool accepted=false;
    if(i==0) {
      first_ready=now=ready;accepted=timeline.initialize(packet,ready);
      if(!accepted) return 3;
    } else {
      if(!advance(ready)) return 2;
      accepted=timeline.stage(packet,ready);
    }
    if(i) events<<',';
    events<<"{\"sequence\":"<<packet.sequence<<",\"accepted\":"<<(accepted?"true":"false")
      <<",\"rejection\":"<<static_cast<int>(timeline.rejection())
      <<",\"position_gap_rad\":"<<timeline.position_gap()
      <<",\"velocity_gap_rad_s\":"<<timeline.velocity_gap()<<'}';
  }
  std::string extra;if(std::cin>>extra) return 2;
  if(!advance(finish)) return 2;
  std::cout<<std::setprecision(17)<<"{\"status\":\"RAW_CONTINUOUS_NUMERICAL_REPLAY_COMPLETE\",\"robot_motion_commanded\":false,"
    <<"\"sampling_dt_seconds\":0.001,\"samples\":"<<samples<<",\"handoffs\":"<<timeline.handoffs()
    <<",\"last_sequence\":"<<timeline.sequence()<<",\"phase\":"<<static_cast<int>(timeline.phase())
    <<",\"events\":["<<events.str()<<"]}\n";
}
