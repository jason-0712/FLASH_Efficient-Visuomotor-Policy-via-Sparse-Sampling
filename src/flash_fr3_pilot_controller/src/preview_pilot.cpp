// Numerical reference preview only: no ROS, libfranka or robot connection.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "flash_fr3_pilot_controller/pilot_core.hpp"
using namespace flash_fr3_pilot_controller;
int main(int argc, char** argv) {
  if (argc != 10) { std::cerr << "usage: preview_pilot CSV q1 q2 q3 q4 q5 q6 q7 delta\n"; return 2; }
  try {
    Joints q{}, zero{};
    for (std::size_t j = 0; j < 7; ++j) q[j] = std::stod(argv[j+2]);
    const double delta = std::stod(argv[9]);
    PilotCore c;
    if (!c.arm(q,q,zero,q,delta,0.,0.)) throw std::runtime_error("invalid initial state");
    std::ofstream csv(argv[1]);
    if (!csv) throw std::runtime_error("cannot create CSV");
    csv << std::setprecision(16) << "elapsed_s";
    for (const char* key : {"q", "dq", "ddq", "jerk"})
      for (int j = 1; j <= 7; ++j) csv << ',' << key << j;
    csv << '\n';
    double excursion=0., speed=0., accel=0., jerk=0.;
    for (int i = 0; i <= 6500; ++i) {
      const double t = i*.001;
      if (!c.step(t,t,0.,q,zero)) throw std::runtime_error("reference rejected");
      const auto& r = c.reference();
      csv << t;
      for (const auto* v : {&r.q,&r.dq,&r.ddq,&r.jerk}) for (double x : *v) csv << ',' << x;
      csv << '\n';
      for (std::size_t j = 0; j < 7; ++j) {
        excursion=std::max(excursion,std::abs(r.q[j]-q[j])); speed=std::max(speed,std::abs(r.dq[j]));
        accel=std::max(accel,std::abs(r.ddq[j])); jerk=std::max(jerk,std::abs(r.jerk[j]));
      }
    }
    if (c.phase() != Phase::Complete) throw std::runtime_error("incomplete trajectory");
    std::cout << std::setprecision(16)
      << "{\"status\":\"NUMERICAL_PREVIEW_PASS\",\"samples\":6501,\"duration_seconds\":6,"
      << "\"excursion_max_rad\":" << excursion << ",\"velocity_max_rad_s\":" << speed
      << ",\"acceleration_max_rad_s2\":" << accel << ",\"jerk_max_rad_s3\":" << jerk
      << ",\"robot_motion_commanded\":false,\"dynamics_simulated\":false}\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
