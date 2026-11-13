#include <fstream>
#include <iomanip>
#include <iostream>
#include "flash_fr3_model_controller/task_core.hpp"
using namespace flash_fr3_model_controller;
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  Plan plan;
  Joints origin;
  for (auto& p : plan) input >> p.duration;
  for (auto& p : plan) for (auto& c : p.c) for (auto& value : c) input >> value;
  for (auto& value : origin) input >> value;
  double extra;
  if (!input || input >> extra || !finite(origin)) return 2;
  std::array<Joints,4> bounds{}, brake{};
  if (!validate_task_plan(plan,origin,&bounds,&brake)) {
    std::cerr << "Model plan rejected: C2, finite values, nominal or watchdog-brake bounds.\n";
    return 3;
  }
  std::ofstream output(argv[2]);
  output << std::setprecision(17) << "time_s";
  for (const auto* key : {"q","dq","ddq","jerk"})
    for (int j=1; j<=7; ++j) output << ',' << key << j;
  output << '\n';
  double offset = 0.;
  for (const auto& p : plan) {
    const int count = static_cast<int>(std::ceil(p.duration/.001));
    for (int i=0; i<=count; ++i) {
      const double t = p.duration*i/count;
      auto ref = p.at(t);
      for (std::size_t j=0; j<7; ++j) ref.q[j] += origin[j];
      output << offset+t;
      for (const auto& value : {ref.q,ref.dq,ref.ddq,ref.jerk})
        for (double v : value) output << ',' << v;
      output << '\n';
    }
    offset += p.duration;
  }
  std::cout << std::setprecision(17) << "{\"status\":\"TASK_SEGMENT_PLAN_PASS\",\"bounds_method\":\"Bernstein convex hull; watchdog brake includes all nominal reference states\",\"nominal_bounds\":[";
  for (int order=0; order<4; ++order) {
    if (order) std::cout << ',';
    std::cout << '[';
    for (std::size_t j=0; j<7; ++j) { if (j) std::cout << ','; std::cout << bounds[order][j]; }
    std::cout << ']';
  }
  std::cout << "],\"watchdog_brake_bounds\":[";
  for (int order=0; order<4; ++order) {
    if (order) std::cout << ',';
    std::cout << '[';
    for (std::size_t j=0; j<7; ++j) { if (j) std::cout << ','; std::cout << brake[order][j]; }
    std::cout << ']';
  }
  std::cout << "],\"robot_motion_commanded\":false}\n";
  return output ? 0 : 4;
}
