#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace flash_fr3_model_controller {
using Joints = std::array<double, 7>;
struct Reference { Joints q{}, dq{}, ddq{}, jerk{}; };
enum class Phase { Waiting = 0, Moving = 1, Braking = 2, LatchedHold = 3, Complete = 4, Error = 5 };
enum class Reason { None = 0, HeartbeatTimeout = 1, StartTimeout = 2, ClockOrDeadline = 3,
                    StateLimit = 4, TrackingError = 5, ReferenceLimit = 6, InitialState = 7 };
constexpr Joints kLower{-2.3476, -1.5454, -2.4937, -2.7714, -2.51, .7773, -2.7045};
constexpr Joints kUpper{2.3476, 1.5454, 2.4937, -.4226, 2.51, 4.2841, 2.7045};
constexpr double kLegSeconds = 3.0;
constexpr double kBrakeSeconds = .25;
constexpr double kHeartbeatSeconds = .20;

inline bool valid_heartbeat(int64_t source, int64_t now, int64_t activation,
                            int64_t previous, bool matching_plan) {
  return matching_plan && activation > 0 && source >= activation && source > previous &&
         now >= source && now - source <= 100000000;
}

inline bool finite(const Joints& a) {
  for (double v : a) if (!std::isfinite(v)) return false;
  return true;
}

// General quintic Hermite segment, in normalized u=t/T; derivatives are SI.
class Segment {
 public:
  void set(const Reference& start, const Joints& end, double seconds) {
    duration_ = seconds;
    for (std::size_t j = 0; j < 7; ++j) {
      const double c0 = start.q[j], c1 = start.dq[j] * seconds;
      const double c2 = .5 * start.ddq[j] * seconds * seconds;
      const double d = end[j] - c0;
      coefficients_[j] = {c0, c1, c2, 10*d - 6*c1 - 3*c2,
                          -15*d + 8*c1 + 3*c2, 6*d - 3*c1 - c2};
    }
  }
  Reference at(double elapsed) const {
    const double u = std::clamp(elapsed / duration_, 0., 1.);
    Reference r;
    for (std::size_t j = 0; j < 7; ++j) {
      const auto& c = coefficients_[j];
      r.q[j] = c[0] + u*(c[1] + u*(c[2] + u*(c[3] + u*(c[4] + u*c[5]))));
      r.dq[j] = (c[1] + u*(2*c[2] + u*(3*c[3] + u*(4*c[4] + u*5*c[5])))) / duration_;
      r.ddq[j] = (2*c[2] + u*(6*c[3] + u*(12*c[4] + u*20*c[5]))) / (duration_*duration_);
      r.jerk[j] = (6*c[3] + u*(24*c[4] + u*60*c[5])) / std::pow(duration_, 3);
    }
    return r;
  }
 private:
  double duration_{1.};
  std::array<std::array<double, 6>, 7> coefficients_{};
};

}  // namespace flash_fr3_model_controller
