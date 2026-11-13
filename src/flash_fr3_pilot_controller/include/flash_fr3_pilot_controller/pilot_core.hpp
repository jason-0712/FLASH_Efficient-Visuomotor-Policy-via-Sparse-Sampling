#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace flash_fr3_pilot_controller {
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

// No ROS or IO. The owner calls step from the native ros2_control update loop.
// Robot clock drives the trajectory; steady clock drives heartbeat/deadlines.
class PilotCore {
 public:
  bool arm(const Joints& desired_q, const Joints& measured_q, const Joints& measured_dq,
           const Joints& expected_q, double delta, double robot_time, double steady_time) {
    phase_ = Phase::Error;
    reason_ = Reason::InitialState;
    if (!finite(desired_q) || !finite(measured_q) || !finite(measured_dq) || !finite(expected_q) ||
        !std::isfinite(delta) || std::abs(delta) > .005 || std::abs(delta) < 1e-6 ||
        !std::isfinite(robot_time) || !std::isfinite(steady_time)) return false;
    for (std::size_t j = 0; j < 7; ++j) {
      if (std::abs(measured_dq[j]) > .01 || std::abs(measured_q[j] - expected_q[j]) > .002 ||
          std::abs(desired_q[j] - measured_q[j]) > .002 || desired_q[j] < kLower[j] + .02 ||
          desired_q[j] > kUpper[j] - .02) return false;
    }
    origin_ = desired_q;
    reference_ = {};
    reference_.q = desired_q;
    Joints end = origin_;
    end[1] += delta;
    out_.set(reference_, end, kLegSeconds);
    Reference halfway;
    halfway.q = end;
    back_.set(halfway, origin_, kLegSeconds);
    previous_robot_ = robot_time;
    previous_steady_ = activated_steady_ = last_progress_steady_ = steady_time;
    phase_ = Phase::Waiting;
    reason_ = Reason::None;
    return true;
  }

  bool step(double robot_time, double steady_time, double heartbeat_age,
            const Joints& measured_q, const Joints& measured_dq) {
    if (phase_ == Phase::Error) return false;
    const double dr = robot_time - previous_robot_, ds = steady_time - previous_steady_;
    if (!std::isfinite(robot_time) || !std::isfinite(steady_time) || !std::isfinite(heartbeat_age) ||
        dr < 0 || ds < 0 || dr > .005 || ds > .010)
      return fail(Reason::ClockOrDeadline);
    if (dr > 0) last_progress_steady_ = steady_time;
    if (steady_time - last_progress_steady_ > .005) return fail(Reason::ClockOrDeadline);
    previous_robot_ = robot_time;
    previous_steady_ = steady_time;
    if (!finite(measured_q) || !finite(measured_dq)) return fail(Reason::StateLimit);
    for (std::size_t j = 0; j < 7; ++j) {
      if (measured_q[j] < kLower[j] || measured_q[j] > kUpper[j] ||
          std::abs(measured_q[j] - origin_[j]) > .02 || std::abs(measured_dq[j]) > .05)
        return fail(Reason::StateLimit);
      if (std::abs(measured_q[j] - reference_.q[j]) > .01) return fail(Reason::TrackingError);
    }
    if (phase_ == Phase::Waiting) {
      if (steady_time - activated_steady_ > 2.) {
        phase_ = Phase::LatchedHold;
        reason_ = Reason::StartTimeout;
      } else if (heartbeat_age >= 0 && heartbeat_age <= kHeartbeatSeconds) {
        started_robot_ = robot_time;
        phase_ = Phase::Moving;
      }
    }
    if (phase_ == Phase::Moving) {
      const double t = robot_time - started_robot_;
      if (t >= 2*kLegSeconds) {
        reference_ = {};
        reference_.q = origin_;
        phase_ = Phase::Complete;
      } else {
        reference_ = t <= kLegSeconds ? out_.at(t) : back_.at(t - kLegSeconds);
        if (heartbeat_age < 0 || heartbeat_age > kHeartbeatSeconds) {
          Joints stop_q;
          for (std::size_t j = 0; j < 7; ++j)
            stop_q[j] = reference_.q[j] + .5*kBrakeSeconds*reference_.dq[j] +
                         kBrakeSeconds*kBrakeSeconds*reference_.ddq[j]/12.;
          brake_.set(reference_, stop_q, kBrakeSeconds);
          started_brake_robot_ = robot_time;
          phase_ = Phase::Braking;
          reason_ = Reason::HeartbeatTimeout;
        }
      }
    }
    if (phase_ == Phase::Braking) {
      const double t = robot_time - started_brake_robot_;
      reference_ = brake_.at(t);
      if (t >= kBrakeSeconds) {
        reference_.dq = reference_.ddq = reference_.jerk = {};
        phase_ = Phase::LatchedHold;
      }
    }
    for (std::size_t j = 0; j < 7; ++j) {
      if (!std::isfinite(reference_.q[j]) || !std::isfinite(reference_.dq[j]) ||
          !std::isfinite(reference_.ddq[j]) || !std::isfinite(reference_.jerk[j]) ||
          std::abs(reference_.q[j] - origin_[j]) > .01 ||
          std::abs(reference_.dq[j]) > .02 || std::abs(reference_.ddq[j]) > .1 ||
          std::abs(reference_.jerk[j]) > 5.) return fail(Reason::ReferenceLimit);
    }
    return true;
  }
  const Reference& reference() const { return reference_; }
  Phase phase() const { return phase_; }
  Reason reason() const { return reason_; }
 private:
  bool fail(Reason r) { phase_ = Phase::Error; reason_ = r; return false; }
  Joints origin_{};
  Reference reference_{};
  Segment out_, back_, brake_;
  Phase phase_{Phase::Error};
  Reason reason_{Reason::InitialState};
  double previous_robot_{0.}, previous_steady_{0.}, activated_steady_{0.};
  double started_robot_{0.}, started_brake_robot_{0.};
  double last_progress_steady_{0.};
};
}  // namespace flash_fr3_pilot_controller
