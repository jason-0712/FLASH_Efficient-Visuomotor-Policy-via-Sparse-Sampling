#pragma once
#include "flash_fr3_model_controller/bounded_types.hpp"

namespace flash_fr3_model_controller {
// Fixed-size, normalized-time power basis. Relative to native q_d at arming.
struct Polynomial {
  double duration{1.};
  std::array<Joints, 7> c{};
  Reference at(double elapsed) const {
    const double u = std::clamp(elapsed/duration, 0., 1.);
    Reference r;
    Joints* result[]{&r.q, &r.dq, &r.ddq, &r.jerk};
    for (int order = 0; order <= 3; ++order)
      for (std::size_t j = 0; j < 7; ++j) {
        double value = 0.;
        for (int k = 6; k >= order; --k) {
          double factor = 1.;
          for (int m = 0; m < order; ++m) factor *= k-m;
          value = value*u + factor*c[k][j];
        }
        (*result[order])[j] = value/std::pow(duration, order);
      }
    return r;
  }
};
inline double choose(int n, int k) {
  double v = 1.;
  for (int i = 1; i <= k; ++i) v *= static_cast<double>(n-i+1)/i;
  return v;
}
// Convex-hull bounds in Bernstein form, not sampled extrema.
inline Joints absolute_bound(const Polynomial& p, int order) {
  Joints result{};
  const int n = 6-order;
  for (std::size_t j = 0; j < 7; ++j)
    for (int i = 0; i <= n; ++i) {
      double b = 0.;
      for (int k = 0; k <= i; ++k) {
        double factor = 1.;
        for (int m = 1; m <= order; ++m) factor *= k+m;
        b += choose(i,k)/choose(n,k)*factor*p.c[k+order][j]/std::pow(p.duration,order);
      }
      result[j] = std::max(result[j], std::abs(b));
    }
  return result;
}
inline Polynomial stopping_polynomial(const Reference& start, double duration) {
  Polynomial p;
  p.duration = duration;
  for (std::size_t j = 0; j < 7; ++j) {
    p.c[0][j] = start.q[j];
    p.c[1][j] = duration*start.dq[j];
    p.c[2][j] = .5*duration*duration*start.ddq[j];
    p.c[3][j] = -p.c[1][j] - (4./3.)*p.c[2][j];
    p.c[4][j] = .5*p.c[1][j] + .5*p.c[2][j];
  }
  return p;
}
using Plan = std::array<Polynomial, 2>;
constexpr std::array<double, 4> kReferenceLimits{.01, .02, .1, 5.};

inline bool validate_plan(const Plan& plan, std::array<Joints,4>* nominal_out = nullptr,
                          std::array<Joints,4>* brake_out = nullptr) {
  std::array<Joints,4> bounds{}, brake_bounds{};
  double total = 0.;
  for (const auto& p : plan) {
    if (!std::isfinite(p.duration) || p.duration < .1 || p.duration > 5.) return false;
    for (const auto& c : p.c) if (!finite(c)) return false;
    total += p.duration;
    for (int order = 0; order <= 3; ++order) {
      const auto bound = absolute_bound(p,order);
      for (std::size_t j = 0; j < 7; ++j)
        bounds[order][j] = std::max(bounds[order][j],bound[j]);
    }
  }
  if (total > 8.) return false;
  const auto initial = plan[0].at(0.);
  const auto left = plan[0].at(plan[0].duration), right = plan[1].at(0.);
  const auto end = plan[1].at(plan[1].duration);
  for (std::size_t j = 0; j < 7; ++j)
    if (std::abs(initial.q[j]) > 1e-10 || std::abs(initial.dq[j]) > 1e-10 ||
        std::abs(initial.ddq[j]) > 1e-10 || std::abs(left.q[j]-right.q[j]) > 1e-9 ||
        std::abs(left.dq[j]-right.dq[j]) > 1e-9 || std::abs(left.ddq[j]-right.ddq[j]) > 1e-9 ||
        std::abs(end.dq[j]) > 1e-9 || std::abs(end.ddq[j]) > 1e-9) return false;
  // Bound any 0.25s watchdog brake from ANY reference in the nominal plan.
  // Its polynomial is linear in starting q,v,a. Unit-response bounds plus
  // triangle inequality include combinations that need not actually occur.
  Reference unit_v{}, unit_a{};
  unit_v.dq.fill(1.); unit_a.ddq.fill(1.);
  const auto response_v = stopping_polynomial(unit_v,kBrakeSeconds);
  const auto response_a = stopping_polynomial(unit_a,kBrakeSeconds);
  for (int order = 0; order <= 3; ++order) {
    const auto bv = absolute_bound(response_v,order), ba = absolute_bound(response_a,order);
    for (std::size_t j = 0; j < 7; ++j) {
      brake_bounds[order][j] = (order == 0 ? bounds[0][j] : 0.) +
                              bv[j]*bounds[1][j] + ba[j]*bounds[2][j];
      if (bounds[order][j] > kReferenceLimits[order]-1e-10 ||
          brake_bounds[order][j] > kReferenceLimits[order]-1e-10) return false;
    }
  }
  if (nominal_out) *nominal_out = bounds;
  if (brake_out) *brake_out = brake_bounds;
  return true;
}

class ModelCore {
 public:
  bool arm(const Joints& desired, const Joints& q, const Joints& dq, const Joints& expected,
           const Plan& plan, double robot_time, double steady_time) {
    phase_ = Phase::Error; reason_ = Reason::InitialState;
    if (!finite(desired) || !finite(q) || !finite(dq) || !finite(expected) ||
        !std::isfinite(robot_time) || !std::isfinite(steady_time) || !validate_plan(plan)) return false;
    std::array<Joints,4> bounds{}, brake_bounds{};
    validate_plan(plan,&bounds,&brake_bounds);
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
      if (q[j] < kLower[j] || q[j] > kUpper[j] || std::abs(q[j]-origin_[j]) > .02 ||
          std::abs(dq[j]) > .05) return fail(Reason::StateLimit);
      if (std::abs(q[j]-reference_.q[j]) > .01) return fail(Reason::TrackingError);
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
          brake_ = stopping_polynomial(reference_,kBrakeSeconds);
          started_brake_robot_ = robot_time;
          phase_ = Phase::Braking; reason_ = Reason::HeartbeatTimeout;
        }
      }
    }
    if (phase_ == Phase::Braking) {
      const double t = robot_time-started_brake_robot_;
      reference_ = brake_.at(t);
      if (t >= kBrakeSeconds) {
        reference_.dq = reference_.ddq = reference_.jerk = {};
        phase_ = Phase::LatchedHold;
      }
    }
    const Joints* values[]{&reference_.q,&reference_.dq,&reference_.ddq,&reference_.jerk};
    for (int order = 0; order <= 3; ++order)
      for (std::size_t j = 0; j < 7; ++j)
        if (!std::isfinite((*values[order])[j]) ||
            std::abs((*values[order])[j]-(order == 0 ? origin_[j] : 0.)) > kReferenceLimits[order])
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
