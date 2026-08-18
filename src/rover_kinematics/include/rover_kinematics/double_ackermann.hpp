// rover_kinematics: double-Ackermann inverse/forward kinematics (pure geometry, ROS-free)
// SUR-AUT-SPEC-001 T0-R3 / TK-3 — design per ADR-0002 (option A).
//
// Conventions (REP-103): x forward, y left, z up. Body twist = (vx [m/s], wz [rad/s]).
// Wheel order: FL, FR, RL, RR. Steer angle: + = wheel nose points left.
// Point-turn (vx≈0, wz≠0) is NOT supported by design (§4.2); ik() reports failure.

#pragma once
#include <array>
#include <cmath>

namespace rover_kinematics {

struct Config {
  double wheelbase;     // L: front-to-rear axle distance [m]
  double track;         // W: left-to-right wheel distance [m]
  double wheel_radius;  // r [m]
  double max_steer;     // |steer| limit [rad]
};

struct WheelCommands {
  std::array<double, 4> steer_rad;   // FL FR RL RR
  std::array<double, 4> omega_rads;  // wheel angular velocity, signed (+ = forward roll)
};

struct BodyTwist {
  double vx;  // [m/s]
  double wz;  // [rad/s]
};

class DoubleAckermann {
 public:
  explicit DoubleAckermann(const Config& c) : cfg_(c) {}

  // Minimum achievable turning radius (to vehicle center), from geometry + steer limit.
  // R_min = L/2 / tan(max_steer) + W/2  (inner front wheel saturates first)
  double min_turning_radius() const {
    return 0.5 * cfg_.wheelbase / std::tan(cfg_.max_steer) + 0.5 * cfg_.track;
  }

  // Twist -> wheel commands. Returns false (and zeroes out) for unachievable requests:
  // point turn (|vx|<eps, |wz|>eps) or radius below min_turning_radius.
  bool ik(const BodyTwist& t, WheelCommands& out) const {
    constexpr double eps = 1e-9;
    out = WheelCommands{{0, 0, 0, 0}, {0, 0, 0, 0}};
    const double L2 = 0.5 * cfg_.wheelbase, W2 = 0.5 * cfg_.track;

    if (std::abs(t.wz) < eps) {                 // straight (or stopped)
      const double w = t.vx / cfg_.wheel_radius;
      out.omega_rads = {w, w, w, w};
      return true;
    }
    if (std::abs(t.vx) < eps) return false;     // point turn: unsupported by design

    const double R = t.vx / t.wz;               // signed radius of vehicle center (+ = ICR left)
    if (std::abs(R) < min_turning_radius() - 1e-9) return false;

    // Wheel positions (x_i, y_i); ICR at (0, R).
    const std::array<double, 4> xs{ L2,  L2, -L2, -L2};
    const std::array<double, 4> ys{ W2, -W2,  W2, -W2};
    const double sgn_v = (t.vx >= 0.0) ? 1.0 : -1.0;

    for (int i = 0; i < 4; ++i) {
      const double dy = R - ys[i];  // never 0: |R| >= min radius > W/2
      out.steer_rad[i] = std::atan(xs[i] / dy);
      const double dist = std::hypot(xs[i], dy);
      out.omega_rads[i] = sgn_v * std::abs(t.wz) * dist / cfg_.wheel_radius;
    }
    return true;
  }

  // Joint states -> body twist (least squares over 4 wheels). TK-3 forward direction.
  // Rigid body: v_i = (vx - wz*y_i, wz*x_i); wheel velocity vector = omega*r*(cos d, sin d).
  BodyTwist fk(const WheelCommands& w) const {
    const double L2 = 0.5 * cfg_.wheelbase, W2 = 0.5 * cfg_.track;
    const std::array<double, 4> xs{ L2,  L2, -L2, -L2};
    const std::array<double, 4> ys{ W2, -W2,  W2, -W2};
    double num = 0, den = 0;
    std::array<double, 4> vix{}, viy{};
    for (int i = 0; i < 4; ++i) {
      const double v = w.omega_rads[i] * cfg_.wheel_radius;
      vix[i] = v * std::cos(w.steer_rad[i]);
      viy[i] = v * std::sin(w.steer_rad[i]);
      num += viy[i] * xs[i];
      den += xs[i] * xs[i];
    }
    BodyTwist t{};
    t.wz = (den > 0) ? num / den : 0.0;
    double sum = 0;
    for (int i = 0; i < 4; ++i) sum += vix[i] + t.wz * ys[i];
    t.vx = 0.25 * sum;
    return t;
  }

  const Config& config() const { return cfg_; }

 private:
  Config cfg_;
};

}  // namespace rover_kinematics
