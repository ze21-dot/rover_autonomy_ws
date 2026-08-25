# ADR-0002: Kinematics package design (T0-R3) — custom C++ double-Ackermann

**Status:** accepted (autonomy + mechanical leads)
**Date:** 2026-08-03
**Refs:** SUR-AUT-SPEC-001 §4.2, §4.3, T0-R3, TK-3; Jira SURA-23

## Context

`geometry_msgs/Twist` (vx, ωz) must be translated into 4 independent steering angles + 4 wheel
velocities (double Ackermann), and the same package must later provide the forward direction
(joint states → body twist → wheel odometry, TK-3) as a candidate Phase 2 EKF input. Nav2's
`min_turning_radius` must match the geometry this package actually realizes (T1a-1 consistency).

Options considered (full analysis in the proposal draft, summarized):
**A** custom C++ package (ROS-free library + thin node) · **B** adapt `steering_controllers_library`
(single-axle assumption to be worked around) · **C** chainable custom controller inside ros2_control.

## Decision

Implement a custom C++ double-Ackermann kinematics package. The package will contain a
ROS-independent, unit-tested kinematics library and a thin ROS 2 node converting
`geometry_msgs/Twist` commands into four steering-position and four wheel-velocity commands.
The same library will later implement forward kinematics and wheel odometry.
`min_turning_radius` will be derived from the rover geometry and steering limits and shared
with the Nav2 configuration. Adaptation of `steering_controllers_library` is rejected for the
initial implementation because its core model assumes single-axle steering and would introduce
additional integration and testing risk.

Rationale highlights: exact fit to our four-independently-steered geometry; cleanest path to the
spec's hand-computed unit-test criterion (AC0-3, `colcon test`); single source of truth for
`min_turning_radius` shared with Smac Hybrid-A*; hardware-interface swap in Phase 3 leaves IK
and everything above unchanged (the spec's central architectural goal).

## Consequences

- `rover_kinematics` split as: header-only geometry library (gtest, no ROS) + thin node
- Point-turn (|vx|≈0, ωz≠0) rejected by design per §4.2 — IK returns failure, never invents output
- `min_turning_radius()` exposed by the library; Nav2 config references the derived value
- TK-3 forward kinematics lives in the same library (least-squares over 4 wheels)
- B re-evaluated only if a future real-time/upstream-controller need arises (would supersede this ADR)
