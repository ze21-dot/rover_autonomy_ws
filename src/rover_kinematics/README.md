# rover_kinematics

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Double-Ackermann inverse kinematics (Twist -> per-wheel steer angle + drive speed) and forward direction (wheel odometry).

## Purpose

T0-R3 design decision is OPEN and must be recorded here before implementation: custom C++ vs adapted steering_controllers_library (single-axle assumption needs workaround) vs cascaded structure. min_turning_radius = R_front_only / 2, derived from URDF wheelbase/track + steer limits — derivation documented in this README when computed.

TK-3: forward direction (/joint_states → body twist → integrated Odometry). Not consumed in Phase 1 loop; validated against GT (drift % over 50 m, AC1-4) as candidate Phase 2 EKF input.

## Interfaces

**Subscribes:** `/cmd_vel` (geometry_msgs/Twist), `/joint_states`.
**Publishes:** per-wheel steer/drive commands (to rover_control), `/wheel_odom` (nav_msgs/Odometry, TK-3).

## How to run the test

`colcon test --packages-select rover_kinematics` — unit tests against hand-computed cases: straight line, pure arc left/right, zero-speed steering (AC0-3). Results committed.

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
