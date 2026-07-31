# rover_control

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

ros2_control: controller_manager configuration and forward command controllers over topic_based_ros2_control.

## Purpose

Bridges controllers to Isaac Sim over /isaac_joint_commands and /isaac_joint_states (Phase 1-2). In Phase 3 only the hardware-interface plugin is swapped for the real one (CAN / micro-ROS / UDP — Phase 3 decision); controllers and above stay identical. This swap is the whole reason ros2_control is here.

Open verification (T0-R2): confirm mixed command interfaces (position: steer, velocity: drive) work in the pinned version; fallback = thin JointState bridge node, finding recorded here.

## Interfaces

**Subscribes:** controller command topics.
**Publishes:** `/isaac_joint_commands`.
**Consumes:** `/isaac_joint_states`.

**Controllers:** `steer_position_controller` (forward position group), `drive_velocity_controller` (forward velocity group) — open for alternatives per §4.3.

## How to run the test

`ros2 control list_controllers` shows both controllers active; publish a test command and observe joint motion in Isaac Sim.

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
