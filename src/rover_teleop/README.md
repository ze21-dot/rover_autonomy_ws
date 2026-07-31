# rover_teleop

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Keyboard/gamepad teleoperation publishing /cmd_vel.

## Purpose

Phase 0 teleoperated baseline (T0-R4): human → /cmd_vel → IK → simulated rover. Exit demo: figure-eight with visibly consistent double-Ackermann geometry, no wheel scrubbing (AC0-1).

## Interfaces

**Publishes:** `/cmd_vel` (geometry_msgs/Twist).

**Params:** speed/steer scaling, deadman button, key bindings in `config/`.

## How to run the test

`ros2 launch rover_teleop teleop.launch.py`; drive figure-eight in flat scene, record screen (AC0-4).

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
