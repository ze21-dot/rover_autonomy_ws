# rover_description

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Robot model: URDF/xacro, meshes, robot_state_publisher launch.

## Purpose

Single source of truth for rover geometry. Wheelbase/track/steer limits here drive min_turning_radius derivation (§4.2) — derived, never copied. base_footprint is a child of base_link per REP-120 (fixed -z ≈ wheel radius on flat terrain; slope handling is TF-4 research).

## Interfaces

**Publishes:** `/robot_description`, static TFs for URDF frames via `robot_state_publisher`.

**Frames:** `base_link` (Isaac articulation root) → `base_footprint`, `camera_link`, wheel/steer frames (REP-103/105/120).

## How to run the test

`ros2 launch rover_description rsp.launch.py`; check TF tree in RViz, verify frame axes follow REP-103 (x-forward/y-left/z-up).

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
