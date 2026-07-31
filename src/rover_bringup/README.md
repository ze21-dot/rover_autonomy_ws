# rover_bringup

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Top-level and simulation bringup: aggregated launch files and parameters for the full pipeline.

## Purpose

Single entry point for the whole stack. One command brings up Isaac Sim side interfaces + ROS 2 side (T0-S8). Enforces TF-3 (exactly one publisher per transform) via mutually exclusive launch arguments, and verifies use_sim_time on every node (CNV-3).

## Interfaces

**Launch files (planned):** `sim_bringup.launch.py` (Phase 0 teleop baseline), `nav_bringup.launch.py` (Phase 1a/1b, `costmap_source:=static|nvblox` mutually exclusive), `localization:=gt|ekf` (Phase 1 vs 2, mutually exclusive per TF-3).

**Params:** aggregated per-phase YAML in `config/` (CNV-5 — no container-only edits, ever).

## How to run the test

`ros2 launch rover_bringup sim_bringup.launch.py` with Isaac Sim scene running; verify clean TF tree at 30+ Hz (`ros2 run tf2_tools view_frames`) and `use_sim_time` true on all nodes (AC0-2).

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
