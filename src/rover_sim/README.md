# rover_sim

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Isaac Sim scene references, action-graph documentation, USD assets (Git LFS).

## Purpose

A scene that exists only on one workstation does not exist — all USD versioned via Git LFS. Action graphs (T0-S4/S5/S6) documented in docs/: actuation (joint command sub), state (/isaac_joint_states + GT TF + /odom + /clock), camera (D435i-equivalent depth/color/camera_info, real intrinsics where practical). Friction values documented with source (T0-S2 — measured/datasheet/literature, never silently invented). Isaac drive-gain ↔ AK servo-mode mapping recorded (T0-S3).

## Interfaces

**Assets:** `usd/` (LFS). **Docs:** `docs/action_graphs.md`, `docs/physics_materials.md`, `docs/environments.md` (flat plane → factory + labyrinth in T1b-3 → Mars-yard AprilTag deferred to Phase 2).

## How to run the test

Single bringup script starts scene + all action graphs from one command (T0-S8); topics verified with `ros2 topic hz`.

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
