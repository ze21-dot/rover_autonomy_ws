# rover_gazebo

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0** + **ADR-0001** (dual-simulator decision). README format per **CNV-4**.

Gazebo Harmonic track for GNSS/RTK localization development. Isaac Sim remains the simulator for perception/NVblox phases; Gazebo exists **only** because Isaac has no native GNSS sensor and the HERE4 RTK role (map → odom global correction) needs a realistic NavSatFix source.

## Purpose

- Reuse the same URDF (`rover_description`) — Gazebo-specific tags live in xacro args, one robot model for both simulators.
- Gazebo NavSat sensor (world with `spherical_coordinates` origin) → raw `NavSatFix`.
- **RTK error model node**: wraps the raw fix with RTK-fixed / RTK-float / dropout states, per-state noise (≈1–2 cm fixed, dm-level float), latency and outage injection — mirroring HERE4 behavior for Phase 3.
- Feeds `rover_localization` (navsat_transform + EKF) exactly as the simulated GNSS node in spec §7 — the localization stack must not know which simulator is underneath.

## Interfaces

**Publishes:** `/gps/fix` (sensor_msgs/NavSatFix, via RTK error model), `/imu` (Gazebo IMU), `/joint_states`, `/clock`.
**Consumes:** joint commands via `ros_gz` bridge (same controller topics as Isaac track where practical).
**Worlds:** `worlds/flat_gnss.sdf` (spherical coords set), later Mars-yard variant.

## How to run the test

`ros2 launch rover_gazebo gnss_sim.launch.py` → verify `/gps/fix` at expected rate, toggle RTK states via the error-model params, confirm `map → odom` from navsat_transform stays consistent with Gazebo ground truth (error reported vs GT).

## Related spec items

Spec §7 (Phase 2 global pose), §4.1 HERE4, TF-2. Decision record: `docs/decisions/0001-dual-simulator-gazebo-rtk.md`.
