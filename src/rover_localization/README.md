# rover_localization

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Phase 2: EKF configuration, simulated GNSS node, AprilTag map. Phase 1: static identity map->odom only.

## Purpose

Phase 1: no estimator runs — GT odom→base_link from Isaac, static identity map→odom (TF-1). Phase 2 (open for research, §7): EKF (robot_localization) fusing wheel odom (TK-3) + sim IMU for odom→base_link; simulated GNSS node (GT + noise/latency/dropout → NavSatFix, mirroring HERE4) for map→odom. Stretch: cuVSLAM, AprilTag corrections. GT TF then republished on diagnostics topic only (TF-2).

## Interfaces

**Phase 2 (planned) — Subscribes:** `/wheel_odom`, `/imu`, `/gps/fix`.
**Publishes:** `odom → base_link` TF (EKF), `map → odom` TF (navsat), diagnostics GT.

## How to run the test

Rerun full Phase 1 test suites under estimated localization; report pose error vs GT.

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
