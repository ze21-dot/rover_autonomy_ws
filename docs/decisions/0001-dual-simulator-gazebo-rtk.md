# ADR-0001: Dual-simulator strategy — Gazebo Harmonic for GNSS/RTK track

**Status:** accepted
**Date:** 2026-07-31
**Refs:** SUR-AUT-SPEC-001 §4.1 (HERE4 RTK), §7 (Phase 2 global pose), TF-2; Jira epic "Gazebo RTK Sim"

## Context

SUR-AUT-SPEC-001 is Isaac-Sim-first. Phase 2 requires a global correction source (map → odom) mirroring the CubePilot HERE4 RTK GNSS. Isaac Sim has no native GNSS sensor; the spec's fallback is a hand-written node sampling GT and adding noise. Meanwhile Gazebo (Harmonic, the official ROS 2 Jazzy pairing) ships a NavSat sensor with world spherical coordinates, plus IMU, giving a full localization sandbox with far less custom code — and it runs on machines without RT-core GPUs, so the localization sub-team can work in parallel without competing for the Isaac workstation.

## Options considered

1. **Isaac-only, custom GT-sampling GNSS node (spec default)** — single simulator, but the noise model is entirely hand-rolled and the Isaac workstation becomes a bottleneck for a second workstream.
2. **Gazebo Harmonic track for GNSS/RTK (chosen)** — native NavSat sensor; RTK realism added by a thin error-model node (fixed/float/dropout states). Cost: URDF must carry Gazebo tags (xacro-gated), and a second simulator is maintained.
3. **Full migration to Gazebo** — rejected: NVblox integration and the Isaac ROS toolchain are release-coupled to Isaac Sim; perception phases stay in Isaac.

## Decision

Isaac Sim remains authoritative for Phases 0–1 (perception, NVblox, Nav2 validation). A parallel **Gazebo Harmonic track** (`rover_gazebo`) develops the GNSS/RTK localization chain (NavSat → RTK error model → navsat_transform + EKF → map → odom). The localization stack (`rover_localization`) must be simulator-agnostic: identical topics, frames (REP-105), and `use_sim_time` discipline, so it drops onto the Isaac track (and Phase 3 hardware) unchanged.

## Consequences

- `rover_description` xacro gains a `simulator:=isaac|gazebo` arg; one URDF, two exports.
- TF-3 still holds per running system: only one simulator runs at a time in any bringup.
- The RTK error-model node's parameters (noise per state, dropout profile) become the reference model that Phase 3 HERE4 data is later validated against.
- Spec text is not silently contradicted: this ADR is the recorded "explicit decision" §3 requires for changes; fold into SUR-AUT-SPEC-001 Rev 1.1 at next revision.
