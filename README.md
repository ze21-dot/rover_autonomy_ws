# rover_autonomy_ws

Autonomous navigation pipeline for the **SuRover** platform. Simulation-first (Isaac Sim), progressing to real hardware (Jetson AGX Orin).

**Governing document: [SUR-AUT-SPEC-001 Rev 1.0](docs/) — the spec wins on every conflict.**

| | |
|---|---|
| Project | SuRover Autonomy |
| Spec | SUR-AUT-SPEC-001 Rev 1.0 |
| Tracking | Jira project `SURA` (epics = phases, stories = spec task IDs) |
| Maintainer | Zeynep Altundal |

## Pinned platform versions (§3 — change only by explicit decision)

| Component | Pinned | Notes |
|---|---|---|
| ROS 2 | **Jazzy** (Ubuntu 24.04) | Isaac ROS release line targets Jazzy |
| Isaac ROS | **4.5.0** (latest at project start) | Managed Dev Docker / CLI environment **only** — no native NVblox installs |
| Isaac Sim | **6.1** | Exact compatible build: verify against Isaac ROS release notes at setup and **record here**: `TODO` |
| GPU | `TODO: model + VRAM` | RTX-class (RT cores) required; Isaac Sim rendering and NVblox share it |
| Jetson (Phase 3) | AGX Orin Dev Kit | Ubuntu 24.04 on Orin expected ~Q3 2026 — **confirm before committing Phase 3 base to Jazzy on-device** |
| Gazebo | **Harmonic** (official Jazzy pairing) | GNSS/RTK track only — see ADR-0001; perception phases stay in Isaac |

## Quickstart

```bash
# 1. Clone (USD assets need LFS)
git lfs install
git clone git@github.com:<org>/rover_autonomy_ws.git
cd rover_autonomy_ws

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Phase 0 bringup (Isaac Sim scene must be running — see src/rover_sim/docs/)
ros2 launch rover_bringup sim_bringup.launch.py
```

All nodes run `use_sim_time:=true` in Phases 1–2 — **verified in every bringup launch file, not assumed** (CNV-3). All configuration lives in version-controlled YAML (CNV-5).

## Repository layout (§9)

One colcon workspace = one git repo. One responsibility per package; every package has a CNV-4 README (purpose / interfaces / how to test).

| Package | Responsibility |
|---|---|
| `rover_bringup` | Top-level + sim launch, aggregated params, TF-3 enforcement |
| `rover_description` | URDF/xacro, meshes, robot_state_publisher |
| `rover_control` | ros2_control: controller_manager + forward command controllers over `topic_based_ros2_control` |
| `rover_kinematics` | Double-Ackermann IK + wheel odometry (TK-3), unit tests |
| `rover_teleop` | Keyboard/gamepad → `/cmd_vel` |
| `rover_navigation` | Nav2 config, behavior trees, maps |
| `rover_localization` | Phase 2 EKF, GNSS sim node, AprilTag map |
| `rover_msgs` | Custom interfaces (empty by design until needed) |
| `rover_sim` | Isaac Sim scenes (Git LFS), action-graph docs |
| `rover_gazebo` | Gazebo Harmonic track: GNSS/RTK worlds, NavSat + RTK error model (ADR-0001) |

`build/`, `install/`, `log/` are git-ignored.

## Phase status

| Phase | Scope | Status | Exit criteria |
|---|---|---|---|
| 0 | Env bringup + teleop baseline | 🔵 in progress | AC0-1 … AC0-4 |
| 1a | Nav2 on GT + static map | ⚪ not started | — |
| 1b | NVblox costmap source | ⚪ not started | AC1-1 … AC1-5 |
| 2 | Noisy-sensor localization | ⚪ not started | spec §7 (TBD detail) |
| 3 | Real hardware (Orin) | ⚪ not started | spec §8 (TBD detail) |

## Open design decisions (research tasks — decision recorded before implementation)

| ID | Decision | Where recorded |
|---|---|---|
| T0-R3 | Kinematics: custom C++ vs adapted `steering_controllers_library` vs cascaded | `src/rover_kinematics/README.md` + ADR in `docs/decisions/` |
| T0-R2 | Mixed command interfaces work in pinned `topic_based_ros2_control`? | `src/rover_control/README.md` |
| T1a-4 | Planner/controller comparison (Smac Hybrid-A*; RPP vs MPPI) | `docs/experiments/` — **required evaluation artifact** |
| TF-4 | base_footprint on slopes: gravity-aligned vs chassis-tilted | ADR in `docs/decisions/` |
| §7 | Phase 2 localization design (what to fuse, global source) | ADR, Phase 2 kickoff |
| ADR-0001 | Dual-simulator: Gazebo Harmonic for GNSS/RTK track | `docs/decisions/0001-…` — **accepted** |

## Contributing

Branch naming, commit format, PR rules, and the Jira ↔ git mapping are in [CONTRIBUTING.md](CONTRIBUTING.md). Short version: every branch and every commit carries a spec task ID (`T0-R3`) or Jira key (`SURA-12`); small PRs; config changes never live only in a container.
