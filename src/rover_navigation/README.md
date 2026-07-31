# rover_navigation

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README format per **CNV-4**.

Nav2 configuration: planner/controller/costmap params, behavior trees, static maps, launch.

## Purpose

Entire Nav2 config is open for research (§6.1) — experiments and results are REQUIRED evaluation artifacts (T1a-4), logged in Jira + docs/experiments/. Candidates: Smac Hybrid-A* (planner), RPP vs MPPI-Ackermann (controller). Back-up recovery DISABLED (single forward camera — reversing goes into unobserved space). Consistency check: planner min_turning_radius must match what the chosen kinematics actually realizes (T1a-1).

## Interfaces

**Consumes:** costmap from `map_server` (1a) or `nvblox_nav2` plugin (1b), TF, `/odom`.
**Publishes:** `/cmd_vel`, plan, costmaps.

**Config:** `config/*.yaml` frozen + git-tagged at AC1-5.

## How to run the test

20-consecutive-goal suite per environment; ≥90% success at 0.25 m / 15° tolerance, zero collisions (AC1-1/2), cross-track ≤0.15 m on fixed regression path (AC1-3).

## Related spec items

See the traceability table in the root README and Jira for the task IDs this package owns.
