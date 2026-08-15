# rover_description

> Part of `rover_autonomy_ws` — governed by **SUR-AUT-SPEC-001 Rev 1.0**. README per **CNV-4**.

Karasimsek rover model. Base URDF exported from SolidWorks (Barış); sim-hardening applied on top (see below).

## Purpose

Single source of truth for rover geometry. All numbers below are **measured from this model** (not invented):

| Parameter | Value | Derivation |
|---|---|---|
| Wheelbase (L) | **0.882 m** | FL/RL steer joint origins along model-Y |
| Track at wheel contact (W) | **0.834 m** | rocker offsets ±0.37 + steer/wheel x-offsets |
| Wheel radius | **0.1465 m** | measured from wheel STL bounding box |
| Wheel width | 0.11 m | wheel STL |
| Steer joint limits | ±1.5708 rad (±90°) | xacro `steer_lower/upper` — **mechanical limit TBD (Barış), see below** |
| Chassis mass | 9.78 kg | SolidWorks inertial export |
| **min_turning_radius** | **R = L/2·cot(δ_max) + W/2 ≈ 0.417 m @ ±90°** | shared with Nav2 (§4.2); recompute when real δ_max lands |

⚠️ `min_turning_radius` currently assumes joint-limit steering (±90°). Physical linkage (ball rods)
may limit travel earlier — **pending answer from mechanical team**. Nav2 config must use the
recomputed value, never this provisional one.

## Sim-hardening changes vs. raw SolidWorks export

1. **REP-103 fix:** model is Y-forward as exported; new root chain
   `base_footprint → base_link (x-forward) → suspension_frame (fixed, yaw −90°)` makes
   `base_link` REP-103/105 compliant without touching baked-in coordinates. Old `base_link`
   renamed `chassis_link`.
2. **Collisions:** visual = original STLs (untouched); collision = primitives
   (wheels: cylinders r=0.1465×0.11 measured from STL; chassis + rocker arms: boxes from mesh
   bounding boxes; decorative links: no collision). Raw mesh collisions (up to 204k triangles)
   destabilize contact physics.
3. **`diff_joints` xacro arg** (default `fixed`): rocker suspension frozen for first spawns;
   pass `diff_joints:=revolute` to activate.
4. **Joint damping** on steer/wheel joints (no controllers yet → prevents free flopping).
5. **Gazebo tags:** wheel friction (mu1/mu2), `JointStatePublisher` system plugin.

## Interfaces

**Publishes (via launch):** `/robot_description`, TF (robot_state_publisher), `/joint_states`
(Gazebo plugin bridged), `/clock` (bridged, CNV-3).

**Launch:**
- `display.launch.py` — RViz + joint_state_publisher_gui (no sim)
- `gazebo_spawn.launch.py` — Gazebo Harmonic empty world + spawn + clock/joint_states bridge (GZ-2)

## How to run the test

```bash
colcon build --packages-select rover_description && source install/setup.bash
ros2 launch rover_description gazebo_spawn.launch.py
# expect: rover settles on ground, no jitter/sink/explode; TF tree clean; /joint_states flowing
ros2 run tf2_tools view_frames   # single root: base_footprint
```

## Related spec items

T0-R1 (this package), §4.2 (min_turning_radius derivation), TF-1, CNV-1/3. Deviations recorded
in Jira GZ-2 comments. Bonus meshes present but unreferenced (differential bar) — next model
iteration, pending Barış.
