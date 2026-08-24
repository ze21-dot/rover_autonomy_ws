# SuRover KARASIMSEK — Isaac Sim 6.0 Mars Traverse (Showcase + Test Matrix)

Procedural Mars-analogue terrain, one fixed seed, one camera; only the rover *mode* changes between runs.
Produces per-run telemetry (CSV), config (JSON), and a real-time 60 s video.

**Pinned environment:** Isaac Sim **6.0.1.0** (cloud standard, see `bootstrap_isaac6.sh`), Python 3.12, PhysX TGS @ 60 Hz, CCD on.
Isaac Sim 5.x is no longer on PyPI; do not attempt to downgrade.

## Files

| File | Purpose |
|---|---|
| `mars_traverse.py` | Scene build + physics + camera + telemetry. Parameterised by env vars (below). |
| `bake_ground.py` | Seamless procedural Mars albedo + normal map (FFT noise) → `/workspace/assets/mars_ground.jpg`, `mars_normal.jpg` |
| `bake_sky.py` | Equirect dome sky (dust bands, horizon haze, sun glow) → `/workspace/assets/mars_sky.jpg` |
| `run_matrix.sh` | Sequential 8-run test matrix, one video per run |
| `../../tools/rover_eval/analyze.py` | Telemetry → metrics table (`runs/metrics.csv`, `runs/metrics.md`) |

## Quick start (cloud instance)

```bash
source /workspace/isaac_env/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES
cd /workspace
python bake_ground.py && python bake_sky.py        # once
RUN=smoke TOTAL=120 CAP=60 python mars_traverse.py  # smoke test (~2 min)
nohup bash run_matrix.sh > matrix_log.txt 2>&1 &    # full matrix (~40 min)
python tools/rover_eval/analyze.py                  # metrics table
```

Prerequisites on the instance: `karasimsek_rigid.usd` and `karasimsek.usd` at `/workspace/` (URDF → USD via `isaacsim.asset.importer.urdf`, `run_asset_transformer=True`, `collision_from_visuals=True`), meshes pulled with `git lfs pull`.

## Parameters (`mars_traverse.py`)

| Env | Default | Meaning |
|---|---|---|
| `RUN` | test | Output dir `/workspace/runs/<RUN>/` |
| `MODEL` | rigid | `rigid` (rocker joints locked) / `revolute` (rocker free) |
| `MODE` | rigid | `rigid` / `passive` (viscous damping only) / `hybrid` (damping + PD common-mode torque) |
| `SPEED` | 7.0 | Wheel target, rad/s (0.145 m radius → 7 ≈ 1.0 m/s) |
| `LANE` | 1 | Lane-keeping steering correction on/off |
| `ROCKER_KD` | 300 | Rocker joint viscous damping (same for passive and hybrid — fair ablation) |
| `KP` / `KD` / `TAU` | 300 / 70 / 60 | Hybrid PD gains and torque clamp |
| `TOTAL` / `CAP` | 3600 / 5 | Physics steps @ 60 Hz / capture every CAP steps (720 frames → 60 s @ 12 fps) |
| `X_END` | 110 | Course end; run stops at x ≥ X_END, timeout, or rollover (>60°) |
| `FLAT` / `MESHCOL` | 0 | Diagnostics only (single flat slab / terrain mesh collision — see limitations) |

## Scene

- Terrain 240 × 240 m heightfield (361² verts). Lane |y| < 10 m contains: wide swells (25–50 cm, 8–14 m base), **one-track ridges** (16–28 cm, σ = 1 m, alternating ±0.95 m offset) and hollows — designed so one wheel is lifted while the other is not (asymmetric excitation for the rocker test).
- Physics ground: **0.5 m horizontal step-tiles** (14 336 static boxes). Terrain-mesh collision (SDF / triangle) does not hold the rover in Isaac 6.0.1 — rover falls through. See limitations.
- Rocks: 257 colliding (|y| ≥ 9 m) + 260 decorative non-colliding in the lane margin (|y| ≥ 1 m).
- Wheel μs/μd = 1.5/1.3, ground 1.4/1.2, restitution 0. Wheel effort cap 48 N·m (AK10-9 peak).
- Camera: 18 mm, look-at locked to a slow-filtered rover centre (does not copy chassis pitch/roll), 12 shots.

## Hybrid controller

τ = −KP·(qL+qR−E0) − KD·d/dt(qL+qR), deadband 0.005 rad, slew 3 N·m/step, clamp ±TAU.
Goal: suppress rocker **common-mode** (qL+qR → 0) while preserving differential articulation (qL−qR).
Passive and hybrid share `ROCKER_KD`; the only difference in hybrid is the added PD torque.

## Known limitations (report these)

1. **Tile artefact.** Continuous flat plane: pitch_std 0.06°. Production terrain with 1 m slope-aligned tiles: 1.17°; with 0.5 m flat tiles (current): **0.52°**. This is a common disturbance input across all runs; use relative comparisons, not absolute σz / pitch.
2. **Rocker hard-stop.** Ridges of 16–28 cm drive |q| to the ±0.5 rad joint limit in both passive and hybrid runs. Roll benefit of the controller is bounded by joint travel — a mechanical design input.
3. **Wheel visual vs collision.** Importer collision sits ~15 cm above the visual wheel mesh; physics is at the correct equilibrium (z = 0.443 m), the visual gap is cosmetic.
4. **URDF placeholders.** `velocity=2.0 rad/s` and `effort=10000` in the URDF are unverified defaults; 7 rad/s is the team-nominated nominal, 28 rad/s is a stress point, not a rated capability.
5. **Rocker architecture unknown** (spring / damper / rigid?) — B/C damping = 300 is a modelling choice, not measured.
6. Sim-to-real is **not** validated; hardware comparison is scheduled for the Jetson Orin deployment (October).

## Process notes

- `py_compile` catches syntax only; grep for orphan references after large edits.
- Never trust "it's bouncing" by eye — isolate with FLAT=1 and telemetry first.
- Do not patch incrementally for hours; rewrite clean.
