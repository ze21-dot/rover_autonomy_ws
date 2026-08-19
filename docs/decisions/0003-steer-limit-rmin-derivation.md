# ADR-0003: Steer limit and R_min derivation (kinematic track correction)

Date: 2026-08-19 | Status: Accepted | Refs: SURA steer task, ADR-0002

## Context
Provisional R_min=0.417 m was derived from track W=0.834 m (STL bounding
boxes = wheel-contact track) and inner-wheel 90 deg mechanical limit.
Baris reported MATLAB showing double-Ackermann center steer saturating
at ~53-57 deg "limited by ICR distance", which is geometrically
impossible with W=0.834 (center steer caps at 46.6 deg). Cross-check
script (steer_crosscheck.py) exposed the contradiction.

## Investigation
Ackermann geometry requires the STEER-AXIS (kingpin) track, not
wheel-contact track. From URDF joint origins:
- rocker joints (leftjoint/rightjoint) at +/-0.37 m from suspension_frame
- steer joints at -/+0.0077 m lateral offset from rocker links
=> half-track 0.3625 m, W_kinematic = 0.725 m (not 0.834)
Wheelbase confirmed: steer joint origins at +/-0.441 => L = 0.882 m.

## Consequences (with L=0.882, W=0.725)
- Center-steer ceiling: atan(0.441/0.3625) = 50.6 deg
  (consistent with Baris's ~53 deg; residual ~2-3 deg likely kingpin
  offset inside steer link - open question, does not change decision)
- Theoretical R_min (inner wheel at 90): W/2 = 0.363 m (singular:
  inner wheel speed -> 0; not operational)
- OPERATIONAL limits adopted:
  - inner wheel steer limit: 80 deg (10 deg margin: servo saturation,
    linkage tolerance, cable management)
  - => R_min operational: 0.44 m
  - Nav2 planner min radius: 0.50 m (planning margin)
- kinematics.yaml track corrected 0.834 -> 0.725 (2026-08-19)
- Old "R_min 0.417 FINAL" retracted permanently.

## Open questions
- Kingpin lateral offset inside steer link STLs (would refine W by
  ~1-3 cm); measure from CAD when Baris shares archive.
- Baris's original MATLAB script (if found) to overlay against
  steer_crosscheck.py output.
