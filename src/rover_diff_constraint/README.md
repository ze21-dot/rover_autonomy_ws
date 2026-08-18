# rover_diff_constraint

Software differential bar for the SuRover hybrid suspension approach
(ADR-0002 spirit: v1 is a Python prototype; promote to C++ once behavior
is validated on marsyard v4).

Real differential bar enforces `q_left = -q_right` mechanically. We spawn
with `diff_joints:=revolute` and enforce the constraint from software.

## Modes

| mode | what it does | when |
|---|---|---|
| `effort` (default) | virtual spring-damper kills the *common mode* `qL+qR`, leaves the *differential mode* free for terrain | preferred — physically honest |
| `position` | commands `[d, -d]` with lowpassed `d=(qL-qR)/2` | only if no effort interface |

## Integration (3 steps)

1. **Find real joint names** and put them in `config/diff_constraint.yaml`:
   ```bash
   ros2 topic echo /joint_states --once | grep -i diff
   ```

2. **Add an effort forward controller** for the two diff joints to your
   ros2_control yaml (order MUST match [left, right] = command order):
   ```yaml
   diff_effort_controller:
     type: forward_command_controller/ForwardCommandController
   diff_effort_controller:
     ros__parameters:
       joints: [<left_diff_joint>, <right_diff_joint>]
       interface_name: effort
   ```
   URDF ros2_control block: both joints need `<command_interface name="effort"/>`
   and state interfaces for position + velocity.

3. **Launch** alongside sim:
   ```bash
   ros2 launch rover_description gazebo_spawn.launch.py world:=marsyard diff_joints:=revolute
   ros2 launch rover_diff_constraint diff_constraint.launch.py
   ```

## Test protocol (marsyard v4)

1. Baseline: drive over the 34-rock field WITHOUT the node, screen-record body roll.
2. Same line WITH the node. Expect: rocker'lar zit calisir, govde roll ~yarilanir.
3. Tune: oscillation -> halve `kp`; too floppy -> double `kp`, keep `kd ~ kp/20`.
4. Log evidence for Jira (SURA hybrid task): `ros2 bag record /joint_states`.

## Known unknowns (BLOCKER for v2)

- **Baris's diff-bar attempt failed for a reason we don't know yet** ("olmamisti,
  sebebi vardi"). Until his notes arrive, treat this as an experiment, not a
  solution. Candidate failure causes to check against his notes: controller
  fighting physics engine, effort interface not exposed in gz, joint limits,
  bullet solver instability on stiff constraints.
- Gains are guesses; tune on marsyard v4.
- Command order [left, right] must match controller yaml order.
