## What

<!-- one or two sentences -->

## Spec / Jira refs

- Spec task(s): <!-- T0-R3, TK-3, ... -->
- Jira: <!-- SURA-nn -->

## Verification

- [ ] Unit tests pass (`colcon test --packages-select <pkg>`)
- [ ] Sim-verified (attach recording / rosbag ref if behavior changed)
- [ ] Not applicable — docs/chore only

## Checklist

- [ ] Package README updated if interfaces changed (CNV-4)
- [ ] Config changes are in version-controlled YAML, own commit (CNV-5)
- [ ] Branch + commits carry spec/Jira IDs
- [ ] `use_sim_time` untouched or re-verified (CNV-3)
- [ ] Exactly one publisher per TF transform preserved (TF-3)
