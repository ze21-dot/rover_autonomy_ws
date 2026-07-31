# Contributing — Git & Documentation Conventions

Short, clean history. Everything traceable to SUR-AUT-SPEC-001 and Jira. No exceptions for "quick fixes".

## Branches

```
main                      always builds, always launches Phase-0 bringup
└── feature/T0-R3-double-ackermann-ik
└── feature/SURA-14-nvblox-bringup
└── fix/T0-S3-drive-gain-mapping
└── exp/T1a-4-rpp-vs-mppi          experiments — may be messy, squash-merged
```

- Prefix: `feature/`, `fix/`, `exp/`, `docs/`.
- Every branch name contains a **spec task ID** (`T0-R3`, `TK-3`, `TF-4`) or **Jira key** (`SURA-nn`).
- `main` is protected: PR + 1 review required. No direct pushes.

## Commits — Conventional Commits + trailer

```
<type>(<package>): <imperative summary, ≤72 chars>

<optional body: what & why, not how>

Refs: T0-R3, SURA-12
```

Types: `feat` `fix` `docs` `test` `refactor` `chore` `exp`.

Examples:

```
feat(rover_kinematics): add double-Ackermann IK for pure-arc case

Refs: T0-R3, SURA-12
```

```
fix(rover_control): correct steer joint interface claim order

Mixed position/velocity claims failed silently when steer group
loaded second; load order now pinned in controller config.

Refs: T0-R2, SURA-9
```

Rules:
- One logical change per commit. Never `wip`, `fix stuff`, `asdf`.
- YAML param changes are their own commit (CNV-5 audit trail).
- Generated files, `build/ install/ log/`, and USD binaries outside LFS never get committed.

## Pull requests

- Small: target **< 400 changed lines** (excl. generated/docs). Bigger → split.
- Template auto-fills (`.github/PULL_REQUEST_TEMPLATE.md`): spec refs, test evidence, config-change declaration.
- Squash-merge for `exp/` branches; merge-commit for features (keeps unit-test commits visible).
- A PR that changes behavior must say how it was verified (unit test, sim run, screen recording link).

## Decision records (ADRs)

Open design decisions (T0-R3, TF-4, Phase 2 localization, …) get a one-page ADR in `docs/decisions/` **before** implementation:

```
docs/decisions/0001-kinematics-package-approach.md
  Status / Context / Options considered / Decision / Consequences / Refs
```

The spec explicitly requires "decide, record the choice and reasoning, then implement" — the ADR is where that lives; the package README links to it.

## Jira ↔ git mapping

| Jira | Git |
|---|---|
| Epic = spec phase (Phase 0, 1a, 1b, 2, 3) | milestone / project board column |
| Story = spec task ID (T0-S1 … TK-3) | branch + `Refs:` trailer |
| Sub-task = concrete step | commits |
| Story "Done" | PR merged **and** the task's test/verification evidence attached to the Jira issue |

Acceptance criteria (AC0-x, AC1-x) are separate verification stories — they close phases, not features.

## Documentation duties (CNV-4)

Touch a package's interfaces → update its README in the same PR. Reviewer checks this; it is a valid reason to block a merge.
