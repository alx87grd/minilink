# MPC / receding-horizon refactor

Status: **canonical implementation docs** (July 2026).  
Implementation branch: **`dev-mpc-v2`** (forked from vision-locked `dev-alex`).

This folder is the **source of truth** for the MPC refactor. When these
files disagree with brainstorm docs, **this folder wins** for names and
sequencing.

## How to read

| Doc | Role |
| --- | --- |
| [vision.md](vision.md) | Locked end-goal contracts (Planner 2×2, `TrajectoryPlan`, `ModelPredictiveController`) |
| [phases.md](phases.md) | E0–E8 order, gates, PR slices, start-here |
| [phase-E0.md](phase-E0.md) … [phase-E8.md](phase-E8.md) | Per-phase execution cards (E0–E8 done) |
| [phase-F-cleanup.md](phase-F-cleanup.md) | Next: `control/mpc` move + hygiene → then [UI plan](../planning-ui-simplification.md) |

**Active phase:** [phase-F-cleanup.md](phase-F-cleanup.md) — expand before coding.

**Progress:** E0–E8 landed on `dev-mpc-v2`. Closing sequence: **F** (`control/mpc`)
then **planning UI** (constructor ceremony). Pipeline B (online scene bind) is
a wanted later feature — **not** in this closing sequence.

## Related requirements (background only)

- [../planning-ui-simplification.md](../planning-ui-simplification.md) — flat planner-family constructor UX (after Phase F)
- [../mpc-controller-architecture.md](../mpc-controller-architecture.md) — R1–R12, Option β
- [../planning-pipeline-architecture.md](../planning-pipeline-architecture.md) — result wrappers / parametric NLP
- [../standard-planning-problems.md](../standard-planning-problems.md) — problem taxonomy (stochastic out of scope)

Legacy monolith pointer: [../receding-horizon-implementation-plan.md](../receding-horizon-implementation-plan.md).

## Locked one-liner

```text
PlanningProblem (+ tf)
  → Planner.solve / solve_trajectory[_from]
  → TrajectoryPlan
  → ModelPredictiveController (System family)  →  compute_command / @ plant
```

Demos, notebook, and package exports use that stack (E5 done).
