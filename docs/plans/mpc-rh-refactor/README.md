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
| [phase-E0.md](phase-E0.md) … [phase-E8.md](phase-E8.md) | Per-phase execution cards (E0–E1 done; E2 ready) |

**Active phase:** [phase-E2.md](phase-E2.md) — implement next; cite it in the PR.

## Related requirements (background only)

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

First demo target after E2: `demo_mpc_hybrid_minimal.py` → `mpc @ plant`.
