# Phase E0 — Foundation

**Status:** implemented (verify gate green before merge)  
**Parent:** [phases.md](phases.md) · contracts: [vision.md](vision.md)

## Goal

Land types, move continuous \(T\) to `PlanningProblem.tf`, strip `tf` from
transcription, and rename `compute_solution` → `solve` with dual result slots —
**no functional change** for demos that set `tf=` explicitly on the problem.

## Scope

| Step | Work |
| --- | --- |
| E0.1 | Add `planning/results.py`: `SolveMetadata`, `TrajectoryPlan`, `PolicyPlan` |
| E0.2 | `PlanningProblem.tf` (`None` / finite / `+inf`; demos set finite `tf=` for trajopt) |
| E0.3 | `FixedGridOptions`: **`n_steps` only** — delete options `tf` |
| E0.4 | Grids from `problem.tf` + `n_steps` (`linspace(0, T, N)`) in MPC + trajopt |
| E0.5 | `Planner`: dual slots; `solve()` replaces `compute_solution` repo-wide |
| E0.6 | Stub typed methods on base; TOP/MPC/RRT/DP store wrapped plans |

## Constraints

- No `RecedingHorizonController` (E2).
- No real `solve_trajectory_from` body (E1) — ABC stubs only.
- No TOP/MPC merge (E4).
- Demos/tests/benchmarks always prescribe `PlanningProblem(..., tf=...)`.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_jax_direct_collocation.py \
       tests/unittest/test_planning_architecture.py \
       tests/unittest/test_rrt.py \
       tests/unittest/test_dynamic_programming.py
ruff check . && ruff format --check .
```

## Exit

- Gate green; `planner.solve()` is the public name
- Transcription has no `tf`; `problem.tf` + `n_steps` build the grid
