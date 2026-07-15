# Phase E1 — Online surface on `MPCPlanner`

**Status:** stub (expand before coding)  
**Depends on:** [E0](phase-E0.md) · contracts: [vision.md](vision.md)

## Goal

Expose traj-family online API without RH controller yet:
`solve_trajectory_from(x0, …) -> TrajectoryPlan`.

## Exit

Online contract exists on `MPCPlanner`; hand demos not required to migrate.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_mpc_solve_trajectory_from.py  # new
```

## Outline (expand when starting)

| Step | Work |
| --- | --- |
| E1.1 | Wrap `step` → `TrajectoryPlan` (metadata from `OptimizationResult`, `warm_state=z`) |
| E1.2 | `solve_trajectory()` ≡ `solve_trajectory_from(problem.x_start)` |
| E1.3 | `solve()` → `solve_trajectory()` |
| E1.4 | Unit tests: metadata, `params` unused keys → error, parity vs `step` |

Next: [phase-E2.md](phase-E2.md).
