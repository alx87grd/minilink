# Phase E1 — Online surface on `MPCPlanner`

**Status:** done  
**Depends on:** [E0](phase-E0.md) · contracts: [vision.md](vision.md)

## Goal

Expose the generic traj-family online API on `MPCPlanner` without RH yet:

```python
plan = mpc.solve_trajectory_from(x0, params=None, initial_guess=None)  # TrajectoryPlan
```

Layering (vision):

| Layer | Owns |
| --- | --- |
| Planner `solve_trajectory_from` | \(x_0\), optional `params`, optional `initial_guess` |
| `TrajectoryPlan.warm_state` | Result artifact (packed \(z\)) |
| RH + `warm_start.py` | Warm-start *orchestration* from last latch (E2) |

No planner kwarg named `warm_start` on the from-API.

## Files

| Path | Change |
| --- | --- |
| `minilink/planning/mpc/planner.py` | Add `solve_trajectory_from`; route `solve` / `solve_trajectory` through it |
| `tests/unittest/test_mpc_solve_trajectory_from.py` | New unit tests |
| [vision.md](vision.md) | Already patched for kwargs / RH warm-start (this refinement) |

## Steps

| Step | Work |
| --- | --- |
| E1.1 | `solve_trajectory_from(x0, *, params=None, initial_guess=None)` → reject unknown `params` keys; `step` → `require_trajectory_plan()` |
| E1.2 | `solve_trajectory(*, initial_guess=None)` ≡ `solve_trajectory_from(problem.x_start, …)` |
| E1.3 | `solve(**kw)` → `solve_trajectory(**kw)` |
| E1.4 | Tests: parity vs `step`, metadata / `warm_state`, `params` unused keys → error, alias parity, `initial_guess` accepted |

`params is None` or `{}` → today’s `bind(x0)` only. Non-empty keys → hard
`ValueError` until scene (E7). `step` keeps returning bare `Trajectory`.

## Constraints

- No `RecedingHorizonController` / RH `warm_start=True` (E2).
- No demo migration (E3).
- No TOP parametric merge (E4).
- Hand demos may keep calling `step`.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_mpc_solve_trajectory_from.py
ruff check . && ruff format --check .
```

## Exit

- Online contract on `MPCPlanner`; `solve` / `solve_trajectory` share that path
- Vision kwargs / RH warm-start story stays consistent
- Demos not required to migrate

Next: [phase-E2.md](phase-E2.md).
