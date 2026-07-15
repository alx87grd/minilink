# Phase E0 — Foundation

**Status:** next / active  
**Parent:** [phases.md](phases.md) · contracts: [vision.md](vision.md)

## Goal

Land types, `PlanningProblem.horizon`, grid resolve, and the `solve` rename
with **no functional change** for demos. Prefer a clean rename in one PR over
long-lived aliases.

## Scope

| Step | Work |
| --- | --- |
| E0.1 | Add `planning/results.py`: `SolveMetadata`, `TrajectoryPlan` (`trajectory`, `metadata`, `warm_state`, `x_dot`/`u_dot=None`, `to_flat`/`from_flat` minimal) |
| E0.2 | Optional `PlanningProblem.horizon`; validation \(T>0\) |
| E0.3 | Grid resolve helper: prefer `problem.horizon`, else `options.tf`; conflict → **hard error** |
| E0.4 | Wire resolve into MPC + trajopt collocation time-grid construction (behavior-identical if demos still pass `tf=` only) |
| E0.5 | `Planner`: dual slots `last_trajectory_plan` / `last_policy_plan`; `solve()` replaces `compute_solution` **repo-wide** (~40–60 call sites) |
| E0.6 | Stub typed methods as `NotImplementedError` on base; TOP/MPC/RRT/DP route `solve()` to current bodies and store into the right slot (may wrap bare `Trajectory` as `TrajectoryPlan` with partial metadata) |

## Files likely touched

| Area | Paths |
| --- | --- |
| New | `minilink/planning/results.py` |
| Problem | `minilink/planning/problems.py` |
| Planner ABC + subclasses | `minilink/planning/planner.py`, trajopt, `planning/mpc/`, RRT, DP |
| Grid / horizon | MPC prepare/transcribe + trajopt collocation time-grid helpers |
| Call sites | demos, tests, any `compute_solution` usage |

Export `TrajectoryPlan` / `SolveMetadata` from the planning package public
surface if there is an established `__init__` pattern — match neighborhood.

## Constraints (do not expand)

- No `RecedingHorizonController` yet (E2).
- No `solve_trajectory_from` behavioral API yet (E1) — stubs on the ABC only.
- Do not merge `MPCPlanner` into TOP (E4).
- Do not change demo UX/plots; demos may keep passing `tf=` only.
- Do not reinvent requirements brainstorms — see related docs linked from
  [README.md](README.md).

## Tests / demos

**Gate (automated)**

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_jax_direct_collocation.py \
       tests/unittest/test_planning_architecture.py \
       tests/unittest/test_rrt.py \
       tests/unittest/test_dynamic_programming.py
ruff check . && ruff format --check .
```

**Smoke (manual / optional):**

- `examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py`
- One trajopt demo (e.g. holonomic corridor or cartpole)
- One RRT demo

**Exit criteria**

- All gate tests green
- Demos unchanged functionally
- Public name is `planner.solve()`, not `compute_solution`
- `problem.horizon` + resolve helper exist; conflict with `options.tf` errors

## Suggested PR

**PR-A** — this phase alone. Cite this file in the PR body.

## Start sequence within E0

1. E0.1–E0.3 — types + horizon + resolve (can land with focused tests first)
2. E0.4 — wire into grids (behavior-identical)
3. E0.5 — rename blast (`compute_solution` → `solve`)
4. E0.6 — ABC stubs + slot storage on subclasses

## Done when

Hand off to [phase-E1.md](phase-E1.md) with green gate and rename complete.
