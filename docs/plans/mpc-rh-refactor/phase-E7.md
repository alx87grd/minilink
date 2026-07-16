# Phase E7 — Online params façade (slim)

**Status:** done (façade) · **pipeline B bind:** deferred  
**Depends on:** [E6](phase-E6.md) · requirements:
[../planning-pipeline-architecture.md](../planning-pipeline-architecture.md) ·
contracts: [vision.md](vision.md)

## Goal (this pass)

Wire the trajopt / MPC **online `params` façade** so callers can pass a mapping
through `solve_trajectory_from` and `compute_command`, with a clear contract:

| `params` | Behavior |
| --- | --- |
| `None` or `{}` | Bind `x0` only (current compile-once path) |
| `{"scene": …}` | **`NotImplementedError`** — reserved for pipeline B |
| Other keys | **`ValueError`** |

Do **not** implement `ObstacleBank`, `J(z, p)`, or tick-time scene bind here.

## Contract

```python
plan = planner.solve_trajectory_from(x0, params=None)   # ok
plan = planner.solve_trajectory_from(x0, params={})     # ok
planner.solve_trajectory_from(x0, params={"scene": {}})  # NotImplementedError

cmd = mpc.compute_command(y, params={"scene": bank})     # NotImplementedError
# latch forwards params → solve_trajectory_from (same gate)
```

`ProblemParameters.scene` is reserved (`None` until pipeline B).

## Checklist

| Step | Work |
| --- | --- |
| E7.0 | Expand this card; sync phases / folder README |
| E7.1 | `reject_unknown_online_params`: scene → NotImplementedError |
| E7.2 | Thread `params` through `MPCTickLatch` → `solve_trajectory_from` |
| E7.3 | Reserve `ProblemParameters.scene`; DESIGN / ROADMAP |
| E7.4 | Tests + ruff; mark façade done |

## Deferred — full pipeline B (E7-full)

**Out of the current RH closing sequence** (F → UI). Keep as a wanted later
feature on ROADMAP. Extend existing `ParametricMathematicalProgram` — **not**
a new NLP type.

| ID | Work |
| --- | --- |
| B3–B4 | `ObstacleBank(K)`, `ProblemParameters.scene` / `SceneParameters` |
| B1–B2 | `J(z, p)` / `bind(p)` on parametric evaluator |
| B5 / B10 | Map façade `params` → bind (stop raising for `scene`) |
| B6 | Perception-style demo + compile-once timing |
| later | Parametric hard `g(z, p)` |

Order: spatial bank → NLP signatures → wire façade → demo (see pipeline doc).

## Constraints

- Soft scene bind without `J(z, p)` is out of scope (would re-JIT / rebuild).
- No ObstacleBank class; no perception demo this pass.
- Rebuild escape hatch remains: new `Scene` → re-`compile_parametric_program()`.

## Gate

```bash
ruff check . && ruff format --check .
pytest tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_planner.py \
       tests/unittest/test_mpc_*.py -q
```

## Exit

Façade + latch threading + reserved `scene` field; full bind documented as
deferred.

Next (closing sequence): [phase-F-cleanup.md](phase-F-cleanup.md) → UI plan.
Pipeline B remains parked here / ROADMAP.
