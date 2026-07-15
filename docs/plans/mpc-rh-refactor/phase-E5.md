# Phase E5 — Retire PoC leaves

**Status:** done  
**Depends on:** [E3](phase-E3.md) / [E4](phase-E4.md) · contracts: [vision.md](vision.md)

## Goal

Hard-cut the public PoC controller surface. Product API is only
`ModelPredictiveController`. Delete `mpc_*_controller` factories and stop
exporting `MPCStatelessController` / `MPCStatefulController` /
`MPCDirectCollocationTranscription`. Keep warm-start / overlays tools.

## Contract

```python
mpc = ModelPredictiveController(planner, dt_mpc=..., warm_start=True)
cmd = mpc.compute_command(y, t=t)          # deploy / hand loop
hybrid = mpc @ plant                       # single-rate hybrid
```

`MPCStatelessController` / `MPCStatefulController` remain private factory
backends (modules stay). No deprecation aliases (pre-1.0 clean rename).

Teaching exceptions: trajopt reference + spatial scene guide may keep
planner-level teaching loops; update stale PoC names in copy.

## Inventory (move before delete)

| Call site | Action |
| --- | --- |
| Hand-loop bicycle demos (closed-loop / obstacle / stadium / wide / multi) | `compute_command` |
| `demo_mpc_hybrid_track_lap.ipynb` | product controller + `@ plant` |
| Unittests on `mpc_*_controller` | retarget to `ModelPredictiveController` |

## Steps

| Step | Work |
| --- | --- |
| E5.0 | Expand this card; sync phases / README mermaid |
| E5.1 | Migrate remaining hand-loop demos → `compute_command` |
| E5.2 | Notebook + unittests off PoC factories |
| E5.3 | Hard-cut `mpc/__init__`; delete factories; docs |
| E5.4 | Pytest / ruff + mandatory demo smoke; mark done |

## Constraints

- Preserve user-tuned demo constants / commented plots.
- No scene `params` (E7); no broadcast (E8).
- Keep `Command`, warm-start helpers, overlays, `mpc_plans_from_rollout`.

## Gate

```bash
ruff check . && ruff format --check .
pytest tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_*.py tests/unittest/test_as_computer.py -q

export MPLBACKEND=Agg SDL_VIDEODRIVER=dummy
# MPC migrated + product + hybrid + teaching (see plan E5.4 smoke table)
```

## Exit

One controller product: `ModelPredictiveController` (System family).

Next: [phase-E6.md](phase-E6.md).
