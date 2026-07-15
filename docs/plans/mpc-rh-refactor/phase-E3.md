# Phase E3 — Migrate flagship demos to `ModelPredictiveController`

**Status:** done  
**Depends on:** [E2](phase-E2.md) · contracts: [vision.md](vision.md)

## Goal

Prove aim UX on flagship demos: hybrid `mpc @ plant` and deploy
`compute_command` hand loops. Keep NLP math / tuning identical where possible;
PoC factory names stay in the library until E5.

## Contract (vision)

```python
mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
hybrid = mpc @ plant              # hybrid demos
cmd = mpc.compute_command(y, t=t) # hand-loop / deploy
```

| Path | Demo role |
| --- | --- |
| `mpc @ plant` | Closed-loop Computer + plant (hybrid scripts) |
| `compute_command` | Hand-loop ZOH + RK4 (straight-line); packed-`z` warm-start |

## Files

| Path | Change |
| --- | --- |
| `examples/scripts/hybrid/demo_mpc_hybrid_minimal.py` | PoC → product + `mpc @ plant` |
| `examples/scripts/hybrid/demo_mpc_hybrid_track_lap.py` | Same wiring |
| `examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py` | Hand loop → `compute_command` |
| `README.md` | Examples table copy for hybrid rows |
| `docs/plans/mpc-rh-refactor/` | Status / active phase |

## Steps

| Step | Work |
| --- | --- |
| E3.1 | Hybrid minimal → `ModelPredictiveController` + `mpc @ plant` |
| E3.2 | Straight-line → `compute_command` (stay hand loop / `RateInputs`) |
| E3.3a | Hybrid track lap → same as E3.1 |
| E3.4 | README + docstring sync; mark E3 done |

## Constraints

- No closed-loop lap / obstacle / stadium / trajopt / spatial-guide migration.
- No PoC name retirement (E5); tests may still import aliases.
- No TOP merge (E4); no scene `params` (E7); no broadcast (E8).
- Do not retune demo gains / `TF` / `STEP_DISP` / commented sections.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_hybrid_straight_line.py \
       tests/unittest/test_mpc_hybrid_warm_start_parity.py
```

**Smoke (required):**

```bash
MPLBACKEND=Agg python examples/scripts/hybrid/demo_mpc_hybrid_minimal.py
MPLBACKEND=Agg python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py
```

```bash
ruff check . && ruff format --check .
```

## Exit

- `demo_mpc_hybrid_minimal.py` uses `mpc @ plant`
- Straight-line uses `compute_command` with product warm-start
- README hybrid rows describe the product API
- Behavior comparable to pre-migration (NLP path unchanged)

Next: [phase-E4.md](phase-E4.md).
