# Phase E3 — Migrate flagship demos to `ModelPredictiveController`

**Status:** stub (expand before coding)  
**Depends on:** [E2](phase-E2.md) · contracts: [vision.md](vision.md)

## Goal

Prove aim UX (`mpc @ plant`, `PlanningProblem(tf=T)`); keep NLP math
identical.

## Exit

`demo_mpc_hybrid_minimal.py` uses `mpc @ plant`; behavior comparable to today.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_hybrid_straight_line.py \
       tests/unittest/test_mpc_hybrid_warm_start_parity.py
```

**Smoke (required):**

```bash
python examples/scripts/hybrid/demo_mpc_hybrid_minimal.py
python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py
```

## Outline (expand when starting)

| Step | Work |
| --- | --- |
| E3.1 | Migrate hybrid minimal → `mpc @ plant` |
| E3.2 | Migrate straight-line → `compute_command` (or hybrid) |
| E3.3 | Optional: track lap + closed-loop lap |
| E3.4 | README / docstring sync |

Next: [phase-E4.md](phase-E4.md). Doc sync: README at this phase.
