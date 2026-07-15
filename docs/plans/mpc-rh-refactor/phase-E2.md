# Phase E2 — `RecedingHorizonController`

**Status:** stub (expand before coding)  
**Depends on:** [E1](phase-E1.md) · contracts: [vision.md](vision.md)

## Goal

Product façade: deploy tick (`compute_command`) **and** `rhc @ plant`
(`export_to_computer` / `__matmul__`). PoC leaves can remain until E3.

## Exit

RH can tick and export; demos still on PoC controllers until E3.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_mpc_stateful_controller.py \
       tests/unittest/test_mpc_export_computer.py \
       tests/unittest/test_receding_horizon_controller.py  # new
```

## Outline (expand when starting)

| Step | Work |
| --- | --- |
| E2.1 | `RecedingHorizonController(planner, dt_mpc=…, warm_start=True)` |
| E2.2 | Tick: optional warm-start helpers → `solve_trajectory_from(..., initial_guess=seed)` → latch |
| E2.3 | `compute_command(y, …)` |
| E2.4 | Warm-start via `mpc/warm_start.py` (orchestration here, not on Planner) |
| E2.5 | `as_step_block` / `export_to_computer` / `__matmul__` |
| E2.6 | Tests vs `solve_trajectory_from` + export smoke |

Next: [phase-E3.md](phase-E3.md).
