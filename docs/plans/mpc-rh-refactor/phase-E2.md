# Phase E2 — `ModelPredictiveController`

**Status:** done  
**Depends on:** [E1](phase-E1.md) · contracts: [vision.md](vision.md)

## Goal

Land the product **System family**: deploy tick (`compute_command`) **and**
`mpc @ plant` (`%` / `export_to_computer` / `__matmul__`) on a real Minilink
`System` / `StepSystem`. PoC factory names may remain as thin aliases until E3/E5.

## Contract (vision)

```python
mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
cmd = mpc.compute_command(y)   # Command
hybrid = mpc @ plant           # instance is the leaf
```

| `warm_start` | Type | State |
| --- | --- | --- |
| `False` | `System` (`n = 0`) | — |
| `True` | `StepSystem` | packed \(z\) on `Computer.x` |

Shared orchestration on the instance: warm-start helpers →
`solve_trajectory_from` → latch → ports + `compute_command`. **Not** a
non-System façade that exports a separate leaf. No separate `as_step_block`
API — `warm_start=True` *is* the step block.

## Files

| Path | Change |
| --- | --- |
| `minilink/planning/mpc/` | `ModelPredictiveController` family (+ `Command`); evolve PoC leaves |
| `minilink/planning/mpc/tick_latch.py` | NLP entry → `solve_trajectory_from` |
| `minilink/planning/mpc/__init__.py` | Export product name |
| `tests/unittest/test_model_predictive_controller.py` | New |
| PoC factories | Thin aliases until E5 |

## Steps

| Step | Work |
| --- | --- |
| E2.1 | Factory / dual subclasses: `ModelPredictiveController(planner, dt_mpc=…, warm_start=…)` |
| E2.2 | Tick: optional warm-start helpers → `solve_trajectory_from(..., initial_guess=seed)` → latch |
| E2.3 | `compute_command(y, …) -> Command` |
| E2.4 | Warm-start via `mpc/warm_start.py` (orchestration on controller, not Planner) |
| E2.5 | Ports + `export_to_computer` / `%` / `__matmul__` on the System instance |
| E2.6 | Tests vs `solve_trajectory_from` + `@` smoke; PoC aliases keep existing tests green |

## Constraints

- No demo migration (E3).
- No PoC name retirement (E5).
- No TOP merge (E4); no scene `params` (E7); no broadcast (E8).

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_mpc_solve_trajectory_from.py \
       tests/unittest/test_mpc_stateful_controller.py \
       tests/unittest/test_mpc_stateless_controller.py \
       tests/unittest/test_mpc_export_computer.py \
       tests/unittest/test_model_predictive_controller.py
ruff check . && ruff format --check .
```

## Exit

- `ModelPredictiveController` ticks and `@` works; demos may still use PoC factories until E3
- Vision System-family + naming stay consistent

Next: [phase-E3.md](phase-E3.md).
