# Phase 1: Step Core — Implementation Plan

**Branch:** `dev-hybrid` (Phase 1a landed: `d131a89`)  
**Spec:** [01-step-core.md](01-step-core.md)  
**Gate:** Phase 1 before [02-step-diagram.md](02-step-diagram.md)

## Goal

Add clock-free discrete evolution as a first-class leaf type alongside `DynamicSystem`, without
complicating the continuous-time core. Phase 1 is leaf-only: difference equations, compile,
rollout, facades, three teaching demos.

**Done when:** Fibonacci / accumulator / logistic-map demos run via `compute_rollout`; `compile()`
dispatches `StepSystem` → `StepEvaluator`; tests + docs gate pass.

## Frozen decisions

| Topic | Phase 1 choice |
| --- | --- |
| Rollout engine | `StepEvaluator.rollout()` via `StepRolloutMixin` (mirror `integrate`) |
| No `StepRunner` / `step_runner.py` | Rollout on evaluator + facades |
| Result type | `StepRollout` (`k`, not `t`) |
| Facades | `compute_rollout()`, `plot_rollout()` |
| Cache | `self.rollout` on `StepSystem` |
| Evaluators | `NumpyStepEvaluator`, `JaxStepEvaluator`; `outputs` dict only |
| Guards | Facade dispatch only — no `Simulator` / `StaticSimulator` internals |
| `plot_rollout` | Plan A: `as_trajectory()` → `plot_time_signals`; zero `graphical/` edits |
| Array layout | `(n, N)` / `(m, N)` Trajectory-compatible |

## Implementation slices

| # | Deliverable | Files |
| --- | --- | --- |
| 1 | `StepSystem` — `step`/`h` with `k`, port boilerplate, `self.rollout` | `core/system.py` |
| 2 | `StepRollout` + `as_trajectory()` | `core/step_rollout.py` |
| 3 | `StepEvaluator`, `StepRolloutMixin`, NumPy + JAX | `compile/evaluators/step_evaluator.py`, `step_rollout.py` (mixin), `jax_utils.py` |
| 4 | `compile()` `StepSystem` branch before `DynamicSystem` | `compile/compiler.py` |
| 5 | `compute_rollout`, `plot_rollout`; optional `compute_trajectory` hint | `core/facades.py` |
| 6 | `ZOHHold` + three demos | `blocks/step.py`, `examples/scripts/step/` |
| 7 | Tests | `test_step_system`, `test_compile_step_leaf`, `test_step_rollout`, `test_facades_rollout` |
| 8 | Docs gate | `01-step-core.md`, `00-master-plan.md`, `DESIGN.md`, `README.md` |

## Implementation order

1. `StepRollout` + `StepSystem`
2. `StepEvaluator` + `StepRolloutMixin.rollout` (NumPy)
3. `compile()` branch
4. `compute_rollout` / `plot_rollout` facades
5. JAX `rollout` (`jax.lax.scan`)
6. `ZOHHold` + demos
7. Tests + docs + gate

## Verification

```bash
ruff check . && ruff format --check .
pytest tests/unittest/test_step_system.py \
       tests/unittest/test_compile_step_leaf.py \
       tests/unittest/test_step_rollout.py \
       tests/unittest/test_facades_rollout.py -q
MPLBACKEND=Agg python examples/scripts/step/demo_step_*.py
pytest
```

## Not created

- `simulation/step_runner.py`, `StepRunner`, `StepSimulator`
