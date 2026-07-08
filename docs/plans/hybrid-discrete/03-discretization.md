# Phase 3: Discretization (conversion tool)

**After [Phase 2](02-step-diagram.md).** Optional analysis verb — **not** on the hybrid MPC critical path.

Wraps continuous dynamics + sample time into a **`StepSystem`** leaf for fully discrete plant
models or teaching. Default hybrid path keeps the plant **continuous** ([Phase 5](05-hybrid-simulation.md)).

## Prerequisites

- `StepSystem` + compile path from Phases 1–2.
- Continuous integration to call from the wrapper (`DynamicsEvaluator.rk4_step` or equivalent).

## `discretize` verb

**File:** `minilink/analysis/discretize.py`

```python
def discretize(
    system: DynamicSystem,
    dt: float,
    method: str = "rk4",
) -> StepSystem:
    ...
```

### Contract

- **Inputs:** `DynamicSystem`, sample time `dt`, method (`euler`, `rk4`, `zoh` for LTI exact).
- **Output:** New `StepSystem` with `step` closing over `(system, dt, method)`.
- **Math:** `x_{k+1} = step(x, u, k)` = ZOH integration of `f` over one interval; `dt` in the
  closure, not a `step` argument.

## Tests

- `test_discretize.py`: `euler` / `rk4` match continuous integration over fixed `dt`.
