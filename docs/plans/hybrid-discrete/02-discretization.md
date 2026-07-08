# Discretization Layer (Conversion)

**Build after [step core (01-step-core.md)](01-step-core.md).** Not on the Phase A hybrid critical path.

Provides a bridge from continuous plants to discrete `StepSystem` blocks when you want a
**fully discrete** plant model (e.g. teach ZOH discretization, run a closed loop in
`StepDiagramSystem` only). Phase A **`HybridSimulator`** keeps the plant continuous and does
**not** require this verb.

## Prerequisites

- `StepSystem` leaf contract + tests (`step(x, u, k)`).
- Continuous integration path to call from the wrapper (`DynamicsEvaluator` / `rk4_step` or equivalent).

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
- **Inputs:** Continuous `DynamicSystem`, sample time `dt`, and numerical integration method (`euler`, `rk4`, or `zoh` for LTI exact).
- **Outputs:** A **new** `StepSystem` whose `step` closes over `(system, dt, method)` — continuous system + sample time → pure stepping block.
- **Math:** `x_{k+1} = step(x, u, k)` by integrating `f(x, u, t)` from $t = k \cdot dt$ to $t = (k+1) \cdot dt$ with $u$ held constant (ZOH). Index `k` replaces continuous `t`; `dt` is fixed in the closure, not a `step` argument.
- **Does not replace** hybrid simulation: sampled controller + continuous plant remains the default MPC/SMC path.

### Tests
- `test_discretize.py`: `euler` and `rk4` match continuous integration of the parent system over fixed `dt`.
