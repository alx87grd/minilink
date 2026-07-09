# Phase 3: Discretization (conversion tool)

**Status: Done** (July 2026). Optional analysis verb — **not** on the hybrid MPC critical path.

Wraps continuous dynamics + sample time into a **`StepSystem`** leaf for fully discrete plant
models or teaching. Default hybrid path keeps the plant **continuous** ([Phase 5](05-hybrid-simulation.md));
static controllers use **`sample_static`** (no internal state).

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

def sample_static(system: System, dt: float) -> System:
    ...
```

### Contract

- **`discretize` inputs:** `DynamicSystem`, sample time `dt`, method (`rk4` only in v1).
- **`discretize` output:** New `StepSystem` with `step` closing over `(system, dt, method)`.
- **Math:** `x_{k+1} = step(x, u, k)` = ZOH integration of `f` over one interval; `dt` in the
  closure, not a `step` argument.
- **`sample_static` inputs:** static `System` (`n = 0`), e.g. Pyro-parity `SlidingModeController`.
- **`sample_static` output:** Wrapper with the same boundary ports; `dt` stored in `params`.
  Sampling and ZOH are enforced by `Computer` / `HybridSimulator`.

## Tests

- `test_discretize.py`: `discretize` RK4 one step matches `rk4_step`; `sample_static` delegates
  `ctl` output.
