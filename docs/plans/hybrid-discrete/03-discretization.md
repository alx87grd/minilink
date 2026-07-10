# Phase 3: Discretization (conversion tool)

**Status: Done** (July 2026). Optional analysis verb — **not** on the hybrid MPC critical path.

Wraps continuous dynamics + sample time into a **`StepSystem`** leaf for fully discrete plant
models or teaching. Default hybrid path keeps the plant **continuous** ([Phase 5](05-hybrid-simulation.md));
static controllers (`n = 0`) plug into :func:`~minilink.core.hybrid_composition.hybrid_closed_loop`
directly — sample time lives on :class:`~minilink.simulation.computer.StepSchedule`, not on the block.

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
) -> DiscretizedDynamicSystem:
    ...
```

### Contract

- **`discretize` inputs:** `DynamicSystem`, optional ``dt=``, optional extra ``params``; sample time lives in ``params["dt"]``.
- **`discretize` output:** ``DiscretizedEulerDynamicSystem`` or ``DiscretizedRK4DynamicSystem``; ``step`` / ``h`` forward ``params`` to the source ``f`` / ``h``.
- **Math:** `x_{k+1} = step(x, u, k; p)` = ZOH integration of `f` over one interval; `t_k = k · p["dt"]`.
- **Static controllers in hybrid loops:** pass the leaf `System` to `hybrid_closed_loop(..., schedule=dt)`.
  Sampling and ZOH are enforced by `Computer` / `HybridSimulator`.

## Tests

- `test_discretize.py`: `discretize` RK4 one step matches `rk4_step`; `h` delegates to source; no `sample_static` (schedule on `Computer` instead).
