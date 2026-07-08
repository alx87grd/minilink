# Layer 1: Pure Discrete Core (No Time)

**Files:** `minilink/core/system.py`, `minilink/core/discrete_diagram.py`, `minilink/simulation/scheduled_step.py`

## `StepSystem`

```python
class StepSystem(System):
    """
    x⁺ = φ(x, u, k; p)
    y  = h(x, u, k; p)
    """
```

### Contract
- Same port/params surface as `DynamicSystem`; `phi` returns **next state** (`x⁺`), not derivative.
- **No `t` argument**: Replaced by integer step index `k` to mirror continuous physical time.
- `solver_info`: `continuous_time_equation=False`.

## `StepDiagramSystem`

- Sibling of `DiagramSystem`, sharing port wiring via `core/wiring.py` mixin.
- **`x⁺ = φ(x, u, k)`** — same stacked loop as continuous `DiagramSystem.f`.
- `compile_step_diagram` yields a `StepEvaluator` exposing **`step(x, u, k)`** which iterates topologically through each block's `phi`.
- Subsystems permitted: `StepSystem`, `StaticSystem`. Rejects `DynamicSystem` at compile.

## Run / Simulation

Pure discrete systems have no concept of physical time; they simply iterate forward.

### `StepRunner`
- Steps the discrete equations forward conceptually `N` times. 
- **No `tf`, no `dt`**.
- Useful for pure state-machine algorithms or un-clocked games (like chess steps).

### `TimedStepSimulator`
- Evaluates a synchronous `StepDiagramSystem` given a fixed sample time mapping, for cases where physical time is needed but no multi-rate orchestrator is required.

### Tests
- `test_step_system.py`: leaf `phi`; `ZOHHold`; `solver_info`.
- `test_step_diagram.py`: wiring; `@` closed loop.
- `test_step_runner.py`: clock-free stepping tests.\n