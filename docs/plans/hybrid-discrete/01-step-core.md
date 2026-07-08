# Layer 1: Pure Discrete Core (No Time)

**Build first.** Standalone stepping framework — no dependency on continuous `f` or `discretize`.
Controllers (`MPCBlock`, `SMCBlock`), games, and difference-equation blocks implement `step`
directly; [discretize (02-discretization.md)](02-discretization.md) is an optional later verb
that **produces** a `StepSystem` from a continuous plant.

**Files:** `minilink/core/system.py`, `minilink/core/step_diagram.py`, `minilink/simulation/step_runner.py`

## `StepSystem`

```python
class StepSystem(System):
    """
    x_{k+1} = step(x, u, k; p)
    y_k     = h(x, u, k; p)
    """

    def step(self, x, u, k=0, params=None):
        ...

    def h(self, x, u, k=0, params=None):
        ...
```

### Contract
- Same port/params surface as `DynamicSystem`; **`step` returns next state** (`x_new`), not a derivative.
- **Integer step index `k`**, not float `t` — mirrors continuous time the way `n` mirrors state dimension (do not use `n` for the index).
- **`k=0` default** — same role as `t=0` on `f` / `h`; turn-based and clock-free paths may ignore it.
- `solver_info`: `continuous_time_equation=False`.
- Do **not** overload `f` on this class.

## `StepDiagramSystem`

- Sibling of `DiagramSystem`, sharing port wiring via `core/wiring.py` mixin.
- **`x_{k+1} = step(x, u, k)`** — same stacked loop as continuous `DiagramSystem.f`.
- `compile_step_diagram` yields a `StepEvaluator` exposing **`step(x, u, k)`** which iterates topologically through each block's `step`.
- Subsystems permitted: `StepSystem`, `StaticSystem`. Rejects `DynamicSystem` at compile.

## Run / Simulation

Pure discrete systems have no concept of physical time; they simply iterate forward.

### `StepRunner`
- Steps the discrete equations forward `N` times by incrementing `k`.
- **No `tf`, no `dt`**.
- Useful for pure state-machine algorithms or un-clocked games (like chess moves).

### `TimedStepSimulator`
- Evaluates a synchronous `StepDiagramSystem` on a uniform grid (`k = 0, 1, …` with `t_k = k · sync_dt` for logging only).
- Single-rate only; multi-rate uses `ScheduledStepOrchestrator` ([03-scheduled-orchestrator.md](03-scheduled-orchestrator.md)).

### Tests
- `test_step_system.py`: leaf `step`; `ZOHHold`; `solver_info`.
- `test_step_diagram.py`: wiring; `@` closed loop.
- `test_step_runner.py`: clock-free stepping tests.
