# Phase 1: Step Core (leaf)

**Build first.** Pure difference-equation blocks — **no clock, no diagrams, no `discretize`.**

Leaf types implement `step(x, u, k)` directly (MPC law, SMC, games, manual difference eqs.).
See [Phase 2 (02-step-diagram.md)](02-step-diagram.md) for composing blocks into diagrams.

**Files:** `minilink/core/system.py`, `minilink/blocks/`

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
- **Integer step index `k`**, not float `t` — do not use `n` (state dimension) for the index.
- **`k=0` default** — same role as `t=0` on `f` / `h`; clock-free paths may ignore `k`.
- `solver_info`: `continuous_time_equation=False`.
- Do **not** overload `f` on this class.
- **No sample time on the class** — clock lives in orchestrators ([Phase 4](04-scheduled-orchestrator.md)).

## `ZOHHold` (optional leaf)

Step-side hold register for teaching / tests; hybrid plant holds live in `HybridSimulator`
([Phase 5](05-hybrid-simulation.md)).

## Tests

- `test_step_system.py`: leaf `step`; `ZOHHold`; `solver_info`.
