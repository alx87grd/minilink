# Phase 1: Step Core (leaf)

**After [Phase 0](00-wiring-refactor.md).** Pure difference-equation blocks — **no wall clock, no
diagrams, no `discretize`.**

Leaf types implement `step(x, u, k)` directly (MPC law, SMC, chess, games, manual difference
eqs.). See [Phase 2 (02-step-diagram.md)](02-step-diagram.md) for composing blocks into diagrams.

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

- Same port/params surface as `DynamicSystem`; **`step` returns next state** (`x_new`), not a
  derivative.
- **Integer step index `k`**, not float `t` — do not use `n` (state dimension) for the index.
- **`k=0` default** — same *role* as `t=0` on flow maps; many blocks ignore `k`.
- **`StepSystem` has no wall time** — `k` is a turn / fire index (chess move, MPC solve index,
  game step). No `dt`, no `t_k`, no seconds on the leaf class.
- `solver_info`: `continuous_time_equation=False`.
- Do **not** overload `f` on this class.
- **No sample time on the class** — `StepSchedule.dt_base` ([Phase 4](04-scheduled-orchestrator.md))
  schedules *when* the orchestrator fires; it does **not** become time inside `step()`.

### Third slot: evolution vs outputs

| Map | Method | Third arg | Returns |
| --- | --- | --- | --- |
| Flow leaf | `f(x, u, t; p)` | `t` float | `dx` |
| Step leaf | `step(x, u, k; p)` | `k` int | `x_new` |
| Step leaf output | `h(x, u, k; p)` | `k` int | `y` |

`h` uses **`k`**, not `t` — aligned with step index, not wall clock. Do **not** pass
`float(k)` or `t_k = k * dt` into `StepSystem` APIs.

### Step diagrams and static blocks

When a `StaticSystem` sits in a `StepDiagramSystem` ([Phase 2](02-step-diagram.md)), shared gather
passes **`k`** into `port.compute(..., third, ...)` (same Python slot as flow `t`). Most static
blocks ignore the third argument. Flow time-varying sources (`Step` at seconds, `WhiteNoise`) are
**out of scope** for step diagrams in v1 unless given discrete counterparts.

## `ZOHHold` (optional leaf)

Step-side hold register for teaching / tests; hybrid plant holds live in `HybridSimulator`
([Phase 5](05-hybrid-simulation.md)).

## Tests

- `test_step_system.py`: leaf `step`; `h(x, u, k)`; `ZOHHold`; `solver_info`; no `f` overload.
