# Phase 1: Step Core (leaf + rollout)

**After [Phase 0](00-wiring-refactor.md).** Pure difference-equation blocks and **leaf-only**
rollout — **no wall clock, no diagrams, no `discretize`.**

Leaf types implement `step(x, u, k)` directly (MPC law, SMC, chess, games, manual difference
eqs.). [Phase 2 (02-step-diagram.md)](02-step-diagram.md) composes blocks into
`StepDiagramSystem`; diagram evaluators plug into the same `StepRunner` API defined here.

**Files:** `minilink/core/system.py`, `minilink/core/compile/` (leaf step path),
`minilink/simulation/step_runner.py`, `minilink/blocks/`, `examples/scripts/step/` (teaching demos)

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
- **Evolution kind is the class type** — use `isinstance(sys, StepSystem)` (or `DynamicSystem`
  for flow), **not** `solver_info["continuous_time_equation"]`. That flag is legacy on
  `System` defaults only; Phase 1 does **not** set or test it on `StepSystem`.
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

## Evolution kind vs `solver_info`

| Question | Answer |
| --- | --- |
| How do callers know flow vs step? | **Class type** — `DynamicSystem` / `DiagramSystem` vs `StepSystem` (Phase 2: `StepDiagramSystem`) |
| Is `solver_info["continuous_time_equation"]` the switch? | **No** — redundant with class type; do not use as primary routing |
| What stays in `solver_info`? | **Flow-only hints** on continuous systems: `smallest_time_constant`, `discontinuous_behavior`, etc. |
| What does `StepSystem` do with `solver_info`? | Inherits `System` defaults; **no Phase 1 contract** to tune or assert flags |

### Phase 1 simulator touch

[`Simulator.select_solver`](../../../minilink/simulation/simulator.py) today gates on
`continuous_time_equation`. Migrate to **`isinstance(sys, StepSystem)`** (or check before
`Simulator` construction) so discrete leaves get a clear error without maintaining a
duplicate meta flag.

`compute_trajectory` on `StepSystem` remains **unsupported** (flow-only façade). Leaf
**`compile_step`** and **`StepRunner`** are in scope — the public pure-step rollout path.

## Leaf compile (`compile_step`)

Mirror flow [`compile()`](../../../minilink/core/compile/compiler.py): one entry point, leaf vs
diagram branch added in Phase 2.

**Phase 1** — `compile_step(system, backend=...)` accepts **`StepSystem` leaf only**; rejects
`DiagramSystem` / `StepDiagramSystem` with a clear error.

```python
def compile_step(system, backend="numpy", verbose=False) -> StepEvaluator:
    ...
```

| Backend | Leaf class | API |
| --- | --- | --- |
| NumPy | `NumpyStepLeafEvaluator` | `.step(x, u, k)`, `.h(x, u, k)`, `.outputs(x, u, k)` |
| JAX (optional) | `JaxStepLeafEvaluator` | same; **separate** JIT from flow (`k` int, not `t` float) |

- Wrap `step` / `h` with frozen `params` and nominal `u` snapshot — same pattern as
  `NumpyLeafEvaluator`.
- `StepSystem.compile()` delegates to `compile_step` (analogous to flow `System.compile()`).
- Base **`StepEvaluator`** protocol (minimal): `.step`, `.h` (and `.outputs` if mirroring flow
  leaf). Diagram evaluators in Phase 2 implement the same surface so **`StepRunner` is unchanged**.

**Do not** route `StepSystem` through flow `compile()` / `DynamicsEvaluator` / RK4.

## `StepRunner` (clock-free rollout)

**File:** `minilink/simulation/step_runner.py`

Advance **`N`** steps on integer **`k`**; **no `tf`, no `dt`**. Turn-based games, unit tests,
teaching demos, algorithmic stepping.

```python
@dataclass
class StepResult:
    k: np.ndarray          # (n_steps + 1,) or per-step indices used
    x: np.ndarray          # (n_steps + 1, n)
    y: np.ndarray          # (n_steps + 1, p) when recorded
    u: np.ndarray          # (n_steps, m) inputs applied

def run_steps(
    evaluator,
    x0,
    *,
    n_steps,
    u=None,                # constant, sequence, or callable u(k)
    record_y=True,
) -> StepResult:
    ...
```

Inner loop: `y = evaluator.h(x, u_k, k)` (optional) then `x = evaluator.step(x, u_k, k)`.
Caller owns whether `k` is `0..N-1` or an explicit sequence — default `k = 0, 1, …`.

**Not in Phase 1:** `TimedStepSimulator` (Phase 2 diagram stopgap), `ScheduledStepOrchestrator`
(Phase 4), hybrid plant integration (Phase 5).

## `ZOHHold` (optional leaf)

Step-side hold register for teaching / tests; hybrid plant holds live in `HybridSimulator`
([Phase 5](05-hybrid-simulation.md)).

## Teaching demos (canonical leaf scripts)

Flat under `examples/scripts/step/`, runnable from repo root. **No** flow `Simulator`, **no**
diagrams — use **`compile_step(block)` + `StepRunner.run_steps`**:

```python
evaluator = block.compile_step()  # or compile_step(block)
result = run_steps(evaluator, block.x0, n_steps=20, u=u_seq)
```

| Demo | File | Teaches |
| --- | --- | --- |
| **Fibonacci** | `demo_step_fibonacci.py` | Pure difference equation; 2-state recurrence \(x_{k+1} = A x_k\); no `u`; `StepRunner` rollout |
| **Discrete accumulator** | `demo_step_accumulator.py` | \(x_{k+1} = x_k + u_k\); scripted `u` sequence; contrast with continuous `Integrator` / `f` |
| **Logistic map** | `demo_step_logistic_map.py` | Scalar nonlinear step; `params["r"]`; `result.x` trajectory |

### Sketch: Fibonacci (simplest iconic)

```text
x = [f_k, f_{k+1}]^T     step →  [f_{k+1}, f_k + f_{k+1}]^T
x0 = [0, 1]^T             h → y = x   (or y = [f_k, f_{k+1}])
```

Print `y[0]` or `x[0]` each `k` — sequence 0, 1, 1, 2, 3, 5, …

### Sketch: discrete accumulator

```text
x_{k+1} = x_k + u[0]      u from port or scripted sequence (e.g. all ones → running sum)
```

### Sketch: logistic map

```text
x_{k+1} = r * x[0] * (1 - x[0])     x0 in (0, 1); scan r or fix r=3.9 for chaos demo
```

**Not chosen for v1 canon** (save for Phase 2+ or optional extras): chess/turn engine (needs
`k` in logic), MPC block (Phase 6), `ZOHHold`-only script (fold into accumulator or Phase 2
diagram demo).

### Validation (demos)

```bash
python examples/scripts/step/demo_step_fibonacci.py
python examples/scripts/step/demo_step_accumulator.py
python examples/scripts/step/demo_step_logistic_map.py
```

Acceptance: exit 0; printed sequences match closed-form spot checks (Fibonacci terms, sum of
`u`, logistic stays in `[0,1]` for `r` in `(0,4]` when `x0` in `(0,1)`).

## Tests

- `test_step_system.py`: leaf `step`; `h(x, u, k)`; `ZOHHold`; `isinstance(StepSystem)`; no `f`
  overload; `Simulator` rejects step leaf without relying on `solver_info` flag.
- `test_compile_step_leaf.py`: `compile_step` on leaf; frozen params; `step`/`h` parity with
  direct calls.
- `test_step_runner.py`: clock-free `run_steps` on leaf evaluator; `StepResult` shapes; optional
  `u(k)` callable.
