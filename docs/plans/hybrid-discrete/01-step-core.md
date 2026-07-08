# Phase 1: Step Core (leaf + rollout)

**After [Phase 0](00-wiring-refactor.md) and [Phase 1a](01a-evolution-map-refactor.md).**
Pure difference-equation blocks and **leaf-only** rollout — **no wall clock, no diagrams,
no `discretize`.**

Leaf types implement `step(x, u, k)` directly (MPC law, SMC, chess, games, manual difference
eqs.). [Phase 2 (02-step-diagram.md)](02-step-diagram.md) composes blocks into
`StepDiagramSystem`; diagram evaluators plug into the same `StepRunner` API defined here.

**Prerequisite:** [01a-evolution-map-refactor.md](01a-evolution-map-refactor.md) lands first
(`f` on `DynamicSystem` only, typed `compile()`, `StaticSimulator` / `Simulator` split).

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
- **No `f` method** — continuous evolution lives on `DynamicSystem` only ([Phase 1a](01a-evolution-map-refactor.md)).
- **Integer step index `k`**, not float `t` — do not use `n` (state dimension) for the index.
- **`k=0` default** — same *role* as `t=0` on flow maps; many blocks ignore `k`.
- **`StepSystem` has no wall time** — `k` is a turn / fire index (chess move, MPC solve index,
  game step). No `dt`, no `t_k`, no seconds on the leaf class.
- **Evolution kind is the class type** — use `isinstance(sys, StepSystem)` (or `DynamicSystem`
  for flow), **not** `solver_info["continuous_time_equation"]`.
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
| What stays in `solver_info`? | **Flow-only hints** on continuous systems |
| What does `StepSystem` do with `solver_info`? | Inherits `System` defaults; no Phase 1 contract to tune flags |

### Rollout and facades

- **`Simulator`** rejects `StepSystem` leaves ([Phase 1a](01a-evolution-map-refactor.md) guard).
- **`compute_trajectory` / `compute_forced`** — **not supported** on `StepSystem` in Phase 1.
  Use **`compile()` + `StepRunner.run_steps(...)`**. A future façade (`compute_trajectory(n_steps=...)`
  or timed adapter) is **deferred** — see [01a deferred table](01a-evolution-map-refactor.md#deferred-not-1a).
- Leaf rollout path: **`compile()`** → `StepEvaluator` → **`run_steps`**.

## Leaf compile (unified `compile()`)

Same verb as flow and static ([Phase 1a](01a-evolution-map-refactor.md)): **`sys.compile()`**
dispatches by type. **No public `compile_step`.**

**Phase 1** — `compile()` on **`StepSystem` leaf only** (diagram branch in Phase 2); rejects
`DiagramSystem` / `StepDiagramSystem` with a clear error until Phase 2.

| Backend | Leaf class | API |
| --- | --- | --- |
| NumPy | `NumpyStepLeafEvaluator` | `.step(x, u, k)`, `.h(x, u, k)`, `.outputs(x, u, k)` |
| JAX (optional) | `JaxStepLeafEvaluator` | same; **separate** JIT from flow (`k` int, not `t` float) |

- Wrap `step` / `h` with frozen `params` and nominal `u` snapshot — same pattern as dynamic leaf.
- Base **`StepEvaluator`** protocol: `.step`, `.h`, `.outputs` (+ `_p` tier). Diagram evaluators
  in Phase 2 implement the same surface so **`StepRunner` is unchanged**.

**Do not** route `StepSystem` through flow `DynamicsEvaluator` / RK4.

## `StepRunner` (clock-free rollout)

**File:** `minilink/simulation/step_runner.py`

Advance **`N`** steps on integer **`k`**; **no `tf`, no `dt`**. Turn-based games, unit tests,
teaching demos, algorithmic stepping.

```python
@dataclass
class StepResult:
    k: np.ndarray          # (n_steps + 1,)
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
Default `k = 0, 1, …`.

**Not in Phase 1:** `TimedStepSimulator` (Phase 2 diagram stopgap), `ScheduledStepOrchestrator`
(Phase 4), hybrid plant integration (Phase 5).

## `ZOHHold` (optional leaf)

Step-side hold register for teaching / tests; hybrid plant holds live in `HybridSimulator`
([Phase 5](05-hybrid-simulation.md)). Spec: [01a ZOHHold](01a-evolution-map-refactor.md#zohhold-phase-1-leaf--spec-for-downstream).

## Teaching demos (canonical leaf scripts)

Flat under `examples/scripts/step/`, runnable from repo root. **No** `Simulator`, **no**
diagrams — use **`compile(block)` + `StepRunner.run_steps`**:

```python
evaluator = block.compile()
result = run_steps(evaluator, block.x0, n_steps=20, u=u_seq)
```

| Demo | File | Teaches |
| --- | --- | --- |
| **Fibonacci** | `demo_step_fibonacci.py` | Pure difference equation; 2-state recurrence; no `u` |
| **Discrete accumulator** | `demo_step_accumulator.py` | \(x_{k+1} = x_k + u_k\); contrast with continuous `Integrator` |
| **Logistic map** | `demo_step_logistic_map.py` | Scalar nonlinear step; `params["r"]` |

### Validation (demos)

```bash
python examples/scripts/step/demo_step_fibonacci.py
python examples/scripts/step/demo_step_accumulator.py
python examples/scripts/step/demo_step_logistic_map.py
```

## Tests

- `test_step_system.py`: leaf `step`; `h(x, u, k)`; no `f`; `ZOHHold`; `Simulator` / `compute_trajectory` reject step leaf.
- `test_compile_step_leaf.py`: `compile()` on step leaf; frozen params; `step`/`h` parity.
- `test_step_runner.py`: clock-free `run_steps`; `StepResult` shapes; optional `u(k)` callable.

Plus Phase 1a tests in [01a-evolution-map-refactor.md](01a-evolution-map-refactor.md#tests-1a).
