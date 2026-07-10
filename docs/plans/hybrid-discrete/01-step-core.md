# Phase 1: Step Core (leaf + rollout)

**Status:** complete (`700f8ea` on `dev-hybrid`, July 2026). Façades finalized in
[Phase 1b](01b-facade-mixin-split.md) (`40c8297`).

**After [Phase 0](00-wiring-refactor.md) and [Phase 1a](01a-evolution-map-refactor.md).**
Pure difference-equation blocks and **leaf-only** rollout — **no wall clock, no diagrams,
no `discretize`.**

Leaf types implement `step(x, u, k)` directly (MPC law, SMC, chess, games, manual difference
eqs.). [Phase 2 (02-step-diagram.md)](02-step-diagram.md) composes blocks into
`StepDiagramSystem`; diagram evaluators plug into the same **`StepEvaluator.rollout`**
API defined here.

**Prerequisite:** [01a-evolution-map-refactor.md](01a-evolution-map-refactor.md) (`f` on
`DynamicSystem` only, typed `compile()`, `StaticSimulator` / `Simulator` split).

**Files:** `minilink/core/system.py`, `minilink/core/step_rollout.py`,
`minilink/core/compile/evaluators/` (step path), `minilink/blocks/step.py`,
`examples/scripts/step/` (teaching demos)

## Continuous ↔ step mirror

| Continuous time | Step (discrete index `k`) |
| --- | --- |
| `Trajectory` — `(t, x, u)` | **`StepRollout`** — `(k, x, u)` + optional `signals` |
| `compute_trajectory()` | **`compute_rollout(n_steps, u=...)`** |
| `plot_trajectory()` | **`plot_rollout()`** |
| `Simulator` + solvers | **None** — `StepEvaluator.rollout()` is the engine |
| `IntegrationMixin.integrate` | **`StepRolloutMixin.rollout`** |
| `self.traj` | **`self.rollout`** |

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
- **`n >= 1`**; static IO remains `System` with `n=0` (no `StaticSystem`).
- **Integer step index `k`**, not float `t` — do not use `n` (state dimension) for the index.
- **`k=0` default** — same *role* as `t=0` on flow maps; many blocks ignore `k`.
- **`StepSystem` has no wall time** — `k` is a turn / fire index (chess move, MPC solve index,
  game step). No `dt`, no `t_k`, no seconds on the leaf class.
- **Evolution kind is the class type** — use `isinstance(sys, StepSystem)` (or `DynamicSystem`
  for flow), **not** `solver_info["continuous_time_equation"]`.
- **No sample time on the class** — `StepSchedule.dt_base` ([Phase 4](04-computer.md))
  schedules *when* the **Computer** fires; it does **not** become time inside `step()`.
- **`self.rollout = None`** — convenience cache for the last rollout (parallel `self.traj`).

### Third slot: evolution vs outputs

| Map | Method | Third arg | Returns |
| --- | --- | --- | --- |
| Flow leaf | `f(x, u, t; p)` | `t` float | `dx` |
| Step leaf | `step(x, u, k; p)` | `k` int | `x_new` |
| Step leaf output | `h(x, u, k; p)` | `k` int | `y` |

`h` uses **`k`**, not `t` — aligned with step index, not wall clock. Do **not** pass
`float(k)` or `t_k = k * dt` into `StepSystem` APIs.

### Step diagrams and static blocks

When a `System` with `n=0` sits in a `StepDiagramSystem` ([Phase 2](02-step-diagram.md)), shared
gather passes **`k`** into `port.compute(..., third, ...)` (same Python slot as flow `t`). Most
static blocks ignore the third argument. Flow time-varying sources (`Step` at seconds,
`WhiteNoise`) are **out of scope** for step diagrams in v1 unless given discrete counterparts.

## Evolution kind vs `solver_info`

| Question | Answer |
| --- | --- |
| How do callers know flow vs step? | **Class type** — `DynamicSystem` / `DiagramSystem` vs `StepSystem` (Phase 2: `StepDiagramSystem`) |
| Is `solver_info["continuous_time_equation"]` the switch? | **No** — redundant with class type; do not use as primary routing |
| What stays in `solver_info`? | **Flow-only hints** on continuous systems |
| What does `StepSystem` do with `solver_info`? | Inherits `System` defaults; no Phase 1 contract to tune flags |

## Rollout and facades

- **No `StepSimulator` / `StepRunner` class** — rollout lives on the compiled step evaluator
  (mirror `integrate` on `DynamicsEvaluator`) plus user facades.
- **`compute_rollout(n_steps, u=...)`** — primary entry on `StepSystem` (mirror
  `compute_trajectory`).
- **`plot_rollout()`** — plots via `StepRollout.as_trajectory()` → existing
  `plot_time_signals` with `abscissa_label="Step [k]"` (reuses time plot pipeline).
- **`compute_trajectory` / `compute_forced`** — not the step user API; Phase 1b removed
  friendly `TypeError` overrides — misuse hits `StaticSimulator` rejection via MRO.
- **Facade dispatch only** — do **not** add `StepSystem` guards inside `Simulator` /
  `StaticSimulator`; keep the continuous path clean.

Leaf rollout path: **`compile()`** → `StepEvaluator` → **`rollout(...)`** or
**`compute_rollout(...)`**.

## `StepRollout` (clock-free result)

**File:** `minilink/core/step_rollout.py` (alongside `trajectory.py`)

```python
@dataclass(frozen=True)
class StepRollout:
    k: np.ndarray       # (N,)  integers 0 … n_steps
    x: np.ndarray       # (n, N)  Trajectory-compatible layout
    u: np.ndarray       # (m, N)  ZOH: u[:, k] applied at step k
    signals: dict[str, np.ndarray] = field(default_factory=dict)  # each (dim, N)
```

- **`N = n_steps + 1`** state samples at `k = 0, …, n_steps`.
- Inputs at `k = 0, …, n_steps - 1`; **hold last input** at the final sample for plot width
  alignment (`u[:, -1] = u[:, -2]` when `n_steps >= 1`).
- **`as_trajectory()`** — maps `k` → `t` float array for `plot_time_signals` only; no wall-clock
  semantics.
- Phase 2 diagram rollouts return the same type.

**Two pipelines (Phase 2 vs 4):** `StepDiagramSystem` is **topology**. **`compute_rollout`**
(Pipeline A) fires every block synchronously. **`Computer.tick(u)`** (Pipeline B,
[Phase 4](04-computer.md)) applies a **firing schedule** with internal buffers — not the same
dynamics when `fire` is non-trivial.

**Not in Phase 1:** `TimedStepSimulator` (Phase 2 diagram stopgap),
**`Computer`** (Phase 4), hybrid plant integration (Phase 5).

## Leaf compile (unified `compile()`)

Same verb as flow and static ([Phase 1a](01a-evolution-map-refactor.md)): **`sys.compile()`**
dispatches by type. **No public `compile_step`.**

**Phase 1** — `compile()` on **`StepSystem` leaf only** (diagram branch in Phase 2); Phase 1
rejected `DiagramSystem` / `StepDiagramSystem` until Phase 2 landed.

**Dispatch order (leaf):** `DiagramSystem` → **`StepSystem`** → `DynamicSystem` → static
`n == 0` → `TypeError`.

| Backend | Leaf class | API |
| --- | --- | --- |
| NumPy | `NumpyStepEvaluator` | `.step(x, u, k)`, `.outputs(x, u, k)`, `.rollout(...)` |
| JAX (optional) | `JaxStepEvaluator` | same; **separate** JIT from flow (`k` int, not `t` float); `rollout` via `jax.lax.scan` |

- Base **`StepEvaluator(OutputEvaluator)`**: `.step`, `.step_p`, `.outputs`, `.outputs_p` (+ `_p`
  tier). **No `.f`**, **no `.h`** on evaluator — boundary outputs are the `outputs` dict
  ([Phase 1a](01a-evolution-map-refactor.md)).
- **`StepRolloutMixin.rollout`** — clock-free loop on integer `k`; input coercion: `u=None`
  (nominal), constant vector, `(n_steps, m)` sequence, or `callable(k)`.
- Wrap `step` / port `compute` with frozen `params` and nominal `u` — same pattern as dynamic
  leaf.
- Diagram evaluators in Phase 2 implement the same surface so **`rollout` is unchanged**.

**Do not** route `StepSystem` through flow `DynamicsEvaluator` / RK4 / `IntegrationMixin`.

## `ZOHHold` (optional leaf)

Step-side hold register for teaching / tests; hybrid plant holds live in `HybridSimulator`
([Phase 5](05-hybrid-simulation.md)). Spec: [01a ZOHHold](01a-evolution-map-refactor.md#zohhold-phase-1-leaf--spec-for-downstream).

**File:** `minilink/blocks/step.py`

## Teaching demos (canonical leaf scripts)

Flat under `examples/scripts/step/`, runnable from repo root. **No** `Simulator`, **no**
diagrams — use **`compute_rollout`** (or low-level `compile().rollout(...)`):

```python
result = fibonacci.compute_rollout(n_steps=20)
# or:
result = fibonacci.compile().rollout(fibonacci.x0, n_steps=20)
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

- `test_step_system.py`: leaf `step`; `h(x, u, k)`; no `f`; `ZOHHold`; `self.rollout` attr.
- `test_compile_step_leaf.py`: `compile()` on step leaf → `NumpyStepEvaluator`; frozen params;
  `step` parity.
- `test_step_rollout.py`: `StepRollout` shapes `(n, N)` / `(m, N)`; `u` constant / sequence /
  `u(k)`; `as_trajectory()`; JAX scan smoke.
- `test_facades_rollout.py`: `compute_rollout` / `plot_rollout`.
- `test_facades_split.py` (Phase 1b): MRO surface; step leaves use `compute_rollout`.
- Extend `test_evaluator_api.py`: step evaluator has `rollout`, no `f` / `integrate`.

Plus Phase 1a tests in [01a-evolution-map-refactor.md](01a-evolution-map-refactor.md#tests-1a).

## Explicit non-goals (Phase 1)

- `StepDiagramSystem`, `discretize()`, `HybridSimulator`, `game()` step branch
- `simulation/step_runner.py`, `StepRunner`, `StepSimulator`
- Deep `StepSystem` guards in `Simulator` / `StaticSimulator`
- Changes to flow `DiagramSystem` compile or simulation paths
