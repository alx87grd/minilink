# Phase 1a: Evolution map refactor (`f` on `DynamicSystem`)

**After [Phase 0](00-wiring-refactor.md), before [Phase 1 step core](01-step-core.md).**
Mechanical refactor: move continuous evolution off the `System` shell, split typed
**compile / evaluator / simulate** paths for static vs dynamic leaves, keep unified user
verbs where they still apply.

**Status:** complete (`d131a89` on `dev-hybrid`). Façade routing superseded by
[Phase 1b](01b-facade-mixin-split.md) (`40c8297`). See
[01a-implementation-plan.md](01a-implementation-plan.md).

Full hybrid context: [00-master-plan.md](00-master-plan.md),
[hybrid-discrete-simulation.md](../hybrid-discrete-simulation.md).

---

## Goal

| Today | Target |
| --- | --- |
| `System.f` default → zeros on all kinds | **`f` only on `DynamicSystem`** (+ flow `DiagramSystem.f`) |
| `compile(any leaf)` → `DynamicsEvaluator` with fake `f` on static | **Typed evaluators** by system kind |
| `compute_trajectory` → always `Simulator` | **Static** → `StaticSimulator`; **dynamic** → `Simulator` |
| Step leaves (Phase 1) would inherit lying `f` | **`StepSystem` has no `f`** — only `step` |

---

## Target class hierarchy

```text
System                         # ports, params, h, metadata, facades — NO f, NO step (n defaults to 0)
├── Source(System)             # y = h(t) — compile/sim as static
├── DynamicSystem              # dx = f(x,u,t), y = h(·)     ← f lives HERE
├── StepSystem                 # x_{k+1} = step(x,u,k)      (Phase 1)
├── DiagramSystem              # stacked f (flow diagrams) — IS-A DynamicSystem (Phase 1b)
└── StepDiagramSystem          # stacked step               (Phase 2)
```

**Routing rule:** evolution kind is **class type** (`isinstance`), not
`solver_info["continuous_time_equation"]`.

---

## Unified `compile()` — one name, typed evaluators

Single entry: `sys.compile()` / `compile(system, backend=...)`. **No public `compile_step`.**

| Input | Evaluator | Core API |
| --- | --- | --- |
| `DynamicSystem` leaf | `NumpyLeafEvaluator` (`DynamicsEvaluator`) | `.f`, `.h`, `.outputs`, RK4, … |
| `System` leaf with `n==0` / `Source` | **`NumpyStaticEvaluator`** (`StaticEvaluator`) | `.outputs` only — **no `.f`** |
| `DiagramSystem` | `NumpyDiagramEvaluator` | diagram `.f` |
| `StepSystem` leaf (Phase 1) | `NumpyStepEvaluator` (`StepEvaluator`) | `.step`, `.outputs` — **no `.f`** |
| `System` leaf with `n>0` (no `f`) | `TypeError` | subclass `DynamicSystem` and implement `f()` |

Dispatch lives in `minilink/core/compile/compiler.py`.

### Evaluator layout (new / moved files)

```text
minilink/core/compile/evaluators/
  output_evaluator.py      # shared: h, h_p, outputs, outputs_p; n, m, p
  static_evaluator.py      # NumpyStaticLeafEvaluator (+ optional JAX later)
  evaluator.py             # DynamicsEvaluator — f, integration (dynamic + diagrams only)
  step_evaluator.py        # Phase 1 — StepEvaluator
  numpy_evaluator.py       # dynamic leaf + diagram only
  jax_evaluator.py         # dynamic leaf + diagram only
```

`DynamicsEvaluator` is **not** used for static leaves (no fake empty `f`).

---

## Simulate / rollout — typed paths

| System kind | Rollout API | Implementation |
| --- | --- | --- |
| `System` leaf (`n==0`) / `Source` | **`compute_trajectory`**, **`compute_forced`** | **`StaticSimulator`** — time grid + `u`; optional outputs in `traj.signals` |
| `DynamicSystem` / `DiagramSystem` | **`compute_trajectory`**, **`compute_forced`** | **`Simulator`** — ODE solvers + `DynamicsEvaluator` |
| `StepSystem` (Phase 1) | **`compute_rollout(...)`** only | **`StepEvaluator.rollout`** — integer `k`, clock-free |

### `compute_trajectory` on step systems — **deferred**

A future option is `compute_trajectory(n_steps=100)` as a façade over `StepEvaluator.rollout`
(clock-free rollout, possibly synthetic `t` on `Trajectory` for plotting only). **Not in 1a or
Phase 1.**

**Decision (July 2026):** keep **`compute_trajectory` as a time-signal API** for **static and
dynamic systems only**. `StepSystem` user API is **`compute_rollout`**; misuse of
`compute_trajectory` hits `StaticSimulator` rejection (Phase 1b — no façade router).

Rationale: avoids conflating step index `k` with simulation time `t` before hybrid orchestration
(Phase 4 `StepSchedule.dt_base`) is landed.

### Facade routing — superseded by Phase 1b

Historical 1a sketch used `_simulate` + `isinstance` routers in `facades.py`. **Landed design:**
[01b-facade-mixin-split.md](01b-facade-mixin-split.md) — `SharedSystemFacades` /
`DynamicSystemFacades` / `StepSystemFacades`; MRO picks `compute_trajectory` implementation.

### `StaticSimulator` (new)

**File:** `minilink/simulation/static_simulator.py`

- Mirror `Simulator` kwargs where sensible: `t0`, `tf`, `n_steps`, `dt`, `compile_backend`.
- `compile()` → `StaticEvaluator`; **does not** use `SolverBackend` or `.f`.
- `solve()` / `solve_forced()`: `x` shape `(0, n_pts)` when `n=0`; broadcast or sample `u`.
- **Recommended:** record `evaluator.outputs(...)` at each `t[i]` in `Trajectory.signals` for
  `plot_time_signals`.
- **`solver` kwarg:** ignored or warn once (not applicable).

Extract shared time-grid logic to `minilink/simulation/time_grid.py` (used by `Simulator` and
`StaticSimulator`).

---

Replace **`if subsystem.n == 0`** skip with **`isinstance(subsystem, DynamicSystem)`** in:

| Location | Change |
| --- | --- |
| `DiagramSystem.f` | Only call `subsystem.f` on `DynamicSystem` |
| `build_execution_plan` state_ops | Only attach `f_func=sys.f` for `DynamicSystem` |

Static subsystems stay in **port ops** only (`port.compute`). Phase 2: reject `StepSystem`
inside flow diagrams at compile time.

---

## `ZOHHold` (Phase 1 leaf — spec for downstream)

Two equivalent latch maps (`x_{k+1} = u_k`, `y_k = x_k`); pick one for v1:

| Option | Port story | Teaches |
| --- | --- | --- |
| **1 — Command latch** | `u` = new command, `y` = held command | ZOH / hold register |
| **2 — Measurement latch** | `u` = new sample, `y` = held measurement | plant→step sample |

Phase 1 default: **Option 1** as generic `ZOHHold` in `minilink/blocks/`.

---

## Code touch list

### Must change (1a)

| Area | Files |
| --- | --- |
| Class split | `minilink/core/system.py` |
| Compile dispatch | `minilink/core/compile/compiler.py`, new `static_evaluator.py`, `output_evaluator.py` |
| Dynamic evaluators | `numpy_evaluator.py`, `jax_evaluator.py` — leaf only for `DynamicSystem` |
| Diagram guards | `minilink/core/diagram.py`, `compiler.py` `build_execution_plan` |
| Static sim | `minilink/simulation/static_simulator.py`, `time_grid.py` |
| Facades | `minilink/core/facades.py` |
| Continuous sim | `simulator.py` — reject `StepSystem` when Phase 1 lands |
| Animator | `graphical/animation/animator.py` — `isinstance(..., DynamicSystem)` for Euler sub-steps |

### Migrate bare `System` subclasses

| File | Class | → |
| --- | --- | --- |
| `benchmarks/systems/network.py` | `SimpleGain` | `System(0)` |
| | `SimpleIntegrator`, `MultiInputNode` | `DynamicSystem` |
| `examples/.../demo_dynamic_bicycle_cascade_path_tracking.py` | `PathPlanner` | `System(0)` |
| `examples/.../demo_advanced_autowire.py` | `DemoPathPlanner` | `System(0)` |

Preserve user tuning in demos — base class / imports only.

### Analysis / planning (guards + docstrings)

`linearize`, `equilibria`, `phase_plane` — expect `DynamicSystem | DiagramSystem` for `sys.f`.
`core/backends.py` — `"direct"` mode documents **`DynamicSystem.f`**.

---

## Tests (1a)

| File | Cases |
| --- | --- |
| `test_system_evolution_maps.py` | no `System.f`; `DynamicSystem.f`; static `Gain` has no `f` |
| `test_compile_static.py` | `compile(Gain)` → static evaluator; `.outputs` parity; no `.f` |
| `test_static_simulator.py` | shapes; `compute_trajectory` on static leaf |
| `test_compile_pipeline.py` | diagram static+dynamic; state_ops on `DynamicSystem` only |
| `test_facades_split.py` | static → `StaticSimulator`, dynamic/diagram → `Simulator` (Phase 1b) |

Regression: full `pytest`; `ruff check .`; `ruff format --check .`.

---

## Gates

- **Phase 0 complete** before 1a.
- **Phase 1a complete** before Phase 1 `StepSystem` / `StepEvaluator.rollout` (clean sibling types).
- Phase 1 adds `StepSystem` + `StepEvaluator` + `compute_rollout` on top of 1a dispatch.

---

## Deferred (not 1a)

| Item | When |
| --- | --- |
| `compute_trajectory` on `StepSystem` | TBD — possibly `StepTimedSimulator` or clock-free `n_steps` façade |
| `StepDiagramSystem` | Phase 2 |
| DESIGN §3 full sync | Phase 13 or when implementation lands |

---

## Verification smoke

```python
from minilink.blocks.routing import Gain
from minilink.blocks.basic import Integrator

g = Gain(2.0, dim=1)
g.compile().outputs(np.array([]), np.array([1.0]), 0.0)
g.compute_trajectory(tf=1.0, n_steps=11)

p = Integrator()
p.compile().f(np.array([0.0]), np.array([1.0]), 0.0)
p.compute_trajectory(tf=1.0, n_steps=11)
```
