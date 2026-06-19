# Minilink Technical Design

Architecture and public contracts. User guide and call chains: [README.md](README.md).

## 1. Design Principles

1. **Math readability first**: e.g. `dx = A @ x + B @ u`.
2. **Pure contracts, convenient boundaries**: equation paths stay functional;
   facades (`compute_trajectory`, `plot_*`, `animate`) live at API boundaries.
3. **Explicit data flow**: visible objects and direct calls; no global backend
   switches or hidden registries.
4. **Backend-native math where simple**: one class for traceable NumPy/JAX algebra.
5. **Specialize only when it clarifies**: `Jax<Plant>` twins when a single class
   would sacrifice readability.

### NumPy and JAX

NumPy required; JAX optional (`minilink[jax]`), imported lazily via
`minilink.core.backends` (`require_jax_numpy()`, `array_module()`). No `minilink.jax`
package, no global mode. Explicit `compile_backend` and evaluator backend args.
`array_module()` only for small hybrid helpers.

## 2. Interface Layers

| Layer | Use | Examples |
| --- | --- | --- |
| 1 Facades | Default | `compute_trajectory`, `plot_trajectory`, `+`/`>>`/`@` |
| 2 Orchestrators | Repeat runs, trajopt, NLP | `Simulator`, `TrajectoryOptimizationPlanner`, `Optimizer` |
| 3 Contracts | Custom wiring, extension | `DiagramSystem.connect`, `compile()`, `MathematicalProgram` |

Import from defining modules; `minilink/__init__.py` is a namespace marker until
exports are frozen ([ROADMAP.md](ROADMAP.md) P1).

## 3. Package Map

Component maturity is tracked only in [ROADMAP.md](ROADMAP.md); this section
describes package ownership. Every package belongs to one of four bands.
Planned packages (homes pre-decided so future content lands without
rearrangement) are listed in [ROADMAP.md §5](ROADMAP.md).

**Framework** — defines what a `System` is and how diagrams execute
(NumPy-only; changes are design events):

| Package | Role |
| --- | --- |
| `core/` | `System` (+ `SystemFacades` mixin), `DiagramSystem`, signals/ports (`signals.py`), backend policy & helpers (`backends.py`), `Trajectory`, sets, costs |
| `core/compile/` | `ExecutionPlan`, compiler, NumPy/JAX evaluators |

**System libraries** — `System` subclasses you drop into a diagram, shelved by
*role in the diagram*, never by implementation technology (linear, Lagrangian,
or neural network alike):

| Package | Role |
| --- | --- |
| `blocks/` | plant-agnostic wiring: sources, `Integrator`, `TransferFunction`, routing (`Sum`/`Gain`/`Mux`/`Demux`), nonlinear (`Saturation`/`DeadZone`/`Relay`), filters, neural (`NeuralNetwork`) |
| `dynamics/` | plants: `abstraction/` mother classes, `catalog/` by physical domain, `engines/` plant-generating kernels (experimental) |
| `control/` | control laws and design factories (`ProportionalController`, `PDController`, `PIDController`, `FilteredPIDController`, `LinearStateFeedbackController`, `lqr`) |
| `estimation/` | online state and parameter estimators (planned) |

**Tools** — verbs on a `System`; they return data or plots and never define
user-facing systems (factories are fine: `linearize_matrices()` returns arrays,
`linearize()` wraps them as an `LTISystem`, and an LQR design function returns a
state-feedback block):

| Package | Role |
| --- | --- |
| `simulation/` | `Simulator`, solvers, forcing |
| `analysis/` | `linearize_matrices` (→ arrays), `linearize` (→ `LTISystem`, FD or JAX), controllability/observability, equilibria, `modal`, selected-channel Bode; more frequency tools planned |
| `planning/` | problems, trajopt |
| `optimization/` | `MathematicalProgram`, `Optimizer` (generic NLP) |
| `identification/` | fit parametric systems to data (planned; physical params and NN weights are the same verb) |
| `graphical/` | signals, phase plane, diagrams, animation |
| `interfaces/` | gymnasium, cosimulation, external-model wrappers (planned) |

**Quarantine** — experimental (TRL < 3); nothing may import these:

| Package | Role |
| --- | --- |
| `symbolic/` | experimental symbolic mechanics (SymPy EoM derivation) |

### Dependency law

- Libraries import only `core`, plus `dynamics/abstraction` interfaces —
  never catalog content. (The abstraction modules are the shared mathematical
  bases of the library band: `blocks/` builds LTI wiring on them, `control/`
  computed torque, `estimation/` EKFs.)
- Libraries may ship **factories for their own blocks** with array-in /
  block-out signatures (`control.lqr(A, B, Q, R) -> StateFeedback`,
  `estimation.kalman_design(A, C, Q, R) -> KalmanFilter`); the linearization
  producing those arrays lives in `analysis/`.
- Tools import `core`; they may consume libraries in demos and benchmarks.
- `graphical/` is imported lazily from anywhere; rendering stays optional.
- Quarantined packages are imported by nothing.

### Placement algorithm

1. New `System` subclass → shelf by diagram role: wiring → `blocks/`, plant →
   `dynamics/catalog/`, control law → `control/`, estimator → `estimation/`.
2. New verb → tool: integrate time (`simulation`), characterize (`analysis`),
   find inputs/policies (`planning`), solve NLPs (`optimization`), fit to data
   (`identification`), render (`graphical`), talk to another ecosystem
   (`interfaces`).
3. Neither, and unproven → quarantine at top level with a TRL tag.

Student-facing taxonomy: wiring blocks come from `blocks/`, plants from
`dynamics/`, controllers from `control/`, and everything is a `System`.
`blocks/` holds plant-agnostic wiring primitives; `dynamics/catalog/equations/`
holds canonical textbook ODEs (integrator chains, `VanderPol`) with graphics,
labels, and bounds for teaching demos — the name overlap (`Integrator` vs
`SimpleIntegrator`) is intentional given those roles.

### Scope: continuous time only

Minilink is continuous-time only by decision (June 2026). Digital control and
discrete dynamics (ZOH/delay blocks, sampled controllers, RNNs, mixed-rate
simulation) are out of scope; the framework may assume continuous-time `f`.
If discrete time ever enters scope, it is a framework design project on
`System` and `core/compile/` scheduling — not an incremental patch.

**Dynamics root:** `DynamicSystem` with `f`, `h`. Reusable bases in
`dynamics/abstraction` (`StateSpaceSystem`, `LTISystem`, `MechanicalSystem`,
`GeneralizedMechanicalSystem`). `StateSpaceSystem` builds its matrices through
methods `A(t, params)`, `B(t, params)`, `C(t, params)`, `D(t, params)` (so
`dx = A(t, params) @ x + B(t, params) @ u`), mirroring `MechanicalSystem.H(q,
params)`; subclasses assemble matrices from `params`. `LTISystem` is the
constant-matrix convenience built from `A, B, C, D` arrays (introspect via
`sys.A()`). Catalog names: `Pendulum`, `CartPole`,
`DynamicBicycle`. Mixed inputs → named ports + concrete allocation hooks; no
`WithPositionInputs` inheritance branches.

## 4. Core Object Contracts

### `System`

- **Math:** `f(x,u,t,params)`, `h(x,u,t,params)`; ports use same signature.
- **Dims:** `n` from constructor; `m` from input ports; `p` from primary output
  `"y"` only (aux `"x"` does not change `p`; no `"y"` ⇒ `p==0`).
- **Ports:** explicit, ID-first; infer `dim` from metadata or default 1. Extract
  slices with `get_port_values_from_u(u, "r", "y")`.
- **DynamicSystem shortcut:** `input_dim`, `output_dim`, `expose_state`,
  `y_dependencies` create standard `u`/`y`/`x`.
- **Control naming:** `r` reference, `y` measurement, `u` control.
- **Visualization contract:** `get_kinematic_geometry`,
  `get_kinematic_transforms`, `get_dynamic_geometry`, `get_camera_transform`
  are part of the core `System` contract in `core/system.py` (graphical
  primitives imported lazily; API still under review).
- **Facades:** user shortcuts only (lazy simulation/graphics); defined on the
  `core.facades.SystemFacades` mixin so `core/system.py` keeps the math,
  port, and visualization contracts. `self.traj` is a convenience cache of
  the latest facade rollout; library code never reads it as an input.

### Native-array equation rule

Applies to `f`, `h`, port compute, sets, costs, transcriptions, `MathematicalProgram`
`J`/`h`/`g`. Native in, native out; no `np.asarray` / `float()` inside equation
paths. Convert at boundaries (evaluators, solvers, plotting, `Trajectory`, I/O).

### Parameters

- `params is None` → `self.params`; any other value overrides (never
  `params or self.params`).
- **Diagram params are nested by subsystem id**: `{"plant": {…}, "ctl": {…}}`.
  `DiagramSystem.params` is a live-view property — the getter assembles
  `{sys_id: subsystem.params}` from live references (subsystems stay the
  single source of truth), the setter distributes by sys_id. Partial dicts are
  allowed (missing sys_id → that block's live `self.params`); unknown sys_ids
  raise; per-subsystem dicts are full replacements at the block level. Nested
  diagrams nest the dict recursively.
- Compile: `bind_params=True` copies params into the plan (frozen tier only).
  The parametric tier (`f_p`/`h_p`/`outputs_p`) takes the nested dict on both
  backends and ignores `bound_params`. On JAX the dict is a pytree argument
  (numeric leaves required): values vary without retracing, and
  `jacobian_f_params` / `jax.grad` differentiate dynamics w.r.t. parameters
  (see `examples/scripts/identification/demo_params_gradient.py`).

### `DiagramSystem`

Composes subsystems by named ports; flattens state; compiled `ExecutionPlan` is
the main execution path (reference recursive path must stay equivalent).
`connect()` validates port existence and dimensions at wiring time and is
quiet by default (`connection_verbose=False`; set `True` for one line per connection).

Shortcuts (`core.composition`): `+` flat add only, `>>` series, `@` closed loop,
`autowire()` conservative fill. Diagram operands are flattened, not nested.
Explicit `add_subsystem` / `connect` remains canonical for general topology.

### `Trajectory`, sets, costs

- `Trajectory`: `t (N,)`, `x (n,N)`, `u (m,N)`, optional `signals`; NumPy reporting object.
- Sets: `margin ≥ 0` feasible; `contains`/`sample` may convert to NumPy.
- Costs: `g(x,u,t)`, `h(x,t)` on `CostFunction` in `core`; attach to
  `PlanningProblem`, not the plant.

## 5. Compilation And Simulation

`compile(system, backend)` → `DynamicsEvaluator` (`numpy`|`jax`|`auto`|`direct`).

Diagrams → `ExecutionPlan` → diagram evaluator. Internal outputs via
`reconstruct_internal_signals`; **`outputs()` / `outputs_p()` are boundary outputs
only** (not diagram internals). Keep `ExecutionPlan.output_slices` and
`external_output_slices` aligned. Do not reintroduce `compute_outputs(..., ports=...)`.

**Default sim API:** `compute_trajectory` / `compute_forced` → `Simulator` →
`Trajectory`. Unconnected inputs use port nominals; time-varying sources belong
in the diagram; forcing via `compute_forced`. Facades default
`compile_backend="numpy"`.

Solver presets: `scipy`, `scipy_stiff`, `scipy_max`, `scipy_ultra`, `scipy_lsoda`,
`euler`, `rk4_fixedsteps` (auto-picked when omitted). Planned: `SimulationOptions`
([ROADMAP.md](ROADMAP.md) P1).

## 6. Optimization And Planning

**NLP:** `minimize J(z)` s.t. `h=0`, `g≥0`, bounds. Pure `MathematicalProgram`;
`Optimizer` binds method preset (`scipy_slsqp`, `scipy_trust_constr`, `ipopt`).

**Planning:** `PlanningProblem` owns system, sets, cost. `X0`/`Xf` authoritative;
`x_start`/`x_goal` are shortcuts/representative points.

**Trajopt:** planner → transcription → `MathematicalProgram` → `Optimizer` →
`Trajectory`. Single backend-native transcription classes; no parallel JAX
transcription types. Transcriptions compile the system (`numpy`/`jax`) and
route `problem.params.system` through the parametric tier `f_p`;
`compile_backend="direct"` calls `system.f` uncompiled (escape hatch).
A `MathematicalProgram` carries the native backend of its callables in its
`backend` field, and the `Optimizer` compiles with it by default.

## 7. Graphics And Benchmarks

Facades delegate to `graphical/`. Time plots: `signals=("x", "u", "block:port")`.
Phase plane: matplotlib default. Diagrams: Graphviz display, Mermaid export;
Plotly under `plotting` extra.

**Camera:** `get_camera_transform` → 4×4 matrix (`camera_matrix`); one contract
for all renderers. Override on `System` for custom views. Camera and kinematic
hooks are still under graphical/animation API review.

All performance benchmarking lives in repo-root `benchmarks/` (helpers,
synthetic fixtures, `run_*` scripts) — outside the shipped package, importing
minilink like an external user, and not a public contract.

**Repo conventions:** Python 3.10+; typed public APIs (except equation paths,
which keep bare signatures per agent.md Textbook Style); lazy optional imports;
namespace `__init__.py` files; plot subpackages may re-export small facades.
