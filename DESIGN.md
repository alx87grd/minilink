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
| `core/` | `System` (+ `SystemFacades` mixin), `DiagramSystem`, signals/ports (`signals.py`), backend policy & helpers (`backends.py`), `Trajectory`, sets, costs, geometry (`geometry.py`) |
| `core/compile/` | `ExecutionPlan`, compiler, NumPy/JAX evaluators |

**System libraries** — `System` subclasses you drop into a diagram, shelved by
*role in the diagram*, never by implementation technology (linear, Lagrangian,
or neural network alike):

| Package | Role |
| --- | --- |
| `blocks/` | plant-agnostic wiring: sources, `Integrator`, `TransferFunction`, routing (`Sum`/`Gain`/`Mux`/`Demux`), nonlinear (`Saturation`/`DeadZone`/`Relay`), filters, neural (`NeuralNetwork`) |
| `dynamics/` | plants: `abstraction/` mother classes, `catalog/` by physical domain, `engines/` plant-generating kernels (experimental) |
| `control/` | control laws and design factories (`ProportionalController`, `ImpedanceController`, `ImpedanceIntegralController`, `FilteredController`, `StateFeedbackController`, `lqr`) |
| `estimation/` | online state and parameter estimators (planned) |

**Tools** — verbs on a `System`; they return data or plots and never define
user-facing systems (factories are fine: `linearize_matrices()` returns arrays,
`linearize()` wraps them as an `LTISystem`, and an LQR design function returns a
state-feedback block):

| Package | Role |
| --- | --- |
| `simulation/` | `Simulator`, solvers, forcing |
| `analysis/` | `linearize_matrices` (→ arrays), `linearize` (→ `LTISystem`, FD or JAX), controllability/observability, equilibria, `modal`, selected-channel Bode; more frequency tools planned |
| `planning/` | problems, trajopt, `spatial/` (scenes), `search/` (RRT) |
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
- **Visualization contract:** keyed `get_kinematic_geometry`, `tf`,
  `get_dynamic_geometry` are part of the core `System` contract in
  `core/system.py` (graphical primitives imported lazily). Camera hints
  (`camera_target`, `camera_scale`, `camera_follow_frame`, …) are resolved by
  the animator via `resolve_camera_from_hints`; custom views use
  `animate(camera=…)`.
  **`tf` returns only computed frames** (body, joints, axles, …); **`"world"` is
  implicit** — the animator injects identity so world-fixed geometry can key to
  `"world"` without every plant returning `"world": I`. In **diagrams**, `"world"`
  stays unprefixed (one shared root); articulated frames are namespaced
  (``vehicle:body``).
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
- **Planning params tiers** (`ProblemParameters`): `system`, `cost`, `sets` today;
  a future `scene` tier for spatial overrides is on the roadmap. **Deferred**
  ([ROADMAP.md §5.5](ROADMAP.md#55-planning)): call-time overrides on base
  `Shape`, `Set`, and `CostFunction` primitives in `core/` — those types declare
  `(t, params)` but still read frozen attributes only until a follow-up pass.

### `DiagramSystem`

Composes subsystems by named ports; flattens state; compiled `ExecutionPlan` is
the main execution path (reference recursive path must stay equivalent).
`connect()` validates port existence and dimensions at wiring time and is
quiet by default (`connection_verbose=False`; set `True` for one line per connection).

Shortcuts (`core.composition`): `+` flat add only, `>>` series, `@` closed loop
(with ``closed_loop(..., feedback="auto"|"y"|"qdq")`` and ``closed_loop_qdq``),
`autowire()` conservative fill — **never inserts Mux**; use `feedback="qdq"` or
`closed_loop_qdq` for explicit `Mux(q, dq)` wiring. Diagram operands are flattened,
not nested. Explicit `add_subsystem` / `connect` remains canonical for general topology.
Visualization: subsystem `"world"` geometry merges into one shared diagram
`"world"` frame; only articulated frames get `{sys_id}:` prefixes.

### Control feedback profiles

Controllers in `control/` are grouped by **feedback profile** (module layout +
optional class attribute `feedback_profile`, not inheritance):

| Profile | Module | Measurement |
| --- | --- | --- |
| `output` | `output.py` | `y` (static output error) |
| `impedance` | `impedance.py` | `y = [pos; rate]` dim `2n` |
| `state` | `state.py` | full state `x` |
| `siso` | `siso.py` | `y` dim `n` only (decoupled loops) |
| `impedance` | `impedance.py` | `[pos; rate]` on `y`; optional robotic `+ g(q)` via `robotic.py` |
| `task` | `robotic.py` | Joint ``[q; dq]`` feedback; internal FK/J; optional ``+ g(q)`` |

Model-based laws (`ComputedTorqueController`, …) live in `modelbased.py` with
tag `modelbased`. See [`docs/plans/robot-control-stack.md`](docs/plans/robot-control-stack.md).

### `Trajectory`, sets, costs, geometry

- `Trajectory`: `t (N,)`, `x (n,N)`, `u (m,N)`, optional `signals`; NumPy reporting object.
- Sets: `margin ≥ 0` feasible; `contains`/`sample` may convert to NumPy. Compose with
  `&` → `IntersectionSet`. `margin(z, t, params)` is threaded by transcriptions via
  `problem.params.sets`; field-backed spatial sets forward that same parameter
  object to their scene queries. Base `Set` subclasses other than `FieldSet` /
  `CallableSet` do not yet read `params` (deferred — see ROADMAP).
- Costs: `g(x,u,t)`, `h(x,t)` on `CostFunction` in `core`; attach to
  `PlanningProblem`, not the plant. Compose with `+` → `SumCost` and `*` →
  `ScaledCost` (e.g. `base + w * obstacle_cost`). `g`/`h` receive
  `problem.params.cost`; `FieldCost` forwards that parameter object to its
  underlying spatial field. Built-in costs such as `QuadraticCost` do not yet
  read `params` (deferred).
- Geometry (`geometry.py`): `Shape.sdf(p)` is the signed distance to a workspace
  *solid* — `< 0` inside (occupied), the dual of an allowable `Set` (`margin ≥ 0`).
  Primitives `Sphere`/`Box`/`Union`/`Inflated`; native-array math path (NumPy and
  JAX-traceable). `sdf(p, t, params)` is threaded by the spatial scene pipeline;
  primitives still use frozen dataclass fields only until parametric shapes land
  (deferred). Foundation for hard obstacles in a :class:`~minilink.planning.spatial.scene.Scene`,
  exported as a free-space `Set` (hard constraint) or soft `CostFunction`.
- **Sign convention** (shared): nonnegative ⇒ feasible/free — `Shape.sdf > 0` outside,
  `ClearanceField.value ≥ 0` collision-free, `Set.margin ≥ 0` feasible. The core stores
  signed *physical* quantities (clearance in length units, density in cost units), never a
  bounded `[0,1]` score: an SDF keeps a well-scaled gradient everywhere, which a squashed
  occupancy would lose. Normalized/occupancy views are derived at the **edge** via
  `as_cost(shaping=...)`, not stored in the field.

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

**Policy synthesis** (`planning/policy_synthesis/`): offline dynamic programming on a
continuous plant. A `StateSpaceGrid` discretizes the `PlanningProblem` — grid *extent*
comes from a `BoxSet`/`BoxInputSet` (so it stays finite), grid *validity* from
`X.contains`/`U.contains` (so `X = bounds & free` still works), and successors from a
forward-Euler step `x_next = x + f(x,u,t)·dt` (the time step `dt` lives on the grid, not
on `System`). `DynamicProgrammingPlanner` runs value iteration backward — `compute_solution`
to tolerance, `solve_steps` for a fixed horizon — returning a cost-to-go `J` and greedy policy
`pi` (action ids); out-of-domain transitions are charged a finite `out_of_bound_cost` (pyro's
`cf.INF`). Three interchangeable backward-step backends share this workflow: `loop` (per-node
Python, pyro's reference), `numpy` (vectorized over the precomputed lookup table, the default),
and `jax` (the same backup as one jitted `lax.while_loop` with `map_coordinates`, built on a
NumPy precompute so any plant works; linear/nearest only). `precompute` trades the `(N,A,n)`
successor table for per-sweep recomputation (memory vs time-varying support). `result.controller()`
returns a `LookupTableController` (a `StaticSystem`, so `controller >> plant` simulates);
`PolicyEvaluator` gives the cost-to-go of any fixed law. Benchmark: `benchmarks/run_dp_backends.py`.

**Spatial scene** (`planning/spatial/`): two domains — **workspace** `p ∈ ℝ²/ℝ³` and
**state** `x`. On W: hard `Shape` obstacles and soft `WorkspaceField` sources live in
`Scene` (`obstacles`, `workspace_fields`). On X: `StateField.value(x)` fuses the robot
placement with scene queries (`clearance_field`, `cost_field`). Export separately —
`clearance_field(body)` for collision (hard `Set` or barrier `CostFunction`) and
`cost_field(body)` for terrain (soft `CostFunction` or hard band via
`as_constraint(upper=...)`). `StateField.value(x)` is a **scalar** (min clearance, max
density over body probes). **Collision reuse:** frameless geometry (`disc`,
`car_outline`, `point_probe`) binds to the **planner** plant with
`bind(sys, geometry, frame="body")`; world probes use ``sys.tf(x,u,t)[frame]``
via :func:`~minilink.core.kinematics.apply` — the same FK as rendering. Frameless
geometry: :func:`~minilink.planning.spatial.collision.disc`,
:func:`~minilink.planning.spatial.collision.point_probe`,
:func:`~minilink.planning.spatial.collision.car_outline`. Shape obstacles with
`quadratic_hinge`, `inverse_barrier`). Compose at `PlanningProblem`:
`X = bounds & free`, `cost = base + w * terrain`. Scene param overrides (moving
obstacles, MPC sweeps) are planned on the roadmap — rebuild `Scene` until then.

**Reference track** (`planning/spatial/paths.py`, `track.py`): workspace centerlines
from waypoint polylines via `from_waypoints` (default `kind="polyline"`), wrapped in
`ReferenceTrack(path, half_width)`. Same export pattern as obstacles —
`distance_field(robot).as_cost(shaping=quadratic_excess)` for soft path following,
`corridor_field(body).as_constraint(lower=0)` for a hard tube. Probe semantics match
clearance (subtract body radius). Compose with obstacles:
`X = bounds & scene.clearance_field(body).as_constraint() & track.corridor_field(body).as_constraint()`.

**Search / RRT** (`planning/search/`): `RRTPlanner(Planner)` owns the invariant loop and
sources every concern from the problem — collision `problem.X.contains` (optional
orchestrator `edge_resolution` densification along edges), goal `problem.Xf`/`x_goal`,
free-space sampling from `problem.X` (direct `Set.sample` or rejection from state
bounds), dynamics `problem.sys.f` — so the system stays pure. After
`compute_solution()`, `reached_goal` and `solution_node` report success vs
best-effort fallback (`return_best_effort`). The two swappable pieces are an injected
`TrajectoryExtender` (`propose(from, toward, problem, rng) → Iterable[Edge]`:
`KinodynamicExtender` forward-integrates controls, `SteeringExtender` connects exactly
via a `SteeringFunction` including `DubinsSteering`) and a `metric(a,b)` callable
(nearest-neighbour distance). Every system is an ODE, so an `Edge` always carries real
`(t,x,u)`; `compute_solution() → Trajectory`. `RRTStarPlanner` extends the attach step
with near-neighbour parent selection and cost-based rewiring (`Tree.rewire`,
`Tree.propagate_cost`); `Edge.cost` is the cost-to-come along the tree. With
`optimize_after_goal`, the search continues after the first goal connection until
the best goal cost stops improving for `convergence_patience` extensions;
`record_history` + `animate_convergence` replay tree growth and path refinement.
``live_plot`` / ``callback`` redraw the tree during ``compute_solution`` (pyro-style);
``live_plot_after_goal_only`` limits updates to the RRT* post-goal convergence phase.
``RRTOptions.nearest_backend`` selects brute-force or SciPy ``cKDTree`` nearest/near
queries (Euclidean L2 only — requires ``metric=euclidean``); see
``benchmarks/run_rrt_nearest_backends.py``. Modest
speedups on low-D obstacle scenes are expected when collision checking and
post-goal tree scans dominate.

## 7. Graphics And Benchmarks

Facades delegate to `graphical/`. Time plots: `signals=("x", "u", "block:port")`.
Phase plane: matplotlib default. Diagrams: Graphviz display, Mermaid export;
Plotly under `plotting` extra.

**Camera:** plain `camera_*` hints on `System` resolve to a 4×4 matrix
(`camera_matrix`) each frame via `resolve_camera_from_hints`; pass
`animate(camera=…)` for a constant matrix or callable override. One contract
for all renderers.

All performance benchmarking lives in repo-root `benchmarks/` (helpers,
synthetic fixtures, `run_*` scripts) — outside the shipped package, importing
minilink like an external user, and not a public contract.

**Repo conventions:** Python 3.10+; typed public APIs (except equation paths,
which keep bare signatures per agent.md Textbook Style); lazy optional imports;
namespace `__init__.py` files; plot subpackages may re-export small facades.
Agents and maintainers run tests in the **`minilink`** conda env from
[environment.yml](environment.yml) ([agent.md §9](agent.md#9-local-environment)).
