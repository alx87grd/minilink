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

Contributing style (textbook rules, workflow): [AGENTS.md](AGENTS.md).

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
| `core/` | `System` (+ façade mixins: `SharedSystemFacades`, `DynamicSystemFacades`, `StepSystemFacades`), `DiagramSystem` (subclasses `DynamicSystem`), shared diagram wiring (`wiring.py`: `WiredDiagramMixin`, gather, topology checks), signals/ports (`signals.py`), backend policy & helpers (`backends.py`), `Trajectory`, sets, costs, geometry (`geometry.py`) |
| `core/compile/` | `ExecutionPlan`, compiler, NumPy/JAX evaluators |

**System libraries** — `System` subclasses you drop into a diagram, shelved by
*role in the diagram*, never by implementation technology (linear, Lagrangian,
or neural network alike):

| Package | Role |
| --- | --- |
| `blocks/` | plant-agnostic wiring: sources, `Integrator`, `TransferFunction`, routing (`Sum`/`Gain`/`Mux`/`Demux`), nonlinear (`Saturation`/`DeadZone`/`Relay`), filters, neural (`NeuralNetwork`) |
| `dynamics/` | plants: `abstraction/` mother classes, `catalog/` by physical domain, `engines/` plant-generating kernels (experimental) |
| `control/` | control laws and design factories (`linear.py`, `pid.py`, `lqr.py`, `impedance.py`, `output.py`, `state.py`, `siso.py`, `modelbased.py`, `robotic.py`) |
| `estimation/` | online state and parameter estimators (planned) |

**Tools** — verbs on a `System`; they return data or plots and never define
user-facing systems (factories are fine: `linearize_matrices()` returns arrays,
`linearize()` wraps them as an `LTISystem`, and an LQR design function returns a
state-feedback block):

| Package | Role |
| --- | --- |
| `simulation/` | `Simulator`, `StaticSimulator`, `Computer`, `StepSchedule`, `HybridSimulator`, solvers, forcing |
| `analysis/` | `linearize_matrices` (→ arrays), `linearize` (→ `LTISystem`, FD or JAX), controllability/observability, equilibria, `modal`, selected-channel Bode; `discretize` for continuous→step plant wrappers; more frequency tools planned |
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

### Continuous-time core; step/hybrid subsidiary

Minilink's **primary framework** is continuous-time: `DynamicSystem`, flow
`DiagramSystem`, `compile` → `DynamicsEvaluator`, `Simulator`, and analysis on
`f`. Plants, control design, teaching, and most library growth target this path.

**Step and hybrid** are a **narrow parallel add-on** — not a second framework of
equal weight. They exist so discrete control laws (MPC, SMC, sampled regulators)
can close the loop on a continuous plant without hand-rolled outer `while` loops.
can close the loop on a continuous plant without hand-rolled outer `while` loops.
(subset only — not full Simulink / discrete-dynamics parity).

**Design trade-off rule:** when step or hybrid work conflicts with continuous-time
clarity, **prefer the continuous core**. Add-ons must stay on **sibling types and
separate compile/sim paths** (`StepSystem` beside `DynamicSystem`, not a flag on
`f`; `HybridDiagram` as two-side glue, not a merged heterogeneous diagram) so
flow APIs, evaluators, and `DiagramSystem` behavior remain unchanged. 

*Rationale:* We rejected adding a `time_domain` flag to `DynamicSystem` or inheriting from it because it reverses the mathematical meaning of `f` (from $dx$ to $x_{new}$), breaks textbook ODE clarity, and confuses analysis tools. Evolution kind must always be determined by type (`isinstance(sys, StepSystem)`), not a solver flag. Do not fold
discrete scheduling, sample-time metadata, or mixed `f`/`step` semantics into
`DynamicSystem`, flow `compile()`, or `Simulator` unless there is a clear
flow-side benefit.

**Dynamics root (continuous):** `DynamicSystem` with `f`, `h`. **Discrete leaf (subsidiary):**
:class:`StepSystem` with `step`, `h` and integer index `k` — no `f`, no wall clock on the
leaf. Reusable bases in
`dynamics/abstraction` (`StateSpaceSystem`, `LTISystem`, `MechanicalSystem`,
`GeneralizedMechanicalSystem`). `StateSpaceSystem` builds its matrices through
methods `A(t, params)`, `B(t, params)`, `C(t, params)`, `D(t, params)` (so
`dx = A(t, params) @ x + B(t, params) @ u`), mirroring `MechanicalSystem.H(q,
params)`; subclasses assemble matrices from `params`. `LTISystem` is the
constant-matrix convenience built from `A, B, C, D` arrays (introspect via
`sys.A()`). Catalog names: `Pendulum`, `CartPole`,
`DynamicBicycle`. **`Manipulator`** (`MechanicalSystem` + task ports `p`, `pdot`;
methods `forward_kinematics`, `J`, `inverse_kinematics`) is the catalog base for
serial arms. Joint impedance / task impedance / computed torque use
``closed_loop_qdq`` or ``feedback="auto"``; see `control/robotic.py` and
`control/modelbased.py`. Mixed inputs → named ports + concrete allocation hooks; no
`WithPositionInputs` inheritance branches.

## 4. Core Object Contracts

### `System`

- **Math:** `h(x,u,t,params)` on the model and port `compute` functions; continuous
  evolution `f(x,u,t,params)` is on :class:`DynamicSystem` only (and stacked on
  :class:`DiagramSystem`).
- **Dims:** `n` defaults to 0 (static IO shell); `m` from input ports; `p` from primary output
  `"y"` only (aux `"x"` does not change `p`; no `"y"` ⇒ `p==0`).
- **Ports:** explicit, ID-first; infer `dim` from metadata or default 1. Extract
  slices with `get_port_values_from_u(u, "r", "y")`.
- **DynamicSystem shortcut:** `input_dim`, `output_dim`, `expose_state`,
  `y_dependencies` create standard `u`/`y`/`x`.
- **StepSystem (discrete leaf):** same port shortcut as `DynamicSystem`; evolution is
  `step(x, u, k, params)` → `x_new`. Facades: `compute_rollout` / `plot_rollout`;
  cache `self.rollout`.
- **Hybrid (computer + plant):** :class:`~minilink.core.hybrid_diagram.HybridDiagram`
  bundles :class:`~minilink.simulation.computer.Computer` (step side + schedule) and a
  continuous :class:`DiagramSystem` plant. Boundary channels use ZOH (computer → plant)
  or sample (plant → computer). :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`
  mirrors :class:`~minilink.simulation.simulator.Simulator` (`t0`/`tf`, `solve`,
  `solve_forced`); plant steps use
  :meth:`~minilink.core.compile.evaluators.numpy_evaluators.IntegrationMixin.integrate_zoh`
  with optional ``plant_dt_inner`` for sub-step integration and plant trajectory
  recording. :class:`~minilink.simulation.hybrid_simulator.HybridSimResult` holds
  ``computer`` (:class:`~minilink.core.step_rollout.StepRollout`, tick ``k``) and
  ``plant`` (:class:`~minilink.core.trajectory.Trajectory`, wall time ``t``); default
  plots use ``plant``.
  Shortcuts: ``block % schedule`` → :class:`~minilink.simulation.computer.Computer`;
  ``Computer @ plant`` and :func:`~minilink.core.hybrid_composition.hybrid_closed_loop`
  (same port auto-wiring as continuous ``ctl @ plant`` via
  :func:`~minilink.core.composition.resolve_standard_feedback`);
  :meth:`~minilink.planning.mpc.controller.MPCStatelessController.export_to_computer` /
  :meth:`~minilink.planning.mpc.step_block.MPCStatefulController.export_to_computer` for warm-start MPC.
  Catalog plant :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycleRateInputsUY`
  exposes standard ``u`` / ``y`` ports for hybrid composition.
  Facades: :meth:`~minilink.core.hybrid_diagram.HybridDiagram.compute_trajectory`,
  :meth:`~minilink.core.hybrid_diagram.HybridDiagram.compute_forced`, and
  :meth:`~minilink.core.hybrid_diagram.HybridDiagram.plot_trajectory` /
  :meth:`~minilink.core.hybrid_diagram.HybridDiagram.animate` cache the plant
  :class:`~minilink.core.trajectory.Trajectory` on ``self.traj`` (same quick-access
  pattern as continuous diagrams) and the full
  :class:`~minilink.simulation.hybrid_simulator.HybridSimResult` on ``self.last_result``;
  tick-indexed computer view on ``self.rollout``.
  Visualization: :meth:`~minilink.core.hybrid_diagram.HybridDiagram.plot_diagram` renders Plant +
  Computer (nested StepDiagram) clusters with dashed ZOH/sample boundary edges;
  :func:`~minilink.graphical.diagrams.build_hybrid_topology` /
  :func:`~minilink.graphical.diagrams.export_hybrid_topology` for Graphviz or Mermaid export.
  Default ``abstract_boundary=True`` collapses diagram external Inputs/Outputs routing nodes
  and anchors hybrid edges on wired subsystem ports.
  and anchors hybrid edges on wired subsystem ports.
- **MPC hybrid block (Phase 6a–6b):** :class:`~minilink.planning.mpc.controller.MPCStatelessController`
  is a static ``System`` (``n=0``) leaf: one :meth:`~minilink.planning.mpc.planner.MPCPlanner.step`
  per Computer tick (memoized across output ports ``u_ff``, ``x_ff``, ``z``).
  :class:`~minilink.planning.mpc.step_block.MPCStatefulController` (Phase 6b) is a
  :class:`StepSystem` with packed decision state ``z``; warm-start via
  :func:`~minilink.planning.mpc.warm_start.mpc_warm_start_guess` from
  ``Computer.x``.   Post-sim horizons:
  :func:`~minilink.planning.mpc.plan_reconstruct.mpc_plans_from_rollout`;
  default animation overlays:
  :func:`~minilink.planning.mpc.animation_overlays.mpc_animation_overlays`.
  :func:`~minilink.planning.mpc.animation_overlays.mpc_animation_overlays`.
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
- **Facades:** user shortcuts only (lazy simulation/graphics); split across
  `core.facades` mixins — `SharedSystemFacades` on `System` (compile, static
  `compute_trajectory`, `plot_trajectory`, `animate`, …),
  `DynamicSystemFacades` on `DynamicSystem` (continuous `compute_trajectory`,
  analysis plots, `game`), `StepSystemFacades` on `StepSystem`
  (`compute_rollout`). **MRO** picks `compute_trajectory` implementation; no
  façade-layer `isinstance` routers. `self.traj` is a convenience cache of
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

Subclasses :class:`~minilink.core.system.DynamicSystem` (diagrams are continuous
evolution systems with a compile fast path). Composes subsystems by named ports;
flattens state; compiled `ExecutionPlan` is the main execution path (reference
recursive path must stay equivalent).
Wiring, port gather, params nesting, and `check_algebraic_loops` live on
`WiredDiagramMixin` in `wiring.py`; `DiagramSystem` adds flow-only `f`,
`compile`, and `reconstruct_internal_signals`. Public `DiagramSystem` API is
unchanged (methods inherited from the mixin).
`connect()` validates port existence and dimensions at wiring time and is
quiet by default (`connection_verbose=False`; set `True` for one line per connection).

Shortcuts (`core.composition`): `+` flat add only, `>>` series, `@` closed loop
(with ``closed_loop(..., feedback="auto"|"y"|"qdq")`` and ``closed_loop_qdq``),
`autowire()` conservative fill — **never inserts Mux**; use `feedback="qdq"` or
`closed_loop_qdq` for explicit `Mux(q, dq)` wiring. Diagram operands are flattened,
not nested. Explicit `add_subsystem` / `connect` remains canonical for general topology.

**Shortcut subsystem ids** default by role: `ref` (sources), `ctl` (controllers),
`sys` (stateful plants), with numeric suffix on collision (`sys2`, …). Override
with ``System.id`` before wiring or explicit ``add_subsystem(..., "plant")``.
Block titles in ``plot_diagram()`` still show ``sys.name`` (human type).
:func:`~minilink.graphical.diagrams.build_diagram_topology` accepts
``abstract_boundary=True`` to omit external Inputs/Outputs routing nodes and record
``boundary_inputs`` / ``boundary_outputs`` port anchors (used by hybrid export).

Visualization: subsystem `"world"` geometry merges into one shared diagram
`"world"` frame; only articulated frames get `{sys_id}:` prefixes.

### Control feedback profiles

Controllers in `control/` are grouped by **feedback profile** (module layout +
optional class attribute `feedback_profile`, not inheritance):

| Profile | Module | Measurement |
| --- | --- | --- |
| `output` | `output.py` | `y` (static output error) |
| `impedance` | `impedance.py` | `y = [pos; rate]` dim `2n`; optional robotic `+ g(q)` via `robotic.py` |
| `state` | `state.py` | full state `x` |
| `siso` | `siso.py` | `y` dim `n` only (decoupled loops) |
| `task` | `robotic.py` | Joint ``[q; dq]`` feedback; internal FK/J; optional ``+ g(q)`` |
| `kinematic` | `robotic.py` | Joint ``q`` only; outputs ``dq`` for speed-controlled plants |
| `modelbased` | `modelbased.py` | ``y = [q; dq]``; computed torque; Pyro sliding mode ``τ = ID(q,dq,ddq_r) - K(q) sign(s)`` |

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

`compile(system, backend)` returns a typed evaluator:

- :class:`DynamicSystem` leaf → :class:`~minilink.core.compile.evaluators.evaluators.DynamicsEvaluator`
  (`NumpyDynamicEvaluator` / `JaxDynamicEvaluator`)
- :class:`StepSystem` leaf → :class:`~minilink.core.compile.evaluators.evaluators.StepEvaluator`
  (`NumpyStepEvaluator` / `JaxStepEvaluator`) — `.step`, `.outputs`, state-only `.rollout`; no `.f`
- static :class:`System` leaf (`n=0`) → :class:`~minilink.core.compile.evaluators.evaluators.StaticEvaluator`
  (`NumpyStaticEvaluator` / `JaxStaticEvaluator`) — `.outputs` only, no `.f`
- :class:`DiagramSystem` → diagram evaluator (same dynamics tier as above)

Leaf `compile` accepts `numpy` or `jax` only (`auto` / `direct` are simulator /
transcription concepts — see :mod:`minilink.core.backends`).

Diagrams → `ExecutionPlan` → diagram evaluator. Internal outputs via
`reconstruct_internal_signals`; **`outputs()` / `outputs_p()` are boundary outputs
only** (not diagram internals). Compiled evaluators expose **`outputs` / `outputs_p`**
(dict keyed by port id); they do not mirror model `h` as a separate evaluator API.

**Evaluator execution tiers (JAX).** Default methods (`.f`, `.f_p`, `.outputs`, …)
are the **fast tier** — JIT-compiled on JAX, eager on NumPy. JAX evaluators also
expose a **trace tier** (`.f_trace`, `.f_trace_p`, `.rk4_step_trace`, …):
pre-JIT flat callables for composition inside outer `jit` / `grad` / `vmap`
(identification losses, C export). Optional `_jit` aliases (`.f_jit` ≡ `.f`)
document the fast tier. NumPy evaluators reject `*_trace` / `*_jit` attributes.
`has_trace_tier` is `True` on JAX evaluators. Details:
[docs/plans/evaluator-trace-tier-api.md](docs/plans/evaluator-trace-tier-api.md).

|  | Bound | Parametric |
| --- | --- | --- |
| Fast (default) | `f` ≡ `f_jit` | `f_p` ≡ `f_jit_p` |
| Trace (JAX only) | `f_trace` | `f_trace_p` |

Same 2×2 for `outputs`, `step`, and integration helpers
(`rk4_step`, `rk4_integrate_zoh`, `rk4_integrate_linear`, `euler_integrate_*`, …)
on JAX dynamics evaluators. Layout:
`evaluators.py` (ABCs), `numpy_evaluators.py`, `jax_evaluators.py`,
`step_rollout.py` (`gather_u`, `StepRolloutMixin`).
Integration naming:
[docs/plans/evaluator-integration-api.md](docs/plans/evaluator-integration-api.md).

**Integration rename (pre-1.0).** Rollout methods use explicit integrator + input
model tokens — no bare `integrate`:

| Old | New |
| --- | --- |
| `integrate` | `rk4_integrate_zoh` |
| `integrate_p` | `rk4_integrate_zoh_p` |
| `rk4_integrate_forced` | `rk4_integrate_linear` |
| `rk4_integrate_forced_p` | `rk4_integrate_linear_p` |

Trace-tier JAX twins follow the same pattern (`*_trace`, `*_trace_p`). See
[docs/plans/evaluator-integration-api.md](docs/plans/evaluator-integration-api.md)
for the full grid.

Keep `ExecutionPlan.output_slices` and `external_output_slices` aligned. Do not
reintroduce `compute_outputs(..., ports=...)`.

**Default sim API:** `compute_trajectory` / `compute_forced` dispatch by **class MRO**
(façade mixins), enforced at simulator constructors:

- :class:`DynamicSystem` (including :class:`DiagramSystem`) →
  :class:`~minilink.simulation.simulator.Simulator` (ODE integration or diagram
  evaluator on a time grid)
- static :class:`System` leaf (`n=0`) →
  :class:`~minilink.simulation.static_simulator.StaticSimulator` (time grid +
  boundary outputs in `Trajectory.signals`; not state evolution)
- :class:`StepSystem` → `compile().rollout(...)` or `compute_rollout(n_steps=...)`
  (state-only :class:`~minilink.core.step_rollout.StepRollout`; signal logging via
  :class:`~minilink.simulation.computer.Computer` / :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`; not `Simulator`)
- :class:`~minilink.core.hybrid_diagram.HybridDiagram` →
  :class:`~minilink.simulation.hybrid_simulator.HybridSimulator` or façade
  `compute_trajectory` / `compute_forced` (hybrid :class:`~minilink.simulation.hybrid_simulator.HybridSimResult`)

`animate` / `plot_trajectory` live on `SharedSystemFacades` for continuous and static
systems; :class:`~minilink.core.hybrid_diagram.HybridDiagram` mirrors the pattern with
``self.traj`` (plant :class:`~minilink.core.trajectory.Trajectory`) and
``self.last_result`` (:class:`~minilink.simulation.hybrid_simulator.HybridSimResult`).
Auto-sim fallback calls `compute_trajectory` (MRO picks engine on homogeneous diagrams).

**Two static paths:** a static *leaf* (`Gain`, `Step` source) uses
`StaticSimulator` — empty state, outputs in `traj.signals`. A static-only
*diagram* (`n=0` stacked state) still subclasses `DynamicSystem` and uses
`Simulator` with the diagram evaluator (signal-flow on a time grid).

Unconnected inputs use port nominals; time-varying sources belong in the diagram;
forcing via `compute_forced`. Facades default `compile_backend="numpy"`.

Solver presets: `scipy`, `scipy_stiff`, `scipy_max`, `scipy_ultra`, `scipy_lsoda`,
`euler` (variable knot spacing), `euler_fixedsteps` (uniform grid via
`euler_integrate_*` rollouts), `rk4_fixedsteps` (auto-picked when omitted). Planned: `SimulationOptions`
([ROADMAP.md](ROADMAP.md) P1).

### Discontinuous closed loops — known issues

Controllers with discontinuous laws (e.g. :class:`~minilink.control.modelbased.SlidingModeController`
``sign(s)``) on a **continuous** :class:`DiagramSystem` closed loop are supported today,
but several solver/logging behaviors are misleading until a dedicated hybrid or
event-handling path lands ([ROADMAP.md](ROADMAP.md) §5.2).

**Algebraic feedback during integration.** Nominal closed-loop runs integrate
``f_ivp(x, t)`` — the diagram evaluator re-solves feedback at **every** call to
``f``. There is no sample-and-hold on the controller torque inside a step.

**Fixed-step RK4 (`rk4_fixedsteps`).** Each step evaluates ``f`` at four intermediate
states (k1–k4). ``u`` and ``sign(s)`` can **flip sign between sub-steps** while the
weighted RK4 update **nearly cancels**, leaving ``x`` almost unchanged on the output
grid. The plant can look stationary even though instantaneous dynamics at the grid
point imply large ``ddq``.

**What the trajectory records.** :class:`~minilink.core.trajectory.Trajectory` stores
``x`` (and boundary ``u``) on the **output time grid only**. It does **not** log
internal RK4 sub-step torques. :meth:`~minilink.core.diagram.DiagramSystem.reconstruct_internal_signals`
and ``plot_trajectory`` evaluate ``ctl:u`` and ``f(x, u, t)`` at those **grid states**
— equivalent to the k1/start-of-interval algebraic map, **not** the effective
piecewise dynamics integrated over ``[t_k, t_k + dt]``. Therefore:

- Reconstructed ``ctl:u`` can appear **smooth or constant** while sub-step torques oscillate.
- ``f``-based ``ddq`` at a grid point can **disagree** with ``Δdq/Δt`` from the stored
  state (especially under ``rk4_fixedsteps``).
- This is **not** a compile-backend bug; NumPy and JAX evaluators show the same pattern.

**SciPy adaptive solvers.** ``scipy`` / ``scipy_stiff`` / ``scipy_lsoda`` on the same
closed loop may stall, overflow, or take extreme substeps near switching (Zeno-like
behavior). Prefer **explicit Euler with a small ``dt``** or **hybrid** sampling for
discontinuous mechanical SMC demos.

**Hybrid contrast.** :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`
holds controller torque constant between computer ticks (ZOH) and samples plant outputs
at tick boundaries — the intended semantics for digital SMC. See
``examples/scripts/hybrid/demo_smc_pendulum_compare.py``.

**Diagnostics.** ``scratch/confirm_smc_solver_bug.py`` compares solvers, ``ddq_f`` vs
numerical ``Δdq/Δt``, and RK4 k1–k4 cancellation on the pendulum SMC demo.

**Mitigation (landed):**
— ``SlidingModeController`` sets ``discontinuous_behavior``; diagrams aggregate the flag;
auto ``select_solver`` picks **Euler** with finer default ``dt``; ``UserWarning`` on every
discontinuous solve (stronger when forcing ``rk4_fixedsteps`` / ``scipy_*``). Evolution
kind is class-type routing only — ``solver_info["continuous_time_equation"]`` was removed.

## 6. Optimization And Planning

**NLP:** `minimize J(z)` s.t. `h=0`, `g≥0`, bounds. Pure `MathematicalProgram`;
`Optimizer` binds method preset (`scipy_slsqp`, `scipy_trust_constr`, `ipopt`).

**Planning:** `PlanningProblem` owns system, sets, cost. `X0`/`Xf` authoritative;
`x_start`/`x_goal` are shortcuts/representative points.

**Trajopt:** planner → transcription → `MathematicalProgram` → `Optimizer` →
`Trajectory`. Single backend-native transcription classes; no parallel JAX
transcription types. NumPy transcriptions build constraints via
`dynamics_function` (compiled evaluator, trace tier when JAX). **JAX transcriptions**
embed `problem.sys.f` directly in the NLP (not evaluator `f` / `f_trace`).
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
returns a `LookupTableController` (a static `System`, so `controller >> plant` simulates);
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

**Workspace cost raster** (`grid.sample_field_costs`, `plotting.plot_cost_field` /
`plot_cost_field_3d` / `plot_cost_field_exports`): rasterize shaped ``FieldCost`` terms
at workspace points ``p = (x, y)`` with a **point probe** body (heading-independent).
Build viz costs with ``bind(sys, point_probe())``; MPC may still use ``car_outline``.

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
which keep bare signatures per [AGENTS.md](AGENTS.md) Textbook Style); lazy optional imports;
namespace `__init__.py` files; plot subpackages may re-export small facades.
Agents and maintainers run tests in the **`minilink`** conda env from
[environment.yml](environment.yml) ([README.md#install](README.md#install)).
