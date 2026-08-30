# Minilink Technical Design

Architecture and public contracts. User guide and call chains: [README.md](README.md).

## 1. Design Principles

1. **Math readability first**: e.g. `dx = A @ x + B @ u`.
2. **Pure contracts, convenient boundaries**: equation paths stay functional;
   facades (`compute_trajectory`, `plot_*`, `animate`) live at API boundaries.
   Models are **stateless** — a `System` holds equations, ports, params, and
   `x0`; evolving state lives in the simulator / returned `Trajectory`, not as
   hidden mutable block state.
3. **Explicit data flow**: visible objects and direct calls; no global backend
   switches or hidden registries.
4. **Backend-native math where simple**: one class for traceable NumPy/JAX algebra.
5. **Specialize only when it clarifies**: `Jax<Plant>` twins when a single class
   would sacrifice readability.

Contributing style (textbook rules, workflow): [AGENTS.md](AGENTS.md).

### Product identity & scope

Minilink is a **Python/JAX block-diagram toolbox** for **modeling, simulating,
controlling, optimizing, and learning** with dynamical systems — equations that
read like textbook math (`dx = f(x,u,t;p)`).

**Distinct edge:** one object model for plants, controllers, diagrams, and
NN/ID blocks. Compose (`@`, `>>`), simulate, analyze, optimize
trajectories/policies, and differentiate / `jit` through the same `f` via
compile backends — without splitting a “sim stack” from a “learning stack.”

**Primary use cases**

1. **Model & teach** — readable continuous plants and closed-loop diagrams.
2. **Control** — classical, model-based, and hybrid digital loops (sampled
   MPC/SMC) on those diagrams.
3. **Optimize** — trajopt / MPC / planning (search, DP) on the same `System`
   and costs/sets.
4. **Learn** — identify parameters, residual dynamics, or NN policies with
   gradients through compiled dynamics.
5. **Scale out plants later** — optional external multibody engines as leaves
   when needed; not the product center.

**Not trying to be:** a Simulink GUI/DAE product, a Multibody/contact OS, an
OCP modeling language, or a batched RL physics engine.

| Toolbox | They own | Minilink vs them |
| --- | --- | --- |
| **Simulink** (+ Stateflow/Simscape) | Industrial diagrams, GUI, DAE, codegen | Same block-diagram idea; code-first, causal, open, differentiable — no GUI/DAE ambition |
| **MATLAB** (CST etc.) | Classical LTI / frequency design | Neighbor for LTI; we center nonlinear systems + optimize/learn in one Python stack |
| **Drake** | Multibody, contact, events, deep MathProg | Complement for teaching / reduced-order / JAX-learning loops; not a second MultibodyPlant |
| **MuJoCo / MJX** | Fast multibody + contact physics | Physics backend we can wrap later; they don’t own control-diagram + optimize/learn UX |
| **CasADi** | Symbolic AD → NLP/OCP | Opt is a *tool on Systems*; we own diagram/sim/control/learn surface around it |
| **acados / Crocoddyl** | Fast deployed MPC/DDP | Solver peers; we stay the Systems lab that can call solvers |
| **Pinocchio** | Fast RBD + derivatives | Algorithm/engine peer — not a diagram framework |
| **Modelica** | Acausal physical networks | We stay causal ODE/blocks (better fit for AD and learning) |
| **python-control** | Classical control in Python | Interop; they stop at LTI, we continue nonlinear + optimize/learn |
| **Brax / similar** | Batched differentiable physics for RL | Neighbor in JAX; we are Systems+control+opt, not an RL physics engine |
| **Pyro** | Teaching dynamics lineage | Successor: keep readability; add diagrams, compile/JAX, optimize, learn |

**Claim:** *Systems-first lab for simulate → control → optimize → learn in
Python/JAX — not a physics OS, not an OCP language, not a Simulink
replacement.*

**Scope practices** (with the principles above): freeze the composition
grammar early; put hard physics behind optional leaves; expose structure
(don’t black-box dynamics); demo-gate maturity; lock build vs run
(wire/validate/compile freezes structure); be hard where the identity is
(compile vs reference parity, JAX twins, discontinuous closed-loop solvers).

Maturity and scheduling: [ROADMAP.md](ROADMAP.md).

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

### Public imports (teaching-first)

Three public layers (shortest → deepest). Prefer the shortest that stays clear;
deep defining-module paths stay valid but are not what README / intro show.

| Layer | Example | Role |
| --- | --- | --- |
| **Root prelude** | `from minilink import Pendulum, ImpedanceController` | README / first-hour only (selective `__all__`) |
| **Band facades** | `from minilink.catalog import Pendulum` · `from minilink.control import ImpedanceController` · `from minilink.analysis import bode` · `from minilink.control.lqr import lqr` · `from minilink.analysis.linearize import linearize` | Canonical course / script API |
| **Defining module** | `from minilink.dynamics.catalog.pendulum.pendulum import Pendulum` | Implementation home; always valid |

Rules:

- `minilink.catalog` is a **teaching alias** of `dynamics/catalog/` (plants still
  live under `dynamics/`; no math moves).
- Band packages (`control/`, `analysis/`, `simulation/`, `control.mpc`, …)
  re-export their teaching symbols via `__all__` / lazy `__getattr__`.
  Exception: when a **module name matches a factory** (e.g. `control.lqr`,
  `analysis.linearize`), import the factory from that module
  (`from minilink.control.lqr import lqr`) — do not shadow the submodule on
  the parent package.
- Root `minilink/__init__.py` is a **small convenience prelude**, not the full
  API — grow it slowly; prefer band imports in longer scripts.
- Do **not** use `from minilink import *`. Do **not** re-export quarantine
  (`symbolic/`, `dynamics/engines/`) from teaching surfaces.

### API stability tiers (v0.1)

The v0.1 teaching release freezes a **stable tier**; the rest is
**provisional** (see [README.md — API stability](README.md#api-stability-v01)
for the user-facing table and [ROADMAP.md](ROADMAP.md) for TRL detail):

- **Stable**: `core/` contracts (`System` family, diagrams, composition
  operators, `Trajectory`, compile facade), `Simulator` / `StaticSimulator`,
  `blocks/`, catalog teaching plants, basic `control/` and `analysis/`.
  Public names and semantics change only with a deprecation note.
- **Provisional**: planning, MPC, hybrid/step, realtime, optimization,
  spatial, placeholders, and the evaluator integration-helper grid beyond the
  frozen subset in §5. These may change between minor releases; hardening
  items live in the [docs/plans/TODO.md](docs/plans/TODO.md) pre-v0.2 backlog.

## 3. Package Map

Component maturity is tracked only in [ROADMAP.md](ROADMAP.md); this section
describes package ownership. Every package belongs to one of four bands.
Homes for planned content are pre-decided in the tables below; scheduling is
in [ROADMAP.md](ROADMAP.md) (teaching-release priorities) and
[docs/plans/TODO.md](docs/plans/TODO.md) (Later / operational backlog).

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
| `catalog/` | **teaching alias** of `dynamics/catalog/` — flat re-exports for short imports (`from minilink.catalog import Pendulum`); ownership stays in `dynamics/` |
| `control/` | control laws and design factories (`lqr.py`, `impedance.py`, `output.py`, `state.py`, `siso.py`, `modelbased.py`, `robotic.py`, **`mpc/`** — RH `ModelPredictiveController`) |
| `estimation/` | online state and parameter estimators (planned) |

**Tools** — verbs on a `System`; they return data or plots and never define
user-facing systems (factories are fine: `linearize_matrices()` returns arrays,
`linearize()` wraps them as an `LTISystem`, and an LQR design function returns a
state-feedback block):

| Package | Role |
| --- | --- |
| `simulation/` | `Simulator`, `StaticSimulator`, `Computer`, `StepSchedule`, `HybridSimulator`, solvers, forcing; `realtime/` (`RealtimeSimulator`, `RealtimeInput`/`RealtimeOutput`, `PygameInput`) |
| `analysis/` | `linearize_matrices` (→ arrays), `linearize` (→ `LTISystem`, FD or JAX), controllability/observability, equilibria, `modal`, selected-channel Bode; `discretize` for continuous→step plant wrappers; more frequency tools planned |
| `planning/` | problems, trajopt, `spatial/` (scenes), `search/` (RRT) |
| `optimization/` | `MathematicalProgram`, `Optimizer` (generic NLP) |
| `identification/` | fit parametric systems to data (planned; physical params and NN weights are the same verb) |
| `graphical/` | signals, phase plane, diagrams, animation |
| `interfaces/` | `Sys2Gym` / `SB3Controller` (gymnasium extra); cosimulation / MJX planned |

**Quarantine** — experimental (TRL < 3); nothing may import these:

| Package | Role |
| --- | --- |
| `symbolic/` | experimental symbolic mechanics (SymPy EoM derivation) |

### Dependency law

- Libraries import only `core`, plus `dynamics/abstraction` interfaces —
  never catalog content. (The abstraction modules are the shared mathematical
  bases of the library band: `blocks/` builds LTI wiring on them, `control/`
  computed torque, `estimation/` EKFs.)
- **Exception:** `control/mpc` may import `planning.trajectory_optimization`
  (receding-horizon controller wraps a traj-family planner).
- Libraries may ship **factories for their own blocks** with array-in /
  block-out signatures (`control.lqr(A, B, Q, R) -> StateFeedback`,
  `estimation.kalman_design(A, C, Q, R) -> KalmanFilter`); the linearization
  producing those arrays lives in `analysis/`.
- Tools import `core`; they may consume libraries in demos and benchmarks.
- `graphical/` is imported lazily from anywhere; rendering stays optional.
- Quarantined packages are imported by nothing.

### Placement algorithm

1. New `System` subclass → shelf by diagram role: wiring → `blocks/`, plant →
   `dynamics/catalog/` (then re-export from domain `__init__` and
   `minilink.catalog` if teaching-facing), control law → `control/`, estimator →
   `estimation/`. Do **not** put plant math under top-level `catalog/` — that
   package is a teaching alias only.
2. New verb → tool: integrate time (`simulation`), characterize (`analysis`),
   find inputs/policies (`planning`), solve NLPs (`optimization`), fit to data
   (`identification`), render (`graphical`), talk to another ecosystem
   (`interfaces`).
3. Neither, and unproven → quarantine at top level with a TRL tag.

Student-facing taxonomy: wiring from `blocks/`, plants from `minilink.catalog`
(or `dynamics/catalog/…`), controllers from `control/`, analysis verbs from
`analysis/` — everything is a `System`.
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

**Vehicle JAX ladder** — :mod:`~minilink.dynamics.catalog.vehicles.jax_vehicles`
(planning / trajopt plants; module-scoped names, no ``Jax`` prefix). Default
``u`` / ``y = x``; named-port twins use the ``Ports`` suffix. Compare:
``examples/projects/car_trajopt/car_trajopt_compare.py`` and
[notebook](examples/projects/car_trajopt/car_trajopt.ipynb).

| Class | $n$ | Input $\mathbf{u}$ | Role |
| --- | --- | --- | --- |
| `Holonomic` | 2 | $[v_x, v_y]$ | holonomic point |
| `HolonomicAccel` | 4 | $[a_x, a_y]$ | holonomic double integrator |
| `BicycleKin` | 3 | $[v, \delta]$ | kinematic bicycle |
| `BicycleAcc` | 5 | $[a_x, \dot\delta]$ | no-slip accel / steer rate |
| `BicycleDyn` | 6 | $[\omega_r, \delta]$ | rigid body + linear tires |
| `BicycleDynRate` | 8 | $[\dot\omega_r, \dot\delta]$ | integrated wheel / steer |
| `BicycleDynTauRate` | 8 | $[\tau_r, \dot\delta]$ | torque + steer rate |
| `BicycleDynServo` | 9 | $[\tau_{\mathrm{cmd}}, \delta_{\mathrm{cmd}}]$ | lagged torque + steer |
| `BicycleDynEngine` | 9 | $[P_{\mathrm{cmd}}, \delta_{\mathrm{cmd}}]$ | lagged **power** + steer |

NumPy bicycle plants remain in :mod:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle`
and :mod:`~minilink.dynamics.catalog.vehicles.steering`. Named envelopes:
:mod:`~minilink.dynamics.catalog.vehicles.car_profile` (`apply_car_profile`).

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
  `step(x, u, k, params)` → `x_new`. Facades: `compute_rollout` / `plot_rollout`
  (state-only ``k/x/u``); cache `self.rollout`. Boundary signal histories:
  :class:`~minilink.simulation.computer.Computer` / hybrid sim, not evaluators.
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
  :meth:`~minilink.control.mpc.controller.ModelPredictiveControllerMixin.export_to_computer`
  for warm-start MPC (also via ``mpc % schedule``).
  Catalog plant :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleDynRate`
  exposes standard ``u`` / ``y`` ports for hybrid composition.
  The JAX fidelity ladder in
  :mod:`~minilink.dynamics.catalog.vehicles.jax_vehicles` runs through
  :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleDynServo`
  (torque lag) and
  :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleDynEngine`
  (wheel-frame power lag + stall torque + engine brake).
  Named vehicle envelopes (parameters + planning limits) live in
  :mod:`~minilink.dynamics.catalog.vehicles.car_profile`
  (``passenger_car``, ``racecar``, ``udes_1_5``); apply with
  :func:`~minilink.dynamics.catalog.vehicles.car_profile.apply_car_profile`.
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
- **Realtime (live sessions):**
  :class:`~minilink.simulation.realtime.simulator.RealtimeSimulator`
  (`simulation/realtime/`) is the third simulation orchestrator, beside
  :class:`~minilink.simulation.simulator.Simulator` and
  :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`. Per frame it
  polls a live :class:`~minilink.simulation.realtime.io.RealtimeInput`
  (``u, should_stop``; nominal ``u`` when ``input=None``), advances the plant
  with evaluator
  :meth:`~minilink.core.compile.evaluators.numpy_evaluators.IntegrationMixin.integrate_zoh`,
  draws live through ``Animator.open_live_scene`` / ``update_live_frame``, and
  publishes to an optional
  :class:`~minilink.simulation.realtime.io.RealtimeOutput` (cosimulation hook).
  **Clocks:** ``frame_dt`` (wall period per rendered frame) is independent of
  ``sim_dt`` (internal integration step; default auto-calibrates at
  :meth:`~minilink.simulation.realtime.simulator.RealtimeSimulator.run` from a
  short ZOH probe so integration stays under a quarter of the frame budget;
  the fine-end cap is the coarser of the live substep ceiling (10/frame) and
  the offline ``Simulator`` auto-``dt`` from ``smallest_time_constant``;
  pass an explicit ``sim_dt`` to skip);
  ``sync="locked"`` (default) advances exactly
  ``frame_dt`` of sim time per frame (reproducible, single JAX trace), while
  ``sync="realtime"`` advances the measured wall elapsed quantized to the
  ``sim_dt`` grid. ``compile_backend=None`` / ``"auto"`` tries JAX then
  NumPy (preferring speed for live sessions); frames that miss the
  ``frame_dt`` budget emit a throttled :class:`UserWarning`. JIT is warmed
  before the clock starts. ``verbose=True`` prints framed setup / completion
  panels (same style as offline ``Simulator``) plus a live per-frame
  compute-vs-``frame_dt`` line. The run returns a
  :class:`~minilink.core.trajectory.Trajectory` (one sample per frame) and the
  ``sys.game()`` facade caches it on ``self.traj``.
  :class:`~minilink.simulation.realtime.pygame_input.PygameInput` maps held
  keys to input-port ``lower_bound``/``upper_bound`` (``mode="hold"``) or
  slews a setpoint at ``rate`` (``mode="rate"``); rendering stays on the
  :class:`~minilink.graphical.animation.renderers.renderer.AnimationRenderer`
  pipeline and is not a ``RealtimeOutput``.
- **MPC hybrid block:** :func:`~minilink.control.mpc.ModelPredictiveController`
  returns a static ``System`` (``n=0``) or :class:`StepSystem` (``warm_start=True``)
  that holds a
  :class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`.
  **JAX path:** :meth:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner.compile_parametric_program`
  once, then each Computer tick is bind + solve (memoized across ports ``u_ff``,
  ``x_ff``, ``z``). **NumPy/direct path:** each tick rebuilds the NLP via
  :meth:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner.solve_trajectory_from`
  (no parametric compile). Warm-start via
  :func:`~minilink.control.mpc.utilities.mpc_warm_start_guess` from
  ``Computer.x``. Deploy / hand loop:
  :meth:`~minilink.control.mpc.controller.ModelPredictiveControllerMixin.compute_command`
  → :class:`~minilink.control.mpc.controller.Command`; telemetry via
  :meth:`~minilink.control.mpc.controller.ModelPredictiveControllerMixin.get_solve_metadata`
  and ``Command.metadata`` (alias of ``plan.metadata``); ``reset()`` clears
  deploy counter, last command, nominal cache, and tick latch. High-rate
  nominal (opt-in):
  :meth:`~minilink.control.mpc.controller.ModelPredictiveControllerMixin.generate_nominal_interpolator`
  then
  :meth:`~minilink.control.mpc.controller.ModelPredictiveControllerMixin.get_nominal_u`
  / ``get_nominal_x`` / ``get_nominal_u_dot`` / ``get_nominal_x_dot``.
  Default ``export_to_computer`` / ``mpc @ plant`` apply ``u_ff`` ZOH; advanced
  :meth:`~minilink.control.mpc.controller.ModelPredictiveControllerMixin.dual_rate_computer`
  (``dt_broadcast``) builds a multi-rate Computer with ``u_nom`` broadcast.
  **Dual-rate packaging (option A):** ``replan`` and ``broadcast`` share the
  same MPC object — after each NLP the latch builds ``NominalCache`` via
  ``generate_nominal_interpolator``; broadcast evaluates ``get_nominal_*``.
  There is **no** port edge between the leaves (Graphviz correctly shows two
  islands). Deploy truth remains the two-timer method API; the Computer is
  convenience packaging, not a pure signal graph. A future port-carried flat
  plan (option B) would make the edge honest at the cost of one
  ``dt_broadcast`` lag under parallel tick semantics. External nodes wrap the
  same methods (no ROS2 package in-tree). Post-sim horizons:
  :func:`~minilink.control.mpc.viz.mpc_plans_from_rollout`;
  default animation overlays:
  :func:`~minilink.control.mpc.viz.mpc_animation_overlays`.
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
  (``vehicle:body``). Controllers may own their own visualization frames the
  same way (e.g. optional `TaskImpedance` `task_force` → `ctl:task_force`); the
  arrow shows the pre-gravity task wrench `f_task`, not `Jᵀ f_task` or `g(q)`.
  Meshcat `native=True` freezes changing `Arrow` / `TorqueArrow` geometry at
  `t=0` — use `native=False` for frame-accurate force playback.
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
  (see `examples/demos/identification/params_gradient.py`).
- **Planning params tiers** (`ProblemParameters`): `system`, `cost`, `sets`
  today; `scene` is reserved (`None`) for pipeline B spatial overrides.
  Online façade on `solve_trajectory_from` / `compute_command`: `params=None`
  or `{}` binds `x0` only; `params={"scene": …}` raises `NotImplementedError`
  until `ParametricMathematicalProgram` gains `J(z, p)` / `ObstacleBank`
  ([planning-pipeline-architecture.md](docs/plans/planning-pipeline-architecture.md)).
  **TODO: Prioritize threading $p$ into JAX parametric programs.** This will allow 
  moving obstacles online without rebuilding the NLP, unlocking real-time dynamic obstacle avoidance.
  **Deferred** ([docs/plans/TODO.md](docs/plans/TODO.md) Later): call-time overrides
  on base `Shape`, `Set`, and `CostFunction` primitives in `core/` — those types
  declare `(t, params)` but still read frozen attributes only until a follow-up
  pass.

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

**Operator mutation semantics (frozen v0.1 decision):** when the left operand of
`+` or `>>` is already a `DiagramSystem`, the shortcut **extends that diagram in
place and returns it** — `bigger = loop + estimator` mutates `loop`. This is
deliberate: subsystem operands are always shared references (never copied), so a
"fresh" diagram would only pretend to isolate state; in-place extension keeps
`a >> b >> c` chains cheap and the object identity honest. Build a new
`DiagramSystem` explicitly when you need an independent topology.

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

<!-- TODO: User Architectural Review — feedback-port declaration contract (v0.2 draft) -->

A controller is an ordinary `System`: explicit ports, and `ctl` as the port
compute — `ctl` *is* the control law. The **feedback-port declaration** is
read-only context for tools, never behavior: the class attribute
`feedback_profile` names the law family, and
`minilink/core/feedback.py` maps it to standard port roles
(`PROFILE_PORTS` registry + `feedback_ports(block)` resolver). Explicit
attrs `measurement_port` / `ref_port` / `control_port` / `plot_space`
override the registry for nonstandard blocks (`LookupTableController`
declares `measurement_port="x"`, `ref_port=None`; MPC declares
`control_port="u_ff"`, no `plot_space`). Resolution is duck-typed
(`getattr`), never `isinstance`.

The declaration drives three tools:

- **`@` composition** resolves the controller side from the declaration
  first (`resolve_standard_feedback` and the hybrid
  `default_computer_boundary_ports`); the name/dimension heuristics
  (`y↔y`, `Mux(q, dq)`, `x↔x`) remain as fallback for undeclared blocks.
- **`plot_control_law()`** on the `Controller` / `DynamicController`
  marker facades (also in `core/feedback.py` — no ports, no state, no
  behavior; static blocks subclass `Controller`, stateful ones
  `DynamicController`). The zero-argument call sweeps the textbook slice
  per profile (error space for error-driven laws, absolute measurement or
  state otherwise), pinning the reference, unswept components, and any
  internal state (`x0`) at nominal; engine in
  `graphical/port_map.py` (module twin for third-party blocks). The
  generic `plot_input_output_map()` lives on every `System` and needs no
  declaration.
- **`PolicyEvaluator(policy=block)`** adapts a declared block into
  `u = policy(x)` (grid state on the measurement port, reference pinned).

An undeclared block still simulates, compiles, and wires manually — the
declaration only unlocks the smart tools.

| Profile | Module | Measurement | `plot_control_law` sweep |
| --- | --- | --- | --- |
| `error` | `output.py` | `y` (error-driven, `u = c(r - y)`) | error `e = r − y` |
| `output` | (reserved) | `y` used absolutely, `u = c(y, r)` (learned policies) | measurement, `r` pinned |
| `impedance` | `impedance.py` | `y = [pos; rate]` dim `2n`; optional robotic `+ g(q)` via `robotic.py` | `(e, de)` plane |
| `state` | `state.py` | full state `x` | `(x0, x1)` |
| `siso` | `siso.py` | `y` dim `n` only (decoupled loops) | error per axis |
| `task` | `robotic.py` | Joint ``[q; dq]`` feedback; internal FK/J; optional ``+ g(q)``; optional task-force arrow | `(q0, dq0)` absolute |
| `kinematic` | `robotic.py` | Joint ``q`` only; outputs ``dq`` for speed-controlled plants | measurement |
| `modelbased` | `modelbased.py` | ``y = [q; dq]``; computed torque; Pyro sliding mode ``τ = ID(q,dq,ddq_r) - K(q) sign(s)`` | `(q0, dq0)` absolute |

**Controller naming conventions (v0.1).** PID-style gain params keys are
uppercase (`Kp`, `Ki`, `Kd`, `K_null`) across `impedance`, `siso`, `robotic`,
and `modelbased`; non-PID shaping constants keep their textbook lowercase
names (`tau`, sliding-mode `lam` / `gain` / `nab`). `tracking_ref` defaults
follow the law's nature and are deliberately **not** uniform: spring-damper
and kinematic laws default to regulation (`tracking_ref=False`,
``r = pos_d``); model-based trajectory laws (`ComputedTorqueController`,
`SlidingModeController`) default to tracking (`tracking_ref=True`,
``r = [q_d; dq_d]``).

**Embedded-model params rule (frozen v0.1 decision).** Model-based controllers
(`ModelJointImpedance`, `TaskImpedance`, `ComputedTorqueController`,
`SlidingModeController`) hold a reference to a plant object and call its model
hooks (`g(q)`, `H(q)`, `inverse_dynamics`) with ``params=None`` — the embedded
model always reads that plant object's **live** ``self.params``. The
controller's own params dict carries gains only and is never forwarded to the
embedded model. Consequences: mutating ``plant.params`` updates both the real
plant and the controller model (they are usually the same object — nominal
model-based control); diagram-level ``params={"sys": {...}}`` overrides and the
evaluator parametric tier reach the plant subsystem but **not** the
controller's embedded copy (which is exactly the model-mismatch semantics —
override the plant to study robustness). A forwarded model-params channel is
deliberately not provided in v0.1.

### `Trajectory`, sets, costs, geometry

- `Trajectory`: `t (N,)`, `x (n,N)`, `u (m,N)`, optional `signals`; NumPy reporting object.
- Sets: `margin ≥ 0` feasible; `contains`/`sample` may convert to NumPy. Compose with
  `&` → `IntersectionSet`. `margin(z, t, params)` is threaded by transcriptions via
  `problem.params.sets`; field-backed spatial sets forward that same parameter
  object to their scene queries. Base `Set` subclasses other than `FieldSet` /
  `CallableSet` do not yet read `params` (deferred — see [docs/plans/TODO.md](docs/plans/TODO.md)).
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
`has_trace_tier` is `True` on JAX evaluators.

|  | Bound | Parametric |
| --- | --- | --- |
| Fast (default) | `f` ≡ `f_jit` | `f_p` ≡ `f_jit_p` |
| Trace (JAX only) | `f_trace` | `f_trace_p` |

Same 2×2 for `outputs`, `step`, and integration helpers
(`rk4_step`, `rk4_integrate_zoh`, `rk4_integrate_linear`, `euler_integrate_*`, …)
on JAX dynamics evaluators. Layout:
`evaluators.py` (ABCs), `numpy_evaluators.py`, `jax_evaluators.py`,
`step_rollout.py` (`gather_u`, `StepRolloutMixin`).

**Frozen evaluator subset (v0.1).** The stable-tier evaluator API is:
`f` / `f_p`, `outputs` / `outputs_p`, `step` / `step_p`, `rollout`,
`rk4_step` / `rk4_step_p`, `integrate_zoh` / `integrate_zoh_rollout` /
`integrate_zoh_p`, and the JAX trace-tier twins of those names
(`f_trace`, `f_trace_p`, …). The rest of the integration-helper grid
(`euler_*`, `*_ivp*`, `rk4_integrate_linear*`, `_jit` aliases) is
**stable-internal**: used by simulators and transcriptions, kept working, but
its names may still change before v1.0 — do not build external code on it.

**Integration rename (pre-1.0).** Rollout methods use explicit integrator + input
model tokens — no bare `integrate`:

| Old | New |
| --- | --- |
| `integrate` | `rk4_integrate_zoh` |
| `integrate_p` | `rk4_integrate_zoh_p` |
| `rk4_integrate_forced` | `rk4_integrate_linear` |
| `rk4_integrate_forced_p` | `rk4_integrate_linear_p` |

Trace-tier JAX twins follow the same pattern (`*_trace`, `*_trace_p`).

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
  (state-only :class:`~minilink.core.step_rollout.StepRollout` ``(k, x, u)``; evaluators
  and the façade do not record boundary signals — logging is owned by
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
([docs/plans/TODO.md](docs/plans/TODO.md) Later).

### Discontinuous closed loops — known issues

Controllers with discontinuous laws (e.g. :class:`~minilink.control.modelbased.SlidingModeController`
``sign(s)``) on a **continuous** :class:`DiagramSystem` closed loop are supported today,
but several solver/logging behaviors are misleading until a dedicated hybrid or
event-handling path lands (hybrid path exists; see discontinuous guidance below and
[ROADMAP.md](ROADMAP.md) teaching-release hardening / [docs/plans/TODO.md](docs/plans/TODO.md)).

**TODO: Add a hard warning.** When a discontinuous controller is wired into a continuous `DiagramSystem`, we should warn the user and recommend wrapping it in a fast `Computer` inside a `HybridDiagram` to enforce physical digital-on-continuous reality.

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
``examples/demos/hybrid/smc_pendulum_rate.py``.

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

**Planning:** `PlanningProblem` owns system, sets, cost, and continuous horizon
`tf` (`None` unset, `+inf` infinite-horizon, or a finite length — trajopt/MPC
require finite `tf` via `require_finite_tf()`). `X0`/`Xf` authoritative;
`x_start`/`x_goal` are shortcuts/representative points. Offline entry is
`Planner.solve()` → `TrajectoryPlan` (traj family) or `PolicyPlan` (policy
family).

**Trajopt:** `TrajectoryOptimizationPlanner` → transcription → NLP →
`TrajectoryPlan`. **I-level constructors** take flat kwargs
(`n_steps=…`, `transcription="direct_collocation"`, `compile_backend=…`,
`optimizer_options={…}`) like `Simulator` / `Optimizer`; teach demos pass
`transcription=` explicitly. Tier-2 still accepts a `Transcription` instance
and/or `options=TrajectoryOptimizationOptions(...)`. Offline
`solve_trajectory` always rebuilds a fresh
`MathematicalProgram`. Optional `compile_parametric_program()` builds a
`ParametricMathematicalProgram` (``h(z, x0)``) once so
`solve_trajectory_from(x0, params=None)` is bind + solve only on the JAX
parametric path; with ``compile_backend='numpy'`` or ``'direct'``, each
from-solve rebuilds the NLP (expected for no-JAX MPC). A one-time speed warning
is emitted only when a JAX planner omits parametric compile.
Online ``params`` is an x0-only façade today (`None`/`{}`); reserved key
``scene`` raises ``NotImplementedError`` until pipeline B extends the parametric
tier to ``J(z, p)`` (see [docs/plans/TODO.md](docs/plans/TODO.md) scene params). Knot count
`n_steps` is a planner flat kwarg (or lives on a custom transcription's
options); the time grid is
`linspace(0, problem.tf, n_steps)` for finite `tf`. Single backend-native
transcription classes; no parallel JAX transcription types. NumPy transcriptions
build constraints via `dynamics_function` (compiled evaluator, trace tier when
JAX). **JAX transcriptions** embed `problem.sys.f` directly in the NLP (not
evaluator `f` / `f_trace`). `compile_backend="direct"` calls `system.f`
uncompiled (escape hatch). A `MathematicalProgram` carries the native backend of
its callables in its `backend` field, and the `Optimizer` compiles with it by
default.

**RRT / DP:** same two-tier idea — flat routine knobs on
`RRTPlanner` / `RRTStarPlanner` / `DynamicProgrammingPlanner`, with
`options=` as the advanced escape. Keep fundamental seams explicit
(`extender`, `StateSpaceGrid`).

**Policy synthesis** (`planning/policy_synthesis/`): offline dynamic programming on a
continuous plant. A `StateSpaceGrid` discretizes the `PlanningProblem` — grid *extent*
comes from a `BoxSet`/`BoxInputSet` (so it stays finite), grid *validity* from
`X.contains`/`U.contains` (so `X = bounds & free` still works), and successors from a
forward-Euler step `x_next = x + f(x,u,t)·dt` (the time step `dt` lives on the grid, not
on `System`). `DynamicProgrammingPlanner` runs value iteration backward — `solve`
to tolerance, `solve_steps` for a fixed horizon — returning a `PolicyPlan` whose
`.policy` holds cost-to-go `J` and greedy policy `pi` (action ids); out-of-domain
transitions are charged a finite `out_of_bound_cost` (pyro's
`cf.INF`). Three interchangeable backward-step backends share this workflow: `loop` (per-node
Python, pyro's reference), `numpy` (vectorized over the precomputed lookup table, the default),
and `jax` (the same backup as one jitted `lax.while_loop` with `map_coordinates`, built on a
NumPy precompute so any plant works; linear/nearest only). `precompute` trades the `(N,A,n)`
successor table for per-sweep recomputation (memory vs time-varying support). Plotting lives on
`DynamicProgrammingPlanner` (`plot_cost2go`, `plot_policy`, `animate_*`, `get_controller`) and
in `planning/policy_synthesis/plotting.py` for loaded results (`plotting.get_controller(result)`).
`get_controller()` returns a `LookupTableController` (a static `System`, so `controller >> plant` simulates);
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
obstacles, MPC sweeps) need pipeline B (`ObstacleBank` + `J(z, p)` bind) —
online `params={"scene": …}` is reserved but raises `NotImplementedError`;
rebuild `Scene` + re-`compile_parametric_program()` until then.

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
`solve()`, `reached_goal` and `solution_node` report success vs
best-effort fallback (`return_best_effort`). The two swappable pieces are an injected
`TrajectoryExtender` (`propose(from, toward, problem, rng) → Iterable[Edge]`:
`KinodynamicExtender` forward-integrates controls, `SteeringExtender` connects exactly
via a `SteeringFunction` including `DubinsSteering`) and a `metric(a,b)` callable
(nearest-neighbour distance). Every system is an ODE, so an `Edge` always carries real
`(t,x,u)`; `solve() → TrajectoryPlan`. `RRTStarPlanner` extends the attach step
with near-neighbour parent selection and cost-based rewiring (`Tree.rewire`,
`Tree.propagate_cost`); `Edge.cost` is the cost-to-come along the tree. With
`optimize_after_goal`, the search continues after the first goal connection until
the best goal cost stops improving for `convergence_patience` extensions;
`record_history` + `animate_convergence` replay tree growth and path refinement.
``live_plot`` / ``callback`` redraw the tree during ``solve`` (pyro-style);
``live_plot_after_goal_only`` limits updates to the RRT* post-goal convergence phase.
``RRTOptions.nearest_backend`` selects brute-force or SciPy ``cKDTree`` nearest/near
queries (Euclidean L2 only — requires ``metric=euclidean``); see
``benchmarks/run_rrt_nearest_backends.py``. Modest
speedups on low-D obstacle scenes are expected when collision checking and
post-goal tree scans dominate.

## 7. Graphics And Benchmarks

Facades delegate to `graphical/`. Time plots: `signals=("x", "u", "block:port")`.
Phase plane: matplotlib default. Diagrams: Graphviz layout (notebook SVG;
script Matplotlib window), Mermaid export;
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
