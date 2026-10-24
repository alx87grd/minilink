# Minilink Technical Design

Architecture and public contracts, under the law of [CONSTITUTION.md](CONSTITUTION.md).

## 1. NumPy and JAX

NumPy required; JAX optional (the `full` extra, or `pip install jax jaxlib`), imported lazily via
`minilink.core.backends` (`require_jax_numpy()`, `array_module()`). No `minilink.jax`
package, no global mode. Explicit `compile_backend` and evaluator backend args.

**One class by default** (`xp = array_module(x)`). Law: CONSTITUTION §6, 'Backend-native
math'.

**Precision policy.**
JAX evaluators enable 64-bit floats on construction; `MINILINK_JAX_X64=0`
opts out for GPU/RL workloads. Tools built on JAX evaluators (trajopt,
`Optimizer`, DP) never require the caller to call `configure_jax` first.

## 2. Interface Layers

| Layer | Use | Examples |
| --- | --- | --- |
| 1 Facades | Teaching, quick looks | `compute_trajectory`, `plot_trajectory`, `jacobian`, `linearize`, `+`/`>>`/`@` (`C >> G` is the loop gain, `C @ G` the loop closed through an Error block, `feedback(L)` an open loop closed on itself) |
| 2 Tools and orchestrators | Scripts, projects, repeat runs, trajopt, NLP | `Simulator`, `analysis.jacobian`, `analysis.bode`, `TrajectoryOptimizationPlanner`, `Optimizer` |
| 3 Contracts | Custom wiring, extension | `DiagramSystem.connect`, `compile()`, `MathematicalProgram` |

### Public imports (teaching-first)

Three public layers (shortest → deepest). Prefer the shortest that stays clear;
deep defining-module paths stay valid but are not what README / intro show.

| Layer | Example | Role |
| --- | --- | --- |
| **Root prelude** | `from minilink import Pendulum, Impedance, lqr, QuadraticCost, DynamicProgrammingPlanner, ReinforcementLearningPlanner` | **The teaching surface, one import line** — every name a student meets (tested as a set in `test_teaching_surface.py`) |
| **Band facades** | `from minilink.catalog import Pendulum` · `from minilink.control import Impedance` · `from minilink.analysis import bode` · `from minilink.control import lqr` · `from minilink.analysis import linearize` | Canonical course / script API |
| **Defining module** | `from minilink.dynamics.catalog.pendulum.pendulum import Pendulum` | Implementation home; always valid |

Rules:

- `minilink.catalog` is a **teaching alias** of `dynamics/catalog/` (plants still
  live under `dynamics/`; no math moves).
- Band packages (`control/`, `analysis/`, `simulation/`, `learning/`, …)
  re-export their teaching symbols via `__all__` / lazy `__getattr__`. A
  module never takes the name of a function it defines (`control/design.py`
  holds `lqr`), so the facade can export the function.
- Root `minilink/__init__.py` exports the **whole teaching surface** (lazily) so
  student-facing code needs one import line; band facades organise the same
  names by role.
- Do **not** use `from minilink import *`. Do **not** re-export the
  `experimental/` tier from teaching surfaces.

### Two lanes (teaching surface vs research lane)

The lane contract is [ROADMAP.md §2](ROADMAP.md#2-two-lanes). The teaching surface is the
root prelude plus the band facades, registered in `minilink/__init__.py` and the band
`__init__` files, and checked as a set by `test_teaching_surface.py`.

## 3. Package Map

Component maturity is tracked only in [ROADMAP.md](ROADMAP.md); this section
describes package ownership. Every package belongs to one of four bands.
Homes for planned content are pre-decided in the tables below; scheduling is
in [ROADMAP.md](ROADMAP.md) §5 and the workboard
[docs/plans/TODO.md](../plans/TODO.md).

**Framework** — defines what a `System` is and how diagrams execute
(NumPy-only; changes are design events):

| Package | Role |
| --- | --- |
| `core/` | `System` (+ its shortcuts, `facades/` by what they do: `simulation.py`, `analysis.py`, `plotting.py`, `animation.py`), the package import tables (`exports.py`), `DynamicSystem`, `DiagramSystem` (subclasses `DynamicSystem`), shared diagram wiring (`wiring.py`: `WiredDiagramMixin`, gather, params nesting), the port-dependency graph (`topology.py`: evaluation order, algebraic loops), the composition grammar (`operators.py`: `+`, `>>`, `autowire`; `closed_loop.py`: `@`, `closed_loop`, `feedback`; `assembly.py`: the operands laid out flat, their ids, default ports and camera, which both build on; `roles.py`: the block roles the port names declare), signals/ports (`signals.py`), backend policy & helpers (`backends.py`), `Trajectory`, sets, distributions (`distributions.py`), costs, geometry (`geometry.py`; **planned package** `core/geometry/` — Path, Track, Scene, bind, spatial Fields, course catalog: [docs/plans/geometry-module.md](../plans/geometry-module.md)) |
| `core/discrete/` | the discrete-time sibling, `x_{k+1} = F(x_k, u_k, k)` on an integer index: `StepSystem` (`step_system.py`), `StepDiagramSystem` (`step_diagram.py`), the hybrid loop (`hybrid_diagram.py`: `HybridDiagram`; `hybrid_composition.py`: `hybrid_closed_loop` and the port resolution behind `Computer @ plant`), `StepRollout` (`step_rollout.py`) |
| `core/compile/` | the compiler, the two execution plans, one evaluator per system kind |

**System libraries** — `System` subclasses you drop into a diagram, shelved by
*role in the diagram*, never by implementation technology (linear, Lagrangian,
or neural network alike):

| Package | Role |
| --- | --- |
| `blocks/` | plant-agnostic wiring: sources, `Integrator`, `TransferFunction` (and `Lead`, its lead compensator), routing (`Error`/`Sum`/`Gain`/`Mux`/`Demux`), nonlinear (`Saturation`), filters (`LowPassFilter`), neural (`MLP`), the step block `UnitDelay` |
| `dynamics/` | plants: `abstraction/` mother classes, `catalog/` by physical domain |
| `catalog/` | **teaching alias** of `dynamics/catalog/` — flat re-exports for short imports (`from minilink.catalog import Pendulum`); ownership stays in `dynamics/` |
| `control/` | control laws and design verbs: `design.py` (`lqr`, `place`, `lqr_finite_horizon`, `lqg`, `trajectory_lqr` on a linear model or a plant; their gains on arrays), `siso.py` (`P`, `PI`, `PD`, `PID`), `state.py`, `impedance.py`, `robotic.py`, `modelbased.py`, `neural.py`, `geometric.py` (`PurePursuit`); `ModelPredictiveController` re-exported from `planning/trajectory_optimization/mpc/`. The port contract is §4, 'Control laws' |
| `estimation/` | state observers and their designs: `observer.py` (`StateObserver`, the innovation form, ports `u`, `y` → `x`, role `observer`), `design.py` (`luenberger`, `kalman`, the duals of `place` and `lqr`) |

**Tools** — verbs on a `System`; they return data or plots and never define
user-facing systems (factories are fine: `linearize_matrices()` returns arrays,
`linearize()` wraps them as an `LTISystem`, and an LQR design function returns a
state-feedback block):

| Package | Role |
| --- | --- |
| `simulation/` | `Simulator`, `StaticSimulator`, solvers, forcing; `discrete/` (`Computer`, the tick engine, and its firing `StepSchedule`; `HybridSimulator`); `realtime/` (`RealtimeSimulator`, `RealtimeInput`/`RealtimeOutput`, `PygameInput`, `LiveSignalPlotOutput`) |
| `analysis/` | one calling pattern `tool(sys, x_bar, u_bar, t, params, *, method="auto", eps)` on any `System` (the linear model as is, the rest linearized at the operating point): `jacobian`, `linearize` (→ `LTISystem`), the one-channel frequency and time tools (`bode`, `pzmap`, `nyquist`, `margins`, `root_locus`, `step_response`, `step_info`, `transfer_function`, the sensitivity functions) with their `plot_` twins, `structural` (controllability, observability), `linear` (the one-channel matrix math, `minreal`), `equilibria`, `modal`, `lyapunov`, `riccati`, `discretization`. The API reference is the docs site |
| `planning/` | `problems.py`, `results.py`, `evaluator.py` (the problem evaluator, `evaluate`), `evaluation.py` (`score_run`, `evaluate_trajectory`, the `Evaluation` record), `comparison.py`; `dynamic_programming/` (value iteration, policy evaluation, `LQRPlanner`, the lookup policy), `trajectory_optimization/` (collocation, shooting; `mpc/`, the receding-horizon block, provisional), `search/` (RRT), `spatial/` (scenes; **retires** into `core/geometry/`) |
| `learning/` | policies learned from interaction: `reinforcement_learning/` (the neural planner, the tabular learners); identification and imitation later |
| `optimization/` | `MathematicalProgram`, `Optimizer` (generic NLP) |
| `graphical/` | signals, phase plane, diagrams, animation |
| `interfaces/` | lazy facade: `Sys2Gym` / `SB3Controller` (gymnasium extra); cosimulation / MJX planned |

**Experimental tier** — unproven modules under `experimental/`:

| Module | Role |
| --- | --- |
| `experimental/symbolic/` | symbolic mechanics (SymPy EoM derivation, Lagrange / Kane) and export |
| `experimental/engines/` | hand-rolled JAX contact worlds, ANCF tire |
| `experimental/c_export.py` | JAX → C transpiler for controller leaves |

### Dependency law

- Libraries import only `core`, plus `dynamics/abstraction` interfaces —
  never catalog content. (The abstraction modules are the shared mathematical
  bases of the library band: `blocks/` builds LTI wiring on them, `control/`
  computed torque, `estimation/` EKFs.)
- **Exception:** the `minilink.control` facade re-exports `ModelPredictiveController`
  from `planning/trajectory_optimization/mpc/` (the receding-horizon block wraps a
  trajectory optimization planner, so it lives with it).
- **Exception:** `learning/` builds on `planning/`: the problem, the planner base,
  the solution and the dynamic programming grid its tabular learners share.
- **Exception:** `learning.reinforcement_learning` and `planning.planner`
  may import `control.neural` and `blocks` (a learned
  law is a control block, the critics share its multilayer perceptron, and an
  open-loop plan is a `TrajectorySource` block).
- **Exception:** `core/closed_loop.py` imports `blocks.routing` (`Error`, `Sum`,
  `Gain`, `Demux`): `@` and `feedback` build those blocks.
- **Exception:** `core/inspect.py` imports `optimization` and `planning`
  (`PlanningProblem`, the spatial fields): `print()` dispatches on every core
  noun, wherever it lives.
- **Exception:** `analysis/` imports `blocks.TransferFunction`: the transfer
  function and sensitivity tools return one, so `bode` and `margins` apply.
- **Exception:** `planning/trajectory_optimization/mpc/` imports `planning.spatial.overlays`
  (its plots draw the track).
- **Exception:** `graphical/animation` imports `planning.spatial.Scene`, to
  draw a scene as an overlay.
- A new import across libraries needs an exception here, with its reason.
- Libraries may ship **design verbs for their own blocks**, a model in, block out
  (`control.design.lqr(sys, Q, R) -> StateFeedback`,
  `estimation.design.kalman(sys, Q_n, R_n) -> StateObserver`), and their gains on
  arrays (`lqr_gain(A, B, Q, R) -> K`); the linearization producing the model lives in
  `analysis/`.
- **Exception:** `estimation.design` imports `control.design.place_gain`: the observer
  gains are the dual designs; and `control.design.lqg` imports `estimation.design.kalman`, inside the
  function: an LQR gain on a Kalman estimate.
- **Exception:** `control/` (and `estimation/`) import `analysis`
  (`derivatives.jacobian`, `linearization.design_model`, `riccati`, `structural`): a design on a plant is
  an adapter on its linearization (RULES 4.3), and the Riccati equations are
  analysis-tier math shared with the planners.
- Tools import `core`; they may consume libraries in demos and benchmarks.
- `graphical/` is imported lazily from anywhere; rendering stays optional.
- The `experimental/` tier is imported by nothing.

### Placement algorithm

1. New `System` subclass → shelf by diagram role: wiring → `blocks/`, plant →
   `dynamics/catalog/` (then re-export from domain `__init__` and
   `minilink.catalog` if teaching-facing), control law → `control/`, estimator →
   `estimation/`. Do **not** put plant math under top-level `catalog/` — that
   package is a teaching alias only.
2. New verb → tool: integrate time (`simulation`), characterize (`analysis`),
   find inputs/policies (`planning`), learn them or fit models to data
   (`learning`), solve NLPs (`optimization`), render (`graphical`), talk to another ecosystem
   (`interfaces`).
3. Neither, and unproven → `experimental/` with a TRL tag.

Student-facing taxonomy: wiring from `blocks/`, plants from `minilink.catalog`
(or `dynamics/catalog/…`), controllers from `control/`, analysis verbs from
`analysis/` — everything is a `System`.
`blocks/` holds plant-agnostic wiring primitives; `dynamics/catalog/equations/`
holds canonical textbook ODEs (the double and triple integrators, `VanderPol`) with
graphics, labels, and bounds for teaching demos; the single integrator is the `Integrator`
block.

### Continuous-time core; step/hybrid subsidiary

The law is CONSTITUTION §2 (invariants 1, 3, 4). Its mechanics here: the evolution kind is
the type (`isinstance(sys, StepSystem)`), never a flag or a solver option, so `f` is always a
rate; step and hybrid live on sibling types and separate compile and simulation paths
(`StepSystem` beside `DynamicSystem`, `HybridDiagram` as two-side glue), and nothing of
theirs enters `DynamicSystem`, flow `compile()` or `Simulator`.

The reusable bases are `dynamics/abstraction`: `StateSpaceSystem` builds its matrices
through methods `A(t, params)`, `B(t, params)`, `C(t, params)`, `D(t, params)`
(`dx = A @ x + B @ u`), as `MechanicalSystem` builds `H(q, params)`; `LTISystem` is the
constant-matrix convenience built from arrays; `Manipulator` (`MechanicalSystem` plus the
task ports `p`, `pdot`; `forward_kinematics`, `J`, `inverse_kinematics`) is the base of the
serial arms, read straight by the robotic laws (`ctl @ plant` on `y = [q; dq]`). Mixed
inputs are named ports with concrete allocation hooks, never inheritance branches. The
vehicle ladder (four teaching rungs, dual-backend) is in the catalog; its research rungs and
the named car profiles live with their scenarios in `examples/projects/car_trajopt/`.

## 4. Core Object Contracts

### `System`

- **Math:** `h(x,u,t,params)` on the model and port `compute` functions; continuous
  evolution `f(x,u,t,params)` is on :class:`DynamicSystem` only (and stacked on
  :class:`DiagramSystem`). **Default output:** on :class:`DynamicSystem` and
  :class:`StepSystem`, `h` returns the full state `y = x` when the `y` port has
  the state dimension (`p == n`) and zeros otherwise — the pyro convention, so a
  plant declared with `output_dim=n` composes with `controller @ plant` without
  overriding `h`.
- **Dims:** `n` defaults to 0 (static IO shell); `m` from input ports; `p` from primary output
  `"y"` only (aux `"x"` does not change `p`; no `"y"` ⇒ `p==0`).
- **Shape:** Inheritance for core system types (`System`, `DynamicSystem`,
  `StepSystem`); composition for diagrams and optional behaviors.
- **Nominal and start, two quantities with one owner each:** `state.nominal_value` is
  the nominal state x̄ and the input ports' `nominal_value` the nominal inputs ū, the
  operating point every analysis and design verb reads when `x_bar` / `u_bar` are
  omitted (`analysis.derivatives.operating_point`); `x0` is the start of a simulation
  and nothing else. Both default to zeros; a diagram derives both from its blocks. Where
  the point lives: on a system, the nominal value of its signals; in a block whose law
  reads it (`OutputReference`, `StateObserver`), its `params`; in a tool, the
  `x_bar` / `u_bar` arguments, defaulting to the nominal (the target of
  `QuadraticCost.from_system` included); on a linear model, `xbar`, `ubar` and `ybar`,
  where its deviations are measured from.
- **Ports:** explicit, ID-first; infer `dim` from metadata or default 1. Extract
  slices with `get_port_values_from_u(u, "r", "y")`.
- **DynamicSystem shortcut:** `input_dim`, `output_dim`, `expose_state`,
  `y_dependencies` create standard `u`/`y`/`x`.
- **StepSystem (discrete leaf):** same port shortcut as `DynamicSystem`; evolution is
  `step(x, u, k, params)` → `x_new`. Facades: `compute_rollout` / `plot_rollout`
  (state-only ``k/x/u``); cache `self.rollout`. Boundary signal histories:
  :class:`~minilink.simulation.discrete.computer.Computer` / hybrid sim, not evaluators.
  A flow `DiagramSystem` refuses it at `add_subsystem` (a `StepDiagramSystem`
  likewise refuses a `DynamicSystem` with states); it meets a continuous plant
  only through a `Computer`: `block % dt @ plant`.
- **Hybrid (computer + plant):** `HybridDiagram` bundles a `Computer` (the step side and its
  `StepSchedule`) and a continuous `DiagramSystem` plant; the boundary channels are ZOH
  (computer → plant) and sample (plant → computer). `HybridSimulator` mirrors `Simulator`
  (`t0` / `tf`, `solve`, `solve_forced`; `plant_dt_inner` sub-steps the plant) and returns
  a `HybridSimResult`: `computer` (a `StepRollout`, tick `k`) and `plant` (a `Trajectory`,
  wall time `t`); the facades cache the plant trajectory on `self.traj`, the full result on
  `self.last_result` and the computer view on `self.rollout`. Shortcuts: `block % schedule`
  → `Computer`; `Computer @ plant` and `hybrid_closed_loop` wire the standard feedback
  ports as `ctl @ plant` does; an MPC block goes through `export_to_computer`, which
  refuses a `dt` other than `dt_mpc`. `plot_diagram` draws the plant and computer clusters
  with dashed boundary edges (`abstract_boundary=True` by default).
- **Realtime (live sessions):** `RealtimeSimulator` (`simulation/realtime/`) is the third
  orchestrator. Per frame it polls a `RealtimeInput` (`u`, `should_stop`; the nominal `u`
  when none), advances the plant by ZOH integration, draws live through the `Animator`
  and publishes to an optional `RealtimeOutput` (a `LiveSignalPlotOutput` beside it when
  `plot=True` or `plot=(signals)`, on `plot_backend`). Two clocks: `frame_dt` (wall period per
  frame) and `sim_dt` (the integration step, auto-calibrated at `run` unless given);
  `sync="locked"` advances `frame_dt` of sim time per frame, `sync="realtime"` the measured
  wall time. The run returns a `Trajectory` (one sample per frame), cached on `self.traj`
  by `sys.game()`. `PygameInput` maps held keys or a connected gamepad (`joystick_axes`,
  3-D `camera_axes` orbit) to the input ports' bounds or slews a setpoint, staying
  windowless when a controller is connected.
- **MPC hybrid block:** `ModelPredictiveController(planner, dt_mpc=)` returns a
  `StepSystem` whose state is the last packed `z` (the default `warm_start=True`), or a
  static `System` with `warm_start=False`.
  On JAX the planner's parametric program is compiled once and each tick is bind + solve,
  memoized across the ports `u`, `x_ff`, `z`; on NumPy each tick rebuilds the NLP. The
  deploy loop is `compute_command` → `Command` (the tick's `PlanningSolution`, `k`,
  `t_solve`, `u_ff`, `x_ff`, `z`, `success`); `reset()` clears the latch. A dual-rate
  `Computer` (`dual_rate_computer(dt_broadcast)`) replans at `dt_mpc` and broadcasts the
  interpolated nominal in between; the two leaves share one block, so there is no port
  edge between them. Post-run horizons and animation overlays: `planning/trajectory_optimization/mpc/viz.py`.
- **Observer:** a block that reads the plant's `u` and `y` and gives the estimate on `x`
  (`StateObserver`; later filters keep the ports, their state may be larger than the
  estimate). Under the id `observer`, top level or inside the controller box, `roles()`
  finds it. A block that reads a measurement and gives the state back with no `u` and no
  `y` is refused as a controller by `@`, `closed_loop`, `%` and `plot_control_law`
  (`core.roles.require_command`): pass it with its gain as
  `closed_loop(gain, plant, observer=obs)`, put the two in a controller box, or design
  the pair with `lqg`. The designs read the plant's command port for `B` (every other
  input held at its nominal; `LQRPlanner` too) and, for an observer, its `y` for `C`: a
  plant without `y` is refused, and another output is designed on
  `linearize(plant, x_bar, of=...)`.
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
  same way (e.g. optional `TaskImpedance` `task_force` → `controller:task_force`); the
  arrow shows the pre-gravity task wrench `f_task`, not `Jᵀ f_task` or `g(q)`.
  Meshcat `native=True` keyframes rigid poses, and turns geometry that changes
  (`Arrow`, `TorqueArrow`, the MPC trail and horizon) into a flipbook: one page
  per run of frames with the same geometry, each shown by a `visible` track. The
  live clip, the saved page and the inline view are frame-accurate; their size
  grows with the number of distinct geometries. `native=False` stays the
  per-frame Python loop. In 3-D, polylines are thin ribbons (matplotlib dash
  and width), not 1-pixel WebGL lines.
  `animate(save=True)` writes the backend file: matplotlib a GIF, meshcat a
  standalone HTML page of that native animation.
- **Facades:** user shortcuts only (lazy simulation/graphics); split across
  `core/facades/` by what they do, one mixin per category and system kind:
  `simulation.py` (`SystemSimulation`: `compile`, static `compute_trajectory`,
  `compute_forced`, `compute_cost`; `DynamicSimulation`: the continuous ones;
  `StepSimulation`: `compute_rollout`), `analysis.py` (`SystemAnalysis`:
  `jacobian`; `DynamicAnalysis`: `linearize`, `transfer_function`,
  `find_equilibrium`; `StepAnalysis`: `jacobian` with `k`), `plotting.py`
  (`SystemPlotting`: `plot_trajectory`, `plot_cost`, the maps, `plot_diagram`;
  `DynamicPlotting`: `plot_phase_plane`, `plot_bode`, `plot_pzmap`,
  `plot_root_locus`, `plot_step_response`; `StepPlotting`: `plot_rollout`) and
  `animation.py` (`SystemAnimation`: `render`, `animate`; `DynamicAnimation`:
  `animate_modal`, `game`). `System`, `DynamicSystem` and `StepSystem` list
  their kind's mixins as bases. The
  analysis shortcuts are written out, so every
  editor shows their parameters, and `tests/unittest/test_system_shortcuts.py` pins each
  to its band function (parameters, defaults, forwarding, docstring target). Every facade is a two-line delegation to the tool's module and
  stores no run state on the system (RULES 2.4): `jacobian` compiles its
  evaluator per call, about a millisecond on the eager JAX path, and loops
  keep the callable from `evaluator.jacobian(of, wrt)` instead. The exception
  is `self.traj`, a convenience cache of the latest facade rollout; library
  code never reads it as an input. **MRO**
  picks `compute_trajectory` implementation; no
  façade-layer `isinstance` routers.

### Native-array equation rule

Applies to every method CONSTITUTION §2 names ('One signature'), and also to port
computes, transcriptions and the `MathematicalProgram` `J`/`h`/`g`.
Native in, native out; no `np.asarray` / `float()` inside equation paths. Convert at
boundaries (evaluators, solvers, plotting, `Trajectory`, I/O). Where an object does not
read `params` yet, the gap is tracked below.

### Parameters

- `params is None` → `self.params`; any other value overrides (never
  `params or self.params`).
- **Diagram params are nested by subsystem id**: `{"plant": {…}, "controller": {…}}`.
  `DiagramSystem.params` is a live-view property — the getter assembles
  `{sys_id: subsystem.params}` from live references (subsystems stay the
  single source of truth), the setter distributes by sys_id. Partial dicts are
  allowed (missing sys_id → that block's live `self.params`); unknown sys_ids
  raise; per-subsystem dicts are full replacements at the block level. Nested
  diagrams nest the dict recursively.
- Compile: a diagram's `bind_params=True` (`diagram.compile(bind_params=True)`) copies params into the plan; the evaluator's
  `params=None` reads them. A `params` dict passed to `f` / `outputs` / `step`
  takes the nested dict on both backends in their place. On JAX the dict is a pytree argument
  (numeric leaves required): values vary without retracing, and
  `evaluator.jacobian("f", "params")` / `jax.grad` differentiate dynamics
  w.r.t. parameters (see `examples/demos/compile/params_gradient.py`).
  `evaluator.jacobian(of, wrt)` returns a callable `(x, u, t, params)` on every
  evaluator (`(x, u, k, params)` on a step evaluator; `jax.jacfwd` eagerly on JAX,
  central differences on NumPy); the
  callable is jit-compatible for blocks written with `array_module` all the
  way through, and diagrams address internal wires as `"block:port"`, as
  output or as an additive perturbation.
- **A problem's parameters are a view of its parts** (as a diagram's are of its
  blocks): `problem.params` is `{"plant", "cost", "X", "U", "X0", "Xf", "x_start",
  "discount_rate"}`, assembled on each access from the parts, which stay the one home of
  each number; assigning a dictionary routes each entry to its part (a singleton `X0`
  moves with `x_start`; a set that holds no parameters refuses a write). Library costs
  hold their parameters on the System pattern (`QuadraticCost.params` is `Q`, `R`, `S`,
  `xbar`, `ubar`, its attributes views); a `SumCost` keys its terms by id like a diagram
  (the class name verbatim, `QuadraticCost2` for a repeat, a scaled term the id of the
  cost it scales) and routes a nested dictionary to them. Anonymous `+` and `sum()` flatten
  into one level of terms; `SumCost.of(**named)` and a cost with an `id` stay nested. A
  `ScaledCost`'s params are `{"weight": w, "cost": {...}}`. A partial dictionary fills the
  missing entries from the composite's own, a leaf's dictionary replaces it whole, and an
  unknown id raises. A cost without parameters has `params = None`, an empty pytree;
  `FieldCost`'s params are its weight. Each set owns its `params`
  (`None` by default). Planners read each part's own parameters; a random plant's
  nominal draw is `nominal_plant_params(problem)`.
  Online façade on `solve_trajectory_from` / `compute_command`: `params=None`
  or `{}` binds `x0` only; `params={"scene": …}` raises `NotImplementedError` (moving
  obstacles online; binding the plant, cost and set parameters per solve is planned as S76).
  **Gap:** call-time `params` on the base `Shape` and field primitives, and the library
  sets reading their own `params` (A5).

### `DiagramSystem`

Subclasses :class:`~minilink.core.system.DynamicSystem` (diagrams are continuous
evolution systems with a compile fast path). Composes subsystems by named ports;
flattens state; compiled `ExecutionPlan` is the main execution path (reference
recursive path must stay equivalent).
Wiring, port gather and params nesting live on `WiredDiagramMixin` in `wiring.py`;
the port-dependency graph is `topology.py`: `check_algebraic_loops` (raises on a loop,
returns nothing; also a diagram method) and `evaluation_order_of` (the port order the
compilers read); `DiagramSystem` adds flow-only `f`,
`compile`, `reconstruct_internal_signals`, and `trajectory_of`; the wiring
methods reach `DiagramSystem` through the mixin.
`loop.trajectory_of(plant, traj=None)` returns one block's trajectory as that
block saw it — its own state and the input it received — on the diagram
trajectory's grid (default: the last one computed). A cost written for the
plant alone scores the plant inside any loop:
`loop.compute_cost(cost, of=plant)`. The input is recomputed from the
stored states with the blocks' current `params`: the ones the simulation used,
unless they were changed after simulating.
Every `System` has the two cost shortcuts over that path:
`sys.compute_cost(cost, of=None, traj=None)` returns `J` as a number and
`sys.plot_cost(...)` plots the running cost and `J(t)`; `of=None` scores the
whole system in its own `(x, u)`, `of=block` one block of a diagram, and `traj`
defaults to the last trajectory computed (a user shortcut; library code passes
it).
`connect()` validates port existence and dimensions at wiring time and is
quiet by default (`connection_verbose=False`; set `True` for one line per connection).
A diagram boundary output (`connect_new_output_port`, and every shortcut's `y`)
derives its direct feedthrough from the current wiring — the boundary inputs its
source reaches through `dependencies` edges — unless declared explicitly. A closed
loop nested with `add_subsystem` therefore declares what the same leaf would, and
closing an outer loop around it is not a false algebraic loop.

Shortcuts (`core.operators`, `core.closed_loop`): `+` flat add only, `>>` series, `@` closed loop
(``closed_loop(C, plant, reference=, disturbance=, noise=, filter=, observer=)`` is
`@` with options: the keywords are words, the ports the letters ``r``, ``d``, ``n``;
each is `True` (an edge port), `False` (none) or a source block driving it inside
the loop, named after the word (``reference=Step()``,
``disturbance=WhiteNoise(...)``); a plant that declares ``d`` or ``n`` owns it,
otherwise a visible `Sum` adds it to the command (``disturbance_sum``) or the
measurement (``noise_sum``); `@` keeps the single input ``r``),
`autowire()` connects by name only: each unconnected input takes the one output
with the same name and size, an output may feed several inputs, and an input
with no match or several stays open and is listed in a warning (`strict=True`
raises); a disturbance `d` or a noise `n` left open reads its nominal value
silently. It never inserts a block.
Sources name their signal at creation (`Step(port="r")`), blocks their ports
with `rename_ports` (the renames of one call apply at once; a collision with an
untouched port is refused). `+` and `>>` lay a diagram operand out flat; only
`@` nests, a box on either side under its role name as one block.
Explicit `add_subsystem` / `connect` remains canonical for general topology.

**No operator modifies its operands.** `+`, `>>` and `@` (the continuous loop
`ctl @ plant` and the sampled loop `ctl % dt @ plant`), like `feedback()`, return
a new diagram that shares the operands' blocks: a diagram operand is laid out
flat in the result with its ids, wiring and boundary ports, and the left operand
keeps its blocks, wiring, ports, name, `x0` and camera settings, which `+` and
`>>` carry into the result. Blocks are shared, never copied, so `a >> b >> c`
chains stay cheap. `connect()`, `autowire()` and `expose()` are methods and
change the diagram they are called on. Known gap (on the workboard): `step_diagram % dt`
on a user-built `StepDiagramSystem` adds its boundary ports to that diagram.

**`A >> B` connects one output of A to one input of B** by a fixed rule on their
edges, with no memory of how either was built: A has one output and B one input;
else exactly one output of A and one input of B share a name; else A has `y` and B
has `u`; else it refuses. A diagram counts its edge (its boundary outputs and its
free boundary inputs), a leaf its ports, and a leaf left operand puts all its
inputs on the edge. Sizes are a check, never a choice. One guard, the only special
case: a source into a block that reads both `r` and `y` is refused, since the
name match would feed the measurement (`Step(port="r")` names the signal). The
result keeps A's free inputs as its inputs and B's `y` (else its only output) as
its output `y`. `feedback(L)` closes an open loop on its command input
(`command_port`: `u`, else the one free input), refuses a block or a diagram
that reads its own measurement, and brings back `y` or the only output.
**`expose(*names)`** names the edge of a box: the open inputs of a name become
one edge input feeding all of them, the one inside output of a name becomes an
edge output, and `"x"` with no such port is the box's own state as an output.

**Shortcut subsystem ids.** `@` names its two sides `controller` and `plant`,
`feedback(leaf)` names the leaf `plant`; `+` and `>>` keep each block's own
name (``System.id``, else the class name verbatim: `Step`, `PID`, `Integrator2` on a repeat).
The reserved names `plant`, `controller` and `observer` are never suffixed: a
second block under one of them is refused.
Block titles in ``plot_diagram()`` read ``name::id``.
A diagram nested with ``add_subsystem`` is drawn expanded: its blocks sit in a
box labelled ``name::id``, at every depth, and the wires go straight to the inner
block ports, with no boundary nodes. ``plot_diagram(expand=False)`` draws each
nested diagram as one block with its boundary ports.
:func:`~minilink.graphical.diagrams.build_diagram_topology` accepts
``abstract_boundary=True`` to omit external Inputs/Outputs routing nodes and record
``boundary_inputs`` / ``boundary_outputs`` port anchors (used by hybrid export).

Visualization: subsystem `"world"` geometry merges into one shared diagram
`"world"` frame; only articulated frames get `{sys_id}:` prefixes.

### Control laws: the port names are the declaration

**Compensators and the Error block.** Classical laws are one family: what is
measured decides the derivative (one measurement per axis, the filtered derivative of
`PD` / `PID`, a pole at `−1/tau`; a measured `[q; dq]`, the rate as read by the impedance
pair), the limits are parameters with infinite defaults, and the integrator hold, the
filtered rate and the clip are the three functions of `control/siso.py` every member
calls. They are written once on the tracking error and take a port-layout switch:
`ports="error"` declares one input `e` and command `u` (the compensator
form, `P`, `PI`, `PD`, `PID`, `Lead`, `TransferFunction(ports="error")`),
`ports="reference"` declares `r` and `y` (the controller form the generic
framework uses). A bare `TransferFunction` stays a plant (`u`, `y`). An
error-driven block (`core.roles.block_roles`: an input `e`, or a single
input with no measurement — a compensator, a transfer function, a plant, a
series diagram `C >> G`) gets an `Error` block (`e = r - y`, ports `+`, `-`,
`e`) inserted by `core.closed_loop` (`@`, `closed_loop` and `feedback` alike). `@` always means controller at
plant and refuses a number on the right; `feedback(sys)` closes any such system
on itself, through a `Gain` (`feedback(sys, K)`) or a sensor or filter block
(`feedback(sys, F)`), the return block named `filter` either way;
`closed_loop(C, plant, filter=F)` puts the same block between the plant's `y`
and what reads the measurement, a noise `n` entering before it, and refuses a
controller that reads `x` without an observer (a filter on the state is an
observer). `closed_loop(gain, plant, observer=E)` wires the estimator by role:
the plant's `y` and the controller's `u` go into `E` (named `observer`), and
`E`'s `x` (labelled x̂) goes to a controller that reads `x` alone, so the plant's
own state is never read; it is the loop the `lqg` box gives. Positive
feedback still uses a signed `Sum`. A scalar error against a vector output takes
component 0 through a visible `Demux`; equal dimensions close a vector loop;
anything else raises and names `feedback(sys, of=(port, index))`. No `Loop`
block, no new operator.

A controller is an ordinary `System`: explicit ports, and `ctl` as the port
compute — `ctl` *is* the control law. **The port names are the declaration**,
and `minilink/core/roles.py` reads them, never a class or an `isinstance`:

- `block_roles(block)` says what a block measures (`y`, `x`, or both), its
  reference `r`, its command `u`, and whether an `Error` block drives it (an
  input `e`, or the one input of a plain compensator; a disturbance `d` or a
  noise `n` is never the entry);
- `command_port(sys)` is the input a decision drives on a plant or a box: `u`,
  else the one input that is not `d` or `n`;
- `roles(diagram)` returns the blocks under the reserved subsystem names
  `plant`, `controller` and `observer` (the observer may sit inside the
  controller; a leaf is its own plant; a sampled loop answers from its two
  sides). One of each per diagram level: a second block under a reserved
  name is refused, never suffixed, so two loops cannot be laid side by side
  (`loop >> loop`, `loop + loop`): run them separately and compare.

The one explicit hint left is `plot_space`, read only by `plot_control_law`
(`plot_space_of`) where the ports cannot say the sweep: the `(e, de)` and
`(q, dq)` planes of the robotic laws, and `None` for a solver call such as MPC.
Otherwise the ports decide: `x` sweeps the state, `y` with a same-size `r` the
error `e = r - y`, `y` alone the measurement, an error-driven block its entry.
There is no profile table and no port override attribute.

The port names drive three tools:

- **`@` composition** reads `block_roles` (the sampled loop through
  `resolve_standard_feedback` and `default_computer_boundary_ports`): every
  controller input named `y` or `x` is wired to the plant output of the same
  name (a mechanical plant's `y` is its state `[q; dq]`, read straight; a
  plant box gives its own state), a controller with neither gets an `Error`
  block on its command entry, the loop's output is the plant's `y`, and a
  box on either side is nested under its role name; dimensions are a check,
  never a choice.
- **`plot_control_law()`** on every `System`, beside the generic
  `plot_input_output_map()`. There are no controller base classes: a
  controller is a `System`, or a `DynamicSystem` when it has state, and the
  port names are what make the method draw. The zero-argument call sweeps
  the textbook slice the ports or `plot_space` name (error space for
  error-driven laws, absolute measurement or state otherwise), pinning the
  reference, unswept components, and any internal state (`x0`) at nominal; a
  block with no law to draw (a plant, a filter, `plot_space = None`) refuses,
  and `plot_input_output_map()` sweeps any input of any block. Engine in `graphical/port_map.py` (module
  twin for third-party blocks).
- **`PolicyEvaluator(policy=block)`** adapts a block that reads the state on
  `x` (or `y`) and commands `u` into `u = policy(x)` (grid state on the
  measurement port, reference pinned).

A block with other port names still simulates, compiles, and wires by hand —
the standard names only unlock the shortcuts and the tools. A law on `y` with
a same-size `r` and a law on `y` used absolutely wire identically and differ
only in what `plot_control_law` sweeps. The library's control modules, by what
they read and what the zero-argument sweep draws:

| Module | Measurement | `plot_control_law` sweep |
| --- | --- | --- |
| (a law on `y` used absolutely, `u = c(y, r)`: learned policies) | `y`, `r` of another size or absent | measurement, `r` pinned |
| `impedance.py` (`Impedance`, `IntegralImpedance`) | `y = [q; dq]` dim `2n`, the rate measured so no filter state; `Kp`, `Kd` are `n×n` (a scalar or vector is diagonal, a matrix couples the axes), `Ki` per axis; the same limits and integrator hold as `siso.py`; optional robotic `+ g(q)` via `robotic.py` | `(e, de)` plane (`plot_space = "error_qdq"`) |
| `state.py` | full state `x`; constant gain `K` (`StateFeedback`, what `lqr` returns: `u = ubar - K (x - r)` on a state reference, `r` defaulting to `xbar`; `OutputReference(K, N)`, the textbook output reference `u = ubar - K (x - xbar) + N r`, `r` of the output's dimension) or a gain schedule `K(t)` interpolated between samples and held or replaced past its end (`TimeVaryingFeedback`, what `lqr_finite_horizon` returns; the schedule itself comes from `lqr_gain_schedule`, the Riccati differential equation propagated exactly through the Hamiltonian system); feedback around a reference trajectory `u = u_d(t) - K(t)(x - x_d(t))` (`TrajectoryFeedback`, no reference port — the trajectory is the reference; what `trajectory_lqr` returns: the plant linearized at every sample of the reference, the Riccati equation swept backward interval by interval from the stationary solution at the end point) | `(x0, x1)` |
| `siso.py` (`P`, `PI`, `PD`, `PID`) | `e` per axis, or `y` with `r` (decoupled loops) | error per axis |
| `robotic.py` (task laws) | Joint ``[q; dq]`` feedback; internal FK/J; optional ``+ g(q)``; optional task-force arrow | `(q0, dq0)` absolute (`plot_space = "absolute_qdq"`) |
| `robotic.py` (kinematic laws) | Joint ``q`` only; outputs ``dq`` for speed-controlled plants | measurement (`plot_space = "measurement"`: the task reference is not a joint error) |
| `modelbased.py` | ``y = [q; dq]``; computed torque; Pyro sliding mode ``τ = ID(q,dq,ddq_r) - K(q) sign(s)`` | `(q0, dq0)` absolute (`plot_space = "absolute_qdq"`) |

**Controller naming conventions.** PID-style gain params keys are
uppercase (`Kp`, `Ki`, `Kd`, `K_null`) across `impedance`, `siso`, `robotic`,
and `modelbased`; non-PID shaping constants keep their textbook lowercase
names (`tau`, sliding-mode `lam` / `gain` / `nab`). `tracking_ref` defaults
follow the law's nature and are deliberately **not** uniform: spring-damper
and kinematic laws default to regulation (`tracking_ref=False`,
``r = pos_d``); model-based trajectory laws (`ComputedTorque`,
`SlidingMode`) default to tracking (`tracking_ref=True`,
``r = [q_d; dq_d]``).

**Embedded-model params rule (frozen).** Model-based controllers
(`JointImpedance`, `TaskImpedance`, `ComputedTorque`,
`SlidingMode`) hold a reference to a plant object and call its model
hooks (`g(q)`, `H(q)`, `inverse_dynamics`) with ``params=None`` — the embedded
model always reads that plant object's **live** ``self.params``. The
controller's own params dict carries gains only and is never forwarded to the
embedded model. Consequences: mutating ``plant.params`` updates both the real
plant and the controller model (they are usually the same object — nominal
model-based control); diagram-level ``params={"plant": {...}}`` overrides, the
evaluators' included, reach the plant subsystem but **not** the
controller's embedded copy (which is exactly the model-mismatch semantics —
override the plant to study robustness). No forwarded model-params channel is
provided.

### `Trajectory`, sets, costs, geometry

- `Trajectory`: `t (N,)`, `x (n,N)`, `u (m,N)`, optional `signals`; array-module native
  (JAX arrays kept) and a JAX pytree, its reporting verbs (`t0`, `tf`, `resample`, `save`,
  the plots) NumPy by contract; a loop's run carries `roles`, so `traj.of("plant")` is the
  plant's run by static indexing.
  `print(traj)` is `N`, the time span, and the array shapes, not the sample dump.
- Sets: `margin ≥ 0` feasible; `contains` is that test on the array module (a traceable
  boolean under JAX); `sample(key, n=None)`
  follows the distributions' draw convention below (`BoxSet.sample` traces);
  `bounding_box()` is the tightest box containing the set, or `None` — tools that need a
  box (decision bounds, a grid's extent, a sampler's box) ask for it and never probe the
  set's type. Compose with
  `&` → `IntersectionSet`. `margin(z, t, params=None)`: a set that holds parameters
  stores `self.params` and reads it when `params` is `None`; the tools pass nothing, so
  each set reads its own. The library sets hold none yet (the gap in 'Parameters').
- Distributions (`core/distributions.py`): a duck type — `dim`, `mean` (an array),
  `sample(key, n=None)` with `key` a NumPy generator, an integer seed or a JAX PRNG key
  (JAX arrays out, traceable); `(dim,)` for one draw, `(n, dim)` with `n`; `support` an
  optional `Set`. `Uniform(lower, upper)` holds its `BoxSet`; the same `sample` convention
  holds on sets. `Gaussian`, `Uniform`, `Particles`; sets are support,
  distributions are probability.
- Randomness (`blocks/sources.py`, `core/distributions.py`): a `Distribution` has no time; a
  noise signal is a block holding draws over a sample period. `WhiteNoise(p, *, psd,
  sample_period, seed, hold)` is white noise of two-sided intensity `psd`, each sample
  `w_k ~ N(0, psd / Δ)` held over `Δ` (`hold="zoh"`; `"linear"` interpolates), so the physics
  does not change with the period; `seed`, `sample_period` and `psd` are params, and `h`
  computes `w(t)` from `(t, params)` with a counter-based cipher (Threefry-2x32, the same bits
  on NumPy and JAX), so the block is pure, traces, and edits without a refresh. `seed=None` is
  the mean. `System.realize(key)` returns the params of one realization: every random block
  reseeded under its own stream, named by its id (a diagram nests it like `params`), or every
  seed `None` for `key=None`; the analysis verbs (`operating_point`, `find_equilibrium`) and a
  `PlanningProblem` on a random system resolve `params=None` through `realize(None)`, so a
  noisy loop is linearized and planned at `E[w] = 0`. Three periods: the block's `Δ`, a
  controller's period, the solver's step. A block publishes `Δ` as
  `solver_info["sample_period"]` (a diagram gathers the minimum) and as its time constant;
  with no solver named the simulator takes fixed-step RK4 at the largest `Δ / n` at or under
  the smooth policy's step, one step per sample when no block declares a time constant, since
  the period is the resolution the user chose, and `dt` buys a finer one (the
  realtime simulator reads only the time constant). A step that does not divide `Δ`, a forced
  adaptive solver, two blocks sharing a seed, and a declared disturbance the stepped tools
  never read are announced. Kalman reads `Q = B_w W B_wᵀ`, `R = D_v V D_vᵀ` for the
  continuous filter, `Q_d = B_w W B_wᵀ Δ`, `R_d = D_v V D_vᵀ / Δ` for the discrete one.
- Costs: `g(x,u,t,params=None)`, `h(x,t,params=None)` on `CostFunction` in `core`; attach
  to `PlanningProblem`, not the plant. The cost is the integrand only: `integrate(traj, *,
  discount_rate, params)` is its trapezoid on the trajectory's array module, `g_max(X, U)`
  and `h_max(X)` its upper bounds over boxes (exact for a quadratic, sampled otherwise),
  `traj.with_cost(cost)` the plotting signals. Compose with `+` → `SumCost` and `*` →
  `ScaledCost` (e.g. `base + w * obstacle_cost`); parameters nest by term id ('Parameters').
  The horizon, the discount, the sets and the price are the problem's.
- Geometry (`geometry.py`; its planned package `core/geometry/` is in §3): `Shape.sdf(p)` is the signed distance to a workspace
  *solid* — `< 0` inside (occupied), the dual of an allowable `Set` (`margin ≥ 0`).
  Primitives `Sphere`/`Box`/`Union`/`Inflated`; native-array math path (NumPy and
  JAX-traceable). `sdf(p, t, params)` is threaded by the spatial scene pipeline;
  primitives read frozen dataclass fields only (the gap in 'Parameters'). Foundation for hard obstacles in a :class:`~minilink.planning.spatial.scene.Scene`,
  exported as a free-space `Set` (hard constraint) or soft `CostFunction`.
- **Sign convention** (shared): nonnegative ⇒ feasible/free — `Shape.sdf > 0` outside,
  `ClearanceField.value ≥ 0` collision-free, `Set.margin ≥ 0` feasible. The core stores
  signed *physical* quantities (clearance in length units, density in cost units), never a
  bounded `[0,1]` score: an SDF keeps a well-scaled gradient everywhere, which a squashed
  occupancy would lose. Normalized/occupancy views are derived at the **edge** via
  `as_cost(shaping=...)`, not stored in the field.
- **Notation collisions** (inherited, closed list; RULES 5.4): `h` — output map on a
  `System`, terminal cost on a `CostFunction`; `g` — running cost, `MechanicalSystem.g(q)`
  gravity, `MathematicalProgram.g ≥ 0`; `B` — `MechanicalSystem.B` actuator map,
  `StateSpaceSystem.B` input matrix; `C` — `MechanicalSystem.C` Coriolis,
  `StateSpaceSystem.C` output matrix. One meaning within each object; no new collision is added.

## 5. Compilation And Simulation

**Build vs run.** Law: CONSTITUTION §6, 'Build vs run'. The frozen structure is the
topology, the port dimensions and the backend; forcing and rollouts are runtime too.

Before lowering, every leaf's textbook hooks are probed once at
`(x0, u_nominal, t=0)` — `f` / `step` and the constructor-made `y` / `x`
ports — and a wrong-length result (or a wrong-length `x0`) raises `ValueError`
naming the block, the hook, and both shapes; a bare scalar is accepted for
`n = 1`, custom port computes are not probed. Leaf, diagram, and step-diagram
compiles all run this check.

Speed lives in batches and compiled integrators, not in single calls: a
jitted `f` call costs about the same as the NumPy one (dispatch dominates),
while `rollout_batch` and the compiled `rollout` run 1000 rollouts of
1000 RK4 steps in tens of milliseconds. Quote those; never a per-call ratio.

`compile(system, backend)` returns one evaluator class per system kind, on either
backend (`backend` picked at construction; JAX imported only by a JAX evaluator):

- :class:`DynamicSystem` → `DynamicEvaluator` (`evaluators/dynamic.py`): `f`, `outputs`,
  the step model
- :class:`DiagramSystem` → `DiagramEvaluator` (`evaluators/diagram.py`): the same, and the
  internal `signals`
- static :class:`System` (`n=0`) → `StaticEvaluator` (`evaluators/static.py`): `outputs` only
- :class:`StepSystem` → `StepEvaluator`, a step diagram → `StepDiagramEvaluator`
  (`evaluators/discrete.py`): `step`, `outputs`, state-only `rollout`; no `f`

On JAX a continuous evaluator also offers `rollout_batch(x0s, u_sequences=None, *, t0,
dt, n_steps, params=None)` — one `vmap` of the RK4 / ZOH rollout over a family of
initial states, input sequences, and params (a params leaf with one more
dimension than the compiled value is swept); the research facade for
parameter-family experiments.

Leaf `compile` accepts `numpy` or `jax` only (`auto` / `direct` are simulator /
transcription concepts — see :mod:`minilink.core.backends`).

Diagrams → `ExecutionPlan` → diagram evaluator. **`outputs()` are boundary outputs
only** (dict keyed by port id, not diagram internals); a diagram's internal signals are
`signals(x, u, t, params=None)` (the buffer) and `compute_internal_signals_dict` (keyed
`"block:port"`). Evaluators do not mirror model `h` as a separate evaluator API.

**One body per method, compiled on JAX.** Every evaluator method takes `params=None`
(`None` reads the parameters compiled in; a dict replaces them, a diagram's block by
block):

    ev.f(x, u, t=0.0, params=None)                                            # dx; a continuous evaluator
    ev.outputs(x, u, t=0.0, params=None)                                      # the boundary ports
    ev.step(x, u, t, params=None, *, dt, solver="rk4", substeps=1)            # x_{k+1}, u held
    ev.rollout(x0, u_seq, t0, dt, params=None, *, solver="rk4", substeps=1, hold="zoh", n_steps=None)
    ev.rollout_batch(x0s, u_sequences=None, *, t0=0.0, dt, n_steps=None, params=None)   # JAX: a family, one vmap

A step evaluator has `step(x, u, k=0, params=None)` (the system's own map on the integer
tick, no knobs), `outputs` and a state-only `rollout(x0, *, n_steps, u=None, params=None)`;
a static evaluator has `outputs` only. On NumPy each method is its plain body. On JAX each
is marked `@compiled` (a step evaluator's `rollout` through its scan, returned as a NumPy
`StepRollout`) (`core/compile/evaluators/compiled.py`): `jax.jit` of the body, built
on first use and cached per evaluator, the knobs (`solver`, `substeps`, `hold`, `n_steps`)
static. In a `params` tree the numbers and arrays are traced; a leaf that is not a number (a
Python `bool` flag, a string) is held static, so a block that branches on a flag in its
`params` compiles with the dict passed too. A compiled method is fast from Python and composes inside an outer `jit`, `grad`,
`vmap` or `scan` with the numbers of the uncompiled body, so there is one name per method:
no parametric twin, no trace tier, no `jit` namespace.

**The step model.** Every sampled consumer steps a continuous plant through `step` and
`rollout`, written once (`core/compile/evaluators/stepping.py`) for both backends.
`solver` is `"rk4"` or `"euler"`, `substeps` splits each period, `hold="zoh"` holds each
row of `u_seq` (shape `(N, m)`) and `hold="foh"` reads `N + 1` knots, linear between
them; `u_seq=None` holds the nominal input for `n_steps` periods; `rollout` returns
`[x_0, …, x_N]`, `t_k = t₀ + k dt`. `discretize`, the RL environment, the gym bridge,
value iteration (`StateSpaceGrid.successor`), shooting, the fixed-step solvers, RRT,
the Lyapunov sampler, the problem evaluator and the hybrid and realtime simulators all
call them, or the one-period formula they share (`step_body`). The adaptive
SciPy solver adapts `f` for `solve_ivp` itself (`simulation/solvers/scipy_ivp.py`).
`discretize(plant, dt, *, solver="rk4", substeps=1) -> StepSystem` is the plant one sample
period later, the input held: `solver` is `"rk4"` or `"euler"` (`substeps` per period) or
`"exact"` (the matrix exponential, a linear model only). Its `step(x, u, k=0, params=None)`
takes the integer tick and binds `t_k = t₀ + k dt`; the sample time is `dt` alone, never a
key of the plant's `params`. The hold belongs to sequences (`rollout`), never to a step.

Layout of `core/compile/`: `compiler.py` (`compile`, `compile_auto`, `compile_diagram`,
`compile_step_diagram`, the shape probe), `plan.py` (`ExecutionPlan` and `StepExecutionPlan`, their
operation rows, and the builders),
and `evaluators/`: `base.py` (the `Evaluator` base, the continuous step
model, `gather_u`), `dynamic.py`, `diagram.py`, `static.py`, `discrete.py` (one class per
system kind, both backends), `compiled.py` (the mark), `stepping.py` (the step model's
formulas and the loop / `lax.scan` that runs them), `jacobian.py` (`jacobian`: central
differences on NumPy, `jax.jacfwd` on the uncompiled body on JAX).

**Frozen evaluator subset.** The stable evaluator API is the methods above: `f`,
`outputs`, `signals`, `step`, `rollout`, `rollout_batch` and `jacobian`, each with
`params=None`, and `nominal_input`.

Keep `ExecutionPlan.output_slices` and `external_output_slices` aligned.

**Default sim API:** `compute_trajectory` / `compute_forced` dispatch by **class MRO**
(façade mixins), enforced at simulator constructors:

- :class:`DynamicSystem` (including :class:`DiagramSystem`) →
  :class:`~minilink.simulation.simulator.Simulator` (ODE integration or diagram
  evaluator on a time grid)
- static :class:`System` leaf (`n=0`) →
  :class:`~minilink.simulation.static_simulator.StaticSimulator` (time grid +
  boundary outputs in `Trajectory.signals`; not state evolution)
- :class:`StepSystem` → `compile().rollout(...)` or `compute_rollout(n_steps=...)`
  (state-only :class:`~minilink.core.discrete.step_rollout.StepRollout` ``(k, x, u)``; evaluators
  and the façade do not record boundary signals — logging is owned by
  :class:`~minilink.simulation.discrete.computer.Computer` / :class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimulator`; not `Simulator`)
- :class:`~minilink.core.discrete.hybrid_diagram.HybridDiagram` →
  :class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimulator` or façade
  `compute_trajectory` / `compute_forced` (hybrid :class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimResult`)

`animate` / `plot_trajectory` live on `SystemAnimation` / `SystemPlotting` for continuous and static
systems; :class:`~minilink.core.discrete.hybrid_diagram.HybridDiagram` mirrors the pattern with
``self.traj`` (plant :class:`~minilink.core.trajectory.Trajectory`) and
``self.last_result`` (:class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimResult`).
Auto-sim fallback calls `compute_trajectory` (MRO picks engine on homogeneous diagrams).

**Two static paths:** a static *leaf* (`Gain`, `Step` source) uses
`StaticSimulator` — empty state, outputs in `traj.signals`. A static-only
*diagram* (`n=0` stacked state) still subclasses `DynamicSystem` and uses
`Simulator` with the diagram evaluator (signal-flow on a time grid).

**Unconnected inputs read their port nominal value — by design, and silently.**
An input left unwired is a constant at its declared `nominal_value` (the
Simulink "ground" semantics), never an error or a warning; `plot_diagram()`
shows it as unconnected. Time-varying sources belong in the diagram; forcing
via `compute_forced`. Facades default `compile_backend="numpy"`.

**Automatic grid.** With neither `n_steps` nor `dt`, the solver is chosen first
(user choice, else `euler` for discontinuous plants, else `scipy`) and the grid
is sized to it: adaptive solvers report on `DEFAULT_N_STEPS = 10001` points (a
plotting resolution — the integrator picks its own steps); fixed-step solvers
take `dt` from `solver_info["smallest_time_constant"]` × `AUTO_DT_SCALE` (0.1,
one owner in `simulation/time_grid.py`), discontinuous loops included. A diagram's hint is
the smallest over its stateful subsystems and any block that sets its own; a
static block left at the default does not count. Both hints are bubbled again
at every `refresh()`, which `Simulator` runs before solving, so a hint belongs on
the block: one set by hand on the diagram is replaced. `StaticSimulator` shares
the 10001-point default.

Solver presets: `scipy`, `scipy_stiff`, `scipy_max`, `scipy_ultra`, `scipy_lsoda`,
`euler` (variable knot spacing), `euler_fixedsteps` (uniform grid via the step
model's Euler rollout), `rk4_fixedsteps` (the RK4 rollout: a sampled input held over
each period, a function of time read at the stage times; auto-picked on JAX only for an explicit uniform grid of ≥ 10 000 points). Both fixed-step backends refuse a non-uniform grid, with one relative check (`atol=0`) shared with the simulator.

### Discontinuous closed loops

A discontinuous law (`SlidingMode`'s `sign(s)`) on a continuous closed loop sets
`discontinuous_behavior`, which diagrams aggregate; the automatic solver choice then takes
explicit Euler at `dt = AUTO_DT_SCALE × τ_min` and warns on every solve (louder when
`rk4_fixedsteps` or a SciPy solver is forced). The reason: the diagram evaluator re-solves
the feedback at every call to `f`, so RK4's four sub-steps can flip `sign(s)` and nearly
cancel, and the adaptive SciPy solvers may stall near the switching surface; the
`Trajectory` records the output grid only, so a reconstructed `controller:u` at grid states
is the start-of-interval map, not the effective dynamics over the step. Digital sliding mode
is a sampled loop: `HybridSimulator` holds the torque between ticks, the intended semantics
(`examples/demos/hybrid/sampled_smc_pendulum.py`).

## 6. Optimization And Planning

**NLP:** `minimize J(z)` s.t. `h=0`, `g≥0`, bounds. Pure `MathematicalProgram`;
`Optimizer(program, z0=None, method=)` binds a method preset named by algorithm: `gd`
and `adam` (the first-order backend, projected on the box), `l-bfgs`, `scipy_slsqp`
(the default), `scipy_trust_constr`, `ipopt`. The program's class is read from its
fields (`unconstrained`, `bounded`, `constrained`); `method="auto"` picks L-BFGS for the
first two and SLSQP for the third, and a method the class does not admit (`gd` on
constraint rows) is refused. `z0` defaults to `program.metadata["z0"]`.

**Planning:** `PlanningProblem` (a plain class; `problem.replace(**fields)` derives one)
owns system, sets (`X` and `U` unconstrained unless declared: `X=plant.state.box`,
`U=plant.input_box`), cost, the continuous horizon `tf`, the discount rate and the price
switch (below). `X0`/`Xf` authoritative; `x_start`/`x_goal` are representative points,
and `x_goal` never creates `Xf` (an exact terminal state is `Xf=SingletonSet(x_goal)`).
A planner that needs a finite box (value iteration's grid, RRT's bang-bang) refuses an
unbounded one with the line to write. Offline entry is
`Planner.solve()` → `PlanningSolution`, one result for every planner: the
`problem` it solved (a reference), the
`policy` (a controller block `u = pi(x)`, a `TrajectorySource` `u = pi(t)`
for the trajectory family — `solution.open_loop` — or a time-varying feedback
`u = pi(x, t)` for a finite-horizon LQR), the `solver` record (one
frozen dataclass per planner with `success` and a one-line `str`; `method`
names it), the
`trajectory` (native for open-loop planners; for a feedback law its nominal
rollout on the planner's grid, filled by `solve(evaluate=True)`, else `None`), the `evaluation`, and
`cost_to_go` where the method produces one (the DP and tabular tables; the
Riccati form of `LQRPlanner`; the RL critic when the training discount equals
the problem's). Everything on the solution is the solver's own claim under its
own model — `cost_to_go` is the method's estimate, never a measurement; a
measurement comes back from the evaluators as its own object. A planner
computes only what its solve computes natively; `solve(evaluate=True,
n_trials=)` fills the rest through the evaluator. `get_controller()` returns
the policy. **The planner's output is a controller or a trajectory, and the
script closes the loop itself** (`solution.policy @ plant`): the solution's
verbs draw what the planner produced — `plot_control_law` (the law over the
problem's box, one colour scale from its inputs), `plot_cost_to_go` (the
field over the box, anchored at the goal), `plot_trajectory` (the plan or the
nominal rollout, titled as such), `plot_cost` — never a closed-loop
simulation; the planner keeps one-line shortcuts to its latest solution.
`compare(VI=..., LQR=..., PPO=...)` → `Comparison` reads named solutions
side by side: `print` is the table of records and evaluations,
`plot_control_law` / `plot_cost_to_go` / `plot_trajectory` the overlays on
one scale; `compare(problem, VI=..., PID=pid, dt=, n_trials=, key=)` also scores every
entry (a solution, a controller, or a given `Evaluation`) by `evaluate` on the same draws
and one backend (NumPy for all when one entry does not trace), and `difference(a, b)` is
the paired `J_a − J_b`. `LQRPlanner
(problem)` is the linear-quadratic method on a `QuadraticCost` problem: `A`,
`B` at the cost's `(xbar, ubar)`, the algebraic Riccati equation for an
infinite `tf`, the differential one swept from `S` for a finite `tf`; its
`RiccatiRecord` holds `K`, `P`, the closed-loop poles (success = Hurwitz) and
the finite-horizon schedule. `control.design` stays the array-in / block-out
factory the planner calls.

**Horizon and the constraint set:** the horizon has one owner, `problem.tf` — a
finite length is a finite horizon and `h` is charged at `tf`; `+inf` or unset is an
infinite horizon and `h` is never charged (`problem.horizon_kind()`). How long to run
an infinite-horizon problem is a tool setting, `episode_length` on the evaluator and
the RL planner.
The discount rate `rho` is the problem's, `problem.discount_rate`; planners convert it
with `problem.discount_factor(dt)` (DP `alpha`, RL `gamma`). `X` and `U` are hard
constraints in every tool: a run that leaves them is a failure. **The price switch**
`infeasible_cost` is `None` (an evaluation reports the plain cost and the facts), `True`
(the price is `problem.cost_bound()`) or a number. `cost_bound(tf=None, *,
discount_rate=None, dt=None)` is `g_max ∫₀ᵀ e^{−ρt} dt + e^{−ρT} h_max` over the declared boxes
(with `dt`, a stepped tool's sum `Σ_{k<N} γᵏ dt = dt (1 − γᴺ)/(1 − γ)`, `γ = e^{−ρ dt}`, in place of the integral),
else the plant's declared ranges; always finite (`1e6` with a warning naming
`infeasible_cost` when no bound exists). Trajopt and RRT keep the sets exact; value
iteration, tabular learning, the RL environment and the Gym bridge always price a failure
by `problem.infeasibility_price()` (the number, else the bound on the tool's own
discount and period; value iteration sweeps to convergence, so its sum is `Σₖ αᵏ dt = dt/(1 − α)`,
the rate `ρ = −ln α / dt`), and DP treats
leaving the *grid* as infeasible too (the table has no value off its domain). The learners also
take a **training zone** (`ReinforcementLearningPlanner(training_zone=)`, the
plant's state box by default): leaving it *truncates* the episode, the critic's
value at that state standing in for the rest, because the model is not studied
there; it is a training choice, not a constraint. An exit is always a failure:
terminating without cost would reward escaping under a cost. `h` keeps its one
job: the end of a finite horizon.

**One scoring formula** (`planning.evaluation.score_run`, every evaluation's): a run is
integrated to `tf` (no event, no cut), then

```
finite              = every x_k and u_k finite
feasibility_margin  = min over the run of X.margin(x_k, t_k), U.margin(u_k, x_k, t_k)
failed              = not finite, or feasibility_margin < 0
reachability_margin = Xf.margin(x(tf), tf)        reached = margin ≥ 0   (Xf = None: reached)
cost                = ∫₀^{tf} e^{−ρt} g dt + e^{−ρ tf} h(x(tf))  (trapezoid; +inf when not finite; no h on an infinite horizon)
J                   = cost, or cost + price · (failed or not reached) with the price on
```

A run that is not finite is scored on zeros behind a safe `where`, so its score stays finite with
the price on; its gradient is not defined (the rollout overflowed before the `where`). A run whose
states stay finite while its cost overflows scores `+inf`, price or not. A trajectory is
scored on its `plant` role; one without roles is scored as the plant's run only when `x`
has the plant's `n`, `u` its `m`, and it carries no recorded signals (a logged run), and a
trajectory with recorded signals and no role is refused.
`evaluate_trajectory(problem, traj)` scores a trajectory already held (a loop's run
through its `plant` role; a logged plant run without signals). RL *trains* on the
left-Riemann discretization of the same running cost (`r_k = -g dt`) with per-step factor
`gamma = exp(-rho dt)`; `exp(-rho t)` is not folded into the reward (that would discount
twice); its plans report `evaluate`.

**Stochastic problem:** `StochasticPlanningProblem(PlanningProblem)` adds
`x0_distribution` (its mean is `x_start`, its support `X0`),
`params_distribution` (`{name: Distribution}` over `sys.params`, dotted names
reaching a diagram's subsystem params such as `"sys.mass"`, one draw per
episode carried through the step model's `params`),
`disturbances` (`{port_id: Distribution}`, a fresh draw per step held on the
port in the stepped environment and `Sys2Gym`; `evaluate` holds those ports at their
nominal value and warns) and `criterion` (`"expectation"` default, `"worst_case"`, or `("cvar", alpha)`, the
mean of the worst `alpha` fraction; an evaluation's `objective` is this reduction of the
runs' `J`, and the RL planner, which optimizes the expectation, refuses the others). A deterministic planner given a stochastic problem plans from
the mean start and warns; `Sys2Gym.from_problem(problem, dt=)` is the
Gymnasium view (draws of `x0`, the exit rule, `h` at a finite horizon). Distributions are §4's duck type. `nominal()` is the certainty-equivalent
`PlanningProblem`. Every planner that takes a `PlanningProblem` takes the
stochastic one.

**The problem evaluator (the second verb, `planning/evaluator.py`):**
`pe = problem.compile(controller, dt, *, solver="rk4", substeps=1, backend="auto")` closes
a controller on the plant (`controller @ plant`; a source block drives the command; a built
loop is taken when its block `plant` is the problem's system) and compiles it once.
`pe.params` is the run's tree: the loop's blocks by id, the problem's own leaves (`"cost"`,
the sets' parameters, `"x_start"`, `"discount_rate"`) and each free input at its nominal
value. `pe.trajectory(params)` is one run to the horizon on the step model, every block
output recorded (`"id:port"`, the plant's input `"plant.u"`, the role `plant`);
`pe.evaluate(params, key, n_trials)` the `Evaluation` (a pytree record, one row per run:
`cost`, `J`, the margins, `failed`, `reached`, `x0`; `objective` the criterion over `J`,
`mean` / `std` / `worst` / `cvar(alpha)` and the three rates), the draws of a stochastic
problem `vmap`ped on JAX, so `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)`
reaches every gain, plant parameter, cost weight and the start. `solver="scipy*"` is the
adaptive integral on NumPy, `dt` its reporting grid; `"auto"` falls back to NumPy when the
loop, the cost or a set does not trace. `pe.program(wrt, bounds, key, n_trials,
params=)` exports the tuning `MathematicalProgram` over the leaves `wrt` (dotted paths,
`"controller.Kp"`): the objective `pe.evaluate(...).objective`, its reverse-mode gradient on
JAX, the per-trial feasibility margin as a row when `X` or `U` is bounded and the
reachability margin when `Xf` is set; `pe.pack` / `pe.unpack` move between the tree and
`z`. `evaluate(problem, controller, *, dt, ...)` is the facade. No backend clips the law:
port bounds are information, and saturation lives inside the law or in a `Saturation`
block. `problem.compile` refuses a sampled controller (a `StepSystem`) and a discrete-time
plant, which join through the hybrid loop; a stochastic problem needs `n_trials`, its
draws `problem.realize(key, n)` → `(x0s, {"params": …, "disturbances": …})`, one key per
call. The program reads the same `J` as the report: with the price on, the price is a step
with no gradient, and a non-finite objective at `z0` warns once. The feedback planners'
reported `trajectory` is their own held-input rollout on their
grid (`nominal_trajectory`), the planner's evidence; `evaluate` scores the continuous loop.

**Reinforcement learning (`learning/reinforcement_learning/`):** two
policy-family planners beside DP, one per representation of the law.

`TabularLearningPlanner(problem, x_grid=, u_grid=, dt=, algorithm=)` learns
the cost-to-go table `Q(x, u)` on a `StateSpaceGrid` from transitions alone,
NumPy only: the state is rounded to its nearest node, the action taken among
the grid's input levels, and every algorithm is a moving average toward its
own sample of the cost-to-go, `Q <- Q + eta (q - Q)`: `QLearning` (target
`c + alpha min Q(x', .)`, off-policy), `SARSA` (the action taken next,
on-policy), `MonteCarloControl` (the cost observed to the episode's end).
Exploration is an object, `EpsilonGreedy` (annealed) or `UCB`. `solve()`
returns a `PlanningSolution` (the greedy `LookupPolicy` as `policy`,
`J = min Q` as `cost_to_go`); `planner.result` is the table as a
`DynamicProgrammingResult` (`J = min Q`, `pi = argmin Q`), so
`plot_cost2go`, `plot_policy` and `get_controller` (a `LookupPolicy`)
are those of value iteration, and the same grid solved by
`DynamicProgrammingPlanner` is the exact answer the learner approaches.

`ReinforcementLearningPlanner(problem, dt=, hidden=, features=, algorithm=)`
learns a neural law in pure JAX: `solve(timesteps=)` → `PlanningSolution`
(the controller as `policy`, a `ReinforcementLearningRecord`, the critic as
`cost_to_go` when the discounts match), `get_controller()` →
`control.neural.NeuralPolicyController` (`u = u_mid + u_half · squash(MLP(z(x)))`,
weights in `params["mlp"]`, features normalized by the state box or a user
map), `solve_trajectory_from(x0)` → a `PlanningSolution` carrying the
learned law's rollout.
What the algorithm trains is a `StochasticPolicy`: the controller block's mean action
with an exploration head around it (`GaussianHead` for the policy-gradient family,
`SquashedGaussianHead` for SAC), one pytree `theta = {"mlp", "head"}`, and the methods
`sample`, `log_prob`, `entropy` and `input(a)`. Critics `ValueFunction` and `QFunction` sit
on the same features. One file per algorithm under `algorithms/` (`REINFORCE`,
`ActorCritic`, `PPO`, `SAC`), each owning only its update rule and train state; three class
attributes declare the family (`on_policy`, `head_kind`, `critic_kind`). Bare JAX, no
Flax/Optax dependency. Training prints one line per update; `verbose=False` silences it.

`RolloutEnvironment(problem, dt=, backend=)` is the problem's semantics as
step functions on either backend — the reward `r = -g dt`, the exit rule and
the horizon — shared by the JAX collectors and the tabular learners, so one object owns
what a training episode means (an evaluation scores the learned law by `evaluate`).
Teaching demos write the observation map by hand
(``z = (cos θ, sin θ, scaled rate)``). `control.angle_features(angles, scales)`
is the same map as a helper for scripts. The action port is `u` when the
plant has one, else its single input port — so an inner loop `impedance @
arm` (input `r`) is a plant for an outer learned law; a step that leaves the
box or blows up (non-finite state) ends the episode. The learned law is a `System`:
`ctl @ plant` compiles on both backends, `linearize` / `jacobian` differentiate
through the network, and the step model (`ev.step(x, u, t, params, dt=…)` with
`params={"controller": ..., "plant": ...}`) gives gradients of a rollout with respect
to the policy weights. Official demos: `examples/demos/rl/` (pendulum swing-up, cart-pole,
drone, rocket); the tutorial chapter is
`examples/tutorial/11_reinforcement_learning.ipynb`; native teaching
notebooks are `pendulum_value_iteration_vs_lqr_vs_rl` and `drone_ppo`. The
spatial scene names (`ReferenceTrack`, `from_waypoints`, `Scene`, `bind`,
`car_outline`, `point_probe`, the shaping helpers, `TrackCorridorOverlay`,
`plot_track`) are exported on the `minilink.planning` facade so track demos
import through the teaching surface.

**Trajopt:** `TrajectoryOptimizationPlanner` → transcription → NLP →
`PlanningSolution` whose policy is a linear `TrajectorySource` of the knot
inputs (the transcriptions integrate a linear input between knots, so
`policy >> plant` replays the plan). `TrajectoryOptimizationRecord.success`
means *the returned plan satisfies the program constraints to
`feasibility_tol`* — nothing else: an iteration-limit stop on a feasible plan
is not a failure, a solver that reports convergence on an infeasible plan is,
and the solver's own flag stays in `message` / `stats`. The worst equality
residual / inequality margin / bound violation are recorded on the record.
Online ticks (`solve_trajectory_from`, the MPC path) report
the solver flag without a residual check. **I-level constructors** take flat kwargs
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
Online ``params`` binds `x0` only; the reserved key ``scene`` raises
``NotImplementedError`` (§4, 'Parameters'). Knot count
`n_steps` is a planner flat kwarg (or lives on a custom transcription's
options); the time grid is
`linspace(0, problem.tf, n_steps)` for finite `tf`. Single backend-native
transcription classes; no parallel JAX transcription types. NumPy transcriptions
build constraints via `dynamics_function` (the compiled evaluator's `f`).
**JAX transcriptions** embed `problem.sys.f` directly in the NLP (not the
evaluator's `f`). `compile_backend="direct"` calls `system.f`
uncompiled (escape hatch). A `MathematicalProgram` carries the native backend of
its callables in its `backend` field, and the `Optimizer` compiles with it by
default.

**RRT / DP:** same two-tier idea — flat routine knobs on
`RRTPlanner` / `RRTStarPlanner` / `DynamicProgrammingPlanner`, with
`options=` as the advanced escape. Keep fundamental seams explicit
(`extender`, `StateSpaceGrid`). The textbook DP setup
`DynamicProgrammingPlanner(problem, x_grid=, u_grid=, dt=)` builds that grid
itself (`grid=` for a custom one), and `solve()` then pins saturated
cost-to-go cells to `out_of_bound_cost` (`clean_infeasible=False` keeps the raw
table; `clean_infeasible_set(tol)` reruns the pass with another tolerance). A
callable `out_of_bound_cost` has no single saturation level, so it leaves the table as
solved.
`ValueIterationRecord.success` means the sweeps converged to `tol` (a
fixed-horizon `solve_steps` always succeeds); the record carries the sweep
count and the last cost-to-go change.

**Dynamic programming** (`planning/dynamic_programming/`): offline dynamic programming on a
continuous plant. A `StateSpaceGrid` discretizes the `PlanningProblem` — grid *extent*
comes from the sets' bounding boxes (so it stays finite), grid *validity* from
the sets (`contains` on NumPy, `margin >= 0` on the JAX precompute, so `X = bounds & free`
works on both); `grid.X` and `grid.U` are the boxes it spans, and successors from one
period of the step model, forward Euler by default (`x_next = x + f(x,u,t)·dt`; the grid's
`solver="rk4"` otherwise; the time step `dt` lives on the grid, not on `System`). `DynamicProgrammingPlanner` runs value iteration backward — `solve`
to tolerance, `solve_steps` for a fixed horizon — returning a `PlanningSolution` whose
`policy` is the `LookupPolicy` of the greedy table and whose `cost_to_go`
interpolates `J` (the raw `DynamicProgrammingResult`, `J` and `pi` as action ids, stays on
`planner.result`); out-of-domain
transitions are charged a finite `out_of_bound_cost` (pyro's
`cf.INF`), or, when it is a callable, its price `out_of_bound_cost(x_next, t)` at each
such transition's successor. Three interchangeable
backward-step backends share this workflow: `loop` (per-node Python, pyro's reference),
`numpy` (vectorized over the precomputed lookup table, the default),
and `jax` (the same backup as one jitted `lax.while_loop` with `map_coordinates`, on tables
built with `vmap`, so the plant and cost must be JAX-traceable; linear/nearest only). The JAX
engine lives in `dp_jax.py` and the progress reports in `progress.py`, so `dp.py` reads as the
textbook algorithm. `precompute` trades the `(N,A,n)`
successor table for per-sweep recomputation (memory vs time-varying support); a precomputed
grid also freezes the running-cost table, so a time-varying `f` or `g` needs `precompute=False`. `get_controller()` returns a `LookupPolicy` (a static `System`); `PolicyEvaluator`
gives the cost-to-go of any fixed law. **Function approximation**
(`dynamic_programming/approximation.py`): linear-in-the-weights fits `f_hat(x | w) = wᵀφ(x)` with fixed
features — `QuadraticFeatures(xbar)` (a quadratic form, the LQR shape; `quadratic_form(w)` reads
`(c, b, S)` back) and `RadialBasisFeatures` (Gaussian bumps, `on_grid` for a mesh of centres),
stacked with `+` — and a `LinearApproximator` that fits by least squares (`fit`) or one
stochastic-gradient step per sample (`sgd_step`). It is the seed of approximate dynamic
programming: fit a cost-to-go from value iteration, or learn one online.

**Spatial scene** (`planning/spatial/`; its planned home `core/geometry/` is in §3): two domains — **workspace** `p ∈ ℝ²/ℝ³` and
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
`X = bounds & free`, `cost = base + w * terrain`. Scene param overrides online are the reserved `scene` tier (§4,
'Parameters'): rebuild the `Scene` and recompile the parametric program.

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
free-space sampling from `problem.X` (direct `Set.sample` or rejection from the bounding box of `X`), dynamics `problem.sys.f` — so the system stays pure. After
`solve()`, `reached_goal` and `solution_node` report success vs
best-effort fallback (`return_best_effort`). The two swappable pieces are an injected
`TrajectoryExtender` (`propose(from, toward, problem, rng) → Iterable[Edge]`:
`KinodynamicExtender` forward-integrates controls, `SteeringExtender` connects exactly
via a `SteeringFunction` including `DubinsSteering`) and a `metric(a,b)` callable
(nearest-neighbour distance). Every system is an ODE, so an `Edge` always carries real
`(t,x,u)`; `solve() → PlanningSolution` with a held (`interpolation="previous"`)
`TrajectorySource` as policy and a `TreeSearchRecord`. `RRTStarPlanner` extends the attach step
with near-neighbour parent selection and cost-based rewiring (`Tree.rewire`,
`Tree.propagate_cost`); `Edge.cost` is the cost-to-come along the tree. With
`optimize_after_goal`, the search continues after the first goal connection until
the best goal cost stops improving for `convergence_patience` extensions;
`record_history` + `animate_convergence` replay tree growth and path refinement.
``live_plot`` / ``callback`` redraw the tree during ``solve``; `RRTOptions.nearest_backend`
selects brute-force or a SciPy ``cKDTree`` for Euclidean nearest queries.

## 7. Graphics And Benchmarks

Facades delegate to `graphical/`. Time plots: `signals=("x", "u", "block:port")`.
Phase plane: matplotlib default. Diagrams: Graphviz layout (notebook SVG;
script Matplotlib window). `print(sys)` is a short text
summary: name, class, `n`, ports with dimensions; a diagram adds its keys, an
`LTISystem` its `A`, `B`, `C`, `D`, a `TransferFunction` its `G(s) = (s + 2) / (s² + 3 s + 2)`.
The same inspect print lives on `Trajectory`, sets, distributions, costs,
`PlanningProblem`, `StateField`, and `MathematicalProgram` (a notebook
last-expression on a `System` is still the Graphviz block).
Plotly under `plotting` extra.

**Camera:** plain `camera_*` hints on `System` resolve to a 4×4 matrix
(`camera_matrix`) each frame via `resolve_camera_from_hints`; pass
`animate(camera=…)` for a constant matrix or callable override. One contract
for all renderers. `camera_scale=None` (the `System` default) means **auto-fit**:
the `Animator` frames the bounding box of everything drawn, fixed over the
whole animation (`fit_camera_to_frames`, margin 1.15; backdrops such as
`ground_line` and `Plane` and force glyphs such as `Arrow` are excluded via
`primitive.camera_fit = False`), so a plant whose
`params` change keeps a sensible view; a numeric `camera_scale` frames the
scene yourself. Meshcat consumes the same matrix: the mouse orbit of its viewer
is anchored at the origin, so the target is honoured by sliding the drawn world
(with the viewer's grid and axes) until the target sits there, and the eye sits
on the view-out side at the `T[3, 3]` distance (elevation capped at the
viewer's default); orbit, pan and zoom stay with the mouse, and a native
animation keyframes the slide alongside the bodies. One backdrop hint sits
beside the camera's: `scene_grid=False` hides the viewer's ground grid and axes
(an underwater or open-sea scene); the flat renderers ignore it.

All performance benchmarking lives in repo-root `benchmarks/` (helpers,
synthetic fixtures, `run_*` scripts) — outside the shipped package, importing
minilink like an external user, and not a public contract.
