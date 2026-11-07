# Minilink Technical Design

Architecture and public contracts, under the law of [CONSTITUTION.md](CONSTITUTION.md).

## 1. NumPy and JAX

NumPy required; JAX optional (the `full` extra, or `pip install jax jaxlib`), imported lazily via
`minilink.core.backends` (`require_jax_numpy()`, `array_module()`). No `minilink.jax`
package, no global mode. Explicit `backend` and evaluator backend args.

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
`__init__` files, and checked as a set by `test_teaching_surface.py`. A name reaches the root
when a lesson uses it: `StepPlanningProblem` (`minilink.planning`), `StepCost` and
`SampledCost` (`minilink.core`) stay on their bands until one does; `StepSystem` is
on the root.

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
| `core/` | `System` (+ its shortcuts, `facades/` by what they do: `simulation.py`, `analysis.py`, `plotting.py`, `animation.py`), the package import tables (`exports.py`), `DynamicSystem`, `DiagramSystem` (subclasses `DynamicSystem`), shared diagram wiring (`wiring.py`: `WiredDiagramMixin`, gather, params nesting), the port-dependency graph (`topology.py`: evaluation order, algebraic loops), the composition grammar (`operators.py`: `+`, `>>`, `autowire`; `closed_loop.py`: `@`, `closed_loop`, `feedback`; `assembly.py`: the operands laid out flat, their ids, default ports and camera, which both build on; `roles.py`: the block roles the port names declare), signals/ports (`signals.py`), backend policy & helpers (`backends.py`), the parameter trees (`params.py`: the path lookup, merge and flatten of nested `params`, one owner), `Trajectory`, sets, distributions (`distributions.py`), costs, geometry (the package `core/geometry/`: `shapes.py`, the solids by their signed distance, and `paths.py`, the reference paths, a polyline open or `closed`; **planned** in it — Track, Scene, mount, spatial Fields, course catalog: [docs/plans/geometry-module.md](../plans/geometry-module.md)) |
| `core/discrete/` | the discrete-time sibling, `x_{k+1} = F(x_k, u_k, k)` on an integer index: `StepSystem` (`step_system.py`), `SampledSystem` (`sampled_system.py`: a system on a clock), `StepDiagramSystem` (`step_diagram.py`), `SampledDiagramSystem` (`sampled_diagram.py`: the sampled loop, its holds and the plant's run inside it) |
| `core/compile/` | the compiler, the two execution plans, one evaluator per system kind |

**System libraries** — `System` subclasses you drop into a diagram, shelved by
*role in the diagram*, never by implementation technology (linear, Lagrangian,
or neural network alike):

| Package | Role |
| --- | --- |
| `blocks/` | plant-agnostic wiring: sources, `Integrator`, `TransferFunction` (and `Lead`, its lead compensator), routing (`Error`/`Sum`/`Gain`/`Mux`/`Demux`), nonlinear (`Saturation`), filters (`LowPassFilter`), neural (`MLP`), the step block `UnitDelay` |
| `dynamics/` | plants: `abstraction/` mother classes, `catalog/` by physical domain |
| `catalog/` | **teaching alias** of `dynamics/catalog/` — flat re-exports for short imports (`from minilink.catalog import Pendulum`); ownership stays in `dynamics/` |
| `control/` | control laws and design verbs: `design.py` (`lqr`, `place`, `lqr_finite_horizon`, `lqg`, `trajectory_lqr` on a linear model or a plant; their gains on arrays), `siso.py` (`P`, `PI`, `PD`, `PID`), `state.py`, `impedance.py`, `robotic.py`, `modelbased.py`, `neural.py`, `stochastic.py` (`GaussianPolicy(mean, log_std, *, sample_period, seed=None)`, `u = μ(x) + σ(x) ε(t)` with `ε` the noise block's counter signal of its seed at `⌊t/Δ⌋`, `seed=None` the mean; `log_prob` and `entropy` as methods; `SquashedGaussianPolicy` in a box), `geometric.py` (the path-tracking laws `PurePursuit`, `Stanley`, `LineOfSight`, each reading a `core/geometry` path, and the heading loop `HeadingController`); `ModelPredictiveController` re-exported from `planning/trajectory_optimization/mpc/`. The port contract is §4, 'Control laws' |
| `estimation/` | state observers and their designs: `observer.py` (`StateObserver`, the innovation form, ports `u`, `y` → `x`, role `observer`), `design.py` (`luenberger`, `kalman`, the duals of `place` and `lqr`) |

**Tools** — verbs on a `System`; they return data or plots and never define
user-facing systems (factories are fine: `linearize_matrices()` returns arrays,
`linearize()` wraps them as an `LTISystem`, and an LQR design function returns a
state-feedback block):

| Package | Role |
| --- | --- |
| `simulation/` | `Simulator`, `StaticSimulator`, solvers, forcing; `discrete/` (`clock_runner.py`: the host clock runner of a sampled loop); `realtime/` (`RealtimeSimulator`, `RealtimeInput`/`RealtimeOutput`, `PygameInput`, `LiveSignalPlotOutput`) |
| `analysis/` | one calling pattern `tool(sys, x_bar, u_bar, t, params, *, method="auto", eps)` on any `System` (the linear model as is, the rest linearized at the operating point): `jacobian`, `linearize` (→ `LTISystem`; a step or sampled system → `DiscreteLTISystem`, `x_{k+1} = A_d x_k + B_d u_k` with its period, its poles in `z`, its frequency tools at `G(e^{jωΔ})` up to `π/Δ`, its `plot_pzmap` and `plot_root_locus` with the unit circle, its `step_response` the samples `y_k` at `t_k = k Δ`), the one-channel frequency and time tools (`bode`, `pzmap`, `nyquist`, `margins`, `root_locus`, `step_response`, `step_info`, `transfer_function`, the sensitivity functions) with their `plot_` twins, `structural` (controllability, observability), `linear` (the one-channel matrix math, `minreal`), `equilibria`, `modal`, `lyapunov`, `riccati`, `discretization`. The API reference is the docs site |
| `planning/` | `problems.py`, `step_problems.py` and `step_face.py` (the step problem and the compiled step face), `results.py`, `policies.py` (a controller block read as its policy `u = π(x, t)`), `evaluator.py` (the problem evaluator, `evaluate`), `evaluation.py` (`score_run`, `evaluate_trajectory`, the `Evaluation` record), `environment.py` (`RolloutEnvironment`: the problem as step functions for the learners and MPPI), `comparison.py`; `dynamic_programming/` (value iteration, policy evaluation, `LQRPlanner`, the lookup policy), `trajectory_optimization/` (collocation, shooting; `compiled.py` the planner's face; `path_integral.py` MPPI; `mpc/`, the receding-horizon block, provisional), `search/` (RRT), `spatial/` (scenes, obstacles and the workspace fields; the paths and shapes they read live in `core/geometry/`) |
| `learning/` | policies learned from interaction: `reinforcement_learning/` (the neural planner, the tabular learners); identification and imitation later |
| `optimization/` | `MathematicalProgram`, `Optimizer` (generic NLP) |
| `graphical/` | signals, phase plane, diagrams, animation |
| `interfaces/` | lazy facade: `Sys2Gym` / `SB3Controller` (gymnasium in the `full` extra; stable-baselines3 from `environment.yml`); cosimulation / MJX planned |

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
  computed torque.)
- **Exception:** the `minilink.control` facade re-exports `ModelPredictiveController` and `PlanFollower`
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
- **Exception:** `analysis/` imports `blocks.TransferFunction`: the transfer
  function and sensitivity tools return one, so `bode` and `margins` apply.
- **Exception:** `planning/trajectory_optimization/mpc/` imports `planning.spatial.overlays`
  (its plots draw the track).
- **Exception:** `graphical/animation` imports `planning.spatial.Scene`, to
  draw a scene as an overlay.
- **Exception:** the LQR planner (`planning/dynamic_programming/lqr.py`) imports
  `control.design` and `control.state`: it solves with the design verb and returns its
  state-feedback law.
- **Exception:** `core/facades/simulation.py` imports `simulation.discrete.clock_runner`
  lazily: a sampled loop's `compute_trajectory(solver="scipy")` runs on the host clock runner.
- **Exception:** `graphical/diagrams/` imports `blocks.routing.Sum`, to draw a summing
  junction.
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
rate; step and sampled systems live on sibling types and separate compile and simulation paths
(`StepSystem` beside `DynamicSystem`), and nothing of theirs enters `DynamicSystem`, flow
`compile()` or `Simulator`.

**The sampled loop is a step diagram on a clock**. `%` means one thing: this
block runs on the computer at period `Δ`. The block's kind (its base class) decides what its
third argument is, never the operator:

| Block | `block % Δ` | Its third argument |
| --- | --- | --- |
| a continuous law (`System` with states) | its Euler difference equation, `z_{k+1} = z_k + Δ·f(z_k, e_k, t_k)` | seconds, `t_k = t0 + kΔ` |
| a static block (`n = 0`) | sampled and held | seconds |
| a tick law (`StepSystem`) | its clock | `k`, its own count of firings, never seconds or `Δ` |
| a sampled-data law (`NativeSampledSystem`) | refused: its period is set at construction | seconds |
| a box | each block on the clock | each block's own |

A static block in a *pure* step diagram (no clock anywhere) receives `t = k`. `block % Δ` is a
`SampledSystem` of kind `"law"`: never integrated inside its period (no substeps, no dense
points, never adaptive). `discretize(sys, Δ, solver=, substeps=)` is the textbook verb, a
discrete-time model (kind `"model"`) for the discrete tools (the step face, value iteration,
the RL environment, z-domain analysis), never needed to close a loop: a loop built on one is a
loop of step systems, which has only its ticks. On a step diagram (a box
of tick laws and static blocks, the MPC box among them) `box % dt` clocks each block with
its own face kept, a static block reading seconds and a tick law its count, and returns a
`SampledDiagramSystem` of the same wiring and kind (`diagram_on_clock`); a box holding a clocked
block is clocked once, block by block. The operators `>>`, `+`
and `@` compose step systems with the meaning they have on continuous ones, on the same assembly
code; the operands decide the algebra (`core/assembly.py`, `algebra_of`): a clocked operand
makes a `SampledDiagramSystem`, a tick law a `StepDiagramSystem`, continuous and static operands
a `DiagramSystem`; a static block joins any of them; a continuous operand with states beside a
step one is refused, naming `%`, except `sampled @ continuous`, the one place the two worlds
meet, where the plant becomes the plant on the controller's base clock (`plant` role, a
`SampledSystem` of kind `"plant"` the user never builds). Its integration step is the continuous
simulator's, one rule (`automatic_step_of`, §5 "Automatic grid"): RK4 at
`Δt = AUTO_DT_SCALE·τ_min` from the plant's physics, `substeps = ⌈Δ / Δt⌉`;
`dt=` at the run verbs (`loop.compute_trajectory(tf, dt=0.01)`, `loop.compute_rollout(N,
dt=0.01)`) and `substeps=` at the compiles (`loop.compile(substeps=20)`,
`problem.compile(mpc % Δ, substeps=20)`) choose another, `solver=` beside them (`"rk4"`,
`"euler"`; `"scipy"` the host runner). More than `UNROLLED_SUBSTEPS` substeps run in one rolled
`lax.scan` under JAX. A sampled diagram holds at most one continuous system, found by kind,
never by id: two are refused ("compose them into one continuous system first"). The refusals
of the grammar, each naming the fix: a clocked plant under a continuous law
(`pid @ (plant % Δ)`: write `(pid % Δ) @ plant`), a tick law meeting a continuous plant with no
clock (`UnitDelay() @ plant`). Composition stays simple: `>>` into a closed loop is always
legal, it feeds the loop's reference, and `@` binds before `>>`, so `filt >> ctl @ plant` is
the filter feeding the loop from outside; a filter on the measurement is `((filt % Δ) >> (ctl %
Δ)) @ plant` with the parentheses, or `closed_loop(ctl, plant, filter=filt)`.
A
`SampledDiagramSystem`'s base period `Δ` is the greatest common divisor of its clocked blocks'
periods (on a nanosecond grid); a block of divisor `d > 1` fires on `k mod d = 0`, its held
outputs state slots after the blocks' states (`hold_index`, labelled `controller:u (memory)`:
the memory of the last firing, one period behind the signal a run records at the same tick),
their start `hold_x0` a leaf of the evaluator's tree (`pe.params["hold_x0"]`, differentiable
like every `x0`; a layout change under a start that is not zero is refused); its compiled plan
(`build_sampled_execution_plan`, nested boxes inlined into one flat schedule) forms
`t_k = t0 + k·Δ` once per tick and hands it to every block whose equations are in seconds
(a static block, a `SampledSystem`'s seconds face), `k // d` to a tick law on a clock, `k` to a
bare tick law; the firing is a `lax.cond` on the tick on JAX, a branch on NumPy, never a
`where`. A bare tick law reads the base count of the loop it runs in, so a tick law designed
for a rate names it (`filt % 0.01`); one inside a nested box clocked slower than the loop's base
is refused at compile, since it would tick faster there than in its box alone. The loop's plant
(the one continuous system, at any depth) is what the dense run keeps the substeps of and the
host runner integrates adaptively; the dense run, the substeps and the host runner exist exactly
when the loop has one. Signals propagate within the tick: a delay is a
block. A run records signals from its own signal pass, by the one `record=` rule of every
diagram run (below, `DiagramSystem`): `compute_trajectory(record=…)` keeps each tick's value
held over its period (the end sample the last tick's) in the trajectory's `signals`,
`compute_rollout(record=)` the same at the ticks; by default the plant's input `"plant:u"`, so
`traj.of("plant")`, the plots, the animation and the MPC overlays compute nothing again; the
planning evaluator's run takes its signals from the same pass (`rollout_with_cost(signals=True)`).
The host clock runner records nothing.

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
  `QuadraticCost.from_system` included); on a linear model, `x_bar`, `u_bar` and `y_bar`,
  where its deviations are measured from.
- **Ports:** explicit, ID-first; infer `dim` from metadata or default 1. Extract
  slices with `get_port_values_from_u(u, "r", "y")`.
- **Constructor shortcuts:** `System(state_dim=0, *, input_dim=, output_dim=, expose_state=,
  y_dependencies=)`, the same on `DynamicSystem(state_dim, ...)` and
  `StepSystem(state_dim, ...)`, create the standard ports `u`/`y`/`x`. Words in the
  constructor, textbook symbols as attributes: `state_dim`, `input_dim`, `output_dim` build
  the block, `n`, `m`, `p` read its dimensions (`m` and `p` totals over the ports). A static
  `System` given an input feeds it through by definition: its `y_dependencies` default to
  `("u",)` (`()` on `DynamicSystem` and `StepSystem`, which declare their feedthrough).
- **StepSystem (discrete leaf):** same port shortcut as `DynamicSystem`; evolution is
  `step(x, u, k, params)` → `x_new`. Facades: `compute_rollout` (a `Trajectory` of the
  ticks, kept on `self.traj` like every run) / `plot_rollout`.
  A flow `DiagramSystem` refuses it at `add_subsystem` (a `StepDiagramSystem`
  likewise refuses a `DynamicSystem` with states); it meets a continuous plant
  only on a clock, in a sampled loop: `block % dt @ plant`.
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
- **Path integral control:** `MPPIPlanner(problem, *, dt, n_rollouts, temperature, noise,
  control_cost, n_iterations, solver, seed)` (`planning/trajectory_optimization/path_integral.py`,
  JAX) plans a held input sequence `U` on a grid of `dt`: each pass samples `V^k = U + ε^k`,
  rolls each out on `RolloutEnvironment(reward="stage")`'s `step(x, t, u, key, params=None, *, t_origin=None)` (the one owner of the stage cost, the exit cost
  and the terminal cost), weighs the samples by `exp(−(S^k − S_min) / λ)` and moves `U` by the
  weighted noise. Its compiled face (`n_z = N m + 1`, `shift` holding the last input,
  `command` the held input, `predicts = False`) is the trajectory face, so
  `ModelPredictiveController(MPPIPlanner(...))` runs in every form; its tree is the problem's
  leaves (`cost`, `discount_rate`, `X`'s numbers among them), `temperature`, `noise`,
  `control_cost` under `params["method"]`, `seed` under `params["seed"]` with the tick folded in.
  `pv.samples(params, z0, n_shown)` redraws a solve's own samples on that key (one `passes`
  owns the sampling for both), so after a run `mpc_sampled_futures(run, mpc, loop=)` rebuilds
  every tick's fan from the solve inputs read back through the loop
  (`SampledDiagramEvaluator.block_input`: the block's ancestors alone, the solve not run);
  `mpc_animation_overlays(..., samples=n)` draws them and `plot_sampled_futures` gives the
  static figure. Each face reads its plan as a trajectory (`planned_trajectory`), and the MPC's
  verbose line appends a planner's own account (MPPI's `S_min`, `effective_samples` of `K`).
- **MPC on the planner's face:** `ModelPredictiveController(planner, *, inputs=None,
  delay=False, warm_start=True, backend=None, verbose=False)` closes a planner's compiled face
  `pv = planner.compile()` on the loop, one solve per tick, `(mpc % dt) @ plant`. Every
  MPC is one architecture, a box of pure blocks (`PredictiveController`, a
  `SampledDiagramSystem`; `mpc % dt` clocks its blocks): `PlanSolver` under the id `solve`
  (static, `(y, z_prev) → z*_k = pv.solve(P_k, z0)`), `PlanCommand` under `command`
  (`z → u_k = pv.command(z, t_k)`, and `x_ref`, `u_ref` from `pv.prediction` when the plan
  carries states) and `PlanMemory` under `memory` (a `UnitDelay` holding the last plan, from
  the default guess); the loop computes each signal once per tick, so a tick solves once. The
  two switches are independent: `warm_start=True` starts each solve from the last plan shifted
  to the tick, its first state knot at the measurement (`pv.shift(z_prev, t_k, x0=x̂_k)`,
  the default guess at `P_k` while there is no plan yet), `False` from the default guess at
  every tick; `delay=True` adds a visible `UnitDelay` under `delay` between the solve and the
  command, so tick `k` applies the plan solved at `k − 1` (one period of computation delay, as
  on a real computer), `False` applies it at once (the textbook MPC). A failed solve (a
  non-finite cost or plan) keeps the last plan. `P_k` is the planner's tree (the solve block's
  `params`: the one tuning path is `controller.solve.cost.R`,
  `pe.params["controller"]["solve"]["cost"]["R"]`) with each leaf named in `inputs` read from
  its port (default `{"x0": "y"}`) and `t0` the tick's time. Delay compensation is
  the user's predictor block on the port that binds `x0`, on the face verb
  `pv.predict_start(x, u, t, Δ, params)` (the planner's model, the command held). The face's
  default guess `pv.initial(params)` traces: under a trace it follows the traced `x0`
  and `t0` (the guess is affine in its start), so a traced loop's first plan equals the
  host's. `verbose=True` prints one line per solve on a host run (the tick's time, success,
  iterations, cost, wall time). Ports out: `u`, `z` (the plan solved at this tick) and
  `x_ref`, `u_ref` when the plan carries states. `PlanFollower(planner)` applies a received
  plan, `u = pv.command(z, t)`: the dual rate is `(mpc % slow) >> (PlanFollower(planner) %
  fast)`, the plan read at the slow tick itself; the one-period-late plan is the user's
  `UnitDelay(dim=n_z)` between them. A native fixed-iteration planner (single shooting,
  `method="adam"`, `n_iterations=K`) makes the tick traceable: `problem.compile(mpc % Δ)` is
  one scan on JAX, `vmap`s over starts and differentiates in the plan's weights. Post-run
  horizons and animation overlays: `planning/trajectory_optimization/mpc/viz.py`
  (`mpc_plans_from_rollout`, `mpc_animation_overlays`, the plan read from the run's `z` or
  from the memory's state). A sampled-data law written in seconds whose equation needs its
  period is a `NativeSampledSystem(state_dim, sample_period=Δ)`: its period set at construction, `%`
  on it refused. Out of the MPC's contract: a shrinking horizon (`N` is structural) and
  event-triggered replanning (a later option in the tick).
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
  `simulation.py` (`SystemSimulation`: `compile`, static `compute_trajectory` (the nominal
  input, or `u=`), `compute_cost`; `DynamicSimulation`: the continuous one;
  `StepSimulation`: `compute_rollout`), `analysis.py` (`SystemAnalysis`:
  `jacobian`; `DynamicAnalysis`: `linearize`, `transfer_function`,
  `find_equilibrium`; `StepAnalysis`: `jacobian` with `k`), `plotting.py`
  (`SystemPlotting`: `plot_trajectory`, `plot_cost`, the maps, `plot_diagram`;
  `DynamicPlotting`: `plot_phase_plane`; `StepPlotting`: `plot_rollout`; both inherit
  `LinearPlotting`: `plot_bode`, `plot_pzmap`, `plot_root_locus`, `plot_step_response`,
  one channel's plots in `s` or in `z`) and
  `animation.py` (`SystemAnimation`: `render`, `animate`; `DynamicAnimation`:
  `animate_modal`, `game`). `System`, `DynamicSystem` and `StepSystem` list
  their kind's mixins as bases. The
  analysis shortcuts are written out, so every
  editor shows their parameters, and `tests/unittest/test_system_shortcuts.py` pins each
  to its band function (parameters, defaults, forwarding, docstring target). Every facade is a two-line delegation to the tool's module and
  stores no run state on the system (RULES 2.4): `jacobian` compiles its
  evaluator per call, about a millisecond on the eager JAX path, and a loop
  holds a compiled evaluator and takes its `jacobian` as the function instead. The exception
  is `self.traj`, a convenience cache of the latest facade rollout; library
  code never reads it as an input. **MRO**
  picks `compute_trajectory` implementation; no
  façade-layer `isinstance` routers.

### Native-array equation rule

Applies to every method CONSTITUTION §2 names ('One signature'), and also to port
computes, transcriptions and the `MathematicalProgram` `J`/`h`/`g`.
Native in, native out; no `np.asarray` / `float()` inside equation paths. Convert at
boundaries (evaluators, solvers, plotting, `Trajectory`, I/O). Every core object reads its
`params`; the written exceptions are listed below.

### Parameters

One model for every object, in seven rules:

1. **Every object has `params`**, one dict of its numbers, `{}` when it has none (the one
   `None` is `MathematicalProgram.params`, a `J(z)` signature on a record); its mathematics
   reads that dict when a call passes `params=None` (never `params or self.params`).
2. **A composite holds its parts' own dicts at the parts' keys**, beside its own numbers, so
   every number has one home and a write anywhere lands in it: `loop.params["plant"] is
   plant.params`, `problem.params["X"] is problem.X.params`. Keys are diagram ids, cost term
   ids, problem roles, a scaled cost's `"cost"`, an intersection's, a union's or a scene's
   members by class name, a repeat numbered (`"BallSet"`, `"BallSet_2"`), so a recorded tree still
   names a member after a reorder. A composite value (an intersection, a field set, a field cost, a union, an
   inflated shape, a track, a scene, a state field, a scaled cost) holds its parts' trees by
   reference (`held_params`); a wrapper is what it wraps (`delegated_params`: an input box its
   box, a uniform law its box, a margin field its set, a sampled view its source); a diagram
   and a sum, with no numbers of their own, assemble the view from their parts at each read.
   A block is added to a diagram once, a cost to a sum once.
3. **Structure is the constructor's**: sizes, ports, a method, a clock, a part's key, a
   leaf's name, which spread a Gaussian takes (`std` or `cov`), which bounds a field set has.
   A value is written by item, `plant.params["m"] = 2.0`; a named attribute is a view of its
   leaf (`box.upper`, `law.mean`, `cost.Q`, `problem.discount_rate`), never a second home.
   Assigning a leaf's `params` declares it anew inside the dict it already holds, so a
   composite holding that dict stays current (`own_params`). Derived quantities (`dim`,
   `cov`, a uniform law's and a particle law's `mean`) are computed, never stored.
4. **A `params` argument is `None` or the whole tree**, the same shape as `.params`: on an
   equation method (`f h tf g margin value sdf sample step`, which read it as given, no
   merge, no check) and on every tool verb (`Simulator.solve`, `ev.f/step/rollout/
   rollout_batch`, `pe.trajectory/evaluate/program`, `planner.solve`, `pv.solve/solution/
   initial`, `Optimizer.solve`). Nothing merges anywhere. A one-off
   tree is the tool's own, edited: a held tool's `params` is a copy (`p = sim.params;
   p["m"] = 2.0; sim.solve(params=p)`); a planner's is live on its problem's parts, so a solve
   that leaves the problem as it is copies it (`p = copy.deepcopy(planner.params)`), and one
   that does not care edits the problem and calls `planner.solve()`. A copy of an object at
   other numbers is Python's own, `copy.deepcopy(plant)` then the edit; there is no copying
   verb.
5. **A tree that crosses a boundary is checked** against the structure it enters (`checked`,
   `require_complete` in `core/params.py`): a tool's `params=`, a composite's assignment. An
   unknown key, a missing one, a subtree for a leaf or a leaf of another shape raises, naming
   the known entries; a distribution standing where a number goes is not shape-checked. The
   bare item write on a leaf (`plant.params["mass"] = 2.0`) is a dict write, unchecked.
6. **A tool snapshots at build**: `tool.params` is what `params=None` runs on (the compiled
   evaluators, the `Simulator`, the problem evaluator, the planner's compiled face, the
   `Optimizer`), and reading it gives a copy, so an edit of it never changes the tool (whose
   frozen form compiled the snapshot as constants); nothing reads the objects or compares
   trees per call. A convenience call (`compute_trajectory`, `problem.compile`,
   `planner.solve`) builds each time, so it sees every edit, and keeps nothing but its last
   answer: Minilink keeps no cache. A held
   tool is the fast route for repeated calls: its pure functions (`pv.solve`, `shift`,
   `command`, `prediction`) compile once, at the build or the first call, and never for new
   numbers, which are their arguments (`tests/unittest/test_no_recompile.py`); a NumPy program
   has no compile, and a face on it rebuilds the program at new numbers. `pv.solution` is the
   face's host-side read-back, on the face's numbers: it builds what it reads back (the plant's
   rates) each call. A planner's `solve(params=)` rebuilds at the tree (`planner_at`): a
   method number the tree changes is given, one it leaves at the planner's value is derived
   again on the new problem (a new discount rate re-derives the factor).
7. **A problem's uncertain numbers are `Distribution`s in `problem.distributions`**, a sparse
   tree of the same shape (`distributions={"plant": {"m": Uniform([0.9], [1.1])}}`, the
   start's law `x0=law`); `problem.params` holds numbers only, and a law written into it is
   refused. `realize(key, n)` turns the laws into numbers with a trial axis.

Every number that is not structure is a leaf: a field's numbers are leaves of the set's and
the cost's tree (`{"field": …}`, a shaping's under `"shaping"`), a shape's and a path's of the
scene's and the track's, an LTI block's matrices of its own (`{"A", "B", "C", "D"}`), a lookup
policy's table and levels, a replayed signal's times and values, a model-based law's model
under `"model"`. A number stays out only for a written structural reason: a size, a count, a
choice of method, a boolean switch, a clock's period (it decides the plan), a summing
junction's signs, the action box of a neural policy (the plant's input bounds), the drawn
geometry of a skin (link lengths drawn, wheel tracks), an LTI model's operating point (it
never enters `f`). `tests/unittest/test_contract.py` walks every public system, cost, set and
law on both backends (`params=None` its defaults, the same values twice, the JAX numbers the
NumPy ones, a finite gradient through a float leaf); `tests/unittest/test_params_model.py`
pins the seven rules. The leaves are numbers and numeric arrays, an integer leaf (a seed)
never in `wrt`; every function is deterministic given its parameters, the seed among them.
- **One keyword per kind of choice.** `backend=` is the array module (`"numpy"`,
  `"jax"`, or `"auto"` where a tool can fall back) on every tool and facade that compiles;
  `solver=` is the time integration; `method=` is the verb's own numerical method (the
  optimizer's algorithm on `Optimizer` and on every planner that optimizes); the
  optimizer's solver adapter, which a method binds, is `Optimizer.adapter`.
- **One word for a run's origin:** `t0`, on every runner (`compute_trajectory`,
  `Simulator`, the evaluator's `rollout`, the host clock runner, the realtime
  run, the gym bridge). A trajectory's first sample is its `t0` property.
- **Two words for two times.** `dt` is the step of a computation and names it at every
  call (`discretize(sys, dt)`, `sys % dt`, `problem.discretize(dt)`, the evaluator's `step(…, dt=)`);
  `sample_period` is the clock a block holds and names it on every such block (the sampled
  system, the noise sources, the seeded policy). `discretize(plant, dt, substeps=4)` shows the
  two: the period above, the integration step `dt / 4` underneath. A noise block's period is
  never a `dt`: its physics does not change with the tool's step.
- **Diagram params are nested by subsystem id**: `{"plant": {…}, "controller": {…}}`,
  each the block's own dict (rule 2); nested diagrams nest recursively. A tree passed to `f`
  / `outputs` / `step`, or assigned to `diagram.params`, names every block and every key.
- Compile: a diagram's compile copies every block's params into the plan, as a leaf's does.
  The evaluator's `params` is that snapshot; `params=None` reads it (rule 6). On JAX the dict is a pytree argument
  (numeric leaves required): values vary without retracing, and
  `evaluator.jacobian("f", "params")` / `jax.grad` differentiate dynamics
  w.r.t. parameters (see `examples/demos/compile/params_gradient.py`).
  `evaluator.jacobian(of, wrt, x, u, t, params)` returns the array at the point on every
  evaluator (`t` the tick `k` on a step evaluator; `jax.jacfwd` eagerly on JAX, central
  differences on NumPy), as `sys.jacobian` does: a derivative is a matrix at a point, and its
  function form is the method itself, `partial(ev.jacobian, "f", "x")`; the evaluator keeps
  each built derivative by `(of, wrt)`, machinery keyed on structure (rule 6); the
  callable is jit-compatible for blocks written with `array_module` all the
  way through, and diagrams address internal wires as `"block:port"`, as
  output or as an additive perturbation.
- **A problem's parameters are its parts' trees by role**: `problem.params` is `{"plant",
  "cost", "X", "U", "X0", "Xf", "x0", "t0", "discount_rate"}`, each part's entry the part's
  own dict (the start's singleton `X0` is not an entry: its point is `x0`, one home), its own
  numbers `x0`, `t0` and `discount_rate` held in the tree (the attributes are views of them).
  Assigning the complete tree routes each entry to its part (a singleton `X0` moves with
  `x0`; a set's entry is written into the set, its one home). Library costs hold their
  parameters on the System pattern (`QuadraticCost.params` is `Q`, `R`, `S`, `x_bar`,
  `u_bar`, its attributes views); a `SumCost` keys its terms by id like a diagram (the class
  name verbatim, `QuadraticCost2` for a repeat, a scaled term the id of the cost it scales).
  Anonymous `+` and `sum()` flatten into one level of terms; `SumCost.of(**named)` and a cost
  with an `id` stay nested. A sum holds each cost object once (`q + 10 * q` is refused;
  write `11 * q`, or add a copy). A `ScaledCost`'s params are `{"weight": w, "cost": {...}}`,
  the weight held there. `FieldCost`'s params are its weight and its field's tree. Planners
  read each part's own parameters; a random plant's nominal draw is
  `nominal_plant_params(problem)`.
  A new start is a number of the tree like a mass: offline, `planner.solve(params=p)` with
  the new `p["x0"]`; in real time, the compiled face `pv.solve(p, z0)` (§6); moving obstacles
  online wait for the geometry module. **Gap:** call-time `params` on the base `Shape` and the workspace
  fields (the geometry module's plan); the fields' wrappers hand their field no tree
  ([QUESTIONS.md](QUESTIONS.md)).

### `DiagramSystem`

Subclasses :class:`~minilink.core.system.DynamicSystem` (diagrams are continuous
evolution systems with a compile fast path). Composes subsystems by named ports;
flattens state; compiled `ExecutionPlan` is the main execution path (reference
recursive path must stay equivalent).
Wiring, port gather and params nesting live on `WiredDiagramMixin` in `wiring.py`;
the port-dependency graph is `topology.py`: `check_algebraic_loops` (raises on a loop,
returns nothing; also a diagram method) and `evaluation_order_of` (the port order the
compilers read); `DiagramSystem` adds flow-only `f`,
`compile` and `reconstruct_internal_signals`; the wiring methods reach `DiagramSystem` through
the mixin, `diagram.of("plant")` the block by its id or role (a path, `"inner/plant"`, inside a
nested diagram) and `subsystem_id` a block or its id. **One block selector on every run:** a
diagram's run carries `roles`, every block's state slice by id (a nested block by its path), so
`traj.of("plant")` is the plant's own `(t, x, u)` by static indexing on a continuous loop as on
a sampled one; the input is the recorded signal `"<block>:u"`, else empty, `(0, N)`.
**What a diagram run records** (ruled 2026-11-06), one rule on every solver and both kinds of
loop: `record="plant"`, the default, the plant's whole input `"plant:u"` when the diagram has a
`plant` block, so `traj.of("plant")` is complete on every default run; `record=None` nothing
(the fastest run: hand sweeps, benchmarks, library loops); a tuple of names (`"block:port"` an
output port, `"block:u"` a block's whole input when it has no output port named `u`, a nested
block by its path, `"plant/arm:u"`); `record="all"` every output port. A lone block records
nothing and refuses names. A fixed-step run reads the record off each step's own first stage
(`rollout(record=)`: `k₁` and the record from one signal pass, the states bit for bit the
unrecorded run's, its program the same when the record is empty); an adaptive run, the
variable-step Euler and a run forced by a function of time take one signal pass per saved
sample after the run, inside the `Simulator`, on the numbers the run was made with. A cost
written for the plant alone scores the plant inside any loop, `loop.compute_cost(cost,
of="plant")` (the block or its id): the recorded run, else the plant's input rebuilt from the
stored states (`block_run`, which, with `reconstruct_internal_signals` and `port_samples`,
takes `params=`, the run's tree, and reads the blocks' current numbers without it). Scoring a
run whose plant input was not recorded (`evaluate_trajectory`, the MPC overlays) is refused,
naming `record=("plant:u",)`.
Every `System` has the two cost shortcuts over that path:
`sys.compute_cost(cost, of=None, traj=None)` returns `J` as a number and
`sys.plot_cost(...)` plots the running cost and `J(t)`; `of=None` scores the
whole system in its own `(x, u)`, `of=block` one block of a diagram, and `traj`
defaults to the last trajectory computed (a user shortcut; library code passes
it).
`connect()` validates port existence and dimensions at wiring time and is
quiet by default (`connection_verbose=False`; set `True` for one line per connection).
It refuses to rewire a subsystem input that already reads a source (`ValueError`, naming
both ends); `disconnect(sys_id, port_id)` opens the input and returns the source it read,
so a rewire is the two calls. At compile, each leaf's `y = h(x, u, t)` is probed once more
per input its `dependencies` leave out (that input moved, `h` evaluated at `x0`, the nominal
inputs and `t = 0`): an output seen to move there with an undeclared input is a hidden
feedthrough the evaluation order would miss, and compile refuses it, naming the block, the
port, the input and the declaration to add. It is a probe at one point, not a proof: a
feedthrough that vanishes there (`h = x u` at `x0 = 0`, a saturated output) passes.
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
keeps its blocks, wiring, ports, name and camera settings, which `+` and `>>`
carry into the result (its `x0` comes with the shared blocks, which own it); a `>>` chain is named after every operand
(`"Step >> PID >> Pendulum"`). Blocks are shared, never copied, so `a >> b >> c`
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
``boundary_inputs`` / ``boundary_outputs`` port anchors.

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
| `state.py` | full state `x`; constant gain `K` (`StateFeedback`, what `lqr` returns: `u = u_bar - K (x - r)` on a state reference, `r` defaulting to `x_bar`; `OutputReference(K, N)`, the textbook output reference `u = u_bar - K (x - x_bar) + N r`, `r` of the output's dimension) or a gain schedule `K(t)` interpolated between samples and held or replaced past its end (`TimeVaryingFeedback`, what `lqr_finite_horizon` returns; the schedule itself comes from `lqr_gain_schedule`, the Riccati differential equation propagated exactly through the Hamiltonian system); feedback around a reference trajectory `u = u_d(t) - K(t)(x - x_d(t))` (`TrajectoryFeedback`, no reference port — the trajectory is the reference; what `trajectory_lqr` returns: the plant linearized at every sample of the reference, the Riccati equation swept backward interval by interval from the stationary solution at the end point) | `(x0, x1)` |
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

**Embedded-model params rule.** Model-based controllers (`JointImpedance`,
`TaskImpedance`, `ComputedTorque`, `SlidingMode`, `TaskKinematic`,
`TaskKinematicNullspace`) hold the plant's equations and their own copy of its
numbers, ``params["model"]``, taken at construction; the model hooks (`g(q)`,
`H(q)`, `inverse_dynamics`, `forward_kinematics`, `J`) are called with that
subtree. A model mismatch is one edit of it (``law.params["model"]["m"] = 2``),
and the model's numbers sweep, draw and differentiate like any other leaf; an
edit of the plant's own ``params`` after the law is built, or a diagram-level
``params={"plant": {...}}`` override, moves the plant and not the model.

### `Trajectory`, sets, costs, geometry

- `Trajectory`: `t (N,)`, `x (n,N)`, `u (m,N)`, optional `signals`; array-module native
  (JAX arrays kept) and a JAX pytree, its reporting verbs (`t0`, `tf`, `resample`, `save`,
  the plots) NumPy by contract; `traj.at(t, name="x")` reads one channel at a time, linear
  between the samples and held outside (`interpolate_schedule`, the one time-interpolation
  the gain schedules read too); a diagram's run carries `roles`, every block's state slice by
  id, so `traj.of("plant")` is the plant's run by static indexing (its input recorded by
  default, `DiagramSystem` below).
  `print(traj)` is `N`, the time span, and the array shapes, not the sample dump.
- Sets: `margin ≥ 0` feasible; `contains` is that test on the array module (a traceable
  boolean under JAX); `sample(key, n=None, params=None)` follows the distributions' draw
  convention below (a set that has a design point gives it for `key=None`) (`BoxSet.sample` traces);
  `bounding_box()` is the tightest box containing the set, or `None` — tools that need a
  box (decision bounds, a grid's extent, a sampler's box) ask for it and never probe the
  set's type. Compose with
  `&` → `IntersectionSet`. `margin(z, t, params=None)`: a set's numbers are its `params`
  (`{"lower", "upper"}` on a box, `{"center", "radius"}` on a ball, `{"point"}` on a
  singleton, the members' by class name `{"BoxSet", "BallSet", …}` on an intersection, `{}` for a set with
  none), read when `params` is `None`, so they enter the problem's tree by role (`X.upper`);
  a set holds its numbers as a plant does (rule 3): `X.params["upper"] = ...` moves it, and
  a problem's entry for the role is the set's own tree.
- Fields (`core/fields.py`): `Field`, a scalar `value(x, u=None, t=0.0, params=None)` (it reads
  the arguments it depends on), `gradient` (`jax.grad` on JAX, central differences on NumPy),
  and four conversions named by what they return: `as_set(lower=, upper=)` (a `FieldSet`;
  `as_set(upper=level)` is the sublevel set), `as_input_set` (an `InputSet` over `u` at `x`),
  `as_cost(weight=, shaping=)` (the running cost `g = weight · shaping(V)`) and
  `as_terminal_cost` (the terminal cost `h`). Time goes by keyword, `V(x, t=t)`: the
  second positional argument is `u`. The shaping functions (`quadratic_hinge`,
  `quadratic_excess`, `inverse_barrier`, `occupancy`) are in `core/shaping.py`.
  `Set.as_cost(weight=, shaping=)` is `Field.as_cost` on the set's least margin
  `V(x) = minᵢ mᵢ(x)`, its default shaping the quadratic hinge `max(−V, 0)²` (zero inside:
  the soft form of the constraint). The workspace fields subclass it (`planning/spatial/`).
  `QuadraticField(center, P, t=None)` is `V = (x − c)ᵀ P(t) (x − c)` with the exact gradient
  `2 P(t) (x − c)`; `P` one symmetric matrix (a non-symmetric one is refused), or a schedule
  `(N, n, n)` on `N ≥ 2` times `t` (the Riccati `S(t)` of `lqr_gain_schedule`), read linearly
  in `t` and held past its ends. The sublevel set of a constant `P`, `as_set(upper=level)`
  with `lower` 0 or `None`, is an `EllipsoidSet` (any other band, or a schedule, a plain
  `FieldSet`): its `params` are `{"center", "P", "level"}`, its `field` derived from them; it
  draws uniformly inside (`sample` on a NumPy generator, an integer seed or a JAX key; like a
  distribution, the center, its design point, for `key=None`) and bounds itself
  (`bounding_box`, `half_width = √(level · diag(P⁻¹))`).
  `V(x)` calls `value`. A region-of-attraction certificate's `V` is its `QuadraticField(x̄, P)`
  and its `region` that ellipsoid; the LQR value `QuadraticField(x̄, S).as_terminal_cost()` ends
  a short-horizon MPC, and `LQRPlanner`'s `cost_to_go` is `QuadraticField(x̄, P)` (or
  `(x̄, S, t)` on a finite horizon, `t` on the problem's clock `t0 … t0 + tf`; its law
  applies the stationary gain `K∞` at `x̄` past `tf`, `K(tf)` held when the algebraic
  equation has no stabilizing solution).
  `GridField(levels, values, t=None, *, method="linear", outside=None)` is a table on a
  regular grid read as a field: multilinear in the cell holding `x` (`"nearest"` its closest
  node; both trace), `"cubic"` and `"quintic"` SciPy's splines on NumPy only; a leading time
  axis on `t` read linearly and held past its ends, the `QuadraticField` rule. Outside the grid
  the edge holds (`x` clamped on each axis), or `V = outside` when a number is given. Its
  `params` are `{"levels", "values"}` and, as given, `t` and `outside`; its gradient is the
  cell's exact multilinear one. The cell reading is one function (`grid_value`), which
  `LookupPolicy` shares for its command table (extending the edge cell past the grid, then
  clipping `u`); the field and the lookup law each take their table into the point's array
  module, so both trace with their table a constant: a JAX closed loop runs the law, and
  `backend="auto"` scores it on JAX like every law that traces (ruled 2026-11-06). Value iteration's `cost_to_go` is one (`StateSpaceGrid.field`).
- Distributions (`core/distributions.py`): a duck type — `dim`, `mean` (an array),
  `sample(key, n=None, params=None)` with `key` a NumPy generator, an integer seed or a JAX
  PRNG key (JAX arrays out, traceable); `(dim,)` for one draw, `(n, dim)` with `n`; `support`
  an optional `Set`. **No key, no draw:** `sample(None)` is the mean at the law's numbers
  (`sample(None, params=p)` the mean at other numbers; `(n, dim)` tiled with `n`), as a noise block's `seed=None` is its mean
  signal: the design point. A library law's numbers are its `params`, and `cov` is the covariance of
  one draw: `Gaussian(mean, std=None, cov=None)` (one spread, `{"mean", "std"}` or
  `{"mean", "cov"}`, a full `cov` drawn through its Cholesky factor), `Uniform(lower, upper)`
  (its `BoxSet`'s bounds, `cov = diag((b − a)² / 12)`), `Particles(points)` (`{"points"}`, the
  law's own covariance); so a family of noise levels vmaps like a family of masses. Sets are
  support, distributions are probability.
- Randomness (`blocks/sources.py`, `core/distributions.py`): a `Distribution` has no time; a
  noise signal is a block holding draws over a sample period. `WhiteNoise(p, *, psd,
  sample_period, seed, hold)` is white noise of two-sided intensity `psd`, each sample
  `w_k ~ N(0, psd / Δ)` held over `Δ` (`hold="zoh"`; `"linear"` interpolates), so the physics
  does not change with the period; `seed`, `sample_period` and `psd` are params, and `h`
  computes `w(t)` from `(t, params)` with a counter-based cipher (Threefry-2x32, the same bits
  on NumPy and JAX), so the block is pure, traces, and edits without a refresh. `seed=None` is
  the mean, and such a block is not random (`is_random` reads the seed): no realization or
  evaluation reseeds it. `System.realize(key)` returns the params of one realization: every random block
  reseeded under its own stream, named by its id (a diagram nests it like `params`), or every
  seed `None` for `key=None`; the analysis verbs (`operating_point`, `find_equilibrium`) and a
  `PlanningProblem` on a random system resolve `params=None` through `realize(None)`, so a
  noisy loop is linearized and planned at `E[w] = 0`. Three periods: the block's `Δ`, a
  controller's period, the solver's step. A block publishes `Δ` as
  `solver_info["sample_period"]` (a diagram gathers the base period of its blocks', their
  greatest common divisor); a hold period places the grid's points and is never a time
  constant. With no solver named the simulator takes fixed-step RK4 at the automatic step
  (§5, "Automatic grid"), rounded down to divide `Δ`. A held block is read like any block,
  at each stage's time: the last RK4 stage of a step ending on a switch reads the next draw,
  first order at the switches, an approximation below what a held-noise model claims (a finer
  `dt` tightens it). A step that does not divide `Δ`, a forced
  adaptive solver, two blocks sharing a seed, and a declared disturbance the stepped tools
  never read are announced. Kalman reads `Q = B_w W B_wᵀ`, `R = D_v V D_vᵀ` for the
  continuous filter (`kalman(sys, Q_n, R_n)`, the intensities); the discrete filter, an
  estimation follow-up (ROADMAP §5.6), will read `Q_d = B_w W B_wᵀ Δ`, `R_d = D_v V D_vᵀ / Δ`.
  `NoiseSource(law, *, sample_period, seed, hold)` is sampled noise: `w_k` the law drawn
  at the counter key `CounterKey(seed, k)` by the same cipher (each library law and `BoxSet`
  draw from one), held over `Δ`, the law's numbers under `params["law"]`; a per-sample
  covariance keeps each sample when `Δ` is edited (a datasheet sensor,
  `NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)`). `WhiteNoise` is its subclass whose
  per-sample law, `N(0, psd / Δ)`, is derived in `h`; both are in `minilink.blocks`.
- Costs: `g(x,u,t,params=None)`, `h(x,t,params=None)` on `CostFunction` in `core`; attach
  to `PlanningProblem`, not the plant. The cost is the integrand only: `integrate(traj, *,
  discount_rate, params)` is its trapezoid on the trajectory's array module, `g_max(X, U)`
  and `h_max(X)` its upper bounds over boxes (exact for a quadratic, sampled otherwise),
  `traj.with_cost(cost)` the plotting signals. Compose with `+` → `SumCost` and `*` →
  `ScaledCost` (e.g. `base + w * obstacle_cost`); parameters nest by term id ('Parameters').
  The horizon, the discount, the sets and the failure cost are the problem's.
- Geometry (`core/geometry/shapes.py`; the rest of its package is in §3): `Shape.sdf(p)` is the signed distance to a workspace
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
1000 RK4 steps in a fraction of a second (0.12–0.16 s on a cloud CPU). Quote those; never a
per-call ratio. The `core_perf` regression suite gates the batch: the pendulum's plain batch
(`rollout_batch.pendulum.plain_s`), a family of rod lengths (`family_s`), and their ratio
(`family_speedup`, gated in CI). A family costs about 4–6× the plain batch on CPU: the
pendulum's mass matrix `H = m l² + I` then differs per member, so each RK4 stage solves `B`
one-by-one systems where the plain batch solves one.

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
transcription concepts — see :mod:`minilink.core.backends`). Every automatic backend
(`compile_auto`, a simulator's `backend="auto"`) reads one rule, `build_on_auto` in
`compiler.py`: JAX, falling back to NumPy on two verdicts only, a missing JAX (the import
raises `ImportError`) or a model that does not trace (the trace check raises
`NotTraceableError`, a `RuntimeError`). Any other error of the JAX build (a wrong shape, a
bad parameter) surfaces as it is, never hidden by a NumPy build.

Diagrams → `ExecutionPlan` → diagram evaluator. Optional **compile-time flatten**
(`compile_diagram(..., flatten=False)` by default): before the plan is built,
:func:`~minilink.core.assembly.flatten_diagram` copies the diagram and inlines each
nested :class:`~minilink.core.diagram.DiagramSystem` block so every leaf sits at the
top level. Inlined ids join with ``__`` (``inner__plant``), the same convention as
expanded topology (`controller__pi`); ``:`` stays ``block:port``, ``/`` stays the
hierarchical plot path on the **unflattened** diagram. After flatten, diagram
``params`` keys are those flat ids; :func:`~minilink.core.assembly.remap_params_nested_to_flat`
rewrites a nested tree for an explicit ``params`` argument. ``+`` / ``>>`` already
inline via :func:`~minilink.core.assembly.inline_diagram`; ``@`` nests until flatten
is requested at compile. :func:`~minilink.core.assembly.flatten_diagram` preserves
global state stacking order: inlined leaves replace each nested box at the same stack
position as that box's inner states, so ``x``, ``x0``, and native
:meth:`~minilink.core.diagram.DiagramSystem.f` agree with the unflattened diagram.
Wires and diagram outputs that read a nested box are rewired to the inner block
behind that port. The returned report's ``id_map`` keys each leaf by its nested path,
``/`` at every level (``"loop/inner/plant"``), to its flat id; the params remap reads it,
so a renamed block (``loop__plant2`` when the name was taken) still gets its params.

**`outputs()` are boundary outputs
only** (dict keyed by port id, not diagram internals); a diagram's internal signals are
`signals(x, u, t, params=None)` (the buffer) and `compute_internal_signals_dict` (keyed
`"block:port"`). `h(x, u=None, t=0.0, params=None)` is the output map, the `"y"` port as
`sys.h` (empty when there is none, a `PID`'s output being its command `"u"`): one contract,
`h` *is* the `"y"` port on every block, plant and evaluator (`test_output_map.py`).

**One body per method, compiled on JAX.** Every evaluator method takes `params=None`
(`None` reads the parameters compiled in; a dict replaces them, a diagram's block by
block):

    ev.f(x, u, t=0.0, params=None)                                            # dx; a continuous evaluator
    ev.outputs(x, u, t=0.0, params=None)                                      # the boundary ports
    ev.step(x, u, t, params=None, *, dt, solver="rk4", substeps=1)            # x_{k+1}, u held
    ev.rollout(x0, u_seq, t0, dt, params=None, *, solver="rk4", substeps=1, hold="zoh", n_steps=None)
    ev.rollout_batch(x0s, u_sequences=None, *, t0=0.0, dt, n_steps=None, params=None)   # JAX: a family, one vmap

A step evaluator has `step(x, u, k=0, params=None, *, t0=0.0)` (the system's own map on
the integer tick, no knobs), `outputs` and
`rollout(x0, u_seq=None, params=None, *, n_steps=None, t0=0.0)`;
a static evaluator has `outputs` only.   On NumPy each method is its plain body. On JAX each
is marked `@compiled` (a step evaluator's `rollout` through its scan, returning native
`(N + 1, n)` arrays) (`core/compile/evaluators/compiled.py`): `jax.jit` of the body, built
on first use and cached per evaluator, the knobs (`solver`, `substeps`, `hold`, `n_steps`)
static. Every leaf of `params` is numeric (floats, numeric arrays, or integer seeds; never
a `bool` or `str` mode switch, which belongs on the constructor), so the whole
`params` tree traces directly. A compiled method is fast from Python and composes inside an outer `jit`, `grad`,
`vmap` or `scan` with the numbers of the uncompiled body, so there is one name per method:
no parametric twin, no trace tier, no `jit` namespace.

**Compile once, in the form the call needs.** A compiled method has two forms behind its one
name: `params=None` is the frozen form (the compiled values are constants of the trace, the
fastest run of one fixed model), a tree is the parametric form (its leaves traced, so new
values never retrace). Before the jitted call, every scalar argument and every leaf of the
tree is made the dtype and shape of the compiled default (`9`, `9.0`, `np.float64(9.0)` and a
0-d array share one trace) and passed as it is: a host leaf transferred by the call, a JAX
leaf already in its compiled dtype and shape passed untouched, so a caller who repeats a value
holds it on the device (`jax.device_put(tree)`) and transfers nothing. No cache and no digest
run per call; the frozen form's only per-call work is one `is None` test. A tree holding a leaf
of more than 1000 numbers (a network's weights) always goes as an argument, never as
constants. The frozen form earns its keep: with the tree as an argument a 1000-step rollout is
13 % slower on a PID loop (13 leaves), 23 % on a three-link arm (20) and 42 % on the racecar
(29), because XLA folds the parameter-only algebra out of the scan body. A value sweep on one
held tool compiles the parametric form once (`sim.solve(params=p)`). A tool built inside a
planner's solve reads the problem's numbers at that solve (`evaluator.call_params()`, once per
solve, never per call). A convenience call (`compute_trajectory`, `problem.compile`,
`closed_loop`) compiles each time, so it sees every edit; speed is a tool held by the user
(`Simulator(sys, backend="jax")`, `pe = problem.compile(ctl)`, `planner.compile()`), built
once and run many times (no cache, no hidden registry). The `jit` sits at the outermost pure
computation of each pipeline: the rollout (not `f`), the problem's trial function, the
fixed-count solve and the fused first-order step, the RL rollout; a host loop (SciPy, a
host SLSQP) calls jitted `f`, Jacobian, objective and gradient.

**The step model.** Every sampled consumer steps a continuous plant through `step` and
`rollout`, written once (`core/compile/evaluators/stepping.py`) for both backends.
`solver` is `"rk4"` or `"euler"`, `substeps` splits each period, `hold="zoh"` holds each
row of `u_seq` (shape `(N, m)`) and `hold="foh"` reads `N + 1` knots, linear between
them; `u_seq=None` holds the nominal input for `n_steps` periods; `rollout` returns
`[x_0, …, x_N]`, `t_k = t₀ + k dt`. `discretize`, the RL environment, the gym bridge,
value iteration (`StateSpaceGrid.successor`), shooting, the fixed-step solvers, RRT,
the Lyapunov sampler, the problem evaluator, the host clock runner and the realtime simulator all
call them, or the one-period formula they share (`step_body`). The adaptive
SciPy solver adapts `f` for `solve_ivp` itself (`simulation/solvers/scipy_ivp.py`).
`discretize(sys, dt, *, solver="rk4", substeps=1) -> SampledSystem` (its method facade
`plant.discretize(dt)`) is the system on a clock of period `dt`, the input held
(`core/discrete/sampled_system.py`). A `SampledSystem` is a view of its source: its `params`,
ports (their functions included) and state metadata are the source's, its `sample_period`
structural. One equation, two faces: the seconds face `update(x, u, t, params=None)`, the state
one period after the sampling instant `t` (a continuous source integrated by `solver`, `"rk4"`
or `"euler"` with `substeps` per period, or `"exact"`, the matrix exponential of a linear model;
a static source has `n = 0`), its output ports functions in seconds; the tick face
`step(x, u, k)` and `h(x, u, k)`, `t_k = k·sample_period` (the origin 0; a consumer with an origin
calls the seconds face). A tick source (a `StepSystem`) keeps its count and has no seconds
face (`counts` is true). The sample time is `dt` alone, never a key of the plant's `params`. The
hold belongs to sequences (`rollout`), never to a step.

Layout of `core/compile/`: `compiler.py` (`compile`, `compile_auto`, `compile_diagram`,
`compile_step_diagram`, the shape probe), `plan.py` (`ExecutionPlan` and `StepExecutionPlan`, their
operation rows, and the builders),
and `evaluators/`: `base.py` (the `Evaluator` base, the continuous step
model, `gather_u`), `dynamic.py`, `diagram.py`, `static.py`, `discrete.py` (one class per
system kind, both backends), `compiled.py` (the mark), `stepping.py` (the step model's
formulas and the loop / `lax.scan` that runs them), `jacobian.py` (`jacobian`: central
differences on NumPy, `jax.jacfwd` on the uncompiled body on JAX).

**Frozen evaluator subset.** The stable evaluator API is the methods above: `f`, `h`,
`outputs`, `signals`, `step`, `rollout`, `rollout_batch` and `jacobian`, each with
`params=None`, and `nominal_input`. A diagram evaluator's `rollout(record=names)` returns
`(x_samples, records)`, the named signals at the start of each period from its first stage.

Keep `ExecutionPlan.output_slices` and `external_output_slices` aligned.

**Default sim API:** `compute_trajectory(tf, u=None)` dispatches by **class MRO**
(façade mixins), enforced at simulator constructors:

- :class:`DynamicSystem` (including :class:`DiagramSystem`) →
  :class:`~minilink.simulation.simulator.Simulator` (ODE integration or diagram
  evaluator on a time grid)
- static :class:`System` leaf (`n=0`) →
  :class:`~minilink.simulation.static_simulator.StaticSimulator` (time grid +
  boundary outputs in `Trajectory.signals`; not state evolution)
- :class:`StepSystem` → `compute_rollout(n_steps, u=None, *, x0=None, t0=0.0)`, a
  :class:`~minilink.core.trajectory.Trajectory` of the ticks (`t = k` on a tick law,
  `t = t0 + k·Δ` on a system on a clock); its compiled evaluator's
  `rollout(x0, u_seq=None, params=None, *, n_steps=None, t0=0.0)` returns the states
  `(N + 1, n)` as native arrays (one `lax.scan` on JAX, traced and differentiable end to end),
  `rollout_batch` maps it over starts, inputs and a parameter family in one `vmap`, and a step
  diagram's `tick(x, u, k)` gives the next state and the boundary outputs from one signal pass;
  on a clock with a continuous plant underneath, `compute_trajectory(tf, u=None, *, x0=None,
  t0=0.0, dt=None, solver=None)` is the same scan with the plant's substeps kept (the evaluator's
  `rollout_dense`, `(N·S + 1, n)`, `S` the plant's substeps, `⌈Δ / dt⌉` when `dt` is given), equal to the ticks bit for
  bit at the ticks, and `solver="scipy"` the host clock runner
  (`simulation/discrete/clock_runner.py`: the same blocks ticked on the host, the plant integrated
  by `solve_ivp` between the held inputs, NumPy), the cross-check; a tick law refuses it

`animate` / `plot_trajectory` live on `SystemAnimation` / `SystemPlotting` for continuous and static
systems; a sampled loop caches its run on ``self.traj`` alike.
Auto-sim fallback calls `compute_trajectory` (MRO picks engine on homogeneous diagrams).

**Two static paths:** a static *leaf* (`Gain`, `Step` source) uses
`StaticSimulator` — empty state, outputs in `traj.signals`, the whole grid one
`vmap` of the jitted outputs on JAX (a loop on NumPy). A static-only
*diagram* (`n=0` stacked state) still subclasses `DynamicSystem` and uses
`Simulator` with the diagram evaluator (signal-flow on a time grid).

**Unconnected inputs read their port nominal value — by design, and silently.**
An input left unwired is a constant at its declared `nominal_value` (the
Simulink "ground" semantics), never an error or a warning; `plot_diagram()`
shows it as unconnected. Time-varying sources belong in the diagram; forcing
via `compute_trajectory(u=)`. Facades default `backend="numpy"`.

**What `n_steps` counts.** One rule, by the kind of verb: on a horizon `[t0, tf]` it counts the
grid's points, endpoints included (`Simulator`, a continuous or static `compute_trajectory`,
a transcription's knots, the LQR gain schedule: `N` points, `N − 1` intervals); on a run of
periods it counts the transitions (`rollout`, `rollout_batch`, `compute_rollout`, a step
problem: `N` periods, `N + 1` samples `x_0 … x_N`). `n_steps=100` on `[0, 1]` is thus a
step of `1/99` on the first and `100` steps of `dt` on the second
(`test_simulation.py` pins each verb).

**Automatic grid.** With neither `n_steps` nor `dt`, the solver is chosen first
(user choice, else `euler` for discontinuous plants, else `rk4_fixedsteps` when a block
holds a signal over a `sample_period`, else `scipy`) and the grid
is sized to it: adaptive solvers report on `DEFAULT_N_STEPS = 10001` points (a
plotting resolution — the integrator picks its own steps); fixed-step solvers
step at the automatic step, one rule wherever a fixed-step run picks its own step (a
continuous diagram, discontinuous loops included; a sampled loop's plant; `problem.compile(law
% dt)`; a planner's evidence): `Δt = AUTO_DT_SCALE × τ_min` (0.1, `core/system.py`,
`automatic_step_of`) over the blocks that have dynamics, rounded down so it divides every
hold period of the loop. An explicit `dt=` at a run and `substeps=` at a compile win. `τ_min`
comes from the physics: a linear block reads it from its eigenvalues in its current `params`
(`τ = 1 / max |λᵢ|`, `fastest_time_constant`: an LTI or state-space block's `A`, an
observer's `A − L C`, a PID's derivative filters; an all-zero spectrum, an integrator,
declares `∞` and bounds nothing); a catalog plant declares its own from its linearization at
the nominal point by a closed formula, as a `fastest_mode()` read from `params` (the
pendulum's `√((m l² + I)/(m g l))`, the cart-pole's unstable pole, the dynamic bicycle's tire
modes at rest, the racecar's); a law with a time constant of its own declares it
(`SlidingMode`'s `1/λ`). A custom plant that declares nothing keeps the 1 ms default; no
plant is linearized automatically. A diagram's hint is the smallest over its stateful
subsystems and any block that sets its own; a static block left at the default does not
count. A diagram's hints are derived from its blocks at each read, and a block whose hint
follows its parameters reads them at each read too, so no tool refreshes before a run and a
hint belongs on the block: one set by hand on the diagram is replaced, and so is one set on
a block that computes its own (change its `params`, or pass `dt=`).
`StaticSimulator` shares the 10001-point default.

**Three owners of a start.** A block owns its `x0`; a diagram's `x0` is a view
stacked from its blocks' (a read-only array; assigning a whole vector writes each slice
through to its block), a sampled loop's held outputs keep their own start (`hold_x0`), and a
`SampledSystem`'s `x0` is its source's. A block's state size is fixed once built, and a
diagram's layout is computed as blocks are added. The problem owns the plant's start (`x0`), and the
evaluator's tree gives every other stateful block its start as a leaf (`controller.x0`,
`observer.x0`), read from the block at compile.

Solver presets: `scipy`, `scipy_stiff` (Radau at `scipy`'s tolerances, `rtol=1e-4`,
`atol=1e-7`, with `∂f/∂x` from the evaluator: forward mode on JAX, central differences on
NumPy), `scipy_max`, `scipy_ultra`, `scipy_lsoda`,
`euler` (variable knot spacing), `euler_fixedsteps` (uniform grid via the step
model's Euler rollout), `rk4_fixedsteps` (the RK4 rollout; auto-picked when a block holds a signal over a `sample_period`, on either backend, and on JAX for an explicit uniform grid of ≥ 10 000 points). Both fixed-step backends refuse a non-uniform grid, with one relative check (`atol=0`) shared with the simulator.
**One input rule on every solver** (`Simulator.solve(u=)`, `compute_trajectory(u=)`): a sampled
input `(m, N)` is held over each sample period (`u(t) = u_k` on `[t_k, t_{k+1})`), and a function
of time is read where the method evaluates `f`: the stage times of RK4, the adaptive solver's own
points, the grid times of Euler (where the held sample is the same read). The automatic solver
choice never changes the input model.

### Discontinuous closed loops

A discontinuous law (`SlidingMode`'s `sign(s)`) on a continuous closed loop sets
`discontinuous_behavior`, which diagrams aggregate; the automatic solver choice then takes
explicit Euler at `dt = AUTO_DT_SCALE × τ_min` and warns on every solve (louder when
`rk4_fixedsteps` or a SciPy solver is forced). The reason: the diagram evaluator re-solves
the feedback at every call to `f`, so RK4's four sub-steps can flip `sign(s)` and nearly
cancel, and the adaptive SciPy solvers may stall near the switching surface; the
`Trajectory` records the output grid only, so a reconstructed `controller:u` at grid states
is the start-of-interval map, not the effective dynamics over the step. Digital sliding mode
is a sampled loop: `(ctl % dt) @ plant` holds the torque between ticks, the intended semantics
(`examples/demos/hybrid/sampled_smc_pendulum.py`).

## 6. Optimization And Planning

**NLP:** `minimize J(z)` s.t. `h=0`, `g≥0`, bounds. Pure `MathematicalProgram`;
`Optimizer(program, z0=None, method=)` binds a method preset named by algorithm: `gd`
and `adam` (the first-order backend, projected on the box), `l-bfgs`, `scipy_slsqp`
(the default), `scipy_trust_constr`, `ipopt`. The program's class is read from its
fields (`unconstrained`, `bounded`, `constrained`); `method="auto"` picks L-BFGS for the
first two and SLSQP for the third, and a method the class does not admit (`gd` on
constraint rows) is refused. `z0` defaults to `program.metadata["z0"]`. A program may
declare parameters: `MathematicalProgram(..., params=tree)` takes callables of
`(z, params)`, `tree` their default; `Optimizer(program).solve(params=, z0=)` takes the
solve's complete tree (`optimizer.params`: the program's numbers and the method's under
`"method"`, a copy edited), checks it once (`core.params.checked`: an unknown or a missing
path raises naming the known ones, a new shape raises) and binds it, so one compiled program is
solved for many values without a recompile; a program without `params` keeps callables of `z` alone. One evaluator
body serves both: it holds the bound tree, the JAX one passes it as a traced argument and
differentiates in `z`. **The stochastic program is the seed case**: every program is
deterministic given its parameters, the seed one of them, an integer leaf found by the key
`seed` and never a decision (`pe.program` refuses a seed in `wrt`); complete trees inside, one
merge at the boundary; sizes structural (the knots `N`, the iterations `K`, a batch size are
constructor arguments, never parameters); the warm start an argument of the solve (`z0=`). The
core's sample-average export is `pe.program(wrt, key=, n_trials=)`: the draws realized once
from `key`, so `J` is deterministic and any method solves it; how a solver uses the seed
(advanced per iteration, `vmap`ped) is an `Optimizer` strategy for later. **The native
methods** `gd` and `adam` are pure laws on the array module of their input
(`optimizers/first_order.py`: `law.init(z0) -> state`, `law.update(grad, state) -> state`, the
gradient oracle beside it, the box projection in the step). `Optimizer(program, method=,
n_iterations=K)` takes exactly `K` steps: one `lax.scan` on a JAX program, so
`solve(params=, z0=)` traces under `jit`, `vmap` and `grad` (the result then holds JAX arrays,
`success=True`, `cost` the final `J`), and a loop of the same steps on NumPy; no callback.
`n_iterations=None` runs to the method's tolerance on the host (`maxiter` its cap, Adam's update
jitted when JAX is at hand, plain NumPy without it). `optimizer.as_step_system(params=None)` is
the solve as a `StepSystem`: the law's state flattened (`z` first), `compute_rollout(K)` the
iterates. The failure cost of
an evaluation follows its cost's numbers: the bound of the build at the compile's numbers, one
computed for the call at other weights, and under a trace the build's (a failure cost is a step,
with no gradient). One method table
serves every solve path: `make_optimizer_backend(method, options=)` gives the preset's adapter
(`adapter.solve(evaluator, z0)`), so a planner's compiled program takes every method, `ipopt`
among them, as the offline `Optimizer` does (OPW-1).

**Planning:** `PlanningProblem` (a plain class; `problem.replace(**fields)` derives one)
owns system, sets (`X` and `U` unconstrained unless declared: `X=plant.state.box`,
`U=plant.input_box`, or a `BoxSet` over the inputs, wrapped), cost, the continuous `horizon` and its start `t0` (the
horizon is `[t0, t0 + tf]`, `0` by default; a run starts there, costs, sets and
references read absolute time, the discount counts from it, the fixed grids of the
transcriptions are `t0 + linspace(0, tf, N)`; a leaf of `problem.params` and of the
evaluator's tree, so `jax.grad` differentiates it), the discount rate and the failure cost
switch (below). **The boundary dials**: `X0=` is the start constraint `x(0) ∈ X0` (a
vector is its singleton; by default the start's singleton, a law's support, else the
singleton at `sys.x0`); `Xf=` the terminal constraint (a vector its singleton, a hard terminal
equality for collocation and shooting; `None` a free end); `x0=` the evaluation start, a
point or a law (by default `X0`'s point: a singleton's, a region's bounding-box centre when
finite and inside, else `sys.x0` when inside), which must belong to `X0`; the cost's `x_bar` the
objective's target. There is no goal shortcut: `problem.goal_point()` (`Xf`'s point, else the
cost's `x_bar`, else the start) is what a heuristic aims at (an initial guess's end, RRT's goal
bias, a plot's anchor); it never constrains a plan. The planners pin the start from `x0`.
A planner that needs a finite box (value iteration's grid, RRT's bang-bang) refuses an
unbounded one with the line to write. Offline entry is
`Planner.solve()` → `PlanningSolution`, one result for every planner: the
`problem` it solved (a reference), the
`controller`, the block a loop closes on (a feedback block `u = pi(x)`, a `TrajectorySource` `u = pi(t)`
for the trajectory family — `solution.open_loop` — or a time-varying feedback
`u = pi(x, t)` for a finite-horizon LQR), the `record` (one
frozen dataclass per planner with `success` and a one-line `str`; `method`
names it; the LQR's `RiccatiRecord` ends its `str` with "U ignored" when the problem's `U`
bounds the command, since the Riccati law never reads `U` (LQR never
warns)), the
`trajectory` (native for open-loop planners; for a feedback law its nominal
rollout on the planner's grid, filled by `solve(evaluate=True)`, else `None`), the `evaluation`, and
`cost_to_go` where the method produces one (a `GridField` for value iteration and
tabular learning, the failure cost past the grid, a `QuadraticField` for `LQRPlanner`,
read `J(x)` or `J(x, t=t)`; the RL critic, when the training discount equals the
problem's, a plain callable of `x`, which `planner.cost_to_go(x, t)` reads at any `t`). Everything on the solution is the solver's own claim under its
own model — `cost_to_go` is the method's estimate, never a measurement; a
measurement comes back from the evaluators as its own object. A planner
computes only what its solve computes natively; `solve(evaluate=True,
n_trials=)` fills the rest through the evaluator. `solution.policy(x, t)` is the
controller's policy as a function, the command `u = π(x, t)` (a feedback controller reads `x`
on its measurement port, its other inputs at their nominal values and its state at its `x0`;
an open-loop one reads `t` alone). The planner mirrors its latest solution:
`planner.controller` (a read-only property), `planner.policy(x, t)`,
`planner.cost_to_go(x, t=None)` (the field read, a `float`), `planner.plot_trajectory` and
`planner.animate`. **The planner's output is a controller or a trajectory, and the
script closes the loop itself** (`solution.controller @ plant`): the solution's
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
and one backend (NumPy for all when one entry does not trace), its `dt` as `evaluate` reads
it (a sampled controller on its own clock, `dt` only for a continuous one), and `difference(a, b)` is
the paired `J_a − J_b`. `LQRPlanner
(problem)` is the linear-quadratic method on a `QuadraticCost` problem: `lqr_matrices`, the
first function of `lqr.py`, lowers it to `(A, B, Q, R, S)` at the cost's `(x_bar, u_bar)`
(`B` and `R` over the command port), then the algebraic Riccati equation for an
infinite horizon, the differential one swept from `S` over a finite horizon. A discounted problem
(`discount_rate = ρ > 0`) is solved as its textbook equivalent, the undiscounted problem on
`A − (ρ/2) I` (the same law, in `x̃ = e^{−ρt/2} x`), with a warning, a discounted LQR being
unusual; the poles stay the plant's, `eig(A − B K)`. Its `RiccatiRecord` holds `K`, `P`, the
closed-loop poles (success = Hurwitz) and the finite-horizon schedule. `control.design` stays
the array-in / block-out factory the planner calls.

**Horizon and the constraint set:** the horizon has one owner, `problem.horizon` — a
finite length is a finite horizon and `h` is charged at `t0 + horizon`; `+inf` or unset is an
infinite horizon and `h` is never charged (`problem.horizon_kind()`). How long to run
an infinite-horizon problem is a tool setting, `episode_length` on the evaluator and
the RL planner.
The discount rate `rho` is the problem's, `problem.discount_rate`; value iteration and the
RL planner read its per-tick `gamma` from the problem's step face, `e^{−ρ dt}` (a step
problem's own `gamma`), and tabular learning from `problem.discount_factor(dt)`, the same
number. `X` and `U` are hard
constraints in every tool: a run that leaves them is a failure. **The failure switch**
`penalize_failure` (a bool, default off: an evaluation reports the plain cost and the
facts) adds the **failure cost** to a failed or missed run's `J`; `failure_cost` is that cost (a
number, `None` for the cost bound). **The cost bound** is one formula owned by the problem,
`cost_bound(horizon=None, *, discount_rate=None, dt=None)`: `g_max ∫₀ᵀ e^{−ρt} dt + e^{−ρT} h_max`
over the declared boxes for the continuous judge, and with a tool's `dt` the same maxima on its
own sum, `g_max dt Σ_{k<N} γᵏ + γᴺ h_max`, `γ = e^{−ρ dt}` (it bounds every stage,
`c_k ≤ g_max dt`), else the plant's declared ranges; always finite (`1e6` with a warning naming
`failure_cost` when no bound exists). The step face reads it at its period
(`face.cost_bound`, `face.cost_bound_over(N, γ)`), each tool over the horizon its own sum runs:
the RL environment over its episode, value iteration over its sweeps (to convergence: the
geometric limit `g_max dt/(1 − γ)`), the face over its `N`; on a finite undiscounted problem
the judge's integral and the three sums are one number. Trajopt and RRT keep the sets exact;
value iteration, tabular learning, the RL environment and the Gym bridge always charge a
failure the failure cost (`failure_cost`, else that bound), and DP treats
leaving the *grid* as infeasible too (the table has no value off its domain). The learners also
take a **training zone** (`ReinforcementLearningPlanner(training_zone=)`, the
plant's state box by default): leaving it *truncates* the episode, the critic's
value at that state standing in for the rest, because the model is not studied
there; it is a training choice, not a constraint. An exit is always a failure:
terminating without cost would reward escaping under a cost. `h` keeps its one
job: the end of a finite horizon.

**One scoring formula** (`planning.evaluation.score_run`, every evaluation's): a run is
integrated from `t0` to `t_end = t0 + tf` (no event, no cut), then

```
finite              = every x_k and u_k finite
feasibility_margin  = min over the run of X.margin(x_k, t_k), U.margin(u_k, x_k, t_k)
failed              = not finite, or feasibility_margin < 0
reachability_margin = Xf.margin(x(t_end), t_end)  reached = margin ≥ 0   (Xf = None: reached)
cost                = C_N + e^{−ρ tf} h(x(t_end), t_end),  C_N = Σ_k γᵏ c_k = ∫ e^{−ρ(t − t0)} g dt  (+inf when not finite; no h on an infinite horizon)
J                   = cost, or with penalize_failure on cost + failure_cost · (failed or not reached),
                      and cost_bound + failure_cost for a run not finite
```

**The stage cost is integrated with the state**: the evaluator's run carries the running
cost `C(t) = ∫ e^{−ρ(τ − t0)} g(x_plant, u_plant, τ) dτ` beside the loop's state, by the same
scheme and substeps, read at the plant block's own state and input in the pass that computes
`dx` (a sampled loop: inside the plant's period, `SampledSystem.update_with_cost`), recorded as
the signal `"plant:running_cost"`; its last value is the run's running cost, the stage costs
`c_k` of the step face summed with their discount. `evaluate_trajectory` reads that signal when
the run carries it (one score) and scores a logged run, which has none, by the trapezoid on its
samples. A run that is not finite is scored on zeros behind a safe `where`, and with the failure
charged its cost term is the problem's `cost_bound(tf)`, the largest cost a finite run can reach
(its `1e6` fallback, unannounced, when the boxes bound no cost and the failure cost is a number): its `J` stays
finite and above that of any finite run, failed or not. Its gradient is not defined (the rollout
overflowed before the `where`). A run whose
states stay finite while its cost overflows scores `+inf`, charged or not. A trajectory is
scored on its `plant` role; one without roles is scored as the plant's run only when `x`
has the plant's `n`, `u` its `m`, and it carries no recorded signals (a logged run), and a
trajectory with recorded signals and no role is refused.
`evaluate_trajectory(problem, traj)` scores a trajectory already held (a loop's run
through its `plant` role; a logged plant run without signals). **The planning problem defines
the problem**: a tool may plan or train on a modified copy of it and its record names
the modification, but the score is always the problem's. RL *trains* on `r_k = −g(x_k, u_k, t_k)
dt`, one call of the cost's `g` at the period's start (`reward="stage"`: `−c_k`, the face's stage
cost `∫₀^dt e^{−ρτ} g dτ`, the score's), with per-step factor `gamma`; `exp(-rho t)` is not folded
into the reward (that would discount twice); its record says "trained on −g dt, scored on the
problem's cost". A tool's own discount (value iteration's `gamma`, RL's `gamma` with the stage
reward) is internal and consistent: the tool plans on the problem at `ρ_tool = −ln γ / dt`, the
stage integral and the step both, names it in its record, and its policy is scored at the
problem's `ρ`.

**A problem stochastic by its parts:** there is no stochastic class; where a number
goes, a law may go. `x0=` takes a point, a `Set` (the uniform law on it, the set its
`X0`) or a `Distribution` (its mean the representative `x0`, its support the default
`X0`); `distributions=` takes laws in the tree's own shape (`{"plant": {"m": Uniform([0.9], [1.1])}}`,
a block inside a diagram plant by its id); `disturbances=` takes source blocks on the plant's
ports (`{"d": WhiteNoise(...)}`; a bare law is refused, naming the fix); `criterion`
(`"expectation"` default, `"worst_case"`, or `("cvar", alpha)`, the mean of the worst `alpha`
fraction; an evaluation's `objective` is this reduction of the runs' `J`, and the RL planner,
which optimizes the expectation, refuses the others). `problem.is_stochastic` is read off the
parts; `problem.params` holds numbers only and `problem.distributions` the laws, the
start's law its `start_law`;
`problem.realize(key, n)` is one tree: the problem's numbers with `"x0"` of shape
`(n, nx)`, each law's leaf drawn `n` times and each random source's `seed` a trial axis of
integers (`drawn_paths()` names them); every consumer draws from its own stream, named by its
path (`child_seed(key, "plant.m")`, a source's seed by its block and the trial), so adding a
law or a source leaves every other draw unchanged, and an integer key draws on the host: the
same trials on NumPy and JAX. `realize(None)` draws nothing: every law at its mean, every seed
`None`; `nominal()` is that tree written into a copy of the problem's parts (the caller's
untouched), the random sources kept with seed `None`, their mean signal. The evaluated loop is exactly the one `closed_loop` builds with the same sources (two forms, one diagram): the source of `d` is its block `disturbance`, of `n` its `noise` (a
`Sum` on the command or the measurement when the plant has no such port), any other port's
under the port's name; its seed is the leaf `"disturbance.seed"`. An evaluation draws a seed per
trial for every random block of the loop, the problem's sources or the loop's own, even on a
deterministic problem: a block's own seed is only its default realization, never the trials' seed. The stepped
environment reads each source at `t_k` with the episode's seed, and draws the plant's laws
only: a law on another number (the cost's, a set's, the discount) is refused, naming its
path. A deterministic planner given a stochastic problem plans
from the mean start and warns; `Sys2Gym(sys, cost, x0=law)` draws each episode's start
from a `Distribution` with its `np_random` (`None`: `sys.x0`), and
`Sys2Gym.from_problem(problem, dt=)` is the Gymnasium view
(draws of `x0`, the exit rule, `h` at a finite horizon). Distributions are §4's duck type.

**The problem evaluator (the second verb, `planning/evaluator.py`):**
`pe = problem.compile(controller, dt=None, *, solver="rk4", substeps=None, backend="auto", episode_length=None)` closes
a controller on the plant (`controller @ plant`; a source block drives the command; a built
loop is taken when its block `plant` is the problem's system) and compiles it once. The
controller's kind decides the loop: a continuous law runs the continuous diagram on the
grid of `dt`; a sampled controller (`pid % 0.01`, a held policy, a loop on a clock) runs its
`SampledDiagramSystem` on its own base period, `dt` omitted reading it and a differing `dt`
refused, naming the period, `solver` and `substeps` its plant's integration per base period (`None`, the
automatic step; a continuous loop's `substeps` is 1 by default); a tick law (an MPC box among
them) with no clock is refused, naming the clock.
`pe.params` is the run's tree: the loop's blocks by id (a stateful block other than the
plant with its `"x0"` leaf), the problem's own leaves (`"cost"`,
the sets' parameters, `"x0"`, `"discount_rate"`) and each free input at its nominal
value. `pe.trajectory(params)` is one run to the horizon on the step model, every block
output recorded (`"id:port"`, the plant's input `"plant:u"`, the role `plant`);
`pe.evaluate(params, key, n_trials)` the `Evaluation` (a pytree record, one row per run:
`cost`, `J`, the margins, `failed`, `reached`, `x0`; `objective` the criterion over `J`,
`mean` / `std` / `worst` / `cvar(alpha)` and the three rates), the draws of a stochastic
problem `vmap`ped on JAX, so `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)`
reaches every gain, plant parameter, cost weight and the start; a tree holding an integer leaf (a
noise block's `seed`) is differentiated with `jax.grad(..., allow_int=True)`, the integer leaves
getting a zero (`float0`) gradient; that reads a gradient, since `float0` leaves refuse
the next tree arithmetic, and tuning goes through `pe.program(wrt)`, which never sees an
integer leaf. `solver="scipy*"` is the
adaptive integral on NumPy, `dt` its reporting grid; `"auto"` falls back to NumPy when the
loop, the cost or a set does not trace. `pe.program(wrt, bounds, key, n_trials,
params=)` exports the tuning `MathematicalProgram` over the leaves `wrt` (dotted paths,
`"controller.Kp"`): the objective `pe.evaluate(...).objective`, its reverse-mode gradient on
JAX, the per-trial feasibility margin as a row when `X` or `U` is bounded and the
reachability margin when `Xf` is set; `pe.pack` / `pe.unpack` move between the tree and
`z`. `evaluate(problem, controller, dt=None, ...)` is the facade: a sampled controller runs on
its clock with `dt` omitted, a continuous one needs `dt`. No backend clips the law:
port bounds are information, and saturation lives inside the law or in a `Saturation`
block. A stochastic problem needs `n_trials`; its draws are `problem.realize(key, n)`, one
tree (above), one key per call, and each trial realizes the loop's random blocks under that
key (trial `i` is `loop.realize(child_seed(key, "trial i"))`), the same noise on both
backends for an integer key, while a deterministic problem runs each block's own seed. The program reads the same `J` as the report: with the failure charged, its cost is a step
with no gradient, and a non-finite objective at `z0` warns once. The feedback planners'
reported `trajectory` is their own held-input rollout on their
grid (`nominal_trajectory`, the problem's step face from `t0`), the planner's evidence;
a planner's evidence (`solve(evaluate=True)`, default off; `planner_evidence` in
`planning/planner.py`, an internal helper the seven planners' `solve()` call) runs on
the real plant: a feedback law with no clock of its own held over each period, `policy % dt`, the plant integrated at the automatic step (§5), the planner's own `J` in its
record beside it, so the evidence does not repeat the planner's own belief; an open-loop
source keeps the interpolation its planner states. Scoring a law is the problem evaluator's:
`problem.compile(ctl % dt).evaluate()`, `substeps=` for the planner's own physics.
`problem.compile(law)` called directly closes a continuous law as a continuous loop.

**What the gradient path is not.** Three limits of the differentiable evaluation:
- **Single shooting is not collocation.** `pe.program(wrt)` differentiates one forward run
  of the loop, single shooting (an open-loop source's leaves, a held input): through an unstable plant over a long horizon its gradients
  explode or vanish and the landscape is nonconvex. `ShootingTranscription` and
  `DirectCollocationTranscription` stay the trajectory-optimization path; the export is an
  evaluation and tuning tool.
- **Backpropagation through the dynamics and policy gradients are complementary.** The
  evaluator differentiates through `f` and the reparameterized draws; REINFORCE, PPO and SAC
  differentiate the log-policy score and never `f`. Short smooth horizons favour the first;
  long horizons, saturations and stiff dynamics the second.
- **Crossover margins jump.** A gain or phase margin is read at a crossover that can appear,
  vanish or switch during a descent, and an eigenvalue has no derivative at a repeated pole:
  the margins are the numbers a course reports, not an NLP objective (a smooth bound, the
  sensitivity peak `M_s`, is the traceable-analysis row of TODO §4).

**The step face (`planning/step_face.py`):** `problem.discretize(dt, solver="rk4",
substeps=1).compile()` is the problem's `StepFace` (`compile` gives each problem its
evaluated face: `problem.compile(controller)` is the controller's evaluator, and refuses
no controller with this pointer; a step problem's `compile()` is its step face): `transition(x, u, k, params)
-> (x_next, c_k)`, one period of the step model with `c_k = ∫₀^dt e^{−ρτ} g dτ` integrated
beside the state by the same scheme (`g·dt` to the last bit under one Euler step),
`N = tf/dt`, `gamma = e^{−ρ dt}`, `terminal`, the sets, `cost_bound`, `failure_cost` and `run(policy)`
(`J = Σ γᵏ c_k + γᴺ h`); `params` is the tree its calls read (the plant at its mean, a
discretized problem's at its origin's, the cost, `x0`, `t0`, the discount). A `StepPlanningProblem(sys, *, N, cost, gamma,
...)` (`planning/step_problems.py`) is the native form on a `StepSystem` with a `StepCost`
(`c(x, u, k)`, `h(x, k)`), its start and sets a `PlanningProblem`'s by composition, its
face `problem.compile()`. Its laws are over its own tree, `params = {"plant", "cost", the
sets', "x0", "t0", "gamma"}` (the cost's numbers among them), and `realize(key, n)` draws
them into that tree as a continuous problem's does (the start from the stream `"x0"`, each
law from its path's; `sample_params(key)` one draw); the face's plant is at its laws' mean.
A native `t0 = k₀` is a whole tick offset: tick `k` charges `c(x_k, u_k, k₀ + k)` discounted
by `γᵏ` and `h` at `k₀ + N`, and the cost bound reads `c` and `h` at `k₀`, so a problem
started at `k₀` scores as from 0 on the cost shifted by `k₀`. `problem.discretize(dt)` is the explicit step problem of a
continuous one, its cost a `SampledCost`, compiling to the face the library builds inside
(`StepFace(problem, dt=)`) to the bit.
Value iteration, the RL environment and a planner's evidence read only this face.

The pipeline (its map: [docs/pipelines.md](../pipelines.md)). Every object carries its own
parameters `p`, and they form one tree `θ = pe.params`, a branch per object under its key; the plant,
cost, sets and start (or, on a stochastic problem, the distributions) belong to the problem,
and the controller closes the loop on the problem's plant at compile. The problem evaluator
is its own object: it simulates the closed loop (integrated forward from `x₀` to `tf`),
scores its `Trajectory` (the cost `J`, and the margin `m` of the run to the sets, negative
when it leaves one), and returns the objective with its `Evaluation`; `pe.evaluate(θ)` alone
evaluates a solution. Tuning is an optional export: `pe.program(wrt)` minimizes the objective
over the chosen leaves of `θ` (dotted paths such as `"controller.Kp"`) with `m ≥ 0` as its
rows, and the optimizer writes the tuned leaves back into `θ`. On a stochastic problem the
noise's seed is a leaf of `θ` like any other, and so are the distributions' own parameters;
the draws `ωᵢ` (starts, drawn parameters, block seeds) are realized from one key, the `n` runs
go in one `vmap`, and the criterion `C` reduces their `Jᵢ`.

**Reinforcement learning (`learning/reinforcement_learning/`):** two
policy-family planners beside DP, one per representation of the law.

`TabularLearningPlanner(problem, x_grid=, u_grid=, dt=, method=)` learns
the cost-to-go table `Q(x, u)` on a `StateSpaceGrid` from transitions alone,
NumPy only: the state is rounded to its nearest node, the action taken among
the grid's input levels, and every algorithm is a moving average toward its
own sample of the cost-to-go, `Q <- Q + eta (q - Q)`: `QLearning` (target
`c + gamma min Q(x', .)`, off-policy), `SARSA` (the action taken next,
on-policy), `MonteCarloControl` (the cost observed to the episode's end).
Exploration is an object, `EpsilonGreedy` (annealed) or `UCB`. `solve()`
returns a `PlanningSolution` (the greedy `LookupPolicy` as `controller`,
`J = min Q` as `cost_to_go`, the tables in `record.tables`); `planner.result` is the
table as it stands, a `DynamicProgrammingResult` (`J = min Q`, `pi = argmin Q`), and
`plot_cost_to_go`, `plot_control_law`, `cost_to_go` and `controller` (a `LookupPolicy`)
are those of value iteration on the latest solution's tables (RULES 2.4), and the same grid solved by
`DynamicProgrammingPlanner` is the exact answer the learner approaches.

`ReinforcementLearningPlanner(problem, dt=, hidden=, features=, method=)`
learns a neural law in pure JAX: `solve(n_timesteps=)` → `PlanningSolution`
(a copy of the trained block as `controller`, a `ReinforcementLearningRecord`, the critic as
`cost_to_go` when the discounts match), `planner.controller` →
`control.neural.NeuralPolicy` (`u = u_mid + u_half · squash(MLP(z(x)))`,
weights in `params["mlp"]`, features normalized by the state box or a user
map); the learned law's rollout with draws is
`evaluate(problem, solution.controller, record=True)`. The block being trained, updated in
place every learning iteration, is `planner.actor`.
What the algorithm trains is a `ExplorationPolicy` (`planner.exploration_policy`): the actor's mean action
with an exploration head around it (`GaussianHead` for the policy-gradient family,
`SquashedGaussianHead` for SAC), one pytree `theta = {"mlp", "head"}`, and the methods
`sample`, `log_prob`, `entropy` and `input(a)`. Critics `ValueFunction` and `QFunction` sit
on the same features. One file per algorithm under `algorithms/` (`REINFORCE`,
`ActorCritic`, `PPO`, `SAC`), each owning only its update rule and train state; three class
attributes declare the family (`on_policy`, `head_kind`, `critic_kind`). Bare JAX, no
Flax/Optax dependency. Training prints one line per update; `verbose=False` silences it.

`RolloutEnvironment(problem, dt=, backend=)` is the problem's semantics as
step functions on either backend — the step face's `transition` with the training reward `r = −g dt` (`reward="stage"`: `r = −c_k`),
the judge's failure rule at the sample (`X` at `x_{k+1}`, `U` at `u_k`; the episode ends at
the first failure, a training device) and the horizon — shared by the JAX collectors
and the tabular learners, so one object owns what a training episode means (an evaluation
scores the learned law by `evaluate`, the law held over each period). The heads draw at a JAX
key or at a `CounterKey(seed, k)`; the collectors carry one policy seed per episode and draw
tick `k = ⌊t/dt⌋` at it, the draw `GaussianPolicy(..., seed=)` makes. The discount is
the face's `γ` (the problem's `e^{−ρ dt}`) unless `gamma=` overrides it, and the record (`gamma`,
`scored_gamma`) says when the two differ. The environment reads the face only: its `N`
(an episode's ticks on a finite horizon), its `γ` and its cost bound over the episode (the
failure charge). So a `StepPlanningProblem` trains through its own face, on its own clock
(no `dt`: a native one's period is one tick, its time the tick `t0 + k`; a discretized one's
its `dt`), its reward `−c_k`, its start and laws drawn by its `sample_x0` and
`sample_params`, no source; a discretized problem steps as its origin at that `dt` with
`reward="stage"`, to the bit. `ReinforcementLearningPlanner` trains on either problem (`dt`
defaults to 0.05 s on a continuous one, none on a step problem); a step problem's learned
law is scored by its face, `problem.compile().run(policy)`, and `solve(evaluate=True)`
refuses it with that pointer.
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
`examples/tutorial/advanced/04_reinforcement_learning.ipynb`; native teaching
notebooks are GRO860's `pendulum_value_iteration_vs_lqr_vs_rl` and `drone_ppo`. The
spatial scene names (`ReferenceTrack`, `from_waypoints`, `Scene`, `mount`,
`car_outline`, `point_probe`, the shaping helpers, `TrackCorridorOverlay`,
`plot_track`) are exported on the `minilink.planning` facade so track demos
import through the teaching surface.

**Trajopt:** `TrajectoryOptimizationPlanner` → transcription → NLP →
`PlanningSolution` whose controller is a linear `TrajectorySource` of the knot
inputs (the transcriptions integrate a linear input between knots, so
`controller >> plant` replays the plan). `TrajectoryOptimizationRecord.success`
means *the returned plan satisfies the program constraints to
`feasibility_tol`* — nothing else: an iteration-limit stop on a feasible plan
is not a failure, a solver that reports convergence on an infeasible plan is,
and the solver's own flag stays in `message` / `stats`. The worst equality
residual / inequality margin / bound violation are recorded on the record, with the
solve's whole account (`optimization_result`, the per-iteration `history` of a
`solve(record_history=True)`, `solve_time_s`, `step_time_s`); the planner keeps only the
diagnostics `last_program` and `last_optimizer` (RULES 2.4). A user's verb (`Optimizer.solve`,
`planner.solve`) always times the backend solve it wraps and fills `solve_time_s`; the
machine-side solves (the compiled face's `pv.solve`, the fixed-iteration pure solve) are traced,
have no wall clock, and leave it `None`.
**I-level constructors** take flat kwargs
(`n_steps=…`, `transcription="direct_collocation"`, `backend=…`) like `Simulator` /
`Optimizer`; teach demos pass `transcription=` explicitly. Tier-2 still accepts a
`Transcription` instance. Offline
`solve` always rebuilds a fresh
`MathematicalProgram`. A JAX transcription's program declares the problem's tree it reads
(PP-2, PP-3): `x0` (the start's residual when `X0` is the start's singleton),
`t0`, the plant's, the cost's, `discount_rate`, and each set it reads as margins by role
(`X` and `U` when not lowered to the decision bounds, a non-singleton `X0`, `Xf`; a box in the
bounds is structure, not a leaf). Two routes replan from new numbers, a new start among them:
offline, `planner.solve(params=p, guess=prev)` with the new `p["x0"]` builds each call; in real
time, `pv = planner.compile()` then `pv.solve(p, z)`, which the MPC uses, compiles once and never
for new numbers (`params` is `None` or the complete tree, checked, an unknown or a missing path
raising and naming the known ones). The
reserved key ``scene`` raises ``NotImplementedError`` until the scene bind (A3). Carrying the
tree stays under 2× the compile, the solve and an evaluator call of the same program with it
frozen (the bicycle MPC is the check). Every transcription's objective is the
problem's, discounted from `t0`, `Σ dt/2 (w_k g_k + w_{k+1} g_{k+1}) + w_N h`,
`w_k = e^{−ρ(t_k − t0)}`. Knot count
`n_steps` is a planner flat kwarg (or lives on a custom transcription's
options); the time grid is
`t0 + linspace(0, problem.horizon, n_steps)` for a finite `horizon`.

**A planner's four kinds of argument** (ruled 2026-11-04, every planner alike):
**structure**, what a change of rebuilds (the grid, the transcription, `n_steps`, `dt`,
`method`, `backend`, the stopping rules `n_iterations`, `max_iterations` and the
tolerances), a constructor keyword read back as an attribute; **numbers**, what a sweep, a
tuning loop or `jax.grad` varies without a rebuild (the problem's leaves, the method's numbers:
a native law's `learning_rate`, a discount `gamma`, MPPI's `temperature`, RRT's `goal_bias`;
the `seed`), given as flat keywords and held in `planner.params` (`params["method"]`),
taken per call by `solve(params=…)`, the complete tree edited (a method number the tree
leaves at its value is derived again on the solve's problem, one it changes is given); **an external solver's knobs** (SciPy `maxiter`,
`ftol`; Ipopt's), the dict `options=` passed through untouched, never in `params`; and
**workflow** (`verbose`, `callback`, `live_plot`, `record_history`, RRT's `live_plot_*` display
knobs and RRT*'s `history_stride`), keywords that change what is shown, never what is computed. The constructor
takes the first three kinds; the workflow is a keyword of the verb, `solve` (and `solve_steps`
where it exists), as on `Optimizer.solve` and `compute_trajectory`, and travels as arguments
down to where it is read, never as planner state. A planner rebuilt for a sweep (`planner_at`,
`build_kwargs`, `rebuilt_on`) inherits no display: `solve(params=…)` hands its own workflow keywords to the
rebuilt planner's `solve`. Value iteration rebuilds its grid with the grid's own options
(`StateSpaceGrid.rebuilt_on`: `precompute`, `precompute_backend` and the grid's build report),
as `gamma=` does. The compiled face's `pv.solve(params, z0)` takes none: it stays pure,
silent and free of plots. There is no settings record on a planner. One exception:
the RL planner's `params` is a report, and its `solve` trains without `params=` (TODO §6).

**The compiled face**:
`planner.compile(backend=None)` gives `pv`, the planner's solve as functions of `(params, z)`:
`z` is the transcription's decision vector with the origin `t0` appended (`pv.n_z`);
`pv.params` the tree (`planner.params`: **one shape for every planner**, the problem's
leaves the plan reads, `"method"` the method's numbers (a native law's `learning_rate`, `b1`,
`b2`, `eps`; MPPI's `temperature`, `noise`, `control_cost`; the RL planner's `gamma` and its
algorithm's and optimizer's numbers; `{}` for a host method, whose name and tolerances are
structure) and `"seed"` (`None` for a method that draws nothing); the method's name is a
constructor option); `pv.initial(params)` the default guess packed;
`pv.solve(params=None, z0=None) -> (z, status)` (`params` `None` or `pv.params` edited:
`pv.params` returns a copy, so an MPC holds one tree and sets its `x0` and `t0` each tick); `pv.shift(z, t)` the knot rows re-read at
`t + offset`, the tail held; `pv.command(z, x, t)` the input at `t`; `pv.prediction(z, t)` the
planned `(x, u)` (collocation); `pv.solution(z, params, status)` the host-side
`PlanningSolution`; `pv.pack(guess)`; `pv.program`. A native method with `n_iterations=K` (a
planner keyword, forwarded to its `Optimizer`) on a box-bounded JAX program (single shooting
with an input box) makes `pv.solve` pure: `jit`, `vmap` over starts, `grad`. The face keeps no
last solution; `planner.solve()` stays the offline verb (callbacks, history, live plot, the
report) and keeps `last_solution` for its shortcuts only (`plot_trajectory`,
`animate`, `controller`, `policy`, `cost_to_go`, RULES 2.4); no solve and no tool reads it. Single backend-native
transcription classes; no parallel JAX transcription types. NumPy transcriptions
build constraints via `dynamics_function` (the compiled evaluator's `f`).
**JAX transcriptions** embed `problem.sys.f` directly in the NLP (not the
evaluator's `f`). `backend="direct"` calls `system.f`
uncompiled (escape hatch). A `MathematicalProgram` carries the native backend of
its callables in its `backend` field, and the `Optimizer` compiles with it by
default.

**RRT / DP:** the four kinds of argument above — keywords on
`RRTPlanner` / `RRTStarPlanner` / `DynamicProgrammingPlanner`, read back as
attributes, their numbers in `params["method"]`. Keep fundamental seams explicit
(`extender`, `StateSpaceGrid`). The textbook DP setup
`DynamicProgrammingPlanner(problem, x_grid=, u_grid=, dt=)` builds that grid
itself (`grid=` for a custom one), and `solve()` then pins saturated
cost-to-go cells to `out_of_bound_cost` (`clean_infeasible=False` keeps the raw
table; `clean_infeasible_set(tol)` reruns the pass with another tolerance). A
callable `out_of_bound_cost` has no single saturation level, so it leaves the table as
solved.
`ValueIterationRecord.success` means the sweeps converged to `tol` (a finite
horizon, and `solve_steps`, always succeed); the record carries `mode`
(`"converged"`, `"finite"` or `"receding"`), the sweep count, the last cost-to-go change
and the tables themselves (`record.tables`, the `J` and `pi` the planner's `result`
and plots read; a finite horizon's stages in `record.tables.schedule`).

**Dynamic programming** (`planning/dynamic_programming/`): offline dynamic programming on a
continuous plant. A `StateSpaceGrid` discretizes the `PlanningProblem` — grid *extent*
comes from the sets' bounding boxes (so it stays finite), grid *validity* from
the sets (`contains` on NumPy, `margin >= 0` on the JAX precompute, so `X = bounds & free`
works on both); `grid.X` and `grid.U` are the boxes it spans. The grid reads the problem's
step face (`grid.face`): the successors are one period of the step model, forward
Euler by default (`x_next = x + f(x,u,t)·dt`; the grid's `solver="rk4"` otherwise; the time
step `dt` lives on the grid, not on `System`), and the backup's stage cost is the face's
`c = ∫₀^dt e^{−ρτ} g dτ` integrated by the same scheme (`g·dt` to the last bit under Euler),
the discount the face's `γ` and the failure cost its rule; a `StepPlanningProblem` (a grid world)
solves on its own step and stage cost, its grid needing no `dt`. `DynamicProgrammingPlanner` runs value iteration backward, and the problem's horizon
picks the mode of `solve` (LQR's rule): an infinite one sweeps until the change per sweep
falls under `tol` (not an error bound on `J`: with `γ < 1` the bound is `γ·tol/(1 − γ)`);
a finite one (`horizon = T`, `N = steps_of(T, dt)`, or a step problem's `n_steps`) runs
exactly `N` sweeps and keeps every stage forward in time, `J_0 … J_N` (`J_N = h`) and the
laws `π_0 … π_{N−1}`, stage `i` at `t_i = t0 + i dt` (`record.tables.schedule = (t, J, pi)`,
refused past 1 GiB of tables, the failure cost summed over the `N` steps). `solve_steps(n)`
is the receding `n`-step law: `n` sweeps, the last table used at every time. The
`PlanningSolution`'s `controller` is the `LookupPolicy` of the greedy table (on a finite horizon
a schedule: stage `i` for `t_i ≤ t < t_{i+1}`, a float tick a hair short of `t_i` counted in
it, and stage 0, the full-horizon law, outside `[t_0, t_N)`: past the horizon the
stationary law approximated without a second solve) and its `cost_to_go`
is `J` as a `GridField` on the grid (`J(x, t)` on a finite horizon, the stages read
linearly in `t`; `grid.field(J)`: read by the planner's interpolation,
the failure cost `out_of_bound_cost` past the grid, the edge held when that cost is a
callable; `planner.cost_to_go` reads it). The backup is one function at the top of `dp.py`,
`bellman_backup(J_next, G, arrival, gamma)`, `J = min_u [G + γ J_next(x_next)]` with its
argmin, each engine passing its reading of the grid (`arrival`, zero off the grid); the
`"loop"` engine stays the double loop written for teaching (the raw `DynamicProgrammingResult`, `J` and `pi` as action ids, is
`solution.record.tables`, and `planner.result` the latest solution's); out-of-domain
transitions are charged a finite `out_of_bound_cost` (pyro's
`cf.INF`), or, when it is a callable, its cost `out_of_bound_cost(x_next, t)` at each
such transition's successor. Three interchangeable
backward-step backends share this workflow: `loop` (per-node Python, pyro's reference),
`numpy` (vectorized over the precomputed lookup table, the default),
and `jax` (the same backup as one jitted `lax.while_loop` with `map_coordinates`, a finite
horizon's `N` sweeps one `lax.scan` stacking the stages, on tables
built with `vmap`, so the plant and cost must be JAX-traceable; linear/nearest only). The JAX
engine lives in `dp_jax.py` and the progress reports in `progress.py`, so `dp.py` reads as the
textbook algorithm. `precompute` trades the `(N,A,n)`
successor table for per-sweep recomputation (memory vs time-varying support); a precomputed
grid also freezes the running-cost table, so a time-varying `f` or `g` needs `precompute=False`. `planner.controller` is a `LookupPolicy` (a static `System`; a variant with another
`interpolation` is built directly, `LookupPolicy(grid, pi, interpolation="nearest")`, with the
stage times on a finite horizon, `LookupPolicy(grid, pi_stages, t)`); `PolicyEvaluator`
gives the cost-to-go of any fixed law. A grid passed to either is built on the problem's own `X`
and `U` (the same objects, a `problem.replace(...)` copy included; another problem's sets are
refused), so the loop and table engines and the evaluator read one owner. **Function approximation**
(`dynamic_programming/approximation.py`): linear-in-the-weights fits `f_hat(x | w) = wᵀφ(x)` with fixed
features — `QuadraticFeatures(x_bar)` (a quadratic form, the LQR shape; `quadratic_form(w)` reads
`(c, b, S)` back) and `RadialBasisFeatures` (Gaussian bumps, `on_grid` for a mesh of centres),
stacked with `+` — and a `LinearApproximator` that fits by least squares (`fit`) or one
stochastic-gradient step per sample (`sgd_step`). It is the seed of approximate dynamic
programming: fit a cost-to-go from value iteration, or learn one online.

**Spatial scene** (`planning/spatial/`; its planned home `core/geometry/` is in §3): two domains — **workspace** `p ∈ ℝ²/ℝ³` and
**state** `x`. On W: hard `Shape` obstacles and soft `WorkspaceField` sources live in
`Scene` (`obstacles`, `workspace_fields`). On X: a `Field`'s `value(x)` fuses the robot
placement with scene queries (`clearance_field`, `cost_field`). Export separately —
`clearance_field(body)` for collision (hard `Set` or barrier `CostFunction`) and
`cost_field(body)` for terrain (soft `CostFunction` or hard band via
`as_set(upper=...)`). The field's `value(x)` is a **scalar** (min clearance, max
density over body probes). **Collision reuse:** frameless geometry (`disc`,
`car_outline`, `point_probe`) binds to the **planner** plant with
`mount(geometry, on=sys, frame="body")`; world probes use ``sys.tf(x,u,t)[frame]``
via :func:`~minilink.core.kinematics.apply` — the same FK as rendering. Frameless
geometry: :func:`~minilink.planning.spatial.collision.disc`,
:func:`~minilink.planning.spatial.collision.point_probe`,
:func:`~minilink.planning.spatial.collision.car_outline`. Shape obstacles with
`quadratic_hinge`, `inverse_barrier`). Compose at `PlanningProblem`:
`X = bounds & free`, `cost = base + w * terrain`. Scene param overrides online are the reserved `scene` tier (§4,
'Parameters'): rebuild the `Scene` and recompile the parametric program.

**Reference track** (`core/geometry/paths.py`, `planning/spatial/track.py`): workspace centerlines
from waypoint polylines via `from_waypoints` (default `kind="polyline"`; `closed=True` adds the
segment back to the first waypoint and wraps the arc length round the loop, a structural
switch held on the path, not in its `params`), wrapped in
`ReferenceTrack(path, half_width)`. Same export pattern as obstacles —
`distance_field(robot).as_cost(shaping=quadratic_excess)` for soft path following,
`corridor_field(body).as_set(lower=0)` for a hard tube. Probe semantics match
clearance (subtract body radius). Compose with obstacles:
`X = bounds & scene.clearance_field(body).as_set() & track.corridor_field(body).as_set()`.

**Workspace cost raster** (`grid.sample_field_costs`, `plotting.plot_cost_field` /
`plot_cost_field_3d` / `plot_cost_field_exports`): rasterize shaped ``FieldCost`` terms
at workspace points ``p = (x, y)`` with a **point probe** body (heading-independent).
Build viz costs with ``mount(point_probe(), on=sys)``; MPC may still use ``car_outline``.

**Search / RRT** (`planning/search/`): `RRTPlanner(Planner)` owns the invariant loop and
sources every concern from the problem — collision `problem.X.contains` (optional
orchestrator `edge_resolution` densification along edges), goal region `problem.Xf` (a ball
written as one, `Xf=BallSet(goal, tol)`; a singleton or no `Xf` is refused), its goal bias
toward `problem.goal_point()`,
free-space sampling from `problem.X` (direct `Set.sample` or rejection from the bounding box of `X`), dynamics `problem.sys.f` — so the system stays pure. After
`solve()`, `reached_goal` and `solution_node` report success vs
best-effort fallback (`return_best_effort`). The two swappable pieces are an injected
`TrajectoryExtender` (`propose(from, toward, problem, rng) → Iterable[Edge]`:
`KinodynamicExtender` forward-integrates controls, `SteeringExtender` connects exactly
via a `SteeringFunction` including `DubinsSteering`) and a `metric(a,b)` callable
(nearest-neighbour distance). Every system is an ODE, so an `Edge` always carries real
`(t,x,u)`; `solve() → PlanningSolution` with a held (`interpolation="previous"`)
`TrajectorySource` as its controller and a `TreeSearchRecord`. `RRTStarPlanner` extends the attach step
with near-neighbour parent selection and cost-based rewiring (`Tree.rewire`,
`Tree.propagate_cost`); `Edge.cost` is the cost-to-come along the tree. With
`optimize_after_goal`, the search continues after the first goal connection until
the best goal cost stops improving for `convergence_patience` extensions;
`solve(record_history=True)` + `animate_convergence` replay tree growth and path refinement.
`solve(callback=…, live_plot=True)` hands each step to both, the tree redrawn beside the
callback; the `nearest_backend` setting
selects brute-force or a SciPy ``cKDTree`` for Euclidean nearest queries.

## 7. Graphics And Benchmarks

Facades delegate to `graphical/`. Time plots: `signals=("x", "u", "block:port")`.
Phase plane: matplotlib default. Diagrams: Graphviz layout (notebook SVG;
script Matplotlib window).

**Inspect (`core/inspect.py`).** One text layout for textbook objects:
line 1 is `{TypeName}, …` identity tokens; further lines are two-space
indented `label: value`. One rule places each text: `core/inspect.py` writes the core
objects' (`System`, `Trajectory`, sets, distributions, costs including sums and quadratic
weights, fields) and the layout helpers, and `inspect_text` reads any other object's own
`str`; a tool band's object writes its text beside its class with those helpers
(`PlanningProblem`, a stochastic one with its laws and criterion, `PlanningSolution`,
`MathematicalProgram`, `Optimizer`, `OptimizationResult`, `Simulator`), so the core printer
imports no tool band. `print(obj)` and IPython `_repr_pretty_` read the same text; an
object without one prints its `repr`. The LaTeX summaries follow the same rule
(`latex_inspect` reads an object's own `_repr_latex_`).
Subclasses may append a deep block after the hub summary (`LTISystem`
matrices, `TransferFunction` `G(s)`, `StateObserver` gain and poles).

**Notebook-rich repr.** A bare last expression on a `System` prefers
`_repr_mimebundle_`: `text/plain` inspect plus `image/svg+xml` diagram
when Graphviz is available; `_repr_svg_` remains for SVG-only consumers.
`PlotResult` displays nothing so plots
are not doubled. Optional `_repr_latex_` on `LTISystem`, `QuadraticCost`,
and `MathematicalProgram` renders the textbook equation shape (not the
callable bodies). Optimizer `verbose=True` frames the same inspect lines
inside the solve panel (`core/reporting.py` rules).

Plotly under the `full` extra.

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

The regression gate lives in repo-root `benchmarks/` (`run_regression_check.py`, its
suites, scenarios, synthetic fixtures and committed baselines) — outside the shipped
package, importing minilink like an external user, and not a public contract.
